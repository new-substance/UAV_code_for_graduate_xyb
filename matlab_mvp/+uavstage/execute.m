function run=execute(w,cfg,mode)
%EXECUTE Online controller. No fixture coordinates, world truth or finish().
assert(ismember(mode,{'calibration','closed_loop'}));
book=uavmvp.ProtocolBook(cfg,1,17);history=uavmvp.radio_history_init(cfg);
state=struct('phase','LOCALIZE','pool_ids',[],'fit',[],'fit_ids',[], ...
    'fit_version',-1,'verification_rounds',0,'proof',[],'publish_time_s',NaN);
events={};attempts={};samples={};fits={};checks={};point_record_ids=[];
reserve=uavstage.reservation(cfg);deadline=cfg.stage.platform_resource_s;
reason='ROUTE_EXHAUSTED';bundle5=[];publish_bundle=[];solver_s=0;
compute_delays=[];measured_delays=[];decision_delay=0;
sample('START');
home=cfg.scene.home_xyz_m(:);
takeoff=uavmvp.fly_to(w,[home(1:2);cfg.scene.flight_height_m],cfg,40);
initial=struct('accepted',false,'reason','TAKEOFF_FAILED');
if takeoff.arrived,initial=uavmvp.visit_reference(w,1,cfg,deadline);end
sample('INITIAL_REFERENCE');
if ~initial.accepted
    reason='INITIAL_REFERENCE_FAILED';
else
    route=cfg.stage.route_xy_m;if strcmp(mode,'calibration'),route=route(1:5,:);end
    for index=1:size(route,1)
        q=[route(index,:)';cfg.scene.flight_height_m];
        [~,ref]=min(vecnorm(cfg.scene.reference_centers_xy_m-q(1:2)',2,2));
        ready=false;
        for recovery=1:cfg.stage.navigation_recovery_cap
            if strcmp(mode,'closed_loop')
                d=cfg.motion_and_budget.calculation_model_s_per_decision;
                w.hold(d);decision_delay=decision_delay+d;
                log('DECISION_COST',struct('modeled_delay_s',d,'point',index));
            end
            got=uavstage.acquire_reference(w,ref,cfg,deadline);
            log('REFERENCE',struct('point',index,'attempt',recovery,'status',got));sample('REFERENCE');
            if ~got.accepted,continue;end
            forecast=uavmvp.predict_leg(w.nav,q,cfg,60);
            if ~forecast.arrived || ~forecast.envelope_inside_area,continue;end
            escape=uavmvp.predict_service_visit(w.nav,q,ref,cfg, ...
                cfg.navigation_candidate.reference_observation_s);
            log('APPROACH_ESCAPE',struct('point',index,'accepted',escape.accepted, ...
                'duration_s',escape.duration_s));
            if ~escape.accepted,continue;end
            move=uavmvp.fly_to(w,q,cfg,forecast.duration_s+2);
            if ~move.arrived,continue;end
            margin=uavmvp.reference_margin(w.nav,ref,cfg);
            if margin.nominal_visible,w.observe_reference(ref);end
            sample('SERVICE_ARRIVAL');
            gate=uavstage.navigation_gate(w.nav,reserve.radio_window_s,cfg);
            pred=uavmvp.predict_hold(w.nav,reserve.radio_window_s,cfg);
            reference_target=[cfg.scene.reference_centers_xy_m(ref,:)';cfg.scene.flight_height_m];
            retreat=uavmvp.predict_leg(pred.nav,reference_target,cfg,60,ref);
            back=uavmvp.reference_return_plan(w.nav,cfg,ref);
            retreat_ok=retreat.arrived && retreat.envelope_inside_area && retreat.reference.envelope_inside;
            enough=w.t+reserve.total_s+retreat.duration_s+reserve.post_radio_reference_s+back.reserve_s<=deadline;
            log('ADMISSION',struct('point',index,'gate',gate,'reservation',reserve, ...
                'conditional_return_available',retreat_ok,'return_reserve_s',back.reserve_s, ...
                'retreat_duration_s',retreat.duration_s,'resource_ok',enough));
            if gate.accepted && retreat_ok && enough,ready=true;break;end
        end
        if ~ready
            log('POINT_SKIPPED',struct('point',index,'reason','NAVIGATION_OR_RESOURCE_ADMISSION'));
            if strcmp(mode,'calibration'),reason='CALIBRATION_POINT_NOT_ADMITTED';break;end
            continue;
        end
        start=w.t;w.hold(cfg.motion_and_budget.settle_s);
        if reserve.extra_service_s>0,w.hold(reserve.extra_service_s);end
        accepted=false;record_id=[];
        for retry=1:cfg.stage.transaction_attempt_cap
            control=book.group_control(1,w.t);packets=w.transmit_group(control);
            for k=1:numel(packets),book.receive_identity(packets{k},packets{k}.received_time_s);end
            before=history.last_id;range_start=w.t;
            if book.nodes(1).identity_seen && w.t-book.nodes(1).last_contact_s<=cfg.css.contact_validity_s
                [history,report,record]=uavmvp.execute_range_service(w,book,history,1,cfg);
            else
                report=struct('accepted',false,'reason','NO_CURRENT_IDENTITY');record=[];
            end
            attempts{end+1}=struct('point',index,'retry',retry,'start_s',range_start,'end_s',w.t, ...
                'history_before',before,'history_after',history.last_id,'report',report);
            if report.accepted
                accepted=true;record_id=numel(book.nodes(1).range_records);break;
            end
            if retry<cfg.stage.transaction_attempt_cap,w.hold(cfg.stage.backoff_s);end
        end
        log('SERVICE',struct('point',index,'start_s',start,'end_s',w.t,'accepted',accepted, ...
            'record_id',record_id,'reservation_s',reserve.total_s));sample('SERVICE_END');
        % Complete RTT first, then obtain an ACTUAL reference before solving.
        % Future image success was only a conditional retreat forecast.
        reacquire=uavmvp.visit_reference(w,ref,cfg,deadline);
        log('POST_SERVICE_REFERENCE',struct('point',index,'status',reacquire));sample('POST_SERVICE_REFERENCE');
        if ~reacquire.accepted,reason='POST_SERVICE_REFERENCE_FAILED';break;end
        compute_gate=uavstage.navigation_gate(w.nav,reserve.compute_s+reserve.stop_s,cfg);
        log('COMPUTE_ADMISSION',struct('point',index,'gate',compute_gate));
        if ~compute_gate.accepted,reason='COMPUTE_NAVIGATION_NOT_ADMITTED';break;end
        if ~accepted
            if strcmp(mode,'calibration'),reason='CALIBRATION_POINT_NO_RANGE';break;end
            continue;
        end
        point_record_ids(end+1)=record_id;
        if strcmp(mode,'calibration')
            if numel(point_record_ids)==5
                bundle5=uavstage.bundle(book.nodes(1).range_records,w.nav,history,point_record_ids,cfg);
                reason='CALIBRATION_QUINTET_CAPTURED';break;
            end
            continue;
        end
        pool=state.pool_ids;
        if ~isempty(pool)
            b=uavstage.bundle(book.nodes(1).range_records,w.nav,history,[pool,record_id],cfg);
            if ~uavmvp.distinct_positions(b.anchors,b.Pnav,cfg,[(1:numel(pool))',repmat(numel(pool)+1,numel(pool),1)])
                log('POSITION_IGNORED',struct('record_id',record_id,'reason','NOT_A_NEW_POSITION'));continue;
            end
        end
        state.pool_ids=[pool,record_id];
        if strcmp(state.phase,'VERIFY')
            b=uavstage.bundle(book.nodes(1).range_records,w.nav,history,[state.fit_ids,record_id],cfg);
            clock=tic;
            if state.fit_version~=w.nav.version
                state.fit=uavstage.fit_position(uavstage.subbundle(b,1:4),cfg);state.fit_version=b.version;
                fits{end+1}=uavstage.compact_fit(state.fit);
                log('FOUR_REFRESH',struct('fit',fits{end},'point',index));
            end
            check=uavstage.verify_position(state.fit,b,cfg);elapsed=charge(toc(clock));
            state.verification_rounds=state.verification_rounds+1;checks{end+1}=check;
            log('HELDOUT',check);
            if elapsed>reserve.compute_s,reason='COMPUTE_RESERVATION_OVERRUN';break;end
            if check.passed
                state.phase='DONE';state.publish_time_s=w.t;publish_bundle=b;
                state.proof=struct('passed',true,'fit_ids',state.fit_ids, ...
                    'heldout_id',record_id,'nav_version',b.version,'check',check);
                book.complete(1,state.proof);reason='ONLINE_VERIFIED';
                for retry=1:cfg.planner.stop_notice_attempt_cap
                    command=book.stop_request(1,w.t);packet=w.transmit_stop(command);
                    if ~isempty(packet),book.receive_stop_ack(packet,packet.received_time_s);end
                    if book.nodes(1).stop_ack,break;end
                end
                log('PUBLISHED',struct('time_s',state.publish_time_s,'x',state.fit.x, ...
                    'fit_ids',state.fit_ids,'heldout_id',record_id,'stop_ack',book.nodes(1).stop_ack));
                break;
            end
            state.phase='LOCALIZE';
            if state.verification_rounds>=cfg.localization.verification_round_cap
                reason='VERIFICATION_LIMIT';break;
            end
        elseif numel(state.pool_ids)>=4
            all=uavstage.bundle(book.nodes(1).range_records,w.nav,history,state.pool_ids,cfg);
            pick_cfg=cfg;pick_cfg.localization.group_geometry_rule='local_information';
            probe=[cfg.scene.width_m;cfg.scene.height_m]/2;
            if ~isempty(state.fit) && all_finite(state.fit.x),probe=state.fit.x;end
            required=[];if numel(state.pool_ids)>4,required=record_id;end
            clock=tic;[chosen,selection]=uavmvp.select_four(all,required,pick_cfg,probe);
            if ~isempty(chosen)
                b=uavstage.bundle(book.nodes(1).range_records,w.nav,history,chosen,cfg);
                state.fit=uavstage.fit_position(b,cfg);state.fit_ids=chosen;state.fit_version=b.version;
                fits{end+1}=uavstage.compact_fit(state.fit);
                log('FOUR_FIT',struct('selection',selection,'fit',fits{end}));
                if state.fit.quality,state.phase='VERIFY';end
            end
            elapsed=charge(toc(clock));
            if elapsed>reserve.compute_s,reason='COMPUTE_RESERVATION_OVERRUN';break;end
        end
    end
end
if isfield(cfg.stage,'replay_compute_delays_s')
    assert(numel(compute_delays)==numel(cfg.stage.replay_compute_delays_s), ...
        'uavstage:replayMismatch','A frozen timing trace must match actual decisions.');
end
module_end=w.t;sample('MODULE_END');
returning=uavmvp.return_via_references(w,cfg,deadline);
run=struct('mode',mode,'reason',reason,'state',state,'events',{events},'attempts',{attempts}, ...
    'nav_samples',{samples},'fits',{fits},'checks',{checks},'history',history, ...
    'protocol_nodes',book.nodes,'sessions',{book.sessions},'calibration_bundle5',bundle5, ...
    'publication_bundle',publish_bundle,'module_end_s',module_end,'returning',returning, ...
    'solver_wall_s',solver_s,'compute_delays_s',compute_delays, ...
    'measured_compute_wall_s',measured_delays,'decision_model_delay_s',decision_delay, ...
    'reservation',reserve);
    function log(kind,data)
        events{end+1}=struct('kind',kind,'time_s',w.t,'nav_version',w.nav.version,'data',data);
    end
    function sample(kind)
        samples{end+1}=struct('kind',kind,'time_s',w.t,'p',w.nav.p, ...
            'Pxy',w.nav.P(1:2,1:2),'nav_version',w.nav.version);
    end
    function duration=charge(measured)
        solver_s=solver_s+measured;measured_delays(end+1)=measured;
        duration=measured;
        if isfield(cfg.stage,'replay_compute_delays_s')
            index=numel(compute_delays)+1;
            assert(index<=numel(cfg.stage.replay_compute_delays_s), ...
                'uavstage:replayMismatch','More decisions than the frozen timing trace.');
            duration=cfg.stage.replay_compute_delays_s(index);
        end
        compute_delays(end+1)=duration;w.hold(duration);
        log('SOLVER_COST',struct('charged_delay_s',duration,'measured_wall_s',measured));
    end
end
function yes=all_finite(x),yes=all(isfinite(x));end
