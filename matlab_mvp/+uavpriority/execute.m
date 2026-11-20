function run=execute(w,cfg,mode)
%EXECUTE Truth-free peripheral service with correlated spatial aggregation.
assert(ismember(mode,{'calibration','closed_loop'}));
book=uavmvp.ProtocolBook(cfg,1,17);history=uavmvp.radio_history_init(cfg);
state=struct('phase','LOCALIZE','pool_ids',[],'fit',[],'fit_ids',[], ...
    'fit_version',-1,'verification_rounds',0,'proof',[],'publish_time_s',NaN);
events={};attempts={};samples={};fits={};checks={};point_record_ids=[];macros={};
reserve=uavpriority.reservation(cfg);deadline=cfg.stage.platform_resource_s;
reason='ROUTE_EXHAUSTED';bundle5=[];publish_bundle=[];solver_s=0;
compute_delays=[];measured_delays=[];decision_delay=0;
extra_count=0;extra_fit_count=0;extra_heldout_count=0;route=[];roles=[];route_refs=[];
sample('START');home=cfg.scene.home_xyz_m(:);
preflight=uavpriority.preflight(cfg);log('PREFLIGHT',preflight);
initial=struct('accepted',false,'reason',preflight.reason);
if preflight.accepted
    takeoff=uavmvp.fly_to(w,[home(1:2);cfg.scene.flight_height_m],cfg,40);
    initial.reason='TAKEOFF_FAILED';
    if takeoff.arrived,initial=uavmvp.visit_reference(w,1,cfg,deadline);end
end
log('INITIAL_REFERENCE',initial);sample('INITIAL_REFERENCE');
if ~initial.accepted
    reason='INITIAL_REFERENCE_FAILED';if ~preflight.accepted,reason=preflight.reason;end
else
    route=cfg.stage.route_xy_m;
    if strcmp(mode,'calibration'),route=cfg.priority.calibration_route_xy_m;end
    roles=zeros(size(route,1),1);route_refs=roles;
    for j=1:size(route,1)
        [~,route_refs(j)]=min(vecnorm(cfg.scene.reference_centers_xy_m-route(j,:),2,2));
    end
    index=0;
    while true
        waiting_verify=strcmp(state.phase,'VERIFY') && extra_heldout_count<cfg.priority.extra_heldout_cap;
        need_fit=index>=size(route,1) && strcmp(state.phase,'LOCALIZE') && extra_fit_count<cfg.priority.extra_fit_cap;
        need_extra=strcmp(mode,'closed_loop') && ~isempty(macros) && (waiting_verify || need_fit);
        if need_extra && (index>=size(route,1) || roles(index+1)==0)
            [extra,choice]=uavpriority.next_extra(w.nav,state.fit,route(1:index,:),cfg);
            if choice.available
                route=[route(1:index,:);extra;route(index+1:end,:)];
                roles=[roles(1:index);1;roles(index+1:end)];extra_count=extra_count+1;
                route_refs=[route_refs(1:index);choice.reference_id;route_refs(index+1:end)];
                if waiting_verify,extra_heldout_count=extra_heldout_count+1;purpose='heldout';
                else,extra_fit_count=extra_fit_count+1;purpose='fit';end
                log('EXTRA_SERVICE_PLANNED',struct('target_xy_m',extra,'choice',choice, ...
                    'count',extra_count,'stage',state.phase,'purpose',purpose));
            end
        end
        if index>=size(route,1),break;end
        index=index+1;
        q=[route(index,:)';cfg.scene.flight_height_m];
        ref=route_refs(index);
        center=[cfg.scene.reference_centers_xy_m(ref,:)';cfg.scene.flight_height_m];
        ready=false;
        for recovery=1:cfg.stage.navigation_recovery_cap
            if strcmp(mode,'closed_loop')
                d=cfg.motion_and_budget.calculation_model_s_per_decision;
                w.hold(d);decision_delay=decision_delay+d;
                log('DECISION_COST',struct('modeled_delay_s',d,'point',index));
            end
            got=uavpriority.acquire_reference(w,ref,cfg,deadline);
            log('REFERENCE',struct('point',index,'attempt',recovery,'status',got));sample('REFERENCE');
            if ~got.accepted,continue;end
            back=uavmvp.reference_return_plan(w.nav,cfg,ref);
            if ~back.available || w.t+reserve.total_s+back.reserve_s>deadline,continue;end
            if norm(q-center)>.01
                escape=uavmvp.predict_service_visit(w.nav,q,ref,cfg, ...
                    cfg.navigation_candidate.reference_max_attempts_per_visit*cfg.navigation_candidate.reference_observation_s);
                log('OFFSET_ESCAPE',struct('point',index,'accepted',escape.accepted,'target',q));
                if ~escape.accepted || w.t+escape.duration_s+reserve.total_s+back.reserve_s>deadline,continue;end
                move=uavmvp.fly_to(w,q,cfg,escape.outward.duration_s+2);
                if ~move.arrived,continue;end
                onsite=uavpriority.observe_onsite(w,ref,cfg,deadline);
                log('OFFSET_IMAGE',struct('point',index,'status',onsite));sample('OFFSET_IMAGE');
                if ~onsite.accepted,continue;end
            end
            waiting=uavpriority.wait_reference(w,ref,reserve.extra_service_s,cfg);
            log('WAIT_REFERENCE',struct('point',index,'status',waiting));sample('WAIT_END');
            if ~waiting.accepted,continue;end
            gate=uavpriority.navigation_gate(w.nav,reserve.atomic_radio_s,cfg);
            retreat_report=uavpriority.retreat_gate(w.nav,reserve.atomic_radio_s,ref,cfg);
            retreat=retreat_report.retreat;back=retreat_report.return_plan;
            retreat_ok=retreat_report.accepted;
            enough=back.available && w.t+reserve.total_s-reserve.extra_service_s+back.reserve_s<=deadline;
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
        start=w.t;w.hold(cfg.motion_and_budget.settle_s);children=[];
        for radio_block=1:cfg.priority.burst_recovery_blocks
          block_start=w.t;
          block_goal=min(cfg.priority.cold_services_per_position, ...
              numel(children)+cfg.priority.services_per_atomic_block);
          for burst=numel(children)+1:block_goal
            slot_ok=false;
            for retry=1:cfg.stage.transaction_attempt_cap
                control=book.group_control(1,w.t);packets=w.transmit_group(control);
                for k=1:numel(packets),book.receive_identity(packets{k},packets{k}.received_time_s);end
                before=history.last_id;range_start=w.t;
                if book.nodes(1).identity_seen && w.t-book.nodes(1).last_contact_s<=cfg.css.contact_validity_s
                    [history,report,~]=uavpriority.range_service(w,book,history,1,cfg);
                else
                    report=struct('accepted',false,'reason','NO_CURRENT_IDENTITY');
                end
                attempts{end+1}=struct('point',index,'burst_slot',burst,'retry',retry, ...
                    'start_s',range_start,'end_s',w.t,'history_before',before, ...
                    'history_after',history.last_id,'report',report);
                if report.accepted
                    slot_ok=true;children(end+1)=numel(book.nodes(1).range_records);break;
                end
                if retry<cfg.stage.transaction_attempt_cap,w.hold(cfg.stage.backoff_s);end
            end
            if ~slot_ok,break;end
          end
          log('RADIO_BLOCK',struct('point',index,'block',radio_block,'start_s',block_start, ...
              'end_s',w.t,'reserved_s',reserve.atomic_radio_s,'completed_services',numel(children)));
          if numel(children)==cfg.priority.cold_services_per_position,break;end
          if radio_block==cfg.priority.burst_recovery_blocks,break;end
          recovered=uavpriority.observe_onsite(w,ref,cfg,deadline);
          log('RADIO_BLOCK_RECOVERY',struct('point',index,'status',recovered));sample('RADIO_BLOCK_RECOVERY');
          if ~recovered.accepted,break;end
          gate=uavpriority.navigation_gate(w.nav,reserve.atomic_radio_s,cfg);
          retreat_report=uavpriority.retreat_gate(w.nav,reserve.atomic_radio_s,ref,cfg);
          back=retreat_report.return_plan;
          enough=back.available && w.t+reserve.atomic_radio_s+reserve.compute_s+reserve.stop_s+back.reserve_s<=deadline;
          log('RADIO_BLOCK_ADMISSION',struct('point',index,'gate',gate,'resource_ok',enough));
          if ~gate.accepted || ~retreat_report.accepted || ~enough,break;end
        end
        accepted=numel(children)==cfg.priority.cold_services_per_position;record_id=[];
        if accepted
            record_id=children(end);
            macros{end+1}=struct('id',record_id,'constituent_ids',children, ...
                'weights',ones(1,numel(children))/numel(children),'point',index,'reference_id',ref, ...
                'start_s',start,'end_s',w.t);
        end
        log('SERVICE',struct('point',index,'start_s',start,'end_s',w.t, ...
            'accepted',accepted,'record_id',record_id,'constituent_ids',children, ...
            'atomic_reservation_s',reserve.atomic_radio_s,'block_cap',reserve.block_cap));sample('SERVICE_END');
        reacquire=uavpriority.acquire_reference(w,ref,cfg,deadline);
        log('POST_SERVICE_REFERENCE',struct('point',index,'status',reacquire));sample('POST_SERVICE_REFERENCE');
        if ~reacquire.accepted,reason='POST_SERVICE_REFERENCE_FAILED';break;end
        compute_gate=uavpriority.navigation_gate(w.nav,reserve.compute_s+reserve.stop_s,cfg);
        log('COMPUTE_ADMISSION',struct('point',index,'gate',compute_gate));
        if ~compute_gate.accepted,reason='COMPUTE_NAVIGATION_NOT_ADMITTED';break;end
        if ~accepted
            if strcmp(mode,'calibration'),reason='CALIBRATION_BURST_INCOMPLETE';break;end
            continue;
        end
        point_record_ids(end+1)=record_id;
        if strcmp(mode,'calibration')
            if numel(point_record_ids)==5
                bundle5=uavpriority.bundle(book.nodes(1).range_records,w.nav,history,macros,point_record_ids,cfg);
                reason='CALIBRATION_QUINTET_CAPTURED';break;
            end
            continue;
        end
        pool=state.pool_ids;
        if ~isempty(pool)
            b=uavpriority.bundle(book.nodes(1).range_records,w.nav,history,macros,[pool,record_id],cfg);
            if ~uavmvp.distinct_positions(b.anchors,b.Pnav,cfg,[(1:numel(pool))',repmat(numel(pool)+1,numel(pool),1)])
                log('POSITION_IGNORED',struct('record_id',record_id,'reason','NOT_A_NEW_POSITION'));continue;
            end
        end
        state.pool_ids=[pool,record_id];
        if strcmp(state.phase,'VERIFY')
            b=uavpriority.bundle(book.nodes(1).range_records,w.nav,history,macros,[state.fit_ids,record_id],cfg);
            clock=tic;
            if state.fit_version~=w.nav.version
                state.fit=uavpriority.fit_position(uavpriority.subbundle(b,1:4),cfg);state.fit_version=b.version;
                fits{end+1}=uavstage.compact_fit(state.fit);
                log('FOUR_REFRESH',struct('fit',fits{end},'point',index));
            end
            check=uavpriority.verify_position(state.fit,b,cfg);elapsed=charge(toc(clock));
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
            % This heldout round is finished. Its valid history can now be
            % used for a NEW four-fit, before seeking another fresh position.
            clock=tic;
            [candidate,chosen,selection]=uavpriority.fit_pool(book.nodes(1).range_records,w.nav, ...
                history,macros,state.pool_ids,record_id,cfg,state.fit);
            if ~isempty(candidate)
                state.fit=candidate;state.fit_ids=chosen;state.fit_version=candidate.nav_version;
                fits{end+1}=uavstage.compact_fit(candidate);
                log('RECOVERY_FOUR_FIT',struct('selection',selection,'fit',fits{end}));
                if candidate.quality,state.phase='VERIFY';end
            end
            additional=charge(toc(clock));
            if elapsed+additional>reserve.compute_s,reason='COMPUTE_RESERVATION_OVERRUN';break;end
        elseif numel(state.pool_ids)>=4
            required=[];if numel(state.pool_ids)>4,required=record_id;end
            clock=tic;
            [candidate,chosen,selection]=uavpriority.fit_pool(book.nodes(1).range_records,w.nav, ...
                history,macros,state.pool_ids,required,cfg,state.fit);
            if ~isempty(candidate)
                state.fit=candidate;state.fit_ids=chosen;state.fit_version=candidate.nav_version;
                fits{end+1}=uavstage.compact_fit(candidate);
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
if preflight.accepted
    returning=uavmvp.return_via_references(w,cfg,deadline);
else
    returning=struct('reason','NOT_LAUNCHED','landing',struct('reason','NOT_LAUNCHED'));
end
run=struct('mode',mode,'reason',reason,'state',state,'events',{events},'attempts',{attempts}, ...
    'nav_samples',{samples},'fits',{fits},'checks',{checks},'history',history, ...
    'protocol_nodes',book.nodes,'sessions',{book.sessions},'calibration_bundle5',bundle5, ...
    'publication_bundle',publish_bundle,'module_end_s',module_end,'returning',returning, ...
    'solver_wall_s',solver_s,'compute_delays_s',compute_delays, ...
    'measured_compute_wall_s',measured_delays,'decision_model_delay_s',decision_delay, ...
    'reservation',reserve,'macros',{macros},'preflight',preflight, ...
    'extra_service_count',extra_count,'extra_fit_count',extra_fit_count, ...
    'extra_heldout_count',extra_heldout_count,'planned_route_xy_m',route,'route_roles',roles, ...
    'planned_reference_ids',route_refs);
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
            delay_index=numel(compute_delays)+1;
            assert(delay_index<=numel(cfg.stage.replay_compute_delays_s), ...
                'uavstage:replayMismatch','More decisions than the frozen timing trace.');
            duration=cfg.stage.replay_compute_delays_s(delay_index);
        end
        compute_delays(end+1)=duration;w.hold(duration);
        log('SOLVER_COST',struct('charged_delay_s',duration,'measured_wall_s',measured));
    end
end
