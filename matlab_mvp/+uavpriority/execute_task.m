function out=execute_task(cfg,seed,count,writer,rule)
%EXECUTE_TASK One physical flight/navigation state, separate terminal states.
% This integration is gated; source presence does not release a new mission.
uavmvp.review_gate();
assert(ismember(count,[1,5,10]) && ismember(rule,{'stage','cost'}));
cfg.scene.terminal_counts=count;cfg.sim.compact_recording=true;
w=uavmvp.MotionWorld(cfg,seed);w.configure_radio(count,17);
book=uavmvp.ProtocolBook(cfg,count,17);states=cell(1,count);histories=states;macros=states;
for id=1:count,histories{id}=uavmvp.radio_history_init(cfg);macros{id}={};end
deadline=cfg.stage.platform_resource_s;r=uavpriority.reservation(cfg);
nr=size(cfg.scene.reference_centers_xy_m,1);attempts=zeros(2*nr,count);
extra_fit=zeros(1,count);extra_verify=zeros(1,count);feedback_count=0;
feedback_delivered=0;
cover=cfg.stage.route_xy_m;cover_index=0;last_extra=false;reason='ROUTE_EXHAUSTED';
exception=[];launched=false;returning=struct('arrived',false,'reason','NOT_LAUNCHED');
try
    gate=uavpriority.preflight(cfg);emit('PREFLIGHT',gate);
    if gate.accepted
        home=cfg.scene.home_xyz_m(:);launched=true;
        takeoff=uavmvp.fly_to(w,[home(1:2);cfg.scene.flight_height_m],cfg,40);
        assert(takeoff.arrived,'uavproject:TakeoffFailed','Takeoff did not arrive.');
        first=uavmvp.visit_reference(w,1,cfg,deadline);
        emit('INITIAL_REFERENCE',struct('accepted',first.accepted,'reason',first.reason));
        assert(first.accepted,'uavproject:InitialReferenceFailed','Initial actual reference failed.');
        while true
            if all([book.nodes.completed]),reason='ALL_ONLINE_VERIFIED';break;end
            [q,ref,owner,choice]=next_point();
            if isempty(q),break;end
            if owner>0
                feedback_count=feedback_count+1;
                if strcmp(states{owner}.phase,'VERIFY'),extra_verify(owner)=extra_verify(owner)+1;
                else,extra_fit(owner)=extra_fit(owner)+1;end
                attempts(choice.candidate_id,owner)=attempts(choice.candidate_id,owner)+1;
                emit('POSITION_FEEDBACK',struct('address',owner,'choice',choice));
            end
            w.hold(cfg.motion_and_budget.calculation_model_s_per_decision);
            [ready,why]=prepare(q,ref);
            emit('POINT_ADMISSION',struct('xy_m',q,'reference_id',ref,'accepted',ready,'reason',why, ...
                'estimated_uav_p_m',w.nav.p,'Pxy_m2',w.nav.P(1:2,1:2),'nav_version',w.nav.version));
            if ~ready,continue;end
            targets=find(~[book.nodes.completed]);responded=false(1,count);
            targets=targets(cellfun(@(s)isempty(s) || ~strcmp(s.phase,'FAILED'),states(targets)));
            for group=unique(ceil(targets/cfg.css.group_size))
                ids=targets(ceil(targets/cfg.css.group_size)==group);
                control=book.group_control(ids,w.t);packets=w.transmit_group(control);
                for k=1:numel(packets)
                    report=book.receive_identity(packets{k},packets{k}.received_time_s);
                    if report.accepted,responded(packets{k}.address)=true;end
                end
            end
            emit('CONTACT',struct('targets',targets,'responded',find(responded)));
            % All first contacts precede supplemental work at this point.
            for id=find(responded)
                if book.nodes(id).completed,continue;end
                if ~isempty(states{id}) && ~isempty(states{id}.pool_ids)
                    previous=uavpriority.bundle(book.nodes(id).range_records,w.nav,histories{id}, ...
                        macros{id},states{id}.pool_ids,cfg);
                    if min(vecnorm(previous.anchors(1:2,:)'-q,2,2))< ...
                            cfg.localization.distinct_horizontal_separation_m+1
                        emit('SERVICE_SKIPPED',struct('address',id,'reason','KNOWN_SPATIAL_POSITION'));continue;
                    end
                end
                if owner~=id
                    candidates=[cfg.scene.reference_centers_xy_m;uavpriority.inset_candidates(cfg)];
                    k=find(vecnorm(candidates-q,2,2)<1e-6,1);
                    if ~isempty(k),attempts(k,id)=attempts(k,id)+1;end
                end
                [ready,why]=prepare(q,ref);
                if ~ready,emit('SERVICE_SKIPPED',struct('address',id,'reason',why));continue;end
                [children,report]=serve(id,ref);emit('SPATIAL_SERVICE',report);
                got=uavpriority.acquire_reference(w,ref,cfg,deadline);
                if ~got.accepted,reason='POST_SERVICE_REFERENCE_FAILED';break;end
                if numel(children)~=cfg.priority.cold_services_per_position,prune();continue;end
                rid=children(end);macros{id}{end+1}=struct('id',rid,'constituent_ids',children);
                pool=[];if ~isempty(states{id}),pool=states{id}.pool_ids;end
                b=uavpriority.bundle(book.nodes(id).range_records,w.nav,histories{id},macros{id},[pool,rid],cfg);
                nav_gate=uavpriority.navigation_gate(w.nav,r.compute_s+r.stop_s,cfg);
                if ~nav_gate.accepted,reason='COMPUTE_NAVIGATION_NOT_ADMITTED';break;end
                timer=tic;[states{id},detail]=uavpriority.update_localization(states{id},b,cfg);
                elapsed=toc(timer);w.hold(elapsed);
                if owner==id && ismember(rid,states{id}.pool_ids),feedback_delivered=feedback_delivered+1;end
                emit('LOCALIZATION',struct('address',id,'phase',states{id}.phase, ...
                    'solver_wall_and_charged_s',elapsed,'detail',detail));
                if elapsed>r.compute_s,reason='COMPUTE_RESERVATION_OVERRUN';break;end
                if strcmp(states{id}.phase,'DONE')
                    book.complete(id,states{id}.proof);
                    for retry=1:cfg.planner.stop_notice_attempt_cap
                        packet=w.transmit_stop(book.stop_request(id,w.t));
                        if ~isempty(packet),book.receive_stop_ack(packet,packet.received_time_s);end
                        if book.nodes(id).stop_ack,break;end
                    end
                    emit('PUBLISHED',struct('address',id,'x',states{id}.fit.x, ...
                        'proof',states{id}.proof,'stop_ack',book.nodes(id).stop_ack));
                end
                prune();
            end
            if ismember(reason,{'POST_SERVICE_REFERENCE_FAILED', ...
                    'COMPUTE_NAVIGATION_NOT_ADMITTED','COMPUTE_RESERVATION_OVERRUN'}),break;end
        end
    else
        reason=gate.reason;
    end
catch problem
    reason='EXCEPTION';exception=bounded_exception(problem);
end
% Returning is still attempted after an event quota exception, without writing
% more events. The reserved outcome records whether the actual return worked.
if launched
    try
        w.prune([]);returning=uavmvp.return_via_references(w,cfg,deadline);
    catch problem
        returning=struct('arrived',false,'reason','RETURN_EXCEPTION');
        if isempty(exception),exception=bounded_exception(problem);end
    end
end
data=w.finish();nodes=repmat(struct('address',0,'discovered',false,'online_completed',false, ...
    'phase','NOT_LOCALIZED','estimate_xy_m',[],'true_position_xy_m',[],'true_error_m',NaN,'false_accept',false, ...
    'quality_radius_m',NaN,'verification_rounds',0,'stop_ack',false,'deep_sleep',false),1,count);
for id=1:count
    s=states{id};nodes(id).address=id;nodes(id).discovered=book.nodes(id).discovered;
    nodes(id).true_position_xy_m=data.radio_truth.terminal_positions(1:2,id);
    nodes(id).online_completed=book.nodes(id).completed;nodes(id).stop_ack=book.nodes(id).stop_ack;
    if ~isempty(s)
        nodes(id).phase=s.phase;nodes(id).verification_rounds=s.verification_rounds;
        if ~isempty(s.fit)
            nodes(id).estimate_xy_m=s.fit.x;nodes(id).quality_radius_m=s.fit.radius;
            nodes(id).true_error_m=norm(s.fit.x-data.radio_truth.terminal_positions(1:2,id));
        end
    end
    nodes(id).false_accept=nodes(id).online_completed && (~isfinite(nodes(id).true_error_m) || ...
        nodes(id).true_error_m>cfg.localization.offline_position_error_limit_m);
    nodes(id).deep_sleep=data.radio_truth.device_states{id}.deep_sleep;
end
landed=false;
if isfield(returning,'landing'),landed=returning.landing.command_completed;end
returned=data.physically_landed && landed && returning.arrived && data.return_error_m<=cfg.motion_and_budget.return_position_tolerance_m;
out=struct('reason',reason,'exception',exception,'count',count,'policy',rule, ...
    'nodes',nodes,'all_discovered',all([nodes.discovered]),'all_online_completed',all([nodes.online_completed]), ...
    'false_accept_count',sum([nodes.false_accept]),'returned',returned, ...
    'return_reason',returning.reason,'return_error_m',data.return_error_m,'total_time_s',data.time_s, ...
    'physical_home_xyz_m',data.true_home,'physical_final_xyz_m',data.true_position, ...
    'physically_landed',data.physically_landed,'return_command_completed',landed,'return_arrived',returning.arrived, ...
    'position_feedback_decisions',feedback_count,'position_feedback_delivered',feedback_delivered, ...
    'extra_fit_plans',extra_fit,'extra_heldout_plans',extra_verify, ...
    'end_to_end_success',all([nodes.online_completed]) && ~any([nodes.false_accept]) && ...
        all([nodes.stop_ack]) && all([nodes.deep_sleep]) && returned);
    function [q,ref,owner,choice]=next_point()
        q=[];ref=NaN;owner=0;choice=[];n_coverage_left=cover_index<size(cover,1);
        if ~n_coverage_left || ~last_extra
            % VERIFY gets a fresh opportunity without suspending coverage.
            for n_phase={'VERIFY','LOCALIZE'}
                if n_coverage_left && strcmp(n_phase{1},'LOCALIZE'),continue;end
                for n_address=1:count
                    n_s=states{n_address};if isempty(n_s) || ~strcmp(n_s.phase,n_phase{1}) || isempty(n_s.fit),continue;end
                    if strcmp(n_phase{1},'VERIFY')
                        if extra_verify(n_address)>=cfg.priority.extra_heldout_cap,continue;end
                    elseif extra_fit(n_address)>=cfg.priority.extra_fit_cap,continue;
                    end
                    n_b=uavpriority.bundle(book.nodes(n_address).range_records,w.nav,histories{n_address}, ...
                        macros{n_address},n_s.pool_ids,cfg);
                    n_selected=uavpriority.choose_service_point(w.nav,n_s.phase,n_s.fit,n_b.anchors(1:2,:)',attempts(:,n_address),cfg,rule);
                    if n_selected.available
                        q=n_selected.xy_m;ref=n_selected.reference_id;owner=n_address;choice=n_selected;last_extra=true;return;
                    end
                end
            end
        end
        if n_coverage_left
            cover_index=cover_index+1;q=cover(cover_index,:);
            [~,ref]=min(vecnorm(cfg.scene.reference_centers_xy_m-q,2,2));last_extra=false;
        end
    end
    function [ready,why]=prepare(xy,ref)
        ready=false;why='NAVIGATION_OR_RETURN_ADMISSION';
        p_q=[xy';cfg.scene.flight_height_m];p_center=[cfg.scene.reference_centers_xy_m(ref,:)';cfg.scene.flight_height_m];
        for p_recovery=1:cfg.stage.navigation_recovery_cap
            p_got=uavpriority.acquire_reference(w,ref,cfg,deadline);if ~p_got.accepted,continue;end
            p_back=uavmvp.reference_return_plan(w.nav,cfg,ref);
            if ~p_back.available || w.t+r.total_s+p_back.reserve_s>deadline,why='RETURN_RESOURCE';continue;end
            if norm(p_q-p_center)>.01
                p_escape=uavmvp.predict_service_visit(w.nav,p_q,ref,cfg, ...
                    cfg.navigation_candidate.reference_max_attempts_per_visit*cfg.navigation_candidate.reference_observation_s);
                if ~p_escape.accepted || w.t+p_escape.duration_s+r.total_s+p_back.reserve_s>deadline,continue;end
                p_moved=uavmvp.fly_to(w,p_q,cfg,p_escape.outward.duration_s+2);if ~p_moved.arrived,continue;end
                p_onsite=uavpriority.observe_onsite(w,ref,cfg,deadline);if ~p_onsite.accepted,continue;end
            end
            p_waiting=uavpriority.wait_reference(w,ref,r.extra_service_s,cfg);if ~p_waiting.accepted,continue;end
            p_gate=uavpriority.navigation_gate(w.nav,r.atomic_radio_s,cfg);
            p_retreat=uavpriority.retreat_gate(w.nav,r.atomic_radio_s,ref,cfg);p_back=p_retreat.return_plan;
            p_enough=p_back.available && w.t+r.total_s-r.extra_service_s+p_back.reserve_s<=deadline;
            if p_gate.accepted && p_retreat.accepted && p_enough,ready=true;why='ADMITTED';return;end
        end
    end
    function [children,report]=serve(id,ref)
        v_start=w.t;children=[];w.hold(cfg.motion_and_budget.settle_s);v_tries=0;
        for v_block=1:cfg.priority.burst_recovery_blocks
            v_goal=min(cfg.priority.cold_services_per_position,numel(children)+cfg.priority.services_per_atomic_block);
            for v_slot=numel(children)+1:v_goal
                v_ok=false;
                for v_retry=1:cfg.stage.transaction_attempt_cap
                    v_tries=v_tries+1;v_control=book.group_control(id,w.t);v_packets=w.transmit_group(v_control);
                    for v_k=1:numel(v_packets),book.receive_identity(v_packets{v_k},v_packets{v_k}.received_time_s);end
                    if book.nodes(id).identity_seen && w.t-book.nodes(id).last_contact_s<=cfg.css.contact_validity_s
                        [histories{id},v_rr,~]=uavpriority.range_service(w,book,histories{id},id,cfg);
                        if v_rr.accepted,children(end+1)=numel(book.nodes(id).range_records);v_ok=true;break;end
                    end
                    if v_retry<cfg.stage.transaction_attempt_cap,w.hold(cfg.stage.backoff_s);end
                end
                if ~v_ok,break;end
            end
            if numel(children)==cfg.priority.cold_services_per_position || v_block==cfg.priority.burst_recovery_blocks,break;end
            v_image=uavpriority.observe_onsite(w,ref,cfg,deadline);if ~v_image.accepted,break;end
            v_gate=uavpriority.navigation_gate(w.nav,r.atomic_radio_s,cfg);
            v_escape=uavpriority.retreat_gate(w.nav,r.atomic_radio_s,ref,cfg);v_back=v_escape.return_plan;
            if ~v_gate.accepted || ~v_escape.accepted || ~v_back.available || ...
                    w.t+r.atomic_radio_s+r.compute_s+r.stop_s+v_back.reserve_s>deadline,break;end
        end
        report=struct('address',id,'start_s',v_start,'end_s',w.t,'tries',v_tries, ...
            'constituent_ids',children,'qualified_spatial_record',numel(children)==cfg.priority.cold_services_per_position);
    end
    function prune()
        pr_keep=[];
        for pr_address=1:count
            pr_s=states{pr_address};if isempty(pr_s) || ismember(pr_s.phase,{'DONE','FAILED'}),continue;end
            for pr_rid=pr_s.pool_ids
                pr_ix=find(cellfun(@(m)m.id==pr_rid,macros{pr_address}),1);
                pr_records=[book.nodes(pr_address).range_records{macros{pr_address}{pr_ix}.constituent_ids}];
                pr_keep=[pr_keep,[pr_records.effective_nav_snapshot_id]]; %#ok<AGROW>
            end
        end
        w.prune(unique(pr_keep));
    end
    function emit(kind,payload)
        writer.event(kind,w.t,payload);
    end
end
function out=bounded_exception(problem)
message=problem.message;message=message(1:min(512,numel(message)));
out=struct('identifier',problem.identifier,'message',message);
end
