function [histories,result]=execute_planned_service(world,reference_id,target,acquire,book,tracker,histories,cfg,deadline,visit_id)
%EXECUTE_PLANNED_SERVICE Execute only currently feasible stages, then re-evaluate.
result=struct('reason','NOT_STARTED','visit',[],'movement',[],'reacquisition',[], ...
    'onsite_reference',[],'planned_target',target,'service_target',[], ...
    'start_s',world.t,'end_s',world.t,'navigation_failed',false,'budget_stop',false);
if acquire
    pre=uavmvp.predict_service_visit(world.nav,target,reference_id,cfg, ...
        cfg.navigation_candidate.reference_max_attempts_per_visit*cfg.navigation_candidate.reference_observation_s);
    frame=uavmvp.predict_hold(pre.outward.nav,cfg.navigation_candidate.reference_observation_s,cfg);
    margin=uavmvp.reference_margin(frame.nav,reference_id,cfg);
    escape=uavmvp.reference_return_plan(pre.reacquisition.nav,cfg,reference_id);
    if ~pre.accepted || ~margin.envelope_inside || ~escape.available
        result.reason='ACQUISITION_FORECAST_REJECTED';return;
    end
    if world.t+pre.duration_s+escape.reserve_s>deadline
        result.reason='ACQUISITION_RETURN_BUDGET';result.budget_stop=true;return;
    end
    move=uavmvp.fly_to(world,target,cfg,pre.outward.duration_s+2);result.movement=move;
    if ~move.arrived,result.navigation_failed=true;result.reason='ACQUISITION_MOVEMENT_FAILED';result.end_s=world.t;return;end
    accepted=false;
    for k=1:cfg.navigation_candidate.reference_max_attempts_per_visit
        [~,r]=world.observe_reference(reference_id);accepted=accepted || r.accepted;
    end
    if ~accepted,result.navigation_failed=true;result.reason='ACQUISITION_REFERENCE_FAILED';result.end_s=world.t;return;end
end
world.hold(cfg.motion_and_budget.calculation_model_s_per_decision);
eligible=false(1,numel(book.nodes));
% A corrected pose need not coincide with the nominal geometric waypoint.
% If it is already a useful new location, serve there; the RTT still binds
% the actual navigation snapshots, not the originally planned coordinates.
for id=1:numel(eligible),eligible(id)=uavmvp.can_add_position(tracker,book,id,world.nav,cfg);end
if any(eligible),target=world.nav.p;end
result.service_target=target;
arrival=uavmvp.predict_leg(world.nav,target,cfg,60);
for id=1:numel(eligible),eligible(id)=uavmvp.can_add_position(tracker,book,id,arrival.nav,cfg);end
window=min(cfg.motion_and_budget.stop_max_dwell_s,uavmvp.first_round_budget(cfg,find(eligible))+cfg.planner.optional_service_window_s);
onsite_s=cfg.navigation_candidate.reference_max_attempts_per_visit*cfg.navigation_candidate.reference_observation_s;
forecast=uavmvp.predict_service_visit(world.nav,target,reference_id,cfg,window,onsite_s);
back=uavmvp.reference_return_plan(forecast.reacquisition.nav,cfg,reference_id);
if ~forecast.accepted || ~back.available
    result.reason='SERVICE_FORECAST_REJECTED';
elseif world.t+forecast.duration_s+back.reserve_s>deadline
    result.reason='SERVICE_RETURN_BUDGET';result.budget_stop=true;
else
    move=uavmvp.fly_to(world,target,cfg,forecast.outward.duration_s+2);result.movement=move;
    if move.arrived
        [histories,result.visit]=uavmvp.service_stop(world,book,tracker,histories,cfg,visit_id,min(deadline-back.reserve_s,world.t+window));
        result.reason=result.visit.reason;
    else,result.reason='SERVICE_MOVEMENT_FAILED';result.navigation_failed=true;end
end
% Try the same two real images here. The pre-service forecast retained the
% fallback flight to the marker center if on-site reacquisition is unavailable.
result.onsite_reference=uavmvp.reobserve_reference(world,reference_id,cfg,deadline);
if result.onsite_reference.accepted
    result.reacquisition=result.onsite_reference;
else
    result.reacquisition=uavmvp.visit_reference(world,reference_id,cfg,deadline);
end
result.navigation_failed=result.navigation_failed || ~result.reacquisition.accepted;
if result.navigation_failed,result.reason='POST_SERVICE_REFERENCE_FAILED';end
tracker.refresh_completed(world.nav,histories);result.end_s=world.t;
end
