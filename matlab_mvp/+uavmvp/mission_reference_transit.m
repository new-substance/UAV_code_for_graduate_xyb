function result=mission_reference_transit(world,from_id,to_id,cfg,deadline)
%MISSION_REFERENCE_TRANSIT At most one observed departure repair and retry.
% No terminal service occurs here. Target-observation failures retain the
% existing target recovery; an unobserved target is not labelled unavailable.
result=struct('accepted',false,'reason','','from_reference_id',from_id,'id',to_id, ...
    'start_s',world.t,'end_s',world.t,'steps',{{}},'initial_gate',[], ...
    'failure_response',[],'repair_input',[],'repair_gate',[], ...
    'repair_attempted',false,'repair_acquired',false,'repair_posterior',[], ...
    'retry_gate',[],'retry_attempted',false);
gate=uavmvp.reference_transit_gate(world.nav,to_id,cfg,deadline);
result.initial_gate=gate;result.reason=gate.reason;if ~gate.accepted,return;end
step=uavmvp.transit_reference(world,from_id,to_id,cfg,deadline);
result.steps{end+1}=step;result.accepted=step.accepted;result.reason=step.reason;result.end_s=world.t;
if step.accepted,return;end
response=uavmvp.return_failure_response(step,from_id);result.failure_response=response;
if ~response.repair_departure,return;end
result.repair_input=uavmvp.nav_prune(world.nav,[]);
plan=uavmvp.departure_repair_plan(world.nav,from_id,cfg,deadline);
result.repair_gate=plan;
if ~plan.accepted,result.reason=plan.reason;return;end
result.repair_attempted=true;
repair=uavmvp.visit_reference(world,from_id,cfg,plan.observation_deadline_s);
repair.kind='MISSION_DEPARTURE_REPAIR';result.steps{end+1}=repair;
result.repair_acquired=repair.accepted;result.reason=repair.reason;result.end_s=world.t;
if ~repair.accepted,return;end
result.repair_posterior=uavmvp.nav_prune(world.nav,[]);
% This new gate uses the actual posterior and current time, never a planned
% covariance reduction. The retry has no further departure-repair loop.
gate=uavmvp.reference_transit_gate(world.nav,to_id,cfg,deadline);
result.retry_gate=gate;result.reason=gate.reason;if ~gate.accepted,return;end
result.retry_attempted=true;
retry=uavmvp.transit_reference(world,from_id,to_id,cfg,deadline);
retry.kind='MISSION_TRANSIT_RETRY';result.steps{end+1}=retry;
result.accepted=retry.accepted;result.reason=retry.reason;result.end_s=world.t;
end
