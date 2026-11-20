function status=acquire_safe_reference(w,to,cfg,deadline)
%ACQUIRE_SAFE_REFERENCE T01 per-leg observation deadline preserves return.
from=w.nav.last_reference_id;
status=struct('accepted',false,'reason','NO_LAST_REFERENCE','steps',{{}});
if ~isfinite(from),return;end
path=uavmvp.reference_path(cfg,from,to,[]);
if ~path.available,status.reason='REFERENCE_GRAPH_DISCONNECTED';return;end
for j=1:numel(path.ids)
    target=path.ids(j);plan=uavmvp.departure_repair_plan(w.nav,target,cfg,deadline);
    if ~plan.accepted,status.reason=plan.reason;return;end
    if j==1
        step=uavmvp.visit_reference(w,target,cfg,plan.observation_deadline_s);
    else
        step=uavmvp.transit_reference(w,path.ids(j-1),target,cfg,plan.observation_deadline_s);
    end
    status.steps{end+1}=struct('reference_id',target,'accepted',step.accepted, ...
        'reason',step.reason,'observation_deadline_s',plan.observation_deadline_s, ...
        'return_reserve_s',plan.return_reserve_s);
    if ~step.accepted,status.reason=step.reason;return;end
    actual_return=uavmvp.reference_return_plan(w.nav,cfg,target);
    if ~actual_return.available,status.reason='REFERENCE_RETURN_NOT_AVAILABLE';return;end
    if w.t+actual_return.reserve_s>deadline,status.reason='REFERENCE_RETURN_BUDGET';return;end
end
status.accepted=true;status.reason='REFERENCE_ACQUIRED_WITH_RETURN_RESERVE';
end
