function status=acquire_reference(w,to,cfg,deadline)
%ACQUIRE_REFERENCE Follow the surveyed graph with actual image updates.
from=w.nav.last_reference_id;
status=struct('accepted',false,'reason','NO_LAST_REFERENCE','steps',{{}});
if ~isfinite(from),return;end
% Recenter on the most recently ACTUALLY observed reference before transit.
step=uavmvp.visit_reference(w,from,cfg,deadline);status.steps{end+1}=step;
if ~step.accepted,status.reason=step.reason;return;end
path=uavmvp.reference_path(cfg,from,to,[]);
if ~path.available,status.reason='REFERENCE_GRAPH_DISCONNECTED';return;end
for j=2:numel(path.ids)
    step=uavmvp.transit_reference(w,path.ids(j-1),path.ids(j),cfg,deadline);
    status.steps{end+1}=step;
    if ~step.accepted,status.reason=step.reason;return;end
end
status.accepted=true;status.reason='REFERENCE_ACQUIRED';
end
