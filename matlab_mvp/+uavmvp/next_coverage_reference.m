function [choice,cursor]=next_coverage_reference(order,cursor,visits,current,cfg)
%NEXT_COVERAGE_REFERENCE Cyclic mapped order, independent of terminal beliefs.
% Selecting a mapped route does not certify its actual visibility or budget.
order=order(:)';n=numel(order);
assert(isequal(sort(order),1:size(cfg.scene.reference_centers_xy_m,1)));
assert(isscalar(cursor) && cursor>=1 && cursor<=n && cursor==floor(cursor));
assert(numel(visits)==n && all(visits>=0) && all(visits==floor(visits)));
choice=struct('accepted',false,'reason','COVERAGE_ATTEMPT_LIMIT', ...
    'selected',[],'scanned_ids',[]);
for k=1:n
    cursor=mod(cursor,n)+1;id=order(cursor);
    choice.scanned_ids(end+1)=id;
    if visits(id)>=cfg.scene.max_sweeps,continue;end
    path=uavmvp.reference_path(cfg,current,id);
    choice.selected=struct('reference_id',id,'path',path);
    choice.accepted=path.available;
    if path.available,choice.reason='FIXED_COVERAGE_ORDER';
    else,choice.reason='REFERENCE_GRAPH_DISCONNECTED';end
    return;
end
end
