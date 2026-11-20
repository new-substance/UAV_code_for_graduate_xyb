function beliefs=planning_belief(tracker,book,histories,nav,cfg)
%PLANNING_BELIEF Coarse marginal likelihood used ONLY to choose actions.
% Does not replace four-point fitting, heldout validation, or task acceptance.
step=cfg.planner.belief_grid_step_m;
[x,y]=meshgrid(step/2:step:cfg.scene.width_m,step/2:step:cfg.scene.height_m);
points=[x(:),y(:)];beliefs=cell(1,numel(tracker.terminals));
for id=1:numel(beliefs)
    state=tracker.terminals(id);ids=state.pool_ids;
    if ismember(state.phase,{'DONE','FAILED'}),continue;end
    if isempty(ids)
        F=zeros(size(points,1),1);bundle=[];
    else
        bundle=uavmvp.record_bundle(book.nodes(id).range_records,nav,histories{id},ids);
        F=uavmvp.range_nll_batch(points,bundle.anchors,bundle.ranges,bundle.Pnav,bundle.Cradio,cfg.scene.terminal_height_m);
    end
    weights=exp(-.5*(F-min(F)));weights(~isfinite(weights))=0;
    if sum(weights)==0,weights=ones(size(weights));end
    weights=weights/sum(weights);mu=points'*weights;delta=points-mu';
    covariance=delta'*(delta.*weights)+step^2/12*eye(2);
    % Deterministic weighted quadrature. It uses no world/future random stream.
    cdf=cumsum(weights);K=cfg.planner.scenario_count;sample=zeros(K,1);
    for k=1:K,sample(k)=find(cdf>=(k-.5)/K,1);end
    phase=state.phase;n=numel(ids);
    if strcmp(phase,'VERIFY'),need=1;
    elseif n<4,need=5-n;
    else
        % One new fit point plus a fresh heldout are mandatory after failure.
        need=2;
        if ~isempty(state.fit) && isfinite(state.fit.radius)
            need=need+min(2,max(0,ceil(log2(max(1,state.fit.radius/cfg.localization.nominal_region_radius_limit_m)))));
        end
    end
    beliefs{id}=struct('points',points,'weights',weights,'samples',points(sample,:), ...
        'mean',mu,'covariance',covariance,'bundle',bundle,'phase',phase,'need',need,'record_count',n);
end
end
