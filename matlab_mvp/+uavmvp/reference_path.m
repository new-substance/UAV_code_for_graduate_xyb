function path=reference_path(cfg,start,goal,blocked)
%REFERENCE_PATH Conditional mapped path; actual visibility is checked per leg.
if nargin<4,blocked=[];end
xy=cfg.scene.reference_centers_xy_m;n=size(xy,1);cost=Inf(1,n);cost(start)=0;
previous=zeros(1,n);used=false(1,n);used(blocked)=true;
for k=1:n
    values=cost;values(used)=Inf;[value,j]=min(values);
    if ~isfinite(value),break;end
    used(j)=true;if j==goal,break;end
    for i=find(~used)
        d=norm(xy(i,:)-xy(j,:));
        if d>cfg.sim.reference_graph_edge_limit_m,continue;end
        new=value+d/cfg.motion_and_budget.horizontal_max_speed_mps+cfg.sim.reference_leg_overhead_s;
        if new<cost(i),cost(i)=new;previous(i)=j;end
    end
end
ids=[];
if isfinite(cost(goal)) && ~ismember(goal,blocked)
    ids=goal;while ids(1)~=start,ids=[previous(ids(1)),ids];end %#ok<AGROW>
end
path=struct('available',~isempty(ids),'ids',ids,'time_proxy_s',cost(goal));
end
