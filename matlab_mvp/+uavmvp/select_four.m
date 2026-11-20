function [ids,detail]=select_four(bundle,required_record,cfg,probe)
%SELECT_FOUR Geometry-only deterministic ranking; measured ranges not used.
% Primary score includes possible distant explanations of synthetic means.
rule=cfg.localization.group_geometry_rule;
assert(ismember(rule,{'global_geometry','local_information'}));
count=numel(bundle.record_ids);ids=[];detail=struct('candidate_count',0,'score',[], ...
    'geometry_rule',rule,'nominal_region',[]);
if count<4,return;end
groups=nchoosek(1:count,4);best=Inf(1,8);points=[];
if strcmp(rule,'global_geometry')
    step=cfg.localization.group_geometry_grid_m;
    [x,y]=meshgrid(unique([0:step:cfg.scene.width_m,cfg.scene.width_m]), ...
        unique([0:step:cfg.scene.height_m,cfg.scene.height_m]));points=[x(:),y(:)];
end
for k=1:size(groups,1)
    group=groups(k,:);records=bundle.record_ids(group);
    if ~isempty(required_record) && ~ismember(required_record,records),continue;end
    ix=reshape(3*(group-1)+(1:3)',1,[]);P=bundle.Pnav(ix,ix);a=bundle.anchors(:,group);
    if ~uavmvp.distinct_positions(a,P,cfg),continue;end
    [~,H,C]=uavmvp.range_model(probe,a,P,bundle.Cradio(group,group),cfg.scene.terminal_height_m);
    [L,flag]=chol(C,'lower');if flag,continue;end
    Z=L\H;I=Z'*Z;if rank(I)<2,continue;end
    Px=I\eye(2);primary=0;proxy=[];
    if strcmp(rule,'global_geometry')
        proxy=uavmvp.geometry_region_proxy(probe,a,P,bundle.Cradio(group,group),cfg,points);
        primary=proxy.radius;
    end
    score=[primary,max(eig(Px)),cond(I),-mean(records),records];
    detail.candidate_count=detail.candidate_count+1;
    different=find(abs(score-best)>1e-12,1);
    if isempty(ids) || (~isempty(different) && score(different)<best(different))
        ids=records;best=score;detail.nominal_region=proxy;
    end
end
if ~isempty(ids),detail.score=best;end
end
