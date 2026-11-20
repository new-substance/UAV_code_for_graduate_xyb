function b=bundle(records,nav,history,macros,ids,cfg)
%BUNDLE Average complete services at each spatial visit, retain all correlations.
ids=reshape(ids,1,[]);source_ids=[];selected=cell(1,numel(ids));
for k=1:numel(ids)
    index=find(cellfun(@(m)m.id==ids(k),macros),1);assert(~isempty(index));
    selected{k}=macros{index};source_ids=[source_ids,selected{k}.constituent_ids]; %#ok<AGROW>
end
assert(numel(unique(source_ids))==numel(source_ids),'uavpriority:overlap','Spatial records cannot share service outcomes.');
source=uavstage.bundle(records,nav,history,source_ids,cfg);
W=zeros(numel(ids),numel(source_ids));bounds=zeros(numel(ids),1);offset=0;
for k=1:numel(ids)
    n=numel(selected{k}.constituent_ids);
    assert(n==cfg.priority.cold_services_per_position,'uavpriority:incompleteMacro','A spatial record needs its declared complete service count.');
    ix=offset+(1:n);offset=offset+n;
    W(k,ix)=1/n;
    a=source.anchors(:,ix);mean_a=mean(a,2);
    spatial=reshape(3*(ix-1)+(1:3)',1,[]);P=source.Pnav(spatial,spatial);
    average=kron(ones(1,n)/n,eye(3));span=0;
    for j=1:n
        J=-average;J(:,3*j-2:3*j)=J(:,3*j-2:3*j)+eye(3);
        span=max(span,norm(a(:,j)-mean_a)+3*sqrt(max(0,max(eig(J*P*J')))));
    end
    meanP=average*P*average';height=mean_a(3)-cfg.scene.terminal_height_m-3*sqrt(meanP(3,3));
    curvature=Inf;if height>0,curvature=span^2/(2*height);end
    errors=cellfun(@(id)records{id}.short_stop_bias_bound_m,num2cell(selected{k}.constituent_ids));
    bounds(k)=mean(errors)+curvature;
end
J=kron(W,eye(3));
b=struct('anchors',source.anchors*W','ranges',W*source.ranges, ...
    'Pnav',J*source.Pnav*J','Pnavigation',J*source.Pnavigation*J', ...
    'Ptiming',J*source.Ptiming*J','Cradio',W*source.Cradio*W', ...
    'record_ids',ids,'nav_ids',ids,'version',source.version, ...
    'navigation_covariance_scale',cfg.stage.navigation_covariance_scale, ...
    'model_bias_bounds_m',bounds,'aggregation_matrix',W, ...
    'source_record_ids',source_ids,'source_bundle',source, ...
    'constituent_ids',{cellfun(@(m)m.constituent_ids,selected,'UniformOutput',false)});
end
