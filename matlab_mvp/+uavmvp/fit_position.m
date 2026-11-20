function fit = fit_position(anchors,ranges,Pnav,Cradio,cfg)
%FIT_POSITION Exactly four locations, global grid + bounded local likelihood.
assert(size(anchors,2)==4 && numel(ranges)==4,'uavmvp:fourPoints','Fit requires exactly four spatial positions.');
assert(all(size(Pnav)==[12,12]) && all(size(Cradio)==[4,4]));
assert(all(isfinite(anchors),'all') && all(isfinite(ranges)));
fit=emptyFit();
if ~uavmvp.distinct_positions(anchors,Pnav,cfg)
    fit.reason='SPATIAL_POSITIONS_NOT_DISTINCT';return;
end
step=cfg.localization.grid_initial_m;
[xx,yy]=meshgrid(unique([0:step:cfg.scene.width_m,cfg.scene.width_m]), ...
    unique([0:step:cfg.scene.height_m,cfg.scene.height_m]));
points=[xx(:),yy(:)];
F=uavmvp.range_nll_batch(points,anchors,ranges,Pnav,Cradio,cfg.scene.terminal_height_m);
surface=reshape(F,size(xx));padded=Inf(size(surface)+2);padded(2:end-1,2:end-1)=surface;
ismin=isfinite(surface);
for a=0:2
    for b=0:2
        if a==1 && b==1,continue;end
        ismin=ismin & surface<=padded((1:size(surface,1))+a,(1:size(surface,2))+b);
    end
end
ids=find(ismin(:));[~,order]=sort(F(ids));ids=ids(order);
if isempty(ids),fit.reason='NO_FINITE_LIKELIHOOD';return;end
seed_cap=16;seed_truncated=numel(ids)>seed_cap;
ids=ids(1:min(seed_cap,numel(ids)));
solutions=zeros(2,numel(ids));values=Inf(1,numel(ids));flags=zeros(1,numel(ids));
for k=1:numel(ids)
    [solutions(:,k),values(k),flags(k)]=uavmvp.local_range_solve(points(ids(k),:),anchors,ranges,Pnav,Cradio,cfg);
end
[value,best]=min(values);x=solutions(:,best);solver_flag=flags(best);
[d,H,C]=uavmvp.range_model(x,anchors,Pnav,Cradio,cfg.scene.terminal_height_m);
[L,invalid]=chol(C,'lower');
if invalid~=0 || ~isfinite(value),fit.reason='INVALID_COVARIANCE';return;end
Z=L\H;information=Z'*Z;
fit.x=x;fit.objective=value;fit.residual=ranges(:)-d;
fit.fit_stat=sum((L\fit.residual).^2);fit.condition=cond(information);
fit.local_exit_flags=flags;fit.seed_truncated=seed_truncated;
if rank(information)<2 || ~isfinite(fit.condition)
    fit.reason='DEGENERATE_GEOMETRY';return;
end
fit.covariance=information\eye(2);
fit.local_radius=sqrt(cfg.localization.region_chi2_df2_level*max(eig(fit.covariance)));
% Full-domain refinement is intentionally used for a checkable MVP. Planning
% must account for this computational cost; no small local ellipse shortcut.
step=cfg.localization.grid_refined_m;
[gx,gy]=meshgrid(unique([0:step:cfg.scene.width_m,cfg.scene.width_m]), ...
    unique([0:step:cfg.scene.height_m,cfg.scene.height_m]));
gp=[gx(:),gy(:)];
gf=uavmvp.range_nll_batch(gp,anchors,ranges,Pnav,Cradio,cfg.scene.terminal_height_m);
[gridbest,index]=min(gf);
if gridbest<value-1e-7
    [x,value,solver_flag]=uavmvp.local_range_solve(gp(index,:),anchors,ranges,Pnav,Cradio,cfg);
    fit.x=x;fit.objective=value;
    [d,H,C]=uavmvp.range_model(x,anchors,Pnav,Cradio,cfg.scene.terminal_height_m);
    L=chol(C,'lower');Z=L\H;information=Z'*Z;
    fit.residual=ranges(:)-d;fit.fit_stat=sum((L\fit.residual).^2);
    fit.condition=cond(information);fit.covariance=information\eye(2);
    fit.local_radius=sqrt(cfg.localization.region_chi2_df2_level*max(eig(fit.covariance)));
end
mask=reshape(gf<=value+cfg.localization.region_chi2_df2_level,size(gx));
fit.component_count=uavmvp.component_count(mask);
fit.grid_point_count=numel(gf);fit.compatible_grid_count=nnz(mask);
if ~any(mask,'all'),fit.reason='REGION_UNRESOLVED_BY_GRID';return;end
fit.region_radius=max(vecnorm(gp(mask(:),:)-x',2,2))+step/sqrt(2);
fit.radius=max(fit.local_radius,fit.region_radius);
fit.region_bounds=[min(gp(mask(:),:),[],1);max(gp(mask(:),:),[],1)];
fit.sensitivity=nan(2,4);
if seed_truncated,fit.reason='UNRESOLVED_SEED_CAP';return;end
if fit.component_count~=1,fit.reason='MULTIPLE_COMPATIBLE_COMPONENTS';return;end
if fit.condition>cfg.localization.information_condition_number_limit,fit.reason='POOR_GEOMETRY';return;end
if fit.fit_stat>cfg.localization.fit_chi2_df2_gate,fit.reason='FIT_INCONSISTENT';return;end
if fit.radius>cfg.localization.nominal_region_radius_limit_m,fit.reason='REGION_TOO_LARGE';return;end
if solver_flag<=0,fit.reason='LOCAL_SOLVER_LIMIT';return;end
% Differentiate the actual C(x)-dependent NLL estimator by refitting only
% these four measurements; the fifth measurement is not an input here.
for k=1:4
    h=max(1e-3,1e-3*sqrt(C(k,k)));rp=ranges(:);rm=rp;rp(k)=rp(k)+h;rm(k)=rm(k)-h;
    [xp,~,fp]=uavmvp.local_range_solve(x,anchors,rp,Pnav,Cradio,cfg);
    [xm,~,fm]=uavmvp.local_range_solve(x,anchors,rm,Pnav,Cradio,cfg);
    if fp<=0 || fm<=0,fit.reason='SENSITIVITY_SOLVER_LIMIT';return;end
    fit.sensitivity(:,k)=(xp-xm)/(2*h);
    if max(norm(xp-x),norm(xm-x))>max(.5,100*h)
        fit.reason='UNSTABLE_SOLUTION_BRANCH';return;
    end
end
if any(~isfinite(fit.sensitivity),'all'),fit.reason='INVALID_SENSITIVITY';return;end
fit.quality=true;fit.reason='READY_FOR_NEW_HELDOUT';
end
function f=emptyFit()
f=struct('x',[NaN;NaN],'covariance',nan(2),'quality',false, ...
    'reason','NOT_FIT','objective',Inf,'fit_stat',Inf,'condition',Inf, ...
    'local_radius',Inf,'region_radius',Inf,'radius',Inf,'component_count',0, ...
    'sensitivity',nan(2,4),'grid_point_count',0,'compatible_grid_count',0, ...
    'seed_truncated',false,'local_exit_flags',[],'region_bounds',nan(2),'residual',nan(4,1));
end
