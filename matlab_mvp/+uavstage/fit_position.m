function fit=fit_position(bundle,cfg)
%FIT_POSITION Exactly four records; actual C(x)-dependent NLL, global grid.
a=bundle.anchors;r=bundle.ranges;P=bundle.Pnav;C=bundle.Cradio;
region_only=isfield(cfg.localization,'quality_method') && strcmp(cfg.localization.quality_method,'grid_conditional');
assert(size(a,2)==4 && numel(r)==4);
fit=struct('x',[NaN;NaN],'objective',Inf,'fit_stat',Inf,'condition',Inf, ...
    'covariance',nan(2),'sensitivity',nan(2,4),'sensitivity_ok',false, ...
    'solver_flag',0,'seed_truncated',false,'grid_xy',[],'grid_nll',[], ...
    'grid_shape',[],'grid_step_m',cfg.localization.grid_refined_m, ...
    'record_ids',bundle.record_ids,'nav_version',bundle.version, ...
    'quality',false,'reason','NOT_SOLVED','local_radius',Inf,'region_radius',Inf, ...
    'radius',Inf,'component_count',0,'compatible_grid_count',0);
if ~uavmvp.distinct_positions(a,P,cfg),fit.reason='POSITIONS_NOT_DISTINCT';return;end
[points,shape]=grid(cfg.localization.grid_initial_m,cfg);
values=uavmvp.range_nll_batch(points,a,r,P,C,cfg.scene.terminal_height_m);
surface=reshape(values,shape);padded=Inf(shape+2);padded(2:end-1,2:end-1)=surface;
minima=isfinite(surface);
for i=0:2
    for j=0:2
        if i==1 && j==1,continue;end
        minima=minima & surface<=padded((1:shape(1))+i,(1:shape(2))+j);
    end
end
ids=find(minima(:));[~,order]=sort(values(ids));ids=ids(order);
if isempty(ids),fit.reason='NO_FINITE_LIKELIHOOD';return;end
fit.seed_truncated=numel(ids)>16;ids=ids(1:min(16,numel(ids)));
for k=ids(:)'
    [x,f,flag]=uavmvp.local_range_solve(points(k,:),a,r,P,C,cfg);
    if f<fit.objective,fit.x=x;fit.objective=f;fit.solver_flag=flag;end
end
[fit.grid_xy,fit.grid_shape]=grid(cfg.localization.grid_refined_m,cfg);
fit.grid_nll=uavmvp.range_nll_batch(fit.grid_xy,a,r,P,C,cfg.scene.terminal_height_m);
[lowest,k]=min(fit.grid_nll);
if lowest<fit.objective-1e-7
    [fit.x,fit.objective,fit.solver_flag]=uavmvp.local_range_solve(fit.grid_xy(k,:),a,r,P,C,cfg);
end
[d,H,Q]=uavmvp.range_model(fit.x,a,P,C,cfg.scene.terminal_height_m);
[L,flag]=chol(Q,'lower');if flag,fit.reason='INVALID_COVARIANCE';return;end
Z=L\H;J=Z'*Z;fit.condition=cond(J);fit.fit_stat=sum((L\(r-d)).^2);
if rank(J)<2
    if ~region_only,fit.reason='DEGENERATE_GEOMETRY';return;end
else
    fit.covariance=J\eye(2);
end
% The stage only uses a stable INTERIOR estimator derivative. Boundary cases
% are retained as incomplete, rather than silently applying an interior rule.
inside=all(fit.x>.01) && all(fit.x<[cfg.scene.width_m;cfg.scene.height_m]-.01);
if ~region_only && inside && fit.solver_flag>0 && ~fit.seed_truncated
    A=zeros(2,4);ok=true;
    for k=1:4
        h=max(1e-3,1e-3*sqrt(Q(k,k)));rp=r;rm=r;rp(k)=rp(k)+h;rm(k)=rm(k)-h;
        [xp,~,fp]=uavmvp.local_range_solve(fit.x,a,rp,P,C,cfg);
        [xm,~,fm]=uavmvp.local_range_solve(fit.x,a,rm,P,C,cfg);
        ok=ok && fp>0 && fm>0 && max(norm(xp-fit.x),norm(xm-fit.x))<=max(.5,100*h);
        A(:,k)=(xp-xm)/(2*h);
    end
    fit.sensitivity=A;fit.sensitivity_ok=ok && all(isfinite(A),'all');
end
if region_only,return;end % common solver only; Q01 is evaluated by its wrapper
fit=uavstage.classify_fit(fit,cfg);
% Refine near the actual online radius boundary, before publishing a result.
% This is a numerical guard, not a continuous-domain containment proof.
if cfg.localization.grid_refined_m>.25 && isfinite(fit.radius) && ...
        abs(fit.radius-cfg.stage.region_radius_limit_m)<=1
    finer=cfg;finer.localization.grid_refined_m=.25;
    fit=uavstage.fit_position(bundle,finer);
end
end

function [points,shape]=grid(step,cfg)
[x,y]=meshgrid(unique([0:step:cfg.scene.width_m,cfg.scene.width_m]), ...
    unique([0:step:cfg.scene.height_m,cfg.scene.height_m]));
points=[x(:),y(:)];shape=size(x);
end
