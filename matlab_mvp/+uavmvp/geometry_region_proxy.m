function proxy=geometry_region_proxy(probe,anchors,Pnav,Cradio,cfg,points)
%GEOMETRY_REGION_PROXY Synthetic mean-range ambiguity score, never acceptance.
% No measured ranges or heldout are inputs. Full covariance varies over the
% coarse domain grid; the actual fit/region is still checked on real data.
step=cfg.localization.group_geometry_grid_m;
if nargin<6
    [x,y]=meshgrid(unique([0:step:cfg.scene.width_m,cfg.scene.width_m]), ...
        unique([0:step:cfg.scene.height_m,cfg.scene.height_m]));
    points=[x(:),y(:)];
end
[nominal,H,C]=uavmvp.range_model(probe,anchors,Pnav,Cradio,cfg.scene.terminal_height_m);
proxy=struct('radius',Inf,'local_radius',Inf,'coarse_radius',Inf, ...
    'grid_step_m',step,'grid_count',size(points,1),'compatible_count',0);
[L,flag]=chol(C,'lower');if flag,return;end
Z=L\H;I=Z'*Z;if rcond(I)<1e-12,return;end
proxy.local_radius=sqrt(cfg.localization.region_chi2_df2_level*max(eig(I\eye(2))));
grid=[points;probe(:)'];
F=uavmvp.range_nll_batch(grid,anchors,nominal,Pnav,Cradio,cfg.scene.terminal_height_m);
mask=isfinite(F) & F<=min(F)+cfg.localization.region_chi2_df2_level;
if ~any(mask),return;end
proxy.compatible_count=nnz(mask);
proxy.coarse_radius=max(vecnorm(grid(mask,:)-probe(:)',2,2))+step/sqrt(2);
proxy.radius=max(proxy.local_radius,proxy.coarse_radius);
end
