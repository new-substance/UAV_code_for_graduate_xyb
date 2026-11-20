function prediction=predict_leg(nav,target,cfg,max_duration,reference_id)
%PREDICT_LEG Expected motion/covariance under current estimated state only.
% No synthetic reference update is credited during the leg. This is a local
% screening model, not a promise that actual reference acquisition succeeds.
if nargin<5,reference_id=[];end
target=target(:);start=nav.t;trace=zeros(0,7);arrived=false;
sigma=sqrt(max(0,diag(nav.P(1:3,1:3))));
inside=all(nav.p(1:2)-3*sigma(1:2)>=0) && ...
    all(nav.p(1:2)+3*sigma(1:2)<=[cfg.scene.width_m;cfg.scene.height_m]);
while nav.t-start<max_duration-1e-10
    if norm(nav.p-target)<=.25 && norm(nav.v)<=.1,arrived=true;break;end
    a=uavmvp.control_acceleration(nav,target,cfg);
    control_end=min(nav.t+.05,start+max_duration);
    while control_end-nav.t>1e-12
        dt=min([cfg.sim.dt,control_end-nav.t]);
        vn=nav.v+a*dt;
        vn(1:2)=vn(1:2)/max(1,norm(vn(1:2))/cfg.motion_and_budget.horizontal_max_speed_mps);
        vn(3)=min(cfg.motion_and_budget.vertical_speed_mps,max(-cfg.motion_and_budget.vertical_speed_mps,vn(3)));
        effective=(vn-nav.v)/dt;
        imu=struct('acc',nav.R'*(effective-cfg.sim.gravity)+nav.ba,'gyro',nav.bg);
        nav=uavmvp.nav_predict(nav,imu,dt,cfg);
    end
    sigma=sqrt(max(0,diag(nav.P(1:3,1:3))));
    inside=inside && all(nav.p(1:2)-3*sigma(1:2)>=0) && ...
        all(nav.p(1:2)+3*sigma(1:2)<=[cfg.scene.width_m;cfg.scene.height_m]);
    trace(end+1,:)=[nav.t,nav.p',sigma']; %#ok<AGROW>
end
reference=[];
if ~isempty(reference_id),reference=uavmvp.reference_margin(nav,reference_id,cfg);end
prediction=struct('nav',nav,'arrived',arrived,'duration_s',nav.t-start, ...
    'envelope_inside_area',inside,'reference',reference,'trace',trace);
end
