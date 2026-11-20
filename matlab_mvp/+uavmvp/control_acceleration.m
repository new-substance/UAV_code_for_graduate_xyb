function a=control_acceleration(nav,target,cfg)
%CONTROL_ACCELERATION Pure online estimated-state feedback for point-mass plant.
delta=target(:)-nav.p;amax=cfg.motion_and_budget.horizontal_acceleration_mps2;
vmax=cfg.motion_and_budget.horizontal_max_speed_mps;
d=norm(delta(1:2));v=zeros(3,1);
if d>0,v(1:2)=min(vmax,sqrt(2*amax*d))*delta(1:2)/d;end
v(3)=sign(delta(3))*min(cfg.motion_and_budget.vertical_speed_mps,sqrt(2*amax*abs(delta(3))));
a=(v-nav.v)/.6;
scale=max(1,norm(a(1:2))/amax);a(1:2)=a(1:2)/scale;
a(3)=min(amax,max(-amax,a(3)));
end
