function prediction=predict_hold(nav,duration,cfg)
%PREDICT_HOLD Expected feedback hold with IMU covariance; no future update.
target=nav.p;finish=nav.t+duration;inside=true;
while finish-nav.t>1e-12
    a=uavmvp.control_acceleration(nav,target,cfg);
    control_end=min(nav.t+.05,finish);
    while control_end-nav.t>1e-12
        dt=min(cfg.sim.dt,control_end-nav.t);
        imu=struct('acc',nav.R'*(a-cfg.sim.gravity)+nav.ba,'gyro',nav.bg);
        nav=uavmvp.nav_predict(nav,imu,dt,cfg);
    end
    sigma=sqrt(max(0,diag(nav.P(1:3,1:3))));
    inside=inside && all(nav.p(1:2)-3*sigma(1:2)>=0) && ...
        all(nav.p(1:2)+3*sigma(1:2)<=[cfg.scene.width_m;cfg.scene.height_m]);
end
prediction=struct('nav',nav,'duration_s',duration,'envelope_inside_area',inside);
end
