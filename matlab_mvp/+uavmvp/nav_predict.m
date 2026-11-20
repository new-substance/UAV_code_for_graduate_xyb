function nav = nav_predict(nav, imu, dt, cfg)
%NAV_PREDICT Only measured IMU input. Map and measurement clones are static.
[next,Phi,F] = uavmvp.nominal_step(nav,imu,dt,cfg.sim.gravity);
P = nav.P;
P(1:15,1:15) = Phi*nav.P(1:15,1:15)*Phi'+uavmvp.process_noise(F,dt,cfg);
if size(P,1)>15
    P(1:15,16:end) = Phi*nav.P(1:15,16:end);
    P(16:end,1:15) = P(1:15,16:end)';
end
nav = next; nav.P = (P+P')/2;
end
