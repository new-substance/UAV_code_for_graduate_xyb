function b=bundle(records,nav,history,ids,cfg)
b=uavmvp.record_bundle(records,nav,history,ids);
[ok,k]=ismember(b.nav_ids,nav.clone_ids);assert(all(ok));
v=reshape(nav.clone_velocities(:,k),[],1);
b.Ptiming=nav.radio_imu_timing_std_s^2*(v*v');
b.Pnavigation=b.Pnav-b.Ptiming;
b.Pnav=cfg.stage.navigation_covariance_scale*b.Pnavigation+b.Ptiming;
b.navigation_covariance_scale=cfg.stage.navigation_covariance_scale;
end
