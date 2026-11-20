function report=retreat_gate(nav,hold_s,ref,cfg)
nav=uavmvp.nav_prune(nav,[]);
prediction=uavmvp.predict_hold(nav,hold_s,cfg);
back=uavmvp.reference_return_plan(prediction.nav,cfg,ref);retreat=back.first_leg;
radius=sqrt(cfg.stage.local_geometry_level*cfg.stage.navigation_covariance_scale* ...
    max(eig(retreat.nav.P(1:2,1:2))))+cfg.priority.datum_translation_bound_m;
report=struct('accepted',back.available && prediction.envelope_inside_area && ...
    radius<=cfg.stage.navigation_radius_limit_m,'return_plan',back,'retreat',retreat, ...
    'hold_s',hold_s,'retreat_radius_m',radius);
end
