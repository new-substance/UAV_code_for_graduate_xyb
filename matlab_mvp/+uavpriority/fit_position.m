function f=fit_position(b,cfg)
relative=cfg;relative.stage.region_radius_limit_m=cfg.stage.region_radius_limit_m-cfg.priority.datum_translation_bound_m;
f=uavstage.fit_position(b,relative);f.model_bias_bounds_m=b.model_bias_bounds_m;
f=uavpriority.classify_fit(f,cfg);
end
