function f=fit_position(b,cfg)
if isfield(cfg.localization,'quality_method') && strcmp(cfg.localization.quality_method,'grid_conditional')
    f=uavstage.fit_position(b,cfg);f=uavpriority.classify_region(f,b,cfg);
    if cfg.localization.grid_refined_m>.25 && isfinite(f.radius) && ...
            abs(f.radius-cfg.stage.region_radius_limit_m)<=1
        finer=cfg;finer.localization.grid_refined_m=.25;f=uavpriority.fit_position(b,finer);
    end
    return;
end
relative=cfg;relative.stage.region_radius_limit_m=cfg.stage.region_radius_limit_m-cfg.priority.datum_translation_bound_m;
f=uavstage.fit_position(b,relative);f.model_bias_bounds_m=b.model_bias_bounds_m;
f=uavpriority.classify_fit(f,cfg);
end
