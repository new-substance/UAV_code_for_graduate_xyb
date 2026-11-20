function f=classify_fit(f,cfg)
if any(~isfinite(f.covariance),'all')
    f.quality=false;f.relative_radius_m=f.radius;
    f.datum_allowance_m=cfg.priority.datum_translation_bound_m;
    f.model_allowance_m=Inf;f.radius=Inf;return;
end
relative=cfg;relative.stage.region_radius_limit_m=cfg.stage.region_radius_limit_m-cfg.priority.datum_translation_bound_m;
f=uavstage.classify_fit(f,relative);f.relative_radius_m=f.radius;
f.datum_allowance_m=cfg.priority.datum_translation_bound_m;
f.model_allowance_m=Inf;
if f.sensitivity_ok
    signs=2*(dec2bin(0:15,4)-'0')-1;
    f.model_allowance_m=max(vecnorm(f.sensitivity*(signs.*f.model_bias_bounds_m(:)').',2,1));
end
f.radius=f.relative_radius_m+f.datum_allowance_m+f.model_allowance_m;
if any(f.model_bias_bounds_m>cfg.priority.aggregate_bias_limit_m)
    f.quality=false;f.reason='AGGREGATE_MODEL_BIAS_TOO_LARGE';
elseif f.quality && f.radius>cfg.stage.region_radius_limit_m
    f.quality=false;f.reason='ABSOLUTE_BUDGET_TOO_LARGE';
end
end
