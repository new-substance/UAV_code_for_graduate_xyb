function gate=preflight(cfg)
bound=cfg.priority.datum_translation_bound_m;
survey=cfg.navigation_candidate.survey_common_translation_std_m;
floor=sqrt(cfg.stage.local_geometry_level*cfg.stage.navigation_covariance_scale)*survey+bound;
gate=struct('accepted',isfinite(bound) && bound>=0 && floor<=cfg.stage.navigation_radius_limit_m ...
    && bound<cfg.stage.region_radius_limit_m,'unobservable_radius_floor_m',floor, ...
    'datum_bound_m',bound,'reason','PREFLIGHT_BUDGET_ACCEPTED');
if ~gate.accepted,gate.reason='UNOBSERVABLE_DATUM_BUDGET_INFEASIBLE';end
end
