function f=classify_region(f,b,cfg)
%CLASSIFY_REGION Q01 four-only nominal region, with bounded model remainder.
% Local Fisher geometry is diagnostic; boundary derivatives are not needed.
f.quality=false;f.quality_kind='grid_conditional';f.model_bias_bounds_m=b.model_bias_bounds_m;
f.relative_radius_m=Inf;f.datum_allowance_m=cfg.priority.datum_translation_bound_m;
f.model_allowance_m=0;f.model_bias_in_region=true;f.radius=Inf;
f.region_mask=false(size(f.grid_nll));f.model_region_inflation_m=NaN;
if any(~isfinite(b.model_bias_bounds_m) | b.model_bias_bounds_m<0) || ...
        any(b.model_bias_bounds_m>cfg.priority.aggregate_bias_limit_m)
    f.reason='AGGREGATE_MODEL_BIAS_TOO_LARGE';return;
end
if isempty(f.grid_nll) || any(~isfinite(f.x)) || ~isfinite(f.objective),return;end
[lower,upper]=uavpriority.likelihood_bounds([f.grid_xy;f.x'],b,cfg.scene.terminal_height_m);
f.region_reference_upper=min(upper);
if ~isfinite(f.region_reference_upper),f.reason='INVALID_COVARIANCE';return;end
mask=isfinite(lower(1:end-1)) & lower(1:end-1)<=f.region_reference_upper+cfg.stage.region_gamma;
f.region_mask=mask;f.compatible_grid_count=nnz(mask);
f.component_count=uavmvp.component_count(reshape(mask,f.grid_shape));
if ~any(mask),f.reason='REGION_UNRESOLVED_BY_GRID';return;end
f.region_radius=max(vecnorm(f.grid_xy(mask,:)-f.x',2,2))+f.grid_step_m/sqrt(2);
f.relative_radius_m=f.region_radius;f.radius=f.region_radius+f.datum_allowance_m;
nominal_mask=f.grid_nll<=f.objective+cfg.stage.region_gamma;
if any(nominal_mask)
    nominal_radius=max(vecnorm(f.grid_xy(nominal_mask,:)-f.x',2,2))+f.grid_step_m/sqrt(2);
    f.model_region_inflation_m=max(0,f.region_radius-nominal_radius);
end
if all(isfinite(f.covariance),'all')
    f.local_radius=sqrt(cfg.stage.local_geometry_level*max(eig(f.covariance)));
else
    f.local_radius=Inf;
end
if f.seed_truncated,f.reason='UNRESOLVED_SEED_CAP';
elseif f.solver_flag<=0,f.reason='LOCAL_SOLVER_LIMIT';
elseif f.component_count~=1,f.reason='MULTIPLE_COMPATIBLE_COMPONENTS';
elseif f.fit_stat>cfg.stage.fit_gate,f.reason='FIT_INCONSISTENT';
elseif f.radius>cfg.stage.region_radius_limit_m,f.reason='REGION_TOO_LARGE';
else,f.quality=true;f.reason='READY_FOR_NEW_HELDOUT';end
end
