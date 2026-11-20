function fit=classify_fit(fit,cfg)
%CLASSIFY_FIT Separate empirical LR threshold from local geometry scale.
fit.quality=false;
if isempty(fit.grid_nll),return;end
mask=reshape(fit.grid_nll<=fit.objective+cfg.stage.region_gamma,fit.grid_shape);
fit.compatible_grid_count=nnz(mask);fit.component_count=uavmvp.component_count(mask);
if ~any(mask,'all'),fit.reason='REGION_UNRESOLVED_BY_GRID';return;end
fit.region_radius=max(vecnorm(fit.grid_xy(mask(:),:)-fit.x',2,2))+fit.grid_step_m/sqrt(2);
fit.local_radius=sqrt(cfg.stage.local_geometry_level*max(eig(fit.covariance)));
fit.radius=max(fit.region_radius,fit.local_radius);
if fit.seed_truncated,fit.reason='UNRESOLVED_SEED_CAP';
elseif fit.solver_flag<=0,fit.reason='LOCAL_SOLVER_LIMIT';
elseif fit.component_count~=1,fit.reason='MULTIPLE_COMPATIBLE_COMPONENTS';
elseif fit.condition>cfg.localization.information_condition_number_limit,fit.reason='POOR_GEOMETRY';
elseif fit.fit_stat>cfg.stage.fit_gate,fit.reason='FIT_INCONSISTENT';
elseif fit.radius>cfg.stage.region_radius_limit_m,fit.reason='REGION_TOO_LARGE';
elseif ~fit.sensitivity_ok,fit.reason='INTERIOR_SENSITIVITY_NOT_VALID';
else,fit.quality=true;fit.reason='READY_FOR_NEW_HELDOUT';end
end
