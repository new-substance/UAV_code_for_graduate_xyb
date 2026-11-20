function c=verify_position(f,b,cfg,require_quality)
if nargin<4,require_quality=true;end
if isfield(cfg.localization,'quality_method') && strcmp(cfg.localization.quality_method,'grid_conditional')
    c=uavpriority.verify_region(f,b,cfg,require_quality);return;
end
c=uavstage.verify_position(f,b,cfg,require_quality);
c.raw_statistic=c.statistic;c.model_residual_bound_m=NaN;
if isfinite(c.variance) && c.variance>0 && f.sensitivity_ok
    [~,H]=uavmvp.range_model(f.x,b.anchors,b.Pnav,b.Cradio,cfg.scene.terminal_height_m);
    w=H(5,:)*f.sensitivity;
    c.model_residual_bound_m=b.model_bias_bounds_m(5)+abs(w)*b.model_bias_bounds_m(1:4);
    c.statistic=max(0,abs(c.residual)-c.model_residual_bound_m)^2/c.variance;
    c.passed=(~require_quality || f.quality) && c.statistic<=cfg.stage.verification_gate;
    c.reason='HELDOUT_INCONSISTENT';if c.passed,c.reason='CONSISTENT_WITH_DECLARED_ENVELOPE';end
end
end
