function out = verify_position(fit,anchors,ranges,Pnav,Cradio,cfg)
%VERIFY_POSITION Fifth record held out. All five positions use ONE nav version.
assert(size(anchors,2)==5 && numel(ranges)==5);
out=struct('passed',false,'reason','FIT_NOT_READY','residual',NaN,'variance',NaN,'statistic',NaN);
if ~fit.quality,return;end
if ~uavmvp.distinct_positions(anchors,Pnav,cfg,[(1:4)',5*ones(4,1)])
    out.reason='HELDOUT_POSITION_NOT_DISTINCT';return;
end
[d,H,C]=uavmvp.range_model(fit.x,anchors,Pnav,Cradio,cfg.scene.terminal_height_m);
A=fit.sensitivity;v=H(5,:)*A;
out.residual=ranges(5)-d(5);
out.variance=C(5,5)+v*C(1:4,1:4)*v'-2*v*C(1:4,5);
if ~isfinite(out.variance) || out.variance<=0
    out.reason='INVALID_PREDICTIVE_VARIANCE';return;
end
out.statistic=out.residual^2/out.variance;
out.passed=isfinite(out.statistic) && out.statistic<=cfg.localization.verification_chi2_df1_gate;
if out.passed,out.reason='CONSISTENT';else,out.reason='HELDOUT_INCONSISTENT';end
end
