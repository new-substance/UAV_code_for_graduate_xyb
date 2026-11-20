function check=verify_position(fit,bundle,cfg,require_quality)
%VERIFY_POSITION Fifth outcome is not an input to fit or four-record choice.
if nargin<4,require_quality=true;end
check=struct('passed',false,'reason','FIT_NOT_READY','residual',NaN, ...
    'variance',NaN,'statistic',NaN,'fit_ids',fit.record_ids,'heldout_id',bundle.record_ids(end), ...
    'nav_version',bundle.version,'raw_correlated_variance',NaN);
assert(numel(bundle.record_ids)==5 && isequal(bundle.record_ids(1:4),fit.record_ids));
assert(~ismember(bundle.record_ids(end),fit.record_ids));
assert(fit.nav_version==bundle.version,'uavstage:mixedVersion','Fit and heldout need the same navigation version.');
if (require_quality && ~fit.quality) || ~fit.sensitivity_ok,return;end
if ~uavmvp.distinct_positions(bundle.anchors,bundle.Pnav,cfg,[(1:4)',5*ones(4,1)])
    check.reason='HELDOUT_NOT_DISTINCT';return;
end
[d,H,C]=uavmvp.range_model(fit.x,bundle.anchors,bundle.Pnav,bundle.Cradio,cfg.scene.terminal_height_m);
v=H(5,:)*fit.sensitivity;
S=C(5,5)+v*C(1:4,1:4)*v'-2*v*C(1:4,5);
check.raw_correlated_variance=S;
check.residual=bundle.ranges(5)-d(5);check.variance=S;
if ~isfinite(S) || S<=0,check.reason='INVALID_PREDICTIVE_VARIANCE';return;end
check.statistic=check.residual^2/S;
check.passed=check.statistic<=cfg.stage.verification_gate;
check.reason='HELDOUT_INCONSISTENT';if check.passed,check.reason='CONSISTENT';end
end
