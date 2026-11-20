function c=verify_region(f,b,cfg,require_quality)
%VERIFY_REGION V01 conditional compatibility over an unchanged four-only set.
% The minimum-statistic point is a witness, never a new published estimate.
if nargin<4,require_quality=true;end
c=struct('passed',false,'reason','FIT_NOT_READY','kind','grid_conditional', ...
    'residual',NaN,'variance',NaN,'statistic',NaN,'raw_statistic',NaN, ...
    'model_residual_bound_m',NaN,'fit_ids',f.record_ids,'heldout_id',b.record_ids(end), ...
    'nav_version',b.version,'raw_correlated_variance',NaN,'witness_xy_m',[], ...
    'candidate_count',0,'estimate_unchanged',true);
assert(numel(b.record_ids)==5 && isequal(b.record_ids(1:4),f.record_ids));
assert(~ismember(b.record_ids(5),f.record_ids));
assert(f.nav_version==b.version,'uavstage:mixedVersion','Fit and heldout require one navigation version.');
bounds=b.model_bias_bounds_m(:);
assert(norm(f.model_bias_bounds_m(:)-bounds(1:4),Inf)<1e-12, ...
    'uavproject:MixedModelBounds','Four-point model bounds changed without refreshing its region.');
members=[b.constituent_ids{:}];
assert(numel(members)==numel(unique(members)),'uavproject:RepeatedConstituent','Fit/heldout raw services overlap.');
if (require_quality && ~f.quality) || ~any(f.region_mask),return;end
if ~uavmvp.distinct_positions(b.anchors,b.Pnav,cfg,[(1:4)',5*ones(4,1)])
    c.reason='HELDOUT_NOT_DISTINCT';return;
end
if any(~isfinite(b.model_bias_bounds_m) | b.model_bias_bounds_m<0) || ...
        any(b.model_bias_bounds_m>cfg.priority.aggregate_bias_limit_m)
    c.reason='AGGREGATE_MODEL_BIAS_TOO_LARGE';return;
end
points=f.grid_xy(f.region_mask,:);c.candidate_count=size(points,1);
[~,~,p]=uavmvp.range_nll_batch(points,b.anchors,b.ranges,b.Pnav,b.Cradio,cfg.scene.terminal_height_m);
eta=p.residual(:,5);
for j=1:4,eta=eta-p.L(:,5,j).*p.whitened(:,j);end
variance=p.L(:,5,5).^2;
% K*L_FF=L_vF. Back substitution avoids a covariance inverse at every grid x.
K=zeros(size(points,1),4);
for j=4:-1:1
    value=p.L(:,5,j);
    for k=j+1:4,value=value-K(:,k).*p.L(:,k,j);end
    K(:,j)=value./p.L(:,j,j);
end
bias=bounds(5)+abs(K)*bounds(1:4);
statistic=max(0,abs(eta)-bias).^2./variance;
statistic(~p.valid | ~isfinite(variance) | variance<=0)=Inf;
[best,index]=min(statistic);
if ~isfinite(best),c.reason='INVALID_PREDICTIVE_VARIANCE';return;end
c.statistic=best;c.residual=eta(index);c.variance=variance(index);
c.raw_correlated_variance=c.variance;c.model_residual_bound_m=bias(index);
c.raw_statistic=c.residual^2/c.variance;c.witness_xy_m=points(index,:);
c.passed=best<=cfg.stage.verification_gate;c.reason='HELDOUT_INCONSISTENT';
if c.passed,c.reason='COMPATIBLE_WITH_FOUR_POINT_REGION';end
end
