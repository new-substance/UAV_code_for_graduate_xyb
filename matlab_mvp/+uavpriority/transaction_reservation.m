function r=transaction_reservation(cfg,phase,pool_count,remaining_s,return_s,wait_paid)
%TRANSACTION_RESERVATION P02 complete bounded transaction, phase costs, return.
assert(ismember(phase,{'GLOBAL','LOCALIZE','VERIFY','NOTICE'}));
assert(isscalar(pool_count) && pool_count>=0 && pool_count==floor(pool_count));
if nargin<6,wait_paid=false;end
assert(isscalar(remaining_s) && ~isnan(remaining_s) && isscalar(return_s) && ~isnan(return_s) && return_s>=0);
r=uavpriority.reservation(cfg);r.phase=phase;r.available=false;r.return_s=return_s;r.remaining_s=remaining_s;
if wait_paid || strcmp(phase,'NOTICE'),r.extra_service_s=0;end
r.max_blocks=r.block_cap;r.min_blocks=ceil(r.spatial_service_target/r.atomic_service_cap);
r.compute_s=cfg.motion_and_budget.calculation_model_s_per_decision;r.stop_s=0;
if pool_count+1>=4 || strcmp(phase,'GLOBAL') || strcmp(phase,'VERIFY'),r.compute_s=cfg.stage.compute_reserve_s;end
if ismember(phase,{'GLOBAL','VERIFY','NOTICE'})
    base=uavstage.reservation(cfg);r.stop_s=base.stop_s;
end
if strcmp(phase,'NOTICE')
    r.min_blocks=0;r.max_blocks=0;r.compute_s=0;r.atomic_radio_s=r.stop_s;
end
assert(r.min_blocks<=r.max_blocks);
fixed=r.extra_service_s+r.compute_s+r.stop_s;
if strcmp(phase,'NOTICE'),fixed=fixed+r.post_radio_reference_s;end
block=r.atomic_radio_s+r.post_radio_reference_s;
r.minimum_total_s=r.min_blocks*block+fixed;
r.block_cap=0;r.total_s=r.minimum_total_s;
if ~isfinite(return_s),return;end
for count=r.max_blocks:-1:r.min_blocks
    duration=count*block+fixed;
    if duration+return_s<=remaining_s
        r.available=true;r.block_cap=count;r.total_s=duration;return;
    end
end
end
