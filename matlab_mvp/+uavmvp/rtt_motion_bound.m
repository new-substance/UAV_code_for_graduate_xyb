function report=rtt_motion_bound(raw,out,previous_clock_bound)
%RTT_MOTION_BOUND Local envelope for short-stop geometry and calibration bias.
% Covariance-derived displacement envelopes are nominal, not hard guarantees.
% The same clock bias persists into later warm records; it is not white noise.
if out.exchange_count==2
    mid_b=mean(raw.timestamps(2:3,:),1);baseline=diff(mid_b);
    assert(baseline>0);
    clock_bound=sum(raw.exchange_displacement_bounds_m)/(2*299792458*baseline);
else
    clock_bound=previous_clock_bound;
end
clock_range=abs(out.kappa_sensitivity)*clock_bound;
report=struct('clock_ratio_bound',clock_bound,'clock_range_bound_m',clock_range, ...
    'curvature_bound_m',raw.range_curvature_bound_m, ...
    'total_bound_m',raw.range_curvature_bound_m+clock_range);
end
