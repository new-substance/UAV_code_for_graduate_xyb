function cfg=apply_stress_profile(cfg,name)
%APPLY_STRESS_PROFILE Explicit development interventions; fixed mission unchanged.
% Model-mismatch overrides affect only physical sensors, never assumed errors.
names={'none','correlated_link','visual_dropout','visual_outage', ...
    'matched_ranging','mismatched_ranging','joint'};
assert(ischar(name) && isrow(name) && ismember(name,names), ...
    'uavmvp:stressProfile','Unknown stress profile.');
if strcmp(name,'none'),return;end
cfg.sim.stress_profile_label=name;
if ismember(name,{'correlated_link','joint'})
    cfg.sim.radio_correlated_bad_states=true;
end
if ismember(name,{'visual_dropout','joint'})
    cfg.navigation_candidate.marker_detect_probability_given_visibility=.5;
end
if ismember(name,{'visual_outage','joint'})
    cfg.sim.reference_outage_intervals_s=[60,120];
end
if strcmp(name,'matched_ranging')
    cfg.ranging.range_white_std_m=10;
    cfg.ranging.device_persistent_bias_std_m=10;
end
if ismember(name,{'mismatched_ranging','joint'})
    cfg.sim.radio_truth_range_white_std_m=10;
    cfg.sim.radio_truth_device_bias_std_m=10;
end
if strcmp(name,'joint')
    cfg.navigation_candidate.timestamp_alignment_std_s=.01;
    cfg.ranging.initial_clock_ratio_std_ppm=2;
    cfg.ranging.clock_ratio_random_walk_ppm_per_sqrt_s=.2;
end
end
