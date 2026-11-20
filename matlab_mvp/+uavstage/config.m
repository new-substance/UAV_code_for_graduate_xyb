function cfg=config(profile)
%CONFIG Direction 1+2 module settings; all new values are simulation inputs.
if nargin<1,profile='nominal';end
cfg=uavmvp.config();
cfg.scene.reference_centers_xy_m=[10,10;25,10;55,10;10,25;10,55;40,10;10,40];
cfg.sim.reference_graph_edge_limit_m=20.1;
cfg.scene.terminal_counts=1;
cfg.sim.terminal_device_class='uavstage.TerminalDevice';
cfg.navigation_candidate.survey_common_translation_std_m=.20;
cfg.motion_and_budget.time_limit_s=1800; % platform-resource model, not task acceptance limit
cfg.motion_and_budget=rmfield(cfg.motion_and_budget,'stop_max_dwell_s'); % old executor deliberately unsupported
cfg.localization.grid_initial_m=1;
cfg.localization.grid_refined_m=.5;
cfg.localization.new_positions_per_terminal_cap=12;
cfg.stage=struct('version','single-terminal-direction12-v4-wide-L','profile',profile, ...
    'protocol','REQUEST_RESPONSE_META_COLD2_WARM1', ...
    'route_xy_m',[25,15;55,15;15,25;15,55;40,15;15,40;32.5,15;47.5,15;15,32.5;15,47.5;15,15;55,7.5], ...
    'calibration_route_count',5,'transaction_attempt_cap',2,'backoff_s',.25, ...
    'compute_reserve_s',8,'reserve_margin_s',1,'navigation_radius_limit_m',8, ...
    'navigation_max_reference_age_s',25,'navigation_recovery_cap',2, ...
    'platform_resource_s',1800,'reference_success_conditional_return',true, ...
    'region_gamma',5.991464547,'local_geometry_level',5.991464547, ...
    'fit_gate',9.210340372,'verification_gate',6.634896601, ...
    'region_radius_limit_m',20,'calibration_quantile',.95, ...
    'navigation_covariance_scale',1, ...
    'region_kind','whole-domain-sampled-likelihood-difference-with-grid-padding', ...
    'publication','first completion snapshot; no post-completion re-ranging or silent revisions');
switch profile
    case 'nominal'
    case 'no_visual'
        cfg.sim.reference_outage_intervals_s=[0,1e6];
    case 'long_service'
        cfg.stage.extra_service_delay_s=20;
    case 'common_survey'
        cfg.navigation_candidate.survey_common_translation_std_m=3;
    case 'frame_mismatch'
        cfg.sim.truth_frame_translation_m=[25;0;0];
    case 'bad_link'
        cfg.sim.radio_correlated_bad_states=true;
    case 'range_mismatch'
        cfg.sim.radio_truth_range_white_std_m=10;
    case 'blind_layout'
        cfg.scene.reference_centers_xy_m=[10,10;35,10;60,10;85,10];
        cfg.sim.reference_graph_edge_limit_m=26;
        cfg.stage.route_xy_m=[35,15;60,15;60,35;35,35;47.5,15;60,25;47.5,35;35,25;47.5,25;25,25;70,25;47.5,45];
    case 'low_noise_control'
        cfg.ranging.range_white_std_m=2;
        cfg.ranging.device_persistent_bias_std_m=1;
    otherwise,error('uavstage:profile','Unknown stage profile.');
end
end
