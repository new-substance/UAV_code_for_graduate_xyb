function cfg = config()
%CONFIG Confirmed development conditions, not measured hardware parameters.
repo = fileparts(fileparts(fileparts(mfilename('fullpath'))));
source = fullfile(repo, 'research_configs', ...
    '完整策略_v2_仿真假设草案_20261111.json');
cfg = jsondecode(fileread(source));
cfg.source_file = source;
cfg.scene.application = 'GNSS-limited military training/support area: temporary monitoring-node localization';
cfg.scene.application_confirmed_date = '2026-11-13';
cfg.scene.application_scope = 'Above-ground surveyed area; no GNSS during the mission. Generic CSS simulation, not field or anti-jamming certification.';
cfg.sim.dt = 1 / cfg.navigation_candidate.imu_rate_hz;
cfg.sim.gravity = [0; 0; -9.80665];
cfg.sim.reference_nis_gate = 26.1244815584; % chi2(8), 0.999; nominal only
cfg.sim.reference_min_depth_m = 0.10;
cfg.sim.camera_noise_scale = 1;
cfg.sim.reference_max_iterations = 30;
cfg.sim.reference_step_tolerance = 1e-7; % whitened prior coordinates
cfg.sim.radio_hardware_delay_s = 1e-6; % known nominal net RTT delay, development model
cfg.sim.radio_correlated_bad_states = false; % explicit stress profile, baseline unchanged
cfg.sim.range_support_span_limit_m = 1; % local short-stop approximation gate
cfg.sim.range_model_bias_bound_limit_m = .10;
cfg.sim.reference_graph_edge_limit_m = 32; % candidate graph, not measured capability
cfg.sim.reference_leg_overhead_s = 8; % conditional transit/settling time proxy
cfg.sim.landing_reserve_s = 65; % development allocation; landing still obeys deadline
cfg.sim.reference_departure_margin_px = 40; % optional short observed departure
cfg.sim.reference_recovery_maneuvers = 2; % finite physical viewpoints, not gate relaxation
cfg.planner.belief_grid_step_m = 5; % planning quadrature only, not localization grid
cfg.planner.local_candidate_radius_m = 6.5;
cfg.planner.local_candidate_angles_deg = 0:45:315;
cfg.planner.local_candidate_pattern = 'circle'; % retained development baseline
cfg.planner.local_candidate_halfwidth_xy_m = [7,5]; % world-axis rectangle, optional
cfg.planner.footprint_candidate_fraction = .85; % nominal image footprint, actual gates still apply
cfg.planner.geometry_proxy = 'four_fit'; % formal fits use four rows, never the whole pool
cfg.planner.prediction_model = 'arrival'; % retained legacy; conditional_reference is a common scoring option
cfg.planner.scoring_ablation = 'none'; % scoring only; common candidates/execution never change
cfg.localization.group_geometry_rule = 'global_geometry'; % coarse synthetic ambiguity, no measured residual ranking
cfg.localization.group_geometry_grid_m = 2.5; % ranking only; formal 0.25 m region grid unchanged
cfg.planner.optional_service_window_s = 1;
cfg.sim.assumption_note = 'Local ENU, body z up; calibrated pinhole, no distortion. Numerical settings are development assumptions.';
assert(cfg.localization.fit_position_count == 4);
assert(cfg.localization.heldout_position_count == 1);
assert(~cfg.navigation_candidate.unknown_terminal_rtt_updates_navigation);
assert(cfg.scene.design_service_slant_m > cfg.scene.flight_height_m);
end
