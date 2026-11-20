function cfg=config(profile,method)
if nargin<1,profile='nominal';end
if nargin<2,method='priority';end
assert(ismember(method,{'priority','no_noise_envelope','no_wait_vision','single_burst'}));
cfg=uavstage.config();[xy,route]=uavpriority.perimeter_layout();
cfg.scene.reference_centers_xy_m=xy;cfg.stage.route_xy_m=route;
cfg.stage.version='priority-envelope-perimeter-v8';cfg.stage.profile=profile;
cfg.sim.reference_graph_edge_limit_m=20.1;
cfg.localization.new_positions_per_terminal_cap=size(route,1);
cfg.ranging.range_white_std_m=10; % declared standard-deviation upper envelope
cfg.sim.radio_truth_range_white_std_m=5; % simulation environment only
cfg.stage.navigation_covariance_scale=1.6917329191759944;
cfg.stage.region_gamma=6.982738097477608;
cfg.priority=struct('method',method,'white_std_upper_m',10,'datum_translation_bound_m',2, ...
    'cold_services_per_position',4,'services_per_atomic_block',2,'wait_refresh_interval_s',2, ...
    'wait_observations',true,'selection_pool_cap',10,'aggregate_bias_limit_m',.20, ...
    'extra_fit_cap',6,'extra_heldout_cap',3,'extra_service_cap',9,'inset_service_offset_m',7.5, ...
    'calibration_route_xy_m',[10,10;30,10;50,10;10,30;10,50], ...
    'datum_assumption','additional unknown fixed horizontal translation bounded by two metres');
switch method
    case 'no_noise_envelope',cfg.ranging.range_white_std_m=5;cfg.priority.white_std_upper_m=5;
    case 'no_wait_vision',cfg.priority.wait_observations=false;
    case 'single_burst',cfg.priority.cold_services_per_position=1;
end
cfg.priority.burst_recovery_blocks=ceil(cfg.priority.cold_services_per_position/ ...
    cfg.priority.services_per_atomic_block)+2;
switch profile
    case 'nominal'
    case 'range_upper',cfg.sim.radio_truth_range_white_std_m=10;
    case 'range_outside',cfg.sim.radio_truth_range_white_std_m=15;
    case 'long_service',cfg.stage.extra_service_delay_s=20;
    case 'frame_inside',cfg.sim.truth_frame_translation_m=[2;0;0];
    case 'frame_outside',cfg.sim.truth_frame_translation_m=[25;0;0];
    case 'common_survey',cfg.navigation_candidate.survey_common_translation_std_m=3;
    case 'datum_unknown',cfg.priority.datum_translation_bound_m=Inf;
    case 'no_visual',cfg.sim.reference_outage_intervals_s=[0,1e6];
    case 'bad_link',cfg.sim.radio_correlated_bad_states=true;
    otherwise,error('uavpriority:profile','Unknown priority profile.');
end
end
