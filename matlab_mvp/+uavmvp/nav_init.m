function nav = nav_init(cfg)
%NAV_INIT Uncertain surveyed launch prior; no access to realized world truth.
n = cfg.navigation_candidate;
nav.p = cfg.scene.home_xyz_m(:);
nav.v = zeros(3,1); nav.R = eye(3);
nav.ba = zeros(3,1); nav.bg = zeros(3,1);
nav.map_p = [cfg.scene.reference_centers_xy_m'; ...
    zeros(1,size(cfg.scene.reference_centers_xy_m,1))];
m = size(nav.map_p,2);
nav.map_R = repmat(eye(3),1,1,m);
nav.clones = zeros(3,0); nav.clone_ids = zeros(1,0);
nav.clone_times = zeros(1,0); nav.next_clone_id = 1;
nav.clone_velocities = zeros(3,0);
nav.radio_imu_timing_std_s = n.timestamp_alignment_std_s;
nav.t = 0; nav.version = 0;
nav.last_reference_id=NaN;nav.last_reference_time_s=-Inf;
s = [repmat(n.initial_position_std_m,1,3), ...
    repmat(n.initial_velocity_std_mps,1,3), ...
    deg2rad([n.initial_roll_pitch_std_deg,n.initial_roll_pitch_std_deg,n.initial_yaw_std_deg]), ...
    repmat(n.initial_acc_bias_std_mps2,1,3), ...
    repmat(n.initial_gyro_bias_std_radps,1,3)];
mapstd = [repmat(n.survey_translation_std_m,1,3), ...
    repmat(deg2rad(n.survey_rotation_std_deg),1,3)];
nav.P = diag([s,repmat(mapstd,1,m)].^2);
% Optional common horizontal survey-frame translation. It moves the launch
% prior and every marker together; repeated relative images cannot remove it.
if isfield(n,'survey_common_translation_std_m') && n.survey_common_translation_std_m>0
    B=zeros(size(nav.P,1),2);B(1:2,:)=eye(2);
    for j=1:m,B(15+6*(j-1)+(1:2),:)=eye(2);end
    nav.P=nav.P+n.survey_common_translation_std_m^2*(B*B');
end
end
