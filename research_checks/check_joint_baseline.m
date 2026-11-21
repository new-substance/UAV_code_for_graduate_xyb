function out=check_joint_baseline()
%CHECK_JOINT_BASELINE Fixed camera/range/quality input, never a task flight.
repo=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(repo,'matlab_mvp'));
setup_project();c=load_project_config();
assert(c.ranging.range_white_std_m==5 && c.sim.radio_truth_range_white_std_m==5);
[points,~]=uavpriority.service_candidates(c);
xy=[60,70;70,75;60,62.5;70,62.5;80,70];
for k=1:size(xy,1),assert(any(vecnorm(points-xy(k,:),2,2)<1e-9));end
[~,ref]=min(vecnorm(c.scene.reference_centers_xy_m-[70,70],2,2));
n=uavmvp.nav_init(c);n.p=[70;70;c.scene.flight_height_m];n.last_reference_id=ref;
[pixels,~,visible]=uavmvp.camera_model(n,ref,c);assert(visible);
[n,image]=uavmvp.nav_update_reference(n, ...
    struct('marker_id',ref,'detected',true,'epoch_s',n.t,'pixels',pixels),c);assert(image.accepted);
entry=uavmvp.predict_service_visit(n,[70;62.5;c.scene.flight_height_m],ref,c,1);
outer=uavmvp.predict_service_visit(n,[70;75;c.scene.flight_height_m],ref,c,1);
assert(entry.accepted && entry.entry_camera_accepted && outer.accepted && outer.entry_camera_accepted);
a=[xy';c.scene.flight_height_m*ones(1,5)];x=[66;75];
s2=c.ranging.range_white_std_m^2/c.priority.cold_services_per_position;
b=struct('anchors',a,'ranges',vecnorm(a-[x;c.scene.terminal_height_m])', ...
    'Pnav',.04*eye(15),'Cradio',s2*eye(5)+c.ranging.device_persistent_bias_std_m^2*ones(5), ...
    'record_ids',1:5,'nav_ids',1:5,'version',1,'model_bias_bounds_m',zeros(5,1), ...
    'aggregation_matrix',eye(5),'constituent_ids',{{1,2,3,4,5}});
b.Pnavigation=b.Pnav;b.Ptiming=zeros(15);
f=uavpriority.fit_position(uavpriority.subbundle(b,1:4),c);assert(f.quality);
v=uavpriority.verify_position(f,b,c);assert(v.passed && v.estimate_unchanged);
pressure=b;pressure.Cradio=10^2/4*eye(5)+c.ranging.device_persistent_bias_std_m^2*ones(5);
pf=uavpriority.fit_position(uavpriority.subbundle(pressure,1:4),c);assert(~pf.quality);
out=struct('all_passed',true,'mission_count',0,'height_m',c.scene.flight_height_m, ...
    'inner_entry_margin_px',entry.entry.reference.pixel_margin, ...
    'outer_entry_margin_px',outer.entry.reference.pixel_margin, ...
    'nominal_quality_radius_m',f.radius,'pressure_quality_radius_m',pf.radius, ...
    'fixed_fresh_heldout_passed',v.passed, ...
    'scope','optimistic noiseless fixed input only; no task feasibility or population coverage claim');
end
