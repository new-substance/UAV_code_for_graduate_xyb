function out=check_return_reservation()
%CHECK_RETURN_RESERVATION No world/flight: reject before invoking any motion.
repo=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(repo,'matlab_mvp'));
setup_project();c=load_project_config('legacy');nav=uavmvp.nav_init(c);nav.p=[10;10;15];nav.v=zeros(3,1);
nav.last_reference_id=1;
room=uavmvp.departure_repair_plan(nav,1,c,1800);assert(room.accepted);
assert(abs(room.observation_deadline_s+room.return_reserve_s-1800)<1e-9);
assert(room.observation_end_bound_s<=room.observation_deadline_s);
[pixels,~,visible]=uavmvp.camera_model(nav,1,c);assert(visible);
obs=struct('marker_id',1,'detected',true,'epoch_s',nav.t,'pixels',pixels);
[nav,update]=uavmvp.nav_update_reference(nav,obs,c);assert(update.accepted);
neighbor=uavmvp.departure_repair_plan(nav,2,c,1800);assert(neighbor.accepted);
nav.t=1790;
% This struct deliberately has no flight/image methods. A rejected step must
% return without attempting either; calling one would fail the test.
w=struct('nav',nav,'t',nav.t);
blocked=uavpriority.acquire_safe_reference(w,2,c,1800);
assert(~blocked.accepted && contains(blocked.reason,'BUDGET') && isempty(blocked.steps));
repair=uavpriority.repair_departure_reference(w,1,c,1800);
assert(~repair.accepted && contains(repair.reason,'BUDGET') && repair.end_s==w.t);
nav.t=0;original=nav;
short=uavmvp.reference_recovery_plan(nav,1,c,1800);
assert(short.available && short.prediction.arrived && short.pixel_margin>=0);
assert(isequaln(nav,original)); % screening never credits a hypothetical image
none=uavmvp.reference_recovery_plan(nav,1,c,0);
assert(~none.available);
none=uavmvp.reference_recovery_plan(nav,1,c,1800,true(4,1));
assert(~none.available);
out=struct('all_passed',true,'mission_count',0,'no_motion_when_return_unreserved',true, ...
    'explicit_observation_deadline',true,'future_visual_update_not_credited',true);
end
