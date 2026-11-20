function status=land_at_home(world,cfg,deadline)
%LAND_AT_HOME Staged visual descent and finite commanded touchdown interval.
% No contact switch/GNSS/barometer is provided to the online controller.
% Physical touchdown is a separate offline outcome from command completion.
home=cfg.scene.home_xyz_m(:);refs=cfg.scene.reference_centers_xy_m;
[distance,id]=min(vecnorm(refs-home(1:2)',2,2));
assert(distance<1e-8,'uavmvp:homeReference','MVP requires a mapped marker at the launch pad.');
status=struct('command_completed',false,'reason','LANDING_NOT_STARTED', ...
    'updates',0,'misses',0,'checkpoints',[],'time_s',world.t);
heights=[15,12,9,6,4,2,1.5];
for height=heights
    if world.t>=deadline,status.reason='LANDING_BUDGET';return;end
    % Re-enter this height only if no more than 0.5 m above current estimate.
    if height>world.nav.p(3)-home(3)+.5,continue;end
    duration=min(25,deadline-world.t);
    move=uavmvp.fly_to(world,[home(1:2);home(3)+height],cfg,duration);
    if ~move.arrived,status.reason='LANDING_MOVEMENT_TIMEOUT';return;end
    margin=uavmvp.reference_margin(world.nav,id,cfg);
    if ~margin.nominal_visible,status.reason='LANDING_PREDICTED_REFERENCE_LOSS';return;end
    accepted=false;
    for attempt=1:cfg.navigation_candidate.reference_max_attempts_per_visit
        if world.t+cfg.navigation_candidate.reference_observation_s>deadline
            status.reason='LANDING_BUDGET';return;
        end
        [~,report]=world.observe_reference(id);
        if report.accepted,status.updates=status.updates+1;accepted=true;
        else,status.misses=status.misses+1;end
    end
    status.checkpoints(end+1)=height;
    if ~accepted,status.reason='LANDING_REFERENCE_UNAVAILABLE';return;end
end
% At least one final low-altitude correction is required before blind contact.
if isempty(status.checkpoints) || status.checkpoints(end)>1.5
    status.reason='LANDING_FINAL_REFERENCE_MISSING';return;
end
nav=world.nav;
sigma_z=sqrt(nav.P(3,3));descent_speed=.30;
remaining=max(0,nav.p(3)-home(3));duration=(remaining+4*sigma_z+.5)/descent_speed+1;
if world.t+duration>deadline,status.reason='LANDING_BUDGET';return;end
% Refuse excessive predicted lateral uncertainty before the unobserved tail.
probe=nav;
imu=struct('acc',-nav.R'*cfg.sim.gravity+nav.ba,'gyro',nav.bg);
for k=1:ceil(duration/cfg.sim.dt)
    probe=uavmvp.nav_predict(probe,imu,min(cfg.sim.dt,duration-(k-1)*cfg.sim.dt),cfg);
end
radius=norm(nav.p(1:2)-home(1:2))+3*sqrt(max(eig(probe.P(1:2,1:2))));
if radius>cfg.motion_and_budget.return_position_tolerance_m
    status.reason='LANDING_PREDICTED_UNCERTAINTY';return;
end
stop=world.t+duration;
while stop-world.t>1e-10
    a=uavmvp.control_acceleration(world.nav,[home(1:2);home(3)-2],cfg);
    a(3)=max(-cfg.motion_and_budget.horizontal_acceleration_mps2, ...
        min(cfg.motion_and_budget.horizontal_acceleration_mps2,(-descent_speed-world.nav.v(3))/.6));
    world.advance(min(.05,stop-world.t),a);
end
status.command_completed=true;status.reason='LANDING_COMMAND_COMPLETED';status.time_s=world.t;
end
