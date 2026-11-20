function status=visit_reference(world,id,cfg,deadline)
%VISIT_REFERENCE Actual flight and at most two measured marker updates.
target=[cfg.scene.reference_centers_xy_m(id,:)';cfg.scene.flight_height_m];
p=uavmvp.predict_leg(world.nav,target,cfg,60,id);
status=struct('arrived',false,'accepted',false,'reason','REFERENCE_FORECAST_REJECTED', ...
    'id',id,'start_s',world.t,'end_s',world.t,'attempts',{{}});
if ~p.arrived || ~p.envelope_inside_area || ~p.reference.envelope_inside,return;end
max_time=p.duration_s+2;
if world.t+max_time+cfg.navigation_candidate.reference_max_attempts_per_visit* ...
        cfg.navigation_candidate.reference_observation_s>deadline
    status.reason='REFERENCE_BUDGET';return;
end
move=uavmvp.fly_to(world,target,cfg,max_time);status.arrived=move.arrived;
if ~move.arrived,status.reason='REFERENCE_MOVEMENT_TIMEOUT';status.end_s=world.t;return;end
for attempt=1:cfg.navigation_candidate.reference_max_attempts_per_visit
    [~,r]=world.observe_reference(id);status.attempts{end+1}=r;
    status.accepted=status.accepted || r.accepted;
end
status.reason='REFERENCE_ACQUIRED';
if ~status.accepted,status.reason='REFERENCE_NOT_ACQUIRED';end
status.end_s=world.t;
end
