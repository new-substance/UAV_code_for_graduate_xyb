function status=recover_reference(world,id,cfg,deadline)
%RECOVER_REFERENCE At most two physical viewpoints after a failed acquisition.
% No threshold inflation, synthetic image or reset of INS uncertainty.
status=struct('accepted',false,'reason','RECOVERY_CAMERA_ENVELOPE_REJECTED','attempts',{{}});
base=cfg.scene.reference_centers_xy_m(id,:)';offsets=[4,0;-4,0;0,4;0,-4];used=false(4,1);
for attempt=1:cfg.sim.reference_recovery_maneuvers
    best=0;best_margin=-Inf;chosen=[];
    for k=find(~used)'
        target=[base+offsets(k,:)';cfg.scene.flight_height_m];
        prediction=uavmvp.predict_leg(world.nav,target,cfg,15);
        frame=uavmvp.predict_hold(prediction.nav,cfg.navigation_candidate.reference_observation_s,cfg);
        margin=uavmvp.reference_margin(frame.nav,id,cfg);
        if prediction.arrived && prediction.envelope_inside_area && frame.envelope_inside_area && margin.envelope_inside && ...
                world.t+prediction.duration_s+2+cfg.navigation_candidate.reference_max_attempts_per_visit* ...
                cfg.navigation_candidate.reference_observation_s<=deadline && margin.pixel_margin>best_margin
            best=k;best_margin=margin.pixel_margin;chosen=prediction;
        end
    end
    if best==0,return;end
    used(best)=true;target=[base+offsets(best,:)';cfg.scene.flight_height_m];start=world.t;
    move=uavmvp.fly_to(world,target,cfg,chosen.duration_s+2);reports={};accepted=false;
    if move.arrived
        for k=1:cfg.navigation_candidate.reference_max_attempts_per_visit
            [~,r]=world.observe_reference(id);reports{end+1}=r;accepted=accepted || r.accepted; %#ok<AGROW>
        end
    end
    status.attempts{end+1}=struct('target',target,'start_s',start,'end_s',world.t, ...
        'movement',move,'reports',{reports},'accepted',accepted);
    if accepted,status.accepted=true;status.reason='REFERENCE_RECOVERED';return;end
end
status.reason='REFERENCE_RECOVERY_LIMIT';
end
