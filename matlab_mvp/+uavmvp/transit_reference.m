function status=transit_reference(world,from_id,to_id,cfg,deadline)
%TRANSIT_REFERENCE Short observed departure if the direct capture margin is poor.
start=world.t;target=[cfg.scene.reference_centers_xy_m(to_id,:)';cfg.scene.flight_height_m];
p=uavmvp.predict_leg(world.nav,target,cfg,60,to_id);preparation=[];
poor=~p.arrived || ~p.envelope_inside_area || ~p.reference.envelope_inside || p.reference.pixel_margin<cfg.sim.reference_departure_margin_px;
if poor && from_id~=to_id
    from=cfg.scene.reference_centers_xy_m(from_id,:)';direction=target(1:2)-from;direction=direction/norm(direction);
    for distance=[4,2]
        look=[from+distance*direction;cfg.scene.flight_height_m];
        escape=uavmvp.predict_service_visit(world.nav,look,from_id,cfg, ...
            cfg.navigation_candidate.reference_max_attempts_per_visit*cfg.navigation_candidate.reference_observation_s);
        frame=uavmvp.predict_hold(escape.outward.nav,cfg.navigation_candidate.reference_observation_s,cfg);
        visible=uavmvp.reference_margin(frame.nav,from_id,cfg);
        if ~escape.accepted || ~visible.envelope_inside || world.t+escape.duration_s>deadline,continue;end
        move=uavmvp.fly_to(world,look,cfg,escape.outward.duration_s+2);reports={};accepted=false;
        if move.arrived
            for k=1:cfg.navigation_candidate.reference_max_attempts_per_visit
                [~,r]=world.observe_reference(from_id);reports{end+1}=r;accepted=accepted || r.accepted; %#ok<AGROW>
            end
        end
        preparation=struct('target',look,'movement',move,'reports',{reports},'accepted',accepted);
        break;
    end
end
status=uavmvp.visit_reference(world,to_id,cfg,deadline);status.start_s=start;status.preparation=preparation;status.recovery=[];
if ~status.accepted && status.arrived
    recovery=uavmvp.recover_reference(world,to_id,cfg,deadline);status.recovery=recovery;
    if recovery.accepted,status.accepted=true;status.reason=recovery.reason;end
end
status.end_s=world.t;
end
