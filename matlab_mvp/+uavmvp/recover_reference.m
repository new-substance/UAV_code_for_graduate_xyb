function status=recover_reference(world,id,cfg,deadline)
%RECOVER_REFERENCE At most two physical viewpoints after a failed acquisition.
% No threshold inflation, synthetic image or reset of INS uncertainty.
status=struct('accepted',false,'reason','RECOVERY_CAMERA_ENVELOPE_REJECTED','attempts',{{}});
used=false(4,1);
for attempt=1:cfg.sim.reference_recovery_maneuvers
    plan=uavmvp.reference_recovery_plan(world.nav,id,cfg,deadline,used);
    if ~plan.available,return;end
    used(plan.index)=true;target=plan.target;chosen=plan.prediction;start=world.t;
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
