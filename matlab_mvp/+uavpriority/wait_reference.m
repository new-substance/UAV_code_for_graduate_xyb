function status=wait_reference(w,ref,duration,cfg)
%WAIT_REFERENCE Interruptible queue only. Images consume the actual wait.
start=w.t;target=start+duration;attempts={};gates={};
status=struct('accepted',true,'reason','WAIT_COMPLETED','requested_s',duration, ...
    'elapsed_s',0,'image_attempts',{{}},'gates',{{}});
if duration==0,return;end
if ~cfg.priority.wait_observations
    reserve=uavpriority.reservation(cfg);
    gate=uavpriority.navigation_gate(w.nav,duration+reserve.atomic_radio_s,cfg);
    retreat=uavpriority.retreat_gate(w.nav,duration+reserve.atomic_radio_s,ref,cfg);
    status.gates={gate};
    if ~gate.accepted || ~retreat.accepted,status.accepted=false;status.reason='BLIND_WAIT_NOT_ADMITTED';return;end
    w.hold(duration);status.elapsed_s=w.t-start;return;
end
image_s=cfg.navigation_candidate.reference_observation_s;
while target-w.t>1e-9
    slice=min(cfg.priority.wait_refresh_interval_s,target-w.t);
    gate=uavpriority.navigation_gate(w.nav,slice,cfg);
    escape=uavpriority.retreat_gate(w.nav,slice,ref,cfg);
    gate.retreat_accepted=escape.accepted;gates{end+1}=gate; %#ok<AGROW>
    if ~gate.accepted || ~escape.accepted
        recovered=uavpriority.observe_onsite(w,ref,cfg,target);
        attempts{end+1}=struct('recovery',recovered); %#ok<AGROW>
        if ~recovered.accepted
            status.accepted=false;status.reason='WAIT_NAVIGATION_RECOVERY_FAILED';break;
        end
        continue;
    end
    if slice>=image_s
        w.hold(slice-image_s);[~,r]=w.observe_reference(ref);
        attempts{end+1}=r; %#ok<AGROW>
    else
        w.hold(slice);
    end
end
status.elapsed_s=w.t-start;status.image_attempts=attempts;status.gates=gates;
end
