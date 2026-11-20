function status=observe_onsite(w,id,cfg,deadline)
%OBSERVE_ONSITE Actual images at current service point; never recenter silently.
status=struct('accepted',false,'reason','ONSITE_IMAGE_FAILED','attempts',{{}});
for k=1:cfg.navigation_candidate.reference_max_attempts_per_visit
    if w.t+cfg.navigation_candidate.reference_observation_s>deadline,break;end
    [~,r]=w.observe_reference(id);status.attempts{end+1}=r;
    if r.accepted,status.accepted=true;status.reason='ACTUAL_ONSITE_IMAGE';break;end
end
end
