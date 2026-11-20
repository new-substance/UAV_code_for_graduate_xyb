function status=acquire_reference(w,id,cfg,deadline)
target=[cfg.scene.reference_centers_xy_m(id,:)';cfg.scene.flight_height_m];
if w.nav.last_reference_id==id && norm(w.nav.p-target)<.5 && norm(w.nav.v)<.2
    attempts={};accepted=false;
    for k=1:cfg.navigation_candidate.reference_max_attempts_per_visit
        if w.t+cfg.navigation_candidate.reference_observation_s>deadline,break;end
        [~,r]=w.observe_reference(id);attempts{end+1}=r;accepted=accepted || r.accepted; %#ok<AGROW>
        if accepted,break;end
    end
    status=struct('accepted',accepted,'reason','ONSITE_IMAGE_FAILED','attempts',{attempts});
    if accepted,status.reason='ACTUAL_ONSITE_REFERENCE';end
else
    status=uavstage.acquire_reference(w,id,cfg,deadline);
end
end
