function status=reobserve_reference(world,id,cfg,deadline)
%REOBSERVE_REFERENCE Actual on-site images, without commanding a return flight.
status=struct('arrived',true,'accepted',false,'reason','REFERENCE_NOT_VISIBLE', ...
    'id',id,'start_s',world.t,'end_s',world.t,'attempts',{{}});
duration=cfg.navigation_candidate.reference_observation_s*cfg.navigation_candidate.reference_max_attempts_per_visit;
forecast=uavmvp.predict_hold(world.nav,duration,cfg);margin=uavmvp.reference_margin(forecast.nav,id,cfg);
if ~forecast.envelope_inside_area || ~margin.envelope_inside,return;end
if world.t+duration>deadline,status.reason='REFERENCE_BUDGET';return;end
for k=1:cfg.navigation_candidate.reference_max_attempts_per_visit
    [~,r]=world.observe_reference(id);status.attempts{end+1}=r;status.accepted=status.accepted || r.accepted;
end
status.reason='REFERENCE_NOT_ACQUIRED';
if status.accepted,status.reason='REFERENCE_REOBSERVED_AT_SERVICE';end
status.end_s=world.t;
end
