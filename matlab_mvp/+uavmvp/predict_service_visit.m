function out=predict_service_visit(nav,target,reference_id,cfg,dwell_s,onsite_observation_s)
%PREDICT_SERVICE_VISIT Screen outward flight, worst dwell and marker reacquisition.
% Credit no future visual update, including one planned before serving.
if nargin<5,dwell_s=cfg.motion_and_budget.stop_max_dwell_s;end
if nargin<6,onsite_observation_s=0;end
assert(isfinite(onsite_observation_s) && onsite_observation_s>=0);
outward=uavmvp.predict_leg(nav,target,cfg,60);
dwell=uavmvp.predict_hold(outward.nav,dwell_s+onsite_observation_s,cfg);
back=uavmvp.predict_leg(dwell.nav,[nav.map_p(1:2,reference_id);cfg.scene.flight_height_m],cfg,60,reference_id);
% The fallback image is exposed AFTER the return flight. Failed on-site
% attempts cost time but supply no covariance reduction before that flight.
reacquisition=uavmvp.predict_hold(back.nav,cfg.navigation_candidate.reference_observation_s,cfg);
reacquisition.reference=uavmvp.reference_margin(reacquisition.nav,reference_id,cfg);
out=struct('accepted',outward.arrived && back.arrived && outward.envelope_inside_area && ...
    dwell.envelope_inside_area && back.envelope_inside_area && reacquisition.envelope_inside_area && ...
    reacquisition.reference.envelope_inside, ...
    'duration_s',outward.duration_s+dwell.duration_s+back.duration_s+reacquisition.duration_s, ...
    'onsite_observation_s',onsite_observation_s,'outward',outward,'dwell',dwell,'back',back,'reacquisition',reacquisition);
end
