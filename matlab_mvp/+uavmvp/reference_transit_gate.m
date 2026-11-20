function gate=reference_transit_gate(nav,reference_id,cfg,deadline)
%REFERENCE_TRANSIT_GATE Shared pre-transit arrival and conditional-time screen.
% This preserves the executor's two checks. Actual acquisition/visibility and
% every later leg are checked separately; no future image is credited here.
% Historical clones cannot affect the current/map marginal during propagation.
nav=uavmvp.nav_prune(nav,[]);
target=[cfg.scene.reference_centers_xy_m(reference_id,:)';cfg.scene.flight_height_m];
forecast=uavmvp.predict_leg(nav,target,cfg,60,reference_id);
back=uavmvp.reference_return_plan(forecast.nav,cfg,reference_id);
required=nav.t+forecast.duration_s+back.reserve_s;
reason='TRANSIT_ADMISSIBLE';
if ~forecast.arrived,reason='TRANSIT_NAVIGATION_REJECTED';
elseif required>deadline,reason='TRANSIT_RETURN_BUDGET';end
gate=struct('accepted',strcmp(reason,'TRANSIT_ADMISSIBLE'),'reason',reason, ...
    'next_reference_id',reference_id,'start_s',nav.t,'duration_s',forecast.duration_s, ...
    'return_reserve_s',back.reserve_s,'required_end_s',required,'deadline_s',deadline, ...
    'slack_s',deadline-required,'arrived',forecast.arrived, ...
    'return_available',back.available,'envelope_inside_area',forecast.envelope_inside_area, ...
    'reference_envelope_inside',forecast.reference.envelope_inside, ...
    'conditional_future_reference_success',true);
end
