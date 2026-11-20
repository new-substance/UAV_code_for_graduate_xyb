function choice=plan_offset_acquisition(nav,id,cfg,deadline)
%PLAN_OFFSET_ACQUISITION Small common geometric alternatives, no terminal truth.
% Prefer inward; if its camera/escape envelope fails, try two oblique offsets.
center=[cfg.scene.width_m;cfg.scene.height_m]/2;
base=cfg.scene.reference_centers_xy_m(id,:)';direction=center-base;
direction=6.5*direction/norm(direction);
choice=struct('accepted',false,'target',[],'prepare',[],'candidates',{{}});
for angle=[0,-pi/6,pi/6]
    R=[cos(angle),-sin(angle);sin(angle),cos(angle)];target=[base+R*direction;cfg.scene.flight_height_m];
    p=uavmvp.predict_service_visit(nav,target,id,cfg, ...
        cfg.navigation_candidate.reference_max_attempts_per_visit*cfg.navigation_candidate.reference_observation_s);
    acquisition=uavmvp.predict_hold(p.outward.nav,cfg.navigation_candidate.reference_observation_s,cfg);
    visible=uavmvp.reference_margin(acquisition.nav,id,cfg);
    escape=uavmvp.reference_return_plan(p.reacquisition.nav,cfg,id);
    allowed=p.accepted && visible.envelope_inside && escape.available && nav.t+p.duration_s+escape.reserve_s<=deadline;
    choice.candidates{end+1}=struct('angle_rad',angle,'target',target,'accepted',allowed, ...
        'pixel_margin',visible.pixel_margin,'escape_pixel_margin',p.reacquisition.reference.pixel_margin, ...
        'area_envelope',p.outward.envelope_inside_area && p.dwell.envelope_inside_area && p.back.envelope_inside_area, ...
        'duration_s',p.duration_s,'return_reserve_s',escape.reserve_s);
    if allowed,choice.accepted=true;choice.target=target;choice.prepare=p;return;end
end
end
