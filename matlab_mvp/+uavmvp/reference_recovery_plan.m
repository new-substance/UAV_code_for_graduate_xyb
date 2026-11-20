function plan=reference_recovery_plan(nav,id,cfg,deadline,used)
%REFERENCE_RECOVERY_PLAN Existing four short viewpoints, no image credit.
if nargin<5,used=false(4,1);end
nav=uavmvp.nav_prune(nav,[]);
base=cfg.scene.reference_centers_xy_m(id,:)';offsets=[4,0;-4,0;0,4;0,-4];
plan=struct('available',false,'index',0,'target',[],'prediction',[],'pixel_margin',-Inf);
for k=find(~used)'
    target=[base+offsets(k,:)';cfg.scene.flight_height_m];
    prediction=uavmvp.predict_leg(nav,target,cfg,15);
    frame=uavmvp.predict_hold(prediction.nav,cfg.navigation_candidate.reference_observation_s,cfg);
    margin=uavmvp.reference_margin(frame.nav,id,cfg);
    if prediction.arrived && prediction.envelope_inside_area && frame.envelope_inside_area && margin.envelope_inside && ...
            nav.t+prediction.duration_s+2+cfg.navigation_candidate.reference_max_attempts_per_visit* ...
            cfg.navigation_candidate.reference_observation_s<=deadline && margin.pixel_margin>plan.pixel_margin
        plan=struct('available',true,'index',k,'target',target,'prediction',prediction,'pixel_margin',margin.pixel_margin);
    end
end
end
