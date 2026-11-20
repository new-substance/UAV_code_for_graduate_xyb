function tests=test_reference_transit
tests=functiontests(localfunctions);
end
function testDepartureUsesActualImagesAndActualTime(t)
cfg=fixture();w=uavmvp.MotionWorld(cfg,9101);uavmvp.fly_to(w,[10;10;15],cfg,40);
before=w.nav.version;start=w.t;r=uavmvp.transit_reference(w,1,5,cfg,120);
verifyTrue(t,r.accepted);verifyNotEmpty(t,r.preparation);verifyTrue(t,r.preparation.accepted);
verifyGreaterThanOrEqual(t,w.nav.version,before+4);verifyGreaterThan(t,w.t-start,25/3);
d=w.finish();verifyGreaterThanOrEqual(t,numel(d.reference_log),4);
end
function testMissingImagesNeverBecomePredictedUpdates(t)
cfg=fixture();cfg.navigation_candidate.marker_detect_probability_given_visibility=0;
w=uavmvp.MotionWorld(cfg,9102);uavmvp.fly_to(w,[10;10;15],cfg,40);before=w.nav.version;
r=uavmvp.transit_reference(w,1,5,cfg,120);
verifyFalse(t,r.accepted);verifyEqual(t,w.nav.version,before);
verifyLessThanOrEqual(t,numel(r.recovery.attempts),cfg.sim.reference_recovery_maneuvers);
verifyLessThanOrEqual(t,w.t,120);w.finish();
end
function cfg=fixture()
cfg=ideal_navigation_profile();layout=reference_layout(12);
cfg.scene.reference_centers_xy_m=layout.scene.reference_centers_xy_m;
cfg.navigation_candidate.marker_detect_probability_given_visibility=1;
cfg.sim.reference_departure_margin_px=Inf;
end
