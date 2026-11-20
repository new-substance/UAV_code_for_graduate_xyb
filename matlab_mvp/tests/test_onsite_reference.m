function tests=test_onsite_reference
tests=functiontests(localfunctions);
end
function testServiceCanEndAtItsActualObservedOffset(t)
[cfg,w,book,tracker,h]=fixture(9201,1);before=w.t;
[~,r]=uavmvp.execute_planned_service(w,1,[16.5;10;15],false,book,tracker,h,cfg,600,1);
verifyFalse(t,r.navigation_failed);verifyTrue(t,r.onsite_reference.accepted);
verifyEqual(t,r.reacquisition.reason,'REFERENCE_REOBSERVED_AT_SERVICE');
verifyGreaterThan(t,norm(w.nav.p(1:2)-[10;10]),5);
verifyGreaterThan(t,w.t,before);d=w.finish();verifyGreaterThan(t,norm(d.true_position(1:2)-[10;10]),5);
end
function testFailedOnsiteImagesConsumeTimeAndUsePhysicalFallback(t)
[cfg,w,book,tracker,h]=fixture(9202,0);
[~,r]=uavmvp.execute_planned_service(w,1,[16.5;10;15],false,book,tracker,h,cfg,600,1);
verifyFalse(t,r.onsite_reference.accepted);verifyTrue(t,r.navigation_failed);
verifyEqual(t,r.reason,'POST_SERVICE_REFERENCE_FAILED');
verifyLessThan(t,norm(w.nav.p-[10;10;15]),.25);
d=w.finish();verifyGreaterThanOrEqual(t,numel(d.reference_log),4);
end
function [cfg,w,book,tracker,h]=fixture(seed,detect)
cfg=ideal_navigation_profile();cfg.navigation_candidate.marker_detect_probability_given_visibility=detect;
cfg.ranging.baseline_stage_probabilities=struct('wake_decode',1,'identity_decode_given_control',1,'range_exchange_valid',1);
w=uavmvp.MotionWorld(cfg,seed);w.configure_radio(1,71,[40;10;0]);
uavmvp.fly_to(w,[16.5;10;15],cfg,40);book=uavmvp.ProtocolBook(cfg,1,71);tracker=uavmvp.TerminalTracker(cfg,book);
h={uavmvp.radio_history_init(cfg)};
end
