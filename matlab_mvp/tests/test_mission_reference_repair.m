function tests=test_mission_reference_repair
tests=functiontests(localfunctions);
end

function testRepairAllowanceMatchesRealVisitWithoutPredictedUpdate(t)
cfg=reference_layout(12);nav=uavmvp.nav_init(cfg);nav.p=[14;10;15];nav.t=50;
nav.v=zeros(3,1);original=nav;
plan=uavmvp.departure_repair_plan(nav,1,cfg,600);
flight=uavmvp.predict_leg(nav,[10;10;15],cfg,60,1);
frames=cfg.navigation_candidate.reference_max_attempts_per_visit*cfg.navigation_candidate.reference_observation_s;
held=uavmvp.predict_hold(flight.nav,2+frames,cfg);
back=uavmvp.reference_return_plan(held.nav,cfg,1);
verifyTrue(t,plan.accepted);verifyEqual(t,plan.movement_allowance_s,flight.duration_s+2);
verifyEqual(t,plan.observation_end_bound_s,nav.t+flight.duration_s+2+frames,'AbsTol',1e-10);
verifyEqual(t,plan.return_reserve_s,back.reserve_s);
verifyEqual(t,plan.observation_deadline_s,600-back.reserve_s);
verifyEqual(t,plan.prediction_version,nav.version);verifyEqual(t,nav,original);
verifyGreaterThan(t,trace(held.nav.P(1:3,1:3)),trace(nav.P(1:3,1:3)));
end

function testRepairRefusesBudgetAndVisibilityFailures(t)
cfg=reference_layout(12);nav=uavmvp.nav_init(cfg);nav.p=[14;10;15];nav.t=50;nav.v=zeros(3,1);
plan=uavmvp.departure_repair_plan(nav,1,cfg,600);
needed=plan.observation_end_bound_s+plan.return_reserve_s;
low=uavmvp.departure_repair_plan(nav,1,cfg,needed-1e-6);
enough=uavmvp.departure_repair_plan(nav,1,cfg,needed+1e-6);
verifyFalse(t,low.accepted);verifyEqual(t,low.reason,'DEPARTURE_REPAIR_RETURN_BUDGET');
verifyTrue(t,enough.accepted);
nav.P(1:3,1:3)=100*eye(3);
bad=uavmvp.departure_repair_plan(nav,1,cfg,600);
verifyFalse(t,bad.accepted);verifyEqual(t,bad.reason,'DEPARTURE_REPAIR_NAVIGATION_REJECTED');
end

function testHealthyTransitPreservesActualTrajectoryAndImages(t)
cfg=fixture();w=uavmvp.MotionWorld(cfg,9101);uavmvp.fly_to(w,[10;10;15],cfg,40);
result=uavmvp.mission_reference_transit(w,1,5,cfg,600);actual=w.finish();
other=uavmvp.MotionWorld(cfg,9101);uavmvp.fly_to(other,[10;10;15],cfg,40);
old=uavmvp.transit_reference(other,1,5,cfg,600);expected=other.finish();
verifyTrue(t,result.accepted);verifyFalse(t,result.repair_attempted);verifyFalse(t,result.retry_attempted);
verifyNumElements(t,result.steps,1);verifyEqual(t,result.steps{1},old);
verifyEqual(t,actual.trajectory,expected.trajectory);verifyEqual(t,actual.reference_log,expected.reference_log);
end

function testObservedTargetFailureDoesNotLoopThroughDeparture(t)
cfg=fixture();cfg.navigation_candidate.marker_detect_probability_given_visibility=0;
w=uavmvp.MotionWorld(cfg,9102);uavmvp.fly_to(w,[10;10;15],cfg,40);before=w.nav.version;
result=uavmvp.mission_reference_transit(w,1,5,cfg,600);
verifyFalse(t,result.accepted);verifyNumElements(t,result.steps,1);
verifyTrue(t,result.steps{1}.arrived);verifyFalse(t,result.repair_attempted);verifyFalse(t,result.retry_attempted);
verifyFalse(t,result.failure_response.repair_departure);
verifyEqual(t,w.nav.version,before);verifyLessThan(t,w.t,600);w.finish();
% An initial budget rejection causes no physical actions at all.
w=uavmvp.MotionWorld(cfg,9103);uavmvp.fly_to(w,[10;10;15],cfg,40);start=w.t;
rejected=uavmvp.mission_reference_transit(w,1,5,cfg,start);
verifyFalse(t,rejected.accepted);verifyEmpty(t,rejected.steps);verifyEqual(t,w.t,start);w.finish();
end

function cfg=fixture()
cfg=ideal_navigation_profile();layout=reference_layout(12);
cfg.scene.reference_centers_xy_m=layout.scene.reference_centers_xy_m;
cfg.navigation_candidate.marker_detect_probability_given_visibility=1;
cfg.sim.reference_departure_margin_px=Inf;
end
