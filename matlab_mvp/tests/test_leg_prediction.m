function tests=test_leg_prediction
tests=functiontests(localfunctions);
end
function testPredictionPreservesMapAndUsesNoAssumedReferenceUpdate(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];
original=nav;out=uavmvp.predict_leg(nav,[35;10;15],cfg,35);
verifyTrue(t,out.arrived);verifyGreaterThan(t,out.duration_s,25/3);
verifyEqual(t,out.nav.version,original.version);
verifyEqual(t,out.nav.map_p,original.map_p);
verifyEqual(t,out.nav.P(16:end,16:end),original.P(16:end,16:end),'AbsTol',1e-15);
verifyGreaterThan(t,trace(out.nav.P(1:3,1:3)),trace(original.P(1:3,1:3)));
verifyGreaterThanOrEqual(t,min(eig(out.nav.P)),-1e-12);
end
function testReferenceArrivalDependsOnPredictedUncertainty(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];
nav.P(1:3,1:3)=100*eye(3);
out=uavmvp.predict_leg(nav,[10;10;15],cfg,1,1);
verifyTrue(t,out.arrived);verifyTrue(t,out.reference.nominal_visible);
verifyFalse(t,out.reference.envelope_inside);
end
function testNominalMotionCostMatchesAnalyticalDistanceBounds(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[60;40;15];nav.P(:)=0;
out=uavmvp.predict_leg(nav,[85;40;15],cfg,40);
verifyTrue(t,out.arrived);verifyLessThan(t,norm(out.nav.p-[85;40;15]),.25);
verifyLessThan(t,norm(out.nav.v),.1);
% Bang-bang time is a lower bound: this feedback controller also settles.
lower=25/cfg.motion_and_budget.horizontal_max_speed_mps+ ...
    cfg.motion_and_budget.horizontal_max_speed_mps/cfg.motion_and_budget.horizontal_acceleration_mps2;
verifyGreaterThanOrEqual(t,out.duration_s,lower-.1);
end
function testDwellIsPropagatedBeforeReacquisition(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];
short=uavmvp.predict_hold(nav,1,cfg);long=uavmvp.predict_hold(nav,10,cfg);
verifyEqual(t,long.nav.version,nav.version);
verifyGreaterThan(t,trace(long.nav.P(1:3,1:3)),trace(short.nav.P(1:3,1:3)));
nav.P(4:6,4:6)=eye(3);
out=uavmvp.predict_service_visit(nav,nav.p,1,cfg);
verifyFalse(t,out.accepted);verifyTrue(t,out.outward.arrived);
verifyGreaterThanOrEqual(t,out.duration_s,cfg.motion_and_budget.stop_max_dwell_s);
end
function testFailedOnsiteAttemptTimePropagatesToFallbackExposure(t)
cfg=ideal_navigation_profile();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];
nav.P(4:6,4:6)=.01*eye(3);
out=uavmvp.predict_service_visit(nav,nav.p,1,cfg,3,1);
% At rest with only uncertain constant velocity, P_pp(T)=T^2 P_vv.
% Three seconds service, one second failed images, then 0.5 s exposure.
T=4.5;verifyEqual(t,out.duration_s,T,'AbsTol',1e-11);
verifyEqual(t,out.reacquisition.nav.t-nav.t,T,'AbsTol',1e-11);
verifyEqual(t,out.reacquisition.nav.P(1:3,1:3),.01*T^2*eye(3),'AbsTol',1e-10);
verifyEqual(t,out.reacquisition.nav.version,nav.version);
verifyGreaterThan(t,out.reacquisition.nav.t,out.back.nav.t);
end
