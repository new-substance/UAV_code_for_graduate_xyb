function tests=test_reference_return
tests=functiontests(localfunctions);
end
function testDisconnectedReferenceGraphCannotInventReturn(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[110;70;15];nav.P(:)=0;
plan=uavmvp.reference_return_plan(nav,cfg,3);
verifyFalse(t,plan.available);verifyTrue(t,isinf(plan.reserve_s));
end
function testGraphUsesMappedGeometryWithoutCreditingFutureObservations(t)
cfg=uavmvp.config();cfg.scene.reference_centers_xy_m=[10,10;35,10;60,10];
nav=uavmvp.nav_init(cfg);nav.p=[60;10;15];nav.P(:)=0;
plan=uavmvp.reference_return_plan(nav,cfg,3);
verifyTrue(t,plan.available);verifyEqual(t,plan.reference_ids,[3,2,1]);
verifyTrue(t,plan.conditional_future_reference_success);
verifyEqual(t,plan.first_leg.nav.version,nav.version);
nav.P(1:3,1:3)=100*eye(3);bad=uavmvp.reference_return_plan(nav,cfg,3);
verifyFalse(t,bad.available);
end
function testFailedMarkerIsExcludedFromSubsequentReturnPath(t)
cfg=reference_layout(12);nav=uavmvp.nav_init(cfg);nav.p=[110;70;15];nav.P(:)=0;
plan=uavmvp.reference_return_plan(nav,cfg,3,[5,6]);
verifyTrue(t,plan.available);verifyFalse(t,any(ismember(plan.reference_ids,[5,6])));
blocked=uavmvp.reference_return_plan(nav,cfg,3,1);verifyFalse(t,blocked.available);
end
function testUnobservedTargetKeepsShortReturnPathAvailable(t)
cfg=reference_layout(12);nav=uavmvp.nav_init(cfg);nav.p=[85;10;15];nav.P(:)=0;
step=struct('accepted',false,'arrived',false,'attempts',{{}},'id',6,'reason','REFERENCE_FORECAST_REJECTED');
action=uavmvp.return_failure_response(step,7);
verifyFalse(t,action.block_target);verifyTrue(t,action.repair_departure);verifyFalse(t,action.stop);
plan=uavmvp.reference_return_plan(nav,cfg,7);
verifyEqual(t,plan.reference_ids,[7,6,5,1]);
bad=uavmvp.reference_return_plan(nav,cfg,7,6);
verifyGreaterThan(t,bad.reserve_s,plan.reserve_s);
end
function testOnlyAttemptedTargetFailureBlocksDestination(t)
step=struct('accepted',false,'arrived',true,'attempts',{{struct('accepted',false)}},'id',6,'reason','REFERENCE_NOT_ACQUIRED');
action=uavmvp.return_failure_response(step,7);
verifyTrue(t,action.block_target);verifyFalse(t,action.repair_departure);
step.arrived=false;step.attempts={};step.reason='REFERENCE_BUDGET';
action=uavmvp.return_failure_response(step,7);
verifyTrue(t,action.stop);verifyFalse(t,action.block_target);verifyFalse(t,action.repair_departure);
end
function testEstimatedProximityDoesNotMeanReferenceWasObserved(t)
cfg=reference_layout(12);nav=uavmvp.nav_init(cfg);nav.p=[60;70;15];nav.P(:)=0;nav.t=20;
verifyTrue(t,isnan(uavmvp.recent_reference_id(nav,cfg)));
obs=struct('marker_id',10,'detected',true,'pixels',uavmvp.camera_model(nav,10,cfg));
[nav,report]=uavmvp.nav_update_reference(nav,obs,cfg);verifyTrue(t,report.accepted);
verifyEqual(t,uavmvp.recent_reference_id(nav,cfg),10);
unchanged=nav;[nav,report]=uavmvp.nav_update_reference(nav,[],cfg);
verifyFalse(t,report.accepted);verifyEqual(t,nav,unchanged);
nav.t=nav.t+2;verifyTrue(t,isnan(uavmvp.recent_reference_id(nav,cfg)));
end
