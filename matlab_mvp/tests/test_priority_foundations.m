function tests=test_priority_foundations
tests=functiontests(localfunctions);
end
function setupOnce(~),setup_priority();end
function testPeripheralGridHasFiveCandidates(t)
cfg=uavpriority.config();xy=cfg.scene.reference_centers_xy_m;
verifyEqual(t,size(xy,1),32);
verifyTrue(t,all(xy(:,1)==10 | xy(:,1)==110 | xy(:,2)==10 | xy(:,2)==70));
[x,y]=meshgrid(0:2.5:120,0:2.5:80);minimum=Inf;
for p=[x(:),y(:)]'
    ranges=sqrt(sum((xy-p').^2,2)+15^2);minimum=min(minimum,sum(ranges<=40));
end
verifyGreaterThanOrEqual(t,minimum,5); % sampled nominal reachability, not a continuous/probability proof
end
function testUnobservablePreflightRefusesIncompatibleAndUnknownDatum(t)
c=uavpriority.config();r=uavpriority.preflight(c);verifyTrue(t,r.accepted);
c=uavpriority.config('common_survey');r=uavpriority.preflight(c);
verifyFalse(t,r.accepted);verifyGreaterThan(t,r.unobservable_radius_floor_m,8);
r=uavpriority.preflight(uavpriority.config('datum_unknown'));verifyFalse(t,r.accepted);
end
function testRepeatedColdServicesKeepPacketQualityAndClockModel(t)
c=uavpriority.config();verifyEqual(t,c.ranging.quality_reported_std_limit_m,8);
verifyEqual(t,c.ranging.range_white_std_m,10);
verifyEqual(t,c.sim.radio_truth_range_white_std_m,5);
verifyEqual(t,c.priority.cold_services_per_position,4);
verifyEqual(t,c.priority.services_per_atomic_block,2);
r=uavpriority.reservation(c);b=uavstage.reservation(c);
verifyGreaterThan(t,r.atomic_radio_s,b.radio_window_s);
end
function testWaitingUsesActualPixelsAndAccountsSensorTime(t)
c=uavpriority.config();w=uavmvp.MotionWorld(c,91001);home=c.scene.home_xyz_m(:);
uavmvp.fly_to(w,[home(1:2);15],c,40);w.observe_reference(1);
before=w.nav.version;start=w.t;
r=uavpriority.wait_reference(w,1,6,c);
verifyTrue(t,r.accepted);verifyEqual(t,w.t-start,6,'AbsTol',1e-8);
verifyGreaterThan(t,w.nav.version,before);raw=w.finish();
verifyGreaterThanOrEqual(t,numel(raw.reference_log),4);
end
function testMissingPixelsDoNotCauseNavigationVersionUpdates(t)
c=uavpriority.config();c.sim.reference_outage_intervals_s=[20,1e6];
c.navigation_candidate.marker_detect_probability_given_visibility=1;
w=uavmvp.MotionWorld(c,91001);home=c.scene.home_xyz_m(:);
uavmvp.fly_to(w,[home(1:2);15],c,40);w.observe_reference(1);
before=w.nav.version;uavpriority.wait_reference(w,1,6,c);
verifyEqual(t,w.nav.version,before);raw=w.finish();
verifyGreaterThanOrEqual(t,numel(raw.reference_log),4);
verifyFalse(t,raw.reference_log{end}.report.accepted);
end
function testActualWaitImagesSupportOtherwiseRejectedFullWindow(t)
c=uavpriority.config();c.navigation_candidate.marker_detect_probability_given_visibility=1;
w=uavmvp.MotionWorld(c,91001);home=c.scene.home_xyz_m(:);
uavmvp.fly_to(w,[home(1:2);15],c,40);w.observe_reference(1);
reserve=uavpriority.reservation(c);
blind=uavpriority.navigation_gate(w.nav,20+reserve.atomic_radio_s,c);
verifyFalse(t,blind.accepted);
before=w.nav.version;r=uavpriority.wait_reference(w,1,20,c);
actual=uavpriority.navigation_gate(w.nav,reserve.atomic_radio_s,c);
verifyTrue(t,r.accepted);verifyTrue(t,actual.accepted);
verifyGreaterThan(t,w.nav.version,before);w.finish();
end
function testUnobservableBudgetRejectionDoesNotLeaveGround(t)
c=uavpriority.config('common_survey');w=uavmvp.MotionWorld(c,91001);
w.configure_radio(1,17,[30;30;0]);o=uavpriority.execute(w,c,'closed_loop');raw=w.finish();
verifyFalse(t,o.preflight.accepted);verifyEqual(t,raw.time_s,0);
verifyEqual(t,raw.true_position,raw.true_home);
verifyEmpty(t,raw.reference_log);verifyEmpty(t,raw.radio_events);
end
function testMissingIdentityHasBoundedRecoveryBlocks(t)
c=uavpriority.config();c.stage.route_xy_m=c.scene.reference_centers_xy_m(1,:);
w=uavmvp.MotionWorld(c,91001);w.configure_radio(1,17,[115;75;0]);
o=uavpriority.execute(w,c,'closed_loop');raw=w.finish();
count=sum(cellfun(@(e)startsWith(e.kind,'GROUP_'),raw.radio_events));
verifyEqual(t,count,8);verifyEmpty(t,o.macros);verifyEqual(t,o.history.last_id,0);
verifyEqual(t,sum(cellfun(@(e)strcmp(e.kind,'RADIO_BLOCK_RECOVERY'),o.events)),3);
end
function testNoResidualBasedSelection(t)
c=uavpriority.config();
n=14;b=struct('anchors',[linspace(10,110,n);10+30*sin(1:n);15*ones(1,n)], ...
    'ranges',zeros(n,1),'Pnav',.01*eye(3*n),'Pnavigation',.01*eye(3*n), ...
    'Ptiming',zeros(3*n),'Cradio',eye(n),'record_ids',1:n,'nav_ids',1:n, ...
    'model_bias_bounds_m',zeros(n,1),'aggregation_matrix',eye(n), ...
    'constituent_ids',{num2cell(1:n)},'version',1);
a=uavpriority.selection_bundle(b,14,c);b.ranges=100*randn(n,1);
z=uavpriority.selection_bundle(b,14,c);
verifyEqual(t,a.record_ids,z.record_ids);verifyEqual(t,numel(a.record_ids),10);
verifyTrue(t,ismember(14,a.record_ids));
end
