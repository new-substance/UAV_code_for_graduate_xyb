function tests=test_planning
tests=functiontests(localfunctions);
end
function testConditionalInformationEqualsJointBatchInformation(t)
cfg=uavmvp.config();x=[40;30];a=[10,70,70,10;10,10,60,60;15,15,15,15];
P=kron(ones(4),diag([9,9,1]))+.04*eye(12);C=25*eye(4)+9*ones(4);
s=uavmvp.conditional_information(x,a,P,C,cfg);
[~,H,V]=uavmvp.range_model(x,a,P,C,0);L=chol(V,'lower');Z=L\H;
expected=(diag(12./[120,80].^2)+Z'*Z)\eye(2);
verifyEqual(t,s.after,expected,'AbsTol',1e-10);
verifyLessThanOrEqual(t,max(eig(s.after-s.before)),1e-10);
end
function testRadioPredictionRetainsHistoryWithoutMutatingIt(t)
cfg=uavmvp.config();h=uavmvp.radio_history_init(cfg);original=h;
[first,id,mode]=uavmvp.predict_radio_record(h,10,cfg);
verifyEqual(t,h,original);verifyEqual(t,mode,'cold');verifyEqual(t,id,1);
[warm,id,mode]=uavmvp.predict_radio_record(first,15,cfg);
verifyEqual(t,id,2);verifyEqual(t,mode,'warm');verifyNotEqual(t,warm.P(3,4),0);
verifyEqual(t,warm.P(3,3),first.P(3,3),'AbsTol',1e-12);
[~,~,mode]=uavmvp.predict_radio_record(warm,50,cfg);verifyEqual(t,mode,'cold');
end
function testSharedTailRewardsOneLocationServingMultipleMembers(t)
cfg=uavmvp.config();need=[2,2];duration=10*ones(3,1);positions=[0,0;10,0;20,0];
shared=[1,1;1,1;1,1];individual=[1,0;0,1;1,0];
a=uavmvp.shared_tail_cost(need,shared,duration,positions,1,cfg);
b=uavmvp.shared_tail_cost(need,individual,duration,positions,1,cfg);
verifyLessThan(t,a.time_proxy_s,b.time_proxy_s);verifyEqual(t,a.unresolved,[0,0]);
verifyGreaterThan(t,sum(b.unresolved),0);
end
function testBlockedReferencePathAndFiniteGeometry(t)
cfg=reference_layout(12);path=uavmvp.reference_path(cfg,1,3);
verifyTrue(t,path.available);verifyEqual(t,path.ids([1,end]),[1,3]);
blocked=uavmvp.reference_path(cfg,1,3,[5,12]);verifyFalse(t,blocked.available);
end
function testTailCannotScheduleBeyondActualRemainingVisits(t)
cfg=uavmvp.config();need=[2,2];q=ones(3,2);duration=10*ones(3,1);points=[0,0;10,0;20,0];
last=uavmvp.shared_tail_cost(need,q,duration,points,1,cfg,0);
two=uavmvp.shared_tail_cost(need,q,duration,points,1,cfg,1);
verifyEmpty(t,last.sequence);verifyEqual(t,last.scheduled_proxy_s,0);
verifyEqual(t,last.unresolved,[1,1]);verifyGreaterThan(t,last.unresolved_penalty_s,0);
verifyNumElements(t,two.sequence,1);verifyEqual(t,two.unresolved,[0,0]);
verifyEqual(t,two.followup_cap,1);
end
function testCandidateGeometryRetainsCircleAndSeparatesRectangle(t)
cfg=reference_layout(12);base=cfg.scene.reference_centers_xy_m(1,:);
circle=uavmvp.service_candidate_points(1,cfg);
verifySize(t,circle,[9,3]);
verifyEqual(t,vecnorm(circle(2:end,1:2)-base,2,2),6.5*ones(8,1),'AbsTol',1e-12);
verifyLessThan(t,norm(circle(2,1:2)-circle(3,1:2)),cfg.localization.distinct_horizontal_separation_m);
cfg.planner.local_candidate_pattern='rectangle';
rectangle=uavmvp.service_candidate_points(1,cfg);
verifySize(t,rectangle,[5,3]);
for j=1:5
    for k=j+1:5
        verifyGreaterThan(t,norm(rectangle(j,1:2)-rectangle(k,1:2)),cfg.localization.distinct_horizontal_separation_m+3);
    end
end
% Nominal separation is necessary, not a certificate: the same physical
% visibility gate must reject a rectangle pushed outside the camera view.
cfg.planner.local_candidate_halfwidth_xy_m=[70,50];
far=uavmvp.service_candidate_points(1,cfg);nav=uavmvp.nav_init(cfg);nav.p=far(2,:)';
margin=uavmvp.reference_margin(nav,1,cfg);verifyFalse(t,margin.envelope_inside);
end
