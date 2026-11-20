function tests=test_reference_budget
tests=functiontests(localfunctions);
end

function testInfeasibleTopScoreUsesFeasibleAlternative(t)
[cfg,nav,rows]=example();original=nav;
a=uavmvp.reference_transit_gate(nav,10,cfg,600);
b=uavmvp.reference_transit_gate(nav,4,cfg,600);
verifyTrue(t,a.arrived && b.arrived);verifyGreaterThan(t,a.required_end_s,b.required_end_s);
deadline=(a.required_end_s+b.required_end_s)/2;
choice=uavmvp.screen_reference_choices(nav,11,rows,cfg,deadline);
verifyTrue(t,choice.accepted);verifyEqual(t,choice.selected.reference_id,4);
verifyEqual(t,choice.candidates(1).first_leg.reason,'TRANSIT_RETURN_BUDGET');
verifyEqual(t,[choice.candidates.score],[rows.score]);verifyEqual(t,nav,original);
verifyEqual(t,choice.selected.first_leg,uavmvp.reference_transit_gate(nav,4,cfg,deadline));
% Both routes to reference 12 and reference 4 start at 4, not at their goal.
verifyEqual(t,choice.candidates(2).first_leg,choice.candidates(3).first_leg);
end

function testAllInfeasibleAndDeadlineBoundary(t)
[cfg,nav,rows]=example();
gate=uavmvp.reference_transit_gate(nav,4,cfg,600);
at=uavmvp.reference_transit_gate(nav,4,cfg,gate.required_end_s);
before=uavmvp.reference_transit_gate(nav,4,cfg,gate.required_end_s-1e-8);
verifyTrue(t,at.accepted);verifyEqual(t,at.slack_s,0);
verifyFalse(t,before.accepted);verifyEqual(t,before.reason,'TRANSIT_RETURN_BUDGET');
none=uavmvp.screen_reference_choices(nav,11,rows,cfg,nav.t);
verifyFalse(t,none.accepted);verifyEmpty(t,none.selected);
verifyEqual(t,none.reason,'NO_FEASIBLE_REFERENCE');verifyNumElements(t,none.candidates,3);
empty=uavmvp.screen_reference_choices(nav,11,struct([]),cfg,600);
verifyFalse(t,empty.accepted);verifyEqual(t,empty.reason,'NO_USEFUL_REFERENCE');
end

function testSharedGateMatchesOriginalFullStateCalculation(t)
[cfg,nav,~]=example();nav=uavmvp.nav_clone(nav,cfg);original=nav;
forecast=uavmvp.predict_leg(nav,[cfg.scene.reference_centers_xy_m(4,:)';cfg.scene.flight_height_m],cfg,60,4);
back=uavmvp.reference_return_plan(forecast.nav,cfg,4);
gate=uavmvp.reference_transit_gate(nav,4,cfg,600);
verifyEqual(t,gate.duration_s,forecast.duration_s);
verifyEqual(t,gate.return_reserve_s,back.reserve_s);
verifyEqual(t,gate.accepted,forecast.arrived && nav.t+forecast.duration_s+back.reserve_s<=600);
verifyEqual(t,nav,original);
end

function testSelfVisitDoesNotCertifyServiceOrFuturePath(t)
[cfg,nav,rows]=example();
self=rows(1);self.reference_id=11;self.path=uavmvp.reference_path(cfg,11,11);
choice=uavmvp.screen_reference_choices(nav,11,self,cfg,nav.t);
verifyTrue(t,choice.accepted);
verifyEqual(t,choice.selected.first_leg.reason,'NO_TRANSIT_SERVICE_UNCHECKED');
verifyFalse(t,isfield(choice.selected.first_leg,'return_reserve_s'));
% A prediction that cannot arrive must retain the navigation rejection.
cfg.motion_and_budget.horizontal_max_speed_mps=.01;
gate=uavmvp.reference_transit_gate(nav,10,cfg,1e6);
verifyFalse(t,gate.accepted);verifyEqual(t,gate.reason,'TRANSIT_NAVIGATION_REJECTED');
end

function [cfg,nav,rows]=example()
cfg=reference_layout(12);nav=uavmvp.nav_init(cfg);nav.t=410;
nav.p=[cfg.scene.reference_centers_xy_m(11,:)';cfg.scene.flight_height_m];
nav.v=zeros(3,1);rows=struct([]);
for id=[10,4,12]
    rows=[rows;struct('reference_id',id,'value',1,'path',uavmvp.reference_path(cfg,11,id),'score',1/id)]; %#ok<AGROW>
end
rows(1).score=3;rows(2).score=2;rows(3).score=1;
end
