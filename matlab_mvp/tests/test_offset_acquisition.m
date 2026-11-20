function tests=test_offset_acquisition
tests=functiontests(localfunctions);
end
function testAllAlternativesRespectObservedNavigationUncertainty(t)
cfg=reference_layout(12);nav=uavmvp.nav_init(cfg);nav.p=[110;70;15];nav.P(:)=0;
choice=uavmvp.plan_offset_acquisition(nav,3,cfg,600);
assertTrue(t,choice.accepted);verifyEqual(t,choice.prepare.outward.nav.version,nav.version);
nav.P(1:3,1:3)=100*eye(3);
choice=uavmvp.plan_offset_acquisition(nav,3,cfg,600);
verifyFalse(t,choice.accepted);verifyEqual(t,numel(choice.candidates),3);
verifyFalse(t,any(cellfun(@(c)c.accepted,choice.candidates)));
end
