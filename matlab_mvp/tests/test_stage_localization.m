function tests=test_stage_localization
tests=functiontests(localfunctions);
end
function setupOnce(~),setup_stage();end
function testActualFourFitAndHeldoutIsolation(t)
cfg=uavstage.config();b=bundle([25,15,15;45,15,15;15,25,15;15,45,15;35,15,15],[30;30]);
f=uavstage.fit_position(uavstage.subbundle(b,1:4),cfg);
verifyLessThan(t,norm(f.x-[30;30]),1e-4);verifyTrue(t,f.sensitivity_ok);
verifyTrue(t,f.quality);check=uavstage.verify_position(f,b,cfg);verifyTrue(t,check.passed);
old=f;b.ranges(5)=b.ranges(5)+100;check=uavstage.verify_position(f,b,cfg);
verifyFalse(t,check.passed);verifyEqual(t,f,old);
end
function testNavigationVersionMustMatch(t)
cfg=uavstage.config();b=bundle([25,15,15;45,15,15;15,25,15;15,45,15;35,15,15],[30;30]);
f=uavstage.fit_position(uavstage.subbundle(b,1:4),cfg);b.version=b.version+1;
verifyError(t,@()uavstage.verify_position(f,b,cfg),'uavstage:mixedVersion');
end
function testGlobalMirrorIsRejectedDespiteLocalRank(t)
cfg=uavstage.config();a=[30,40,15;50,40,15;70,40,15;90,40,15];b=bundle(a,[60;60]);
b.Cradio=.1*eye(4);b.Pnav(:)=0;b.Pnavigation(:)=0;b.Ptiming(:)=0;
f=uavstage.fit_position(b,cfg);
[~,H]=uavmvp.range_model([60;60],b.anchors,b.Pnav,b.Cradio,0);
verifyEqual(t,rank(H),2);verifyFalse(t,f.quality);verifyGreaterThan(t,f.component_count,1);
verifyGreaterThan(t,f.region_radius,39);
end
function testWiderAnglesImproveGlobalRegionWithoutRelaxingGates(t)
cfg=uavstage.config();cfg.stage.region_gamma=9.421345153984678;
old=bundle([25,15,15;45,15,15;15,25,15;15,45,15],[30;30]);
wide=bundle([25,15,15;55,15,15;15,25,15;15,55,15],[30;30]);
old.Pnav=.08*eye(12);wide.Pnav=old.Pnav;
old.Cradio=14*eye(4)+9*ones(4);wide.Cradio=old.Cradio;
a=uavstage.fit_position(old,cfg);b=uavstage.fit_position(wide,cfg);
verifyGreaterThan(t,a.radius,20);verifyLessThan(t,b.radius,20);
verifyLessThan(t,b.radius,a.radius-4);
end
function b=bundle(a,x)
n=size(a,1);a=a';ranges=vecnorm(a-[x;0])';P=.04*eye(3*n);
b=struct('anchors',a,'ranges',ranges,'Pnav',P,'Pnavigation',P,'Ptiming',zeros(3*n), ...
    'Cradio',5*eye(n)+ones(n),'record_ids',1:n,'nav_ids',1:n,'version',3, ...
    'navigation_covariance_scale',1);
end
