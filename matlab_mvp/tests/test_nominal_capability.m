function tests=test_nominal_capability
tests=functiontests(localfunctions);
end
function testAnalyticalCommonBiasInverseMatchesFullEnumeration(t)
cfg=uavmvp.config();x=[60;40];a=[20,60,100,100,60,20;20,10,20,60,70,60;15,15,15,15,15,15];
result=nominal_quartet_geometry(x,a,cfg);groups=nchoosek(1:6,4);best=Inf;
for k=1:size(groups,1)
    ids=groups(k,:);delta=[x;0]-a(:,ids);H=(delta(1:2,:)./vecnorm(delta))';
    C=25*eye(4)+9*ones(4);P=(H'*(C\H))\eye(2);
    best=min(best,sqrt(cfg.localization.region_chi2_df2_level*max(eig(P))));
end
verifyEqual(t,result.radius,best,'AbsTol',1e-10);
verifyNumElements(t,result.ids,4);verifyNotEmpty(t,result.fifth_ids);
end
function testFootprintGridUsesSameNominalCameraAndSeparatedPoints(t)
cfg=reference_layout(12);cfg.planner.local_candidate_pattern='footprint';
points=uavmvp.service_candidate_points(6,cfg);verifySize(t,points,[9,3]);
nav=uavmvp.nav_init(cfg);nav.P(:)=0;
for k=1:size(points,1)
    nav.p=points(k,:)';[~,~,visible]=uavmvp.camera_model(nav,6,cfg);verifyTrue(t,visible);
end
[distinct,lower]=uavmvp.distinct_positions(points',zeros(27),cfg);
verifyTrue(t,distinct);verifyGreaterThanOrEqual(t,min(lower),7);
verifyEqual(t,cfg.navigation_candidate.camera.fx_px,400); % no wider camera was substituted
end
