function tests=test_group_geometry
tests=functiontests(localfunctions);
end
function testGlobalGeometryRejectsLocalMirrorPreference(t)
[cfg,b,x]=fixture();cfg.localization.group_geometry_rule='local_information';
[old,~]=uavmvp.select_four(b,[],cfg,x);verifyEqual(t,old,[2,3,4,5]);
oldfit=fit_group(old,b,x,cfg);verifyFalse(t,oldfit.quality);verifyGreaterThan(t,oldfit.component_count,1);
cfg.localization.group_geometry_rule='global_geometry';
[ids,detail]=uavmvp.select_four(b,[],cfg,x);verifyTrue(t,ismember(1,ids));
fit=fit_group(ids,b,x,cfg);verifyTrue(t,fit.quality,fit.reason);
verifyGreaterThan(t,fit.local_radius,oldfit.local_radius);
verifyLessThan(t,fit.radius,20);verifyLessThan(t,detail.nominal_region.radius,20);
end
function testGlobalRankingIgnoresResidualAndRequiresNewPosition(t)
[cfg,b,x]=fixture();[ids,detail]=uavmvp.select_four(b,5,cfg,x);
b.ranges=[-100;500;20;999;1];[other,again]=uavmvp.select_four(b,5,cfg,x);
verifyEqual(t,ids,other);verifyEqual(t,detail,again);verifyTrue(t,ismember(5,ids));verifyNumElements(t,ids,4);
% Neither a scoring grid nor a plausible predicted region permits a fifth
% fitting row; the heldout remains the separate tracker's responsibility.
verifyNumElements(t,unique(ids),4);
end
function fit=fit_group(ids,b,x,cfg)
a=b.anchors(:,ids);ranges=vecnorm(a-[x;0])';
fit=uavmvp.fit_position(a,ranges,zeros(12),b.Cradio(ids,ids),cfg);
end
function [cfg,b,x]=fixture()
cfg=uavmvp.config();x=[5;5];a=[10,39.16119859781912,39.99343831738294,10,16.5; ...
    10,14.993438317382942,5.83880140218088,40,40;15,15,15,15,15];
a=a+cfg.navigation_candidate.antenna_lever_arm_body_m(:);
b=struct('record_ids',1:5,'anchors',a,'Pnav',zeros(15),'Cradio',25*eye(5)+9*ones(5), ...
    'ranges',vecnorm(a-[x;0])');
end
