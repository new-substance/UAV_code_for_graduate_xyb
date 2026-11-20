function tests=test_four_fit_geometry
tests=functiontests(localfunctions);
end
function testFourRowProxyMatchesIndependentJointEnumeration(t)
[cfg,x,a,P,C]=fixture();s=uavmvp.four_fit_geometry(x,a,P,C,cfg);
groups=nchoosek(1:size(a,2),4);before=Inf;after=Inf;
for k=1:size(groups,1)
    ids=groups(k,:);d=a(:,ids)-[x;0];u=d./vecnorm(d);H=-u(1:2,:)';
    covariance=(H'*(C(ids,ids)\H))\eye(2);
    radius=sqrt(cfg.localization.region_chi2_df2_level*max(eig(covariance)));
    if ismember(size(a,2),ids),after=min(after,radius);else,before=min(before,radius);end
end
verifyEqual(t,s.before.radius,before,'AbsTol',1e-11);
verifyEqual(t,s.after.radius,after,'AbsTol',1e-11);
verifyNumElements(t,s.after.ids,4);verifyTrue(t,ismember(size(a,2),s.after.ids));
end
function testCommonTranslationCannotBeAveragedAway(t)
[cfg,x,a,P,C]=fixture();base=uavmvp.four_fit_geometry(x,a,P,C,cfg);
P=kron(ones(size(a,2)),diag([36,36,0]));
shared=uavmvp.four_fit_geometry(x,a,P,C,cfg);
verifyEqual(t,shared.before.covariance-base.before.covariance,36*eye(2),'AbsTol',1e-9);
verifyEqual(t,shared.after.covariance-base.after.covariance,36*eye(2),'AbsTol',1e-9);
verifyGreaterThan(t,shared.after.radius,6*sqrt(cfg.localization.region_chi2_df2_level));
end
function testCoincidentProspectivePointIsNeverUseful(t)
[cfg,x,a,P,C]=fixture();a(:,end)=a(:,1);
% There can still be groups using the new point in place of the old one,
% but no group may count those coincident rows as two distinct positions.
s=uavmvp.four_fit_geometry(x,a,P,C,cfg);
verifyFalse(t,all(ismember([1,size(a,2)],s.after.ids)));
verifyEqual(t,s.radius_reduction,0,'AbsTol',1e-12);
end
function [cfg,x,a,P,C]=fixture()
cfg=uavmvp.config();x=[60;40];
a=[20,60,100,100,60,20;20,10,20,60,70,60;15,15,15,15,15,15];
P=zeros(18);C=25*eye(6)+9*ones(6);
end
