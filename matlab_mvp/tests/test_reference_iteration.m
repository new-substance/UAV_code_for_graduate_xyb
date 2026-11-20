function tests=test_reference_iteration
tests=functiontests(localfunctions);
end

function testNonlinearUpdateAgainstIndependentThreeDimensionalMAP(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;18];
nav.P(:)=0;W=diag([4,3,2]);nav.P(1:3,1:3)=W*W';
truth=nav;truth.p=[12;9;15];z=uavmvp.camera_model(truth,1,cfg);
z=z+[.3;-.2;.1;.4;-.3;-.2;-.1;.2];
obs=struct('marker_id',1,'detected',true,'pixels',z);
[updated,report]=uavmvp.nav_update_reference(nav,obs,cfg);
verifyTrue(t,report.accepted,report.reason);
f=@(q) objective(q,nav,W,z,cfg);
eta=fminsearch(f,zeros(3,1),optimset('Display','off','TolX',1e-10,'TolFun',1e-10,'MaxIter',1000));
verifyLessThan(t,norm(updated.p-(nav.p+W*eta)),2e-6);
verifyLessThan(t,report.final_cost,report.initial_cost);
% Independent local information formula, same single observation.
[~,H]=uavmvp.camera_model(updated,1,cfg);
expected=inv(inv(nav.P(1:3,1:3))+H(:,1:3)'*H(:,1:3));
verifyLessThan(t,norm(updated.P(1:3,1:3)-expected,'fro'),1e-10);
end

function testLinearObservationIsCountedExactlyOnceWithSingularClone(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];
nav.P(:)=0;nav.P(1,1)=4;[nav,id]=uavmvp.nav_clone(nav,cfg);
truth=nav;truth.p(1)=11;z=uavmvp.camera_model(truth,1,cfg);
obs=struct('marker_id',1,'detected',true,'pixels',z);
[~,H]=uavmvp.camera_model(nav,1,cfg);
expected=1/(1/4+H(:,1)'*H(:,1));
[updated,report]=uavmvp.nav_update_reference(nav,obs,cfg);
verifyTrue(t,report.accepted);verifyEqual(t,updated.P(1,1),expected,'AbsTol',1e-12);
[~,P]=uavmvp.nav_archive(updated,id);verifyEqual(t,P(1,1),expected,'AbsTol',1e-12);
verifyGreaterThanOrEqual(t,min(eig(updated.P)),-1e-12);
end

function testPriorCoordinateProjectionJacobian(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];
d=zeros(size(nav.P,1),1);d(1:3)=[1;-.4;2];d(7:9)=[.1;-.08;.2];
d(19:21)=[.02;-.03;.04];[current,G]=uavmvp.nav_inject(nav,d);
[~,H]=uavmvp.camera_model(current,1,cfg);J=zeros(size(H));h=1e-5;
for k=1:numel(d)
    e=zeros(size(d));e(k)=h;
    plus=uavmvp.nav_inject(nav,d+e);minus=uavmvp.nav_inject(nav,d-e);
    J(:,k)=(uavmvp.camera_model(plus,1,cfg)-uavmvp.camera_model(minus,1,cfg))/(2*h);
end
verifyLessThan(t,max(abs(J-H*G),[],'all'),1e-6);
end

function value=objective(eta,nav,W,z,cfg)
nav.p=nav.p+W*eta;residual=z-uavmvp.camera_model(nav,1,cfg);
value=eta'*eta+residual'*residual;
end
