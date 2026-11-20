function tests=test_localization
tests=functiontests(localfunctions);
end
function testBatchLikelihoodAgainstDirectCholesky(t)
[cfg,a,r,P,C]=fixture();rs=RandStream('mt19937ar','Seed',8101);
Z=randn(rs,12);P=P+.01*(Z*Z');points=[5,5;12,16;30,22;17.3,13.2];
[F,chi]=uavmvp.range_nll_batch(points,a,r,P,C,0);
for k=1:size(points,1)
    [d,~,V]=uavmvp.range_model(points(k,:),a,P,C,cfg.scene.terminal_height_m);
    L=chol(V,'lower');e=L\(r-d);
    verifyEqual(t,chi(k),e'*e,'RelTol',1e-12);
    verifyEqual(t,F(k),e'*e+2*sum(log(diag(L))),'RelTol',1e-12);
end
end
function testExactGeometryRecoveryAndHeldout(t)
[cfg,a,r,P,C,x]=fixture();
fit=uavmvp.fit_position(a,r,P,C,cfg);
verifyTrue(t,fit.quality,fit.reason);verifyLessThan(t,norm(fit.x-x),.002);
a5=[a,[20;1;15]];r5=[r;norm(a5(:,5)-[x;0])];
V=eye(5)+.3^2*ones(5);P5=kron(eye(5),.01*eye(3));
check=uavmvp.verify_position(fit,a5,r5,P5,V,cfg);
verifyTrue(t,check.passed);verifyLessThan(t,abs(check.residual),.002);
r5(5)=r5(5)+50;check=uavmvp.verify_position(fit,a5,r5,P5,V,cfg);
verifyFalse(t,check.passed);
end
function testGlobalMirrorDespiteFullLocalRank(t)
cfg=uavmvp.config();cfg.scene.width_m=40;cfg.scene.height_m=40;
a=[2,12,25,38;20,20,20,20;15,15,15,15];x=[18;12];r=vecnorm(a-[x;0])';
P=zeros(12);C=.1^2*eye(4);[~,H]=uavmvp.range_model(x,a,P,C,0);
verifyEqual(t,rank(H),2);
fit=uavmvp.fit_position(a,r,P,C,cfg);
verifyFalse(t,fit.quality);verifyGreaterThanOrEqual(t,fit.component_count,2);
verifyEqual(t,fit.reason,'MULTIPLE_COMPATIBLE_COMPONENTS');
end
function testBoundarySolutionAgainstIndependentOneDimensionalMinimum(t)
cfg=uavmvp.config();cfg.scene.width_m=40;cfg.scene.height_m=30;
a=[2,38,38,2;2,2,28,28;15,15,15,15];r=vecnorm(a-[17;34;0])';
P=zeros(12);C=eye(4);[x,F,flag]=uavmvp.local_range_solve([18;29],a,r,P,C,cfg);
edge=@(u)uavmvp.range_nll_batch([u,30],a,r,P,C,0);
[u,expected]=fminbnd(edge,0,40,optimset('TolX',1e-10));
verifyGreaterThan(t,flag,0);verifyEqual(t,x(2),30,'AbsTol',1e-10);
verifyEqual(t,x(1),u,'AbsTol',2e-5);verifyEqual(t,F,expected,'AbsTol',1e-8);
outside=uavmvp.range_nll_batch([x(1),30.001],a,r,P,C,0);
verifyLessThan(t,outside,F); % active upper constraint, not zero unconstrained gradient
end
function testCommonTranslationLeavesAllRangesUnchanged(t)
[~,a,r,~,~,x]=fixture();shift=[4;-3;0];
verifyEqual(t,vecnorm(a+shift-([x;0]+shift))',r,'AbsTol',1e-12);
end
function testRepeatedFitOrHeldoutLocationIsRejected(t)
[cfg,a,r,P,C]=fixture();a(:,4)=a(:,3);r(4)=r(3);
fit=uavmvp.fit_position(a,r,P,C,cfg);
verifyFalse(t,fit.quality);verifyEqual(t,fit.reason,'SPATIAL_POSITIONS_NOT_DISTINCT');
[cfg,a,r,P,C]=fixture();fit=uavmvp.fit_position(a,r,P,C,cfg);
check=uavmvp.verify_position(fit,[a,a(:,1)],[r;r(1)],.01*eye(15),eye(5)+.09*ones(5),cfg);
verifyFalse(t,check.passed);verifyEqual(t,check.reason,'HELDOUT_POSITION_NOT_DISTINCT');
end
function testRelativeCovarianceUsesCrossTerms(t)
cfg=uavmvp.config();a=[0,6;0,0;15,15];
shared=kron(ones(2),100*eye(3));
verifyTrue(t,uavmvp.distinct_positions(a,shared,cfg));
verifyFalse(t,uavmvp.distinct_positions(a,100*eye(6),cfg));
end
function testCommonNavigationUncertaintyDoesNotAverageAway(t)
h=[1,0;0,1;-1,0;0,-1];S=diag([9,4]);C=4*eye(4)+h*S*h';
Px=(h'*(C\h))\eye(2);
verifyEqual(t,Px,S+2*eye(2),'AbsTol',1e-12);
end
function testActualEstimatorSensitivity(t)
[cfg,a,r,P,C,x]=fixture();fit=uavmvp.fit_position(a,r,P,C,cfg);
[~,H,V]=uavmvp.range_model(x,a,P,C,0);
A=(H'*(V\H))\(H'/V);
% Isotropic independent anchor error here makes C(x) constant. The NLL
% derivative should reduce to GLS; general C(x) is checked by refit in code.
verifyLessThan(t,max(abs(fit.sensitivity-A),[],'all'),.003);
end
function testHeldoutCrossTermMonteCarlo(t)
cfg=uavmvp.config();rs=RandStream('mt19937ar','Seed',8102);
a=[0,10,20,5,15;0,0,20,20,5;15,15,15,15,15];x=[9;11];
P=kron(ones(5),diag([4,9,1]));C=4*eye(5)+9*ones(5);
[d,H,V]=uavmvp.range_model(x,a,P,C,0);
A=(H(1:4,:)'*(V(1:4,1:4)\H(1:4,:)))\(H(1:4,:)'/V(1:4,1:4));
fit=struct('quality',true,'x',x,'sensitivity',A);
out=uavmvp.verify_position(fit,a,d,P,C,cfg);
eta=chol(V,'lower')*randn(rs,5,30000);
e=eta(5,:)-H(5,:)*A*eta(1:4,:);
verifyLessThan(t,abs(var(e)/out.variance-1),.04);
naive=V(5,5)+H(5,:)*A*V(1:4,1:4)*A'*H(5,:)';
verifyGreaterThan(t,abs(naive-out.variance),1);
end
function testSensitivityAgainstImplicitHessian(t)
[cfg,a,r,P,C]=fixture();rs=RandStream('mt19937ar','Seed',8103);
Z=randn(rs,12);P=P+.02*(Z*Z');r=r+[.8;-.4;.3;1.2];
fit=uavmvp.fit_position(a,r,P,C,cfg);verifyTrue(t,fit.quality,fit.reason);
x=fit.x;hx=.002;hr=.002;Hess=zeros(2);mixed=zeros(2,4);
f=@(q,y)uavmvp.range_nll_batch(q(:)',a,y,P,C,0);f0=f(x,r);
for j=1:2
    e=zeros(2,1);e(j)=hx;
    Hess(j,j)=(f(x+e,r)-2*f0+f(x-e,r))/hx^2;
    for k=1:4
        d=zeros(4,1);d(k)=hr;
        mixed(j,k)=(f(x+e,r+d)-f(x+e,r-d)-f(x-e,r+d)+f(x-e,r-d))/(4*hx*hr);
    end
end
e=[hx;0];d=[0;hx];
Hess(1,2)=(f(x+e+d,r)-f(x+e-d,r)-f(x-e+d,r)+f(x-e-d,r))/(4*hx^2);
Hess(2,1)=Hess(1,2);implicit=-Hess\mixed;
verifyLessThan(t,max(abs(fit.sensitivity-implicit),[],'all'),.005);
end
function [cfg,a,r,P,C,x]=fixture()
cfg=uavmvp.config();cfg.scene.width_m=40;cfg.scene.height_m=30;
a=[2,38,38,2;2,2,28,28;15,15,15,15];x=[17.3;13.2];
r=vecnorm(a-[x;0])';P=.01*eye(12);C=eye(4)+.3^2*ones(4);
end
