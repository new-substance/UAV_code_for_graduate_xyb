function tests=test_radio_formulas
tests=functiontests(localfunctions);
end
function testDeclaredFrameBudget(t)
c=uavmvp.config();s=uavmvp.radio_timing(c,3);
verifyEqual(t,s.symbol_s,.000512,'AbsTol',1e-14);
verifyEqual(t,s.exchange_s,.145192,'AbsTol',1e-12);
verifyEqual(t,s.cold_success_s,.345192,'AbsTol',1e-12);
verifyEqual(t,s.cold_max_s,.6,'AbsTol',1e-12);
verifyGreaterThan(t,s.group_s,uavmvp.radio_timing(c,1).group_s);
end
function testFirstRoundTenTerminals(t)
c=uavmvp.config();s=uavmvp.radio_timing(c,3);groups=[3,3,3,1];total=0;
for n=groups
    g=uavmvp.radio_timing(c,n);
    total=total+g.group_s+c.css.wake_ready_s+n*(s.frames.identity+c.css.guard_s+s.cold_max_s);
end
total=total+c.motion_and_budget.settle_s+ ...
    c.navigation_candidate.reference_max_attempts_per_visit*c.navigation_candidate.reference_observation_s;
verifyEqual(t,total,8.679568,'AbsTol',1e-12);
verifyLessThanOrEqual(t,total,c.motion_and_budget.stop_max_dwell_s);
verifyGreaterThan(t,total,8);
end
function testClockOffsetAndSkewCancellation(t)
c=299792458;r=37.2;aa=1+15e-6;ab=1-20e-6;
hw=80e-9;
st=stamps(r,aa,ab,12,-7,[100,100.2],.05,hw);
out=uavmvp.rtt_solve(st,aa,hw);
verifyEqual(t,out.kappa,aa/ab,'AbsTol',2e-12);
verifyEqual(t,out.range_m,r,'AbsTol',.0001);
warm=uavmvp.rtt_solve(st(:,1),aa,hw,aa/ab);
verifyEqual(t,warm.range_m,r,'AbsTol',1e-5);
uncorrected=c/2*((st(4,1)-st(1,1))-(st(3,1)-st(2,1)));
verifyGreaterThan(t,abs(uncorrected-r),100);
end
function testColdJointJacobian(t)
st=stamps(20,1,1+10e-6,.7,-.3,[0,.2],.05,0);
out=uavmvp.rtt_solve(st,1,0);h=1e-7;J=zeros(2,8);
for k=1:8
    p=st;m=st;p(k)=p(k)+h;m(k)=m(k)-h;
    op=uavmvp.rtt_solve(p,1,0);om=uavmvp.rtt_solve(m,1,0);
    J(:,k)=([op.range_m;op.kappa]-[om.range_m;om.kappa])/(2*h);
end
verifyLessThan(t,norm(J(1,:)-out.jacobian(1,:))/norm(out.jacobian(1,:)),2e-8);
verifyLessThan(t,max(abs(J(2,:)-out.jacobian(2,:))),2e-8);
end
function testSameDataCalibrationCovarianceMonteCarlo(t)
sigma=5/299792458;V=sigma^2*eye(8);
st=stamps(20,1,1,.7,-.3,[0,.2],.05,0);
out=uavmvp.rtt_solve(st,1,0,[],V);rs=RandStream('mt19937ar','Seed',7102);
samples=12000;errors=zeros(2,samples);
for k=1:samples
    o=uavmvp.rtt_solve(st+reshape(sigma*randn(rs,8,1),4,2),1,0);
    errors(:,k)=[o.range_m-out.range_m;o.kappa-out.kappa];
end
empirical=cov(errors');scale=sqrt(diag(out.joint_covariance));
normalized=(empirical-out.joint_covariance)./(scale*scale');
verifyLessThan(t,max(abs(normalized),[],'all'),.05);
verifyGreaterThan(t,abs(out.joint_covariance(1,2)),0);
end
function testHardwareBiasAndWarmSkewSensitivity(t)
r=30;hw=2*3/299792458;
st=stamps(r,1,1,0,0,[0,.2],.05,hw);
out=uavmvp.rtt_solve(st(:,1),1,0,1);
verifyEqual(t,out.range_m,r+3,'AbsTol',1e-7);
dk=.2e-6;other=uavmvp.rtt_solve(st(:,1),1,0,1+dk);
verifyEqual(t,other.range_m-out.range_m,out.kappa_sensitivity*dk,'AbsTol',1e-8);
end
function st=stamps(r,aa,ab,ba,bb,start,delay,hardware)
tau=r/299792458+hardware/2;
t1=start;t2=t1+tau;t3=t2+delay;t4=t3+tau;
st=[aa*t1+ba;ab*t2+bb;ab*t3+bb;aa*t4+ba];
end
