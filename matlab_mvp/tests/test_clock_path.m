function tests=test_clock_path
tests=functiontests(localfunctions);
end
function testKernelAgainstBrownianAnalyticMoments(t)
terms=[0,2;1,1];W=eye(2);C=uavmvp.clock_function_covariance(terms,W,0,1);
% Var integral_0^2 W = 8/3; Cov(integral_0^2 W,W(1))=1.5.
verifyEqual(t,C,[8/3,1.5;1.5,1],'AbsTol',1e-12);
terms=[100,100.05;100.2,100.25;100.3,100.3];
C=uavmvp.clock_function_covariance(terms,eye(3),.04,.2);
verifyGreaterThan(t,min(eig(C)),0);
end
function testClockPathConstantRateAndQueryOrder(t)
cfg=uavmvp.config();cfg.ranging.initial_clock_ratio_std_ppm=0;
cfg.ranging.clock_ratio_random_walk_ppm_per_sqrt_s=0;
p=uavmvp.ClockPath(cfg,10,7501);[stamp,rate]=p.sample([0,1.7,9.93]);
verifyEqual(t,stamp-stamp(1),[0,1.7,9.93],'AbsTol',2e-12);verifyEqual(t,rate,ones(1,3));
[other,~]=p.sample(1.7);verifyEqual(t,other,stamp(2),'AbsTol',0);
[column,column_rate]=p.sample([0;1.7;9.93]);
verifyEqual(t,column,stamp','AbsTol',0);verifyEqual(t,column_rate,rate','AbsTol',0);
end
function testIntegratedPhysicalClockMonteCarlo(t)
cfg=uavmvp.config();cfg.ranging.initial_clock_ratio_std_ppm=0;
cfg.ranging.clock_ratio_random_walk_ppm_per_sqrt_s=.1;
q=cfg.ranging.clock_ratio_random_walk_ppm_per_sqrt_s*1e-6;
samples=zeros(2,4000);T=.4;
for k=1:size(samples,2)
    path=uavmvp.ClockPath(cfg,.5,7600+k);[st,rate]=path.sample([0,T]);
    samples(:,k)=[st(2)-st(1)-T;rate(2)-1]/q;
end
C=[T^3/3,T^2/2;T^2/2,T];empirical=cov(samples');scale=sqrt(diag(C));
verifyLessThan(t,max(abs((empirical-C)./(scale*scale')),[],'all'),.06);
end
function testWarmDriftCorrectionMatchesIndependentIntegralFunctional(t)
cfg=uavmvp.config();cfg.ranging.device_persistent_bias_std_m=0;
cfg.ranging.initial_clock_ratio_std_ppm=0;h=uavmvp.radio_history_init(cfg);
cold=struct('range_m',30,'kappa',1,'exchange_count',2,'joint_covariance',zeros(2), ...
    'kappa_sensitivity',-299792458*.05/2);
[h,~]=uavmvp.radio_history_append(h,cold,1,cfg,'cold');
warm=cold;warm.exchange_count=1;warm.kappa_sensitivity=-299792458*.05/2;
warm=uavmvp.rtt_clock_drift(warm,[2,2.05],2.2,1,cfg,'warm');
[h,id]=uavmvp.radio_history_append(h,warm,2.2,cfg,'warm');
% Exact calibration at t=1: range error = -c/2 integral_2^2.05 [alpha(t)-alpha(1)] dt.
terms=[2,2.05;1,1;2.2,2.2];c=299792458;
W=[-c/2,c*.05/2,0;0,-1,1];
expected=uavmvp.clock_function_covariance(terms,W,0,cfg.ranging.clock_ratio_random_walk_ppm_per_sqrt_s*1e-6);
actual=h.P([2+id,2],[2+id,2]);
verifyLessThan(t,norm((actual-expected)./sqrt(diag(expected)*diag(expected)'),'fro'),1e-10);
verifyGreaterThanOrEqual(t,min(eig(h.P)),-1e-14);
end
function testColdDriftAgainstPhysicalClockSampling(t)
cfg=uavmvp.config();cfg.ranging.initial_clock_ratio_std_ppm=0;
c=299792458;r=30;t1=[1,1.2];t2=t1+r/c;t3=t2+.05;t4=t3+r/c;epoch=1.4;
nominal=[t1;t2;t3;t4];out=uavmvp.rtt_solve(nominal,1,0,[],zeros(8));
out=uavmvp.rtt_clock_drift(out,[t2',t3'],epoch,1,cfg,'cold');
errors=zeros(2,4000);
for k=1:size(errors,2)
    path=uavmvp.ClockPath(cfg,1.5,17600+k);st=nominal;
    st(2,:)=path.sample(t2);st(3,:)=path.sample(t3);
    [~,rate]=path.sample(epoch);measured=uavmvp.rtt_solve(st,1,0);
    errors(:,k)=[measured.range_m-r;measured.kappa-1/rate];
end
C=out.clock_drift_joint_covariance;scale=sqrt(diag(C));empirical=cov(errors');
verifyLessThan(t,max(abs((empirical-C)./(scale*scale')),[],'all'),.08);
verifyLessThan(t,max(abs(mean(errors,2)./scale)),.08);
end
