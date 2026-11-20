function tests=test_rtt_motion
tests=functiontests(localfunctions);
end
function testMovingEndpointsClockBiasAgainstExactTimestamps(t)
% Independently construct asymmetric propagation in two physical exchanges.
c=299792458;ab=1+2e-7;beta=17;start=[2,2.2];reply=.07;
rf=[24,24.01];rr=[24.015,24.042];st=zeros(4,2);
for k=1:2
    t2=start(k)+rf(k)/c;t3=t2+reply;t4=t3+rr(k)/c;
    st(:,k)=[start(k);ab*t2+beta;ab*t3+beta;t4];
end
out=uavmvp.rtt_solve(st,1,0,[],zeros(8));
dmb=diff(mean(st(2:3,:),1));expected=((rr(2)-rf(2))-(rr(1)-rf(1)))/(2*c*dmb);
verifyEqual(t,out.kappa-1/ab,expected,'AbsTol',3e-14);
raw=struct('timestamps',st,'exchange_displacement_bounds_m',abs(rr-rf),'range_curvature_bound_m',.002);
b=uavmvp.rtt_motion_bound(raw,out,0);
verifyLessThanOrEqual(t,abs(out.kappa-1/ab),b.clock_ratio_bound+3e-14);
verifyLessThan(t,abs(out.range_m-mean([rf,rr])-out.kappa_sensitivity*expected),2e-5);
warm=uavmvp.rtt_solve(st(:,1),1,0,out.kappa,zeros(4));
bw=uavmvp.rtt_motion_bound(raw,warm,b.clock_ratio_bound);
verifyEqual(t,bw.clock_ratio_bound,b.clock_ratio_bound);
verifyGreaterThan(t,bw.total_bound_m,bw.curvature_bound_m);
end
function testSymmetricMovingExchangeAndStaticLimit(t)
raw=struct('timestamps',[0,.2;.01,.21;.08,.28;.09,.29], ...
    'exchange_displacement_bounds_m',[0,0],'range_curvature_bound_m',0);
out=uavmvp.rtt_solve(raw.timestamps,1,0,[],zeros(8));
b=uavmvp.rtt_motion_bound(raw,out,Inf);verifyEqual(t,b.total_bound_m,0);
end
