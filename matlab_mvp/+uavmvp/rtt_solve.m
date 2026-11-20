function out = rtt_solve(stamps,alpha_a,hardware_delay_s,kappa,stamp_cov)
%RTT_SOLVE One warm or two cold exchanges; stamps rows = A1,B2,B3,A4.
% Clock units are nominal seconds. Hardware delay is in physical seconds.
% On cold call omit kappa (or []); covariance propagates SAME used timestamps.
if nargin<4,kappa=[];end
if nargin<5,stamp_cov=zeros(numel(stamps));end
assert(size(stamps,1)==4 && all(isfinite(stamps),'all') && alpha_a>0);
c=299792458;ne=size(stamps,2);
assert(all(size(stamp_cov)==numel(stamps)));
da=stamps(4,:)-stamps(1,:);db=stamps(3,:)-stamps(2,:);
assert(all(da>0 & db>=0),'uavmvp:invalidTimestamps','Noncausal timestamp interval.');
ga=zeros(1,4*ne);gb=ga;
ga(1:4:end)=-1/ne;ga(4:4:end)=1/ne;
gb(2:4:end)=-1/ne;gb(3:4:end)=1/ne;
gk=zeros(1,numel(stamps));
if isempty(kappa)
    assert(ne==2,'uavmvp:coldExchangeCount','Cold calibration requires two complete exchanges.');
    ma=(stamps(1,:)+stamps(4,:))/2;
    mb=(stamps(2,:)+stamps(3,:))/2;
    delta_a=ma(2)-ma(1);delta_b=mb(2)-mb(1);
    assert(delta_a>0 && delta_b>0,'uavmvp:clockBaseline','Need increasing separated clock midpoints.');
    kappa=delta_a/delta_b;
    gma=[-.5,0,0,-.5,.5,0,0,.5];
    gmb=[0,-.5,-.5,0,0,.5,.5,0];
    gk=(gma-kappa*gmb)/delta_b;
else
    assert(isscalar(kappa) && isfinite(kappa) && kappa>0);
end
range=c/2*((mean(da)-kappa*mean(db))/alpha_a-hardware_delay_s);
gr=c/(2*alpha_a)*(ga-kappa*gb-mean(db)*gk);
J=[gr;gk];C=J*stamp_cov*J';C=(C+C')/2;
out=struct('range_m',range,'kappa',kappa,'jacobian',J, ...
    'joint_covariance',C,'kappa_sensitivity',-c*mean(db)/(2*alpha_a), ...
    'exchange_count',ne,'reply_interval_node_s',mean(db));
% For warm use add the external kappa variance/cross-covariance separately.
end
