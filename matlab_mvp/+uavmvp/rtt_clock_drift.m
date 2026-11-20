function out=rtt_clock_drift(out,reply_intervals,epoch,alpha_a,cfg,mode)
%RTT_CLOCK_DRIFT First-order within-service clock drift correction.
% Intervals are B receive->send times in A's physical-time scale, reconstructed
% from available timestamps. A tiny propagation-time approximation is allowed
% only as an explicitly evaluated timing-model approximation, not ground truth.
alpha_b=alpha_a/out.kappa;
q_kappa=cfg.ranging.clock_ratio_random_walk_ppm_per_sqrt_s*1e-6;
q_alpha=alpha_b^2/alpha_a*q_kappa;
assert(size(reply_intervals,2)==2 && all(reply_intervals(:,2)>reply_intervals(:,1)) && ...
    epoch>=max(reply_intervals,[],'all'));
if strcmp(mode,'cold')
    assert(size(reply_intervals,1)==2 && out.exchange_count==2);
    times=reshape(reply_intervals',[],1);anchor=min(times);keep=times>anchor;
    Bcolumns=[2,3,6,7];coefficients=out.jacobian(:,Bcolumns);
    terms=[anchor*ones(sum(keep),1),times(keep);epoch,epoch];
    W=[coefficients(:,keep),[0;alpha_a/alpha_b^2]];
    drift=uavmvp.clock_function_covariance(terms,W,0,q_alpha);
    out.clock_drift_joint_covariance=drift;
    out.joint_covariance=out.joint_covariance+drift;
elseif strcmp(mode,'warm')
    assert(size(reply_intervals,1)==1 && out.exchange_count==1);
    duration=diff(reply_intervals);c=299792458;
    % e_kappa(epoch) contains clock drift up to epoch. The extra range error
    % is -(c/2 alpha_b) integral(alpha(t)-alpha(epoch)) dt and is correlated
    % with that state; adding it independently would double-count drift.
    terms=[reply_intervals;epoch,epoch];
    W=[-c/(2*alpha_b),c*duration/(2*alpha_b);0,alpha_a/alpha_b^2];
    correction=uavmvp.clock_function_covariance(terms,W,0,q_alpha);
    out.within_service_drift=struct('extra_range_variance',correction(1,1), ...
        'clock_extra_range_covariance',correction(1,2),'start_s',reply_intervals(1));
else
    error('uavmvp:radioKind','Expected cold or warm.');
end
end
