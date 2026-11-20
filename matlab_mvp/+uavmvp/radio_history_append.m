function [history,id,quality]=radio_history_append(history,out,time,cfg,kind)
%RADIO_HISTORY_APPEND One space observation, retaining cold/warm covariance.
% Cold out must include all calibration error covariance appropriate to its
% epoch. Within-exchange drift is NOT silently modeled by between-epoch Q.
previous_epoch=history.time_s;history=uavmvp.radio_history_predict(history,time,cfg);
n=size(history.P,1);T=[eye(n);zeros(1,n)];T(end,1)=1;
Q=zeros(n+1);
switch kind
    case 'cold'
        assert(out.exchange_count==2 && all(size(out.joint_covariance)==[2,2]));
        T(2,:)=0; % fresh calibration errors, not a known exact clock
        Q([end,2],[end,2])=out.joint_covariance;
        history.kappa=out.kappa;history.calibration_time_s=time;
    case 'warm'
        assert(isfinite(history.kappa) && out.exchange_count==1);
        assert(time-history.calibration_time_s<=cfg.ranging.clock_calibration_max_age_s, ...
            'uavmvp:staleCalibration','Expired calibration requires cold service.');
        assert(abs(out.kappa-history.kappa)<1e-12, ...
            'uavmvp:clockMismatch','Use the recorded clock estimate.');
        T(end,2)=out.kappa_sensitivity;Q(end,end)=out.joint_covariance(1,1);
    otherwise
        error('uavmvp:radioKind','Expected cold or warm service.');
end
history.P=T*history.P*T'+Q;history.P=(history.P+history.P')/2;
if strcmp(kind,'warm') && isfield(out,'within_service_drift')
    d=out.within_service_drift;
    assert(d.start_s>=previous_epoch,'uavmvp:overlappingRadioEpochs', ...
        'Within-service drift correction assumes previous measurements precede this exchange.');
    % Correlated extra error is independent of past records and bias, but not
    % of the propagated current clock error. Preserve both cross and diagonal.
    history.P(end,end)=history.P(end,end)+d.extra_range_variance+ ...
        2*out.kappa_sensitivity*d.clock_extra_range_covariance;
    history.P(2,end)=history.P(2,end)+d.clock_extra_range_covariance;
    history.P(end,2)=history.P(2,end);
end
history.last_id=history.last_id+1;id=history.last_id;
history.ranges_m(id)=out.range_m;history.times_s(id)=time;history.kinds{id}=kind;
sigma=sqrt(history.P(end,end));
quality=struct('reported_std_m',sigma,'accepted',isfinite(out.range_m) && ...
    out.range_m>=0 && isfinite(sigma) && sigma<=cfg.ranging.quality_reported_std_limit_m);
end
