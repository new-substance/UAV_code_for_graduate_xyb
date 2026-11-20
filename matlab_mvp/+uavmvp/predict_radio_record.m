function [predicted,new_id,mode]=predict_radio_record(history,start,cfg)
%PREDICT_RADIO_RECORD Nominal covariance forecast, never an observed range.
% The history is a value copy; no protocol or terminal state is changed.
timing=uavmvp.radio_timing(cfg);mode='warm';kappa=history.kappa;ne=1;
if ~isfinite(kappa) || start+timing.exchange_s+1e-5-history.calibration_time_s>cfg.ranging.clock_calibration_max_age_s
    mode='cold';kappa=1;ne=2;
end
alpha_b=1/kappa;c=299792458;nominal_range=20;stamps=zeros(4,ne);intervals=zeros(ne,2);
for k=1:ne
    a=start+(k-1)*cfg.css.cold_request_min_separation_s;
    b=a+nominal_range/c;send=b+timing.reply_interval_s;
    receive=send+nominal_range/c+cfg.sim.radio_hardware_delay_s;
    stamps(:,k)=[a;alpha_b*b;alpha_b*send;receive];intervals(k,:)=[b,send];
end
estimate=kappa;if strcmp(mode,'cold'),estimate=[];end
out=uavmvp.rtt_solve(stamps,1,cfg.sim.radio_hardware_delay_s,estimate, ...
    (cfg.ranging.range_white_std_m/c)^2*eye(numel(stamps)));
epoch=start+timing.exchange_s+(ne-1)*cfg.css.cold_request_min_separation_s+1e-5;
out=uavmvp.rtt_clock_drift(out,intervals,epoch,1,cfg,mode);
[predicted,new_id]=uavmvp.radio_history_append(history,out,epoch,cfg,mode);
end
