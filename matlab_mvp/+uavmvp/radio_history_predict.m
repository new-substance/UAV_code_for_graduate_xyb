function history=radio_history_predict(history,time,cfg)
%RADIO_HISTORY_PREDICT Independent future clock drift preserves old cross terms.
assert(isfinite(time) && time>=history.time_s);
dt=time-history.time_s;
history.P(2,2)=history.P(2,2)+(cfg.ranging.clock_ratio_random_walk_ppm_per_sqrt_s*1e-6)^2*dt;
history.time_s=time;
end
