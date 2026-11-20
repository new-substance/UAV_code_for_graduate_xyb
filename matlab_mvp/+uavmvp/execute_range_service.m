function [history,report,record]=execute_range_service(world,book,history,id,cfg)
%EXECUTE_RANGE_SERVICE Online bridge: only decoded stamps/nav/error models.
mode='warm';timing=uavmvp.radio_timing(cfg);
if ~isfinite(history.kappa) || world.t+timing.exchange_s+1e-5-history.calibration_time_s>cfg.ranging.clock_calibration_max_age_s
    mode='cold';
end
[nav_id,initial]=world.clone_antenna();
request=book.begin_ranging(id,world.t,nav_id,mode);raw=world.measure_service(request);
record=[];report=struct('accepted',false,'reason','SERVICE_NO_COMPLETE_RESULT','first_discovery',false);
if isempty(raw),return;end
if ~raw.motion_accepted
    report.reason='SHORT_STOP_MODEL_INVALID';world.prune(setdiff(world.nav.clone_ids,raw.effective_nav_snapshot_id));return;
end
kappa=[];if strcmp(mode,'warm'),kappa=history.kappa;end
sigma=cfg.ranging.range_white_std_m/299792458;
out=uavmvp.rtt_solve(raw.timestamps,1,cfg.sim.radio_hardware_delay_s,kappa,sigma^2*eye(numel(raw.timestamps)));
previous_bound=0;
if isfield(history,'kappa_motion_bound'),previous_bound=history.kappa_motion_bound;end
motion=uavmvp.rtt_motion_bound(raw,out,previous_bound);
if motion.total_bound_m>cfg.sim.range_model_bias_bound_limit_m
    report.reason='SHORT_STOP_CLOCK_MOTION_INVALID';
    world.prune(setdiff(world.nav.clone_ids,raw.effective_nav_snapshot_id));return;
end
% Reconstruct physical reply intervals on the local A time scale; propagation
% (sub-microsecond here) is omitted only in the drift covariance integral.
% The scheduler's request epoch is a causal lower bound. Clipping only this
% covariance integration endpoint avoids negative apparent gaps from A1 noise;
% it does not modify any timestamp used to solve the range or clock ratio.
start=max(request.measurement_epoch_s,raw.timestamps(1,:)');
duration=out.kappa*(raw.timestamps(3,:)-raw.timestamps(2,:))';
out=uavmvp.rtt_clock_drift(out,[start,start+duration],raw.received_time_s,1,cfg,mode);
[history,radio_id,quality]=uavmvp.radio_history_append(history,out,raw.received_time_s,cfg,mode);
history.kappa_motion_bound=motion.clock_ratio_bound;
record=struct('mission_id',request.mission_id,'address',id,'kind','RANGE_META', ...
    'session_id',request.session_id,'sequence',1,'crc_valid',true,'range_m',out.range_m, ...
    'reported_std_m',quality.reported_std_m,'exchange_valid',true(1,request.exchange_count), ...
    'exchange_sequences',request.sequences,'nav_snapshot_id',request.nav_snapshot_id, ...
    'measurement_epoch_s',request.measurement_epoch_s,'effective_nav_snapshot_id',raw.effective_nav_snapshot_id, ...
    'received_time_s',raw.received_time_s, ...
    'effective_epoch_s',raw.effective_epoch_s,'initial_nav_snapshot',initial,'radio_history_id',radio_id, ...
    'short_stop_bias_bound_m',motion.total_bound_m,'motion_envelope',motion,'mode',mode);
report=book.receive_range(record,raw.received_time_s);
if ~report.accepted,world.prune(setdiff(world.nav.clone_ids,raw.effective_nav_snapshot_id));end
end
