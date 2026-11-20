function parts=radio_components(raw,online,cfg)
%RADIO_COMPONENTS Replay declared linear covariance by independent sources.
% Means use the SAME decoded timestamps. This is offline covariance auditing.
white_cfg=cfg;white_cfg.ranging.device_persistent_bias_std_m=0;
white_cfg.ranging.initial_clock_ratio_std_ppm=0;
white_cfg.ranging.clock_ratio_random_walk_ppm_per_sqrt_s=0;
clock_cfg=cfg;clock_cfg.ranging.device_persistent_bias_std_m=0;
clock_cfg.ranging.range_white_std_m=0;
white=uavmvp.radio_history_init(white_cfg);clock=uavmvp.radio_history_init(clock_cfg);
for j=1:numel(online.attempts)
    a=online.attempts{j};
    if a.history_after==a.history_before,continue;end
    found=[];
    for k=1:numel(raw.radio_events)
        e=raw.radio_events{k};
        if strcmp(e.kind,'RANGE_SERVICE') && abs(e.start_s-a.start_s)<1e-7,found=e;break;end
    end
    assert(~isempty(found) && ~isempty(found.raw));
    white=append(white,white_cfg,found);clock=append(clock,clock_cfg,found);
end
n=online.history.last_id;ids=1:n;
W=uavmvp.radio_history_covariance(white,ids);K=uavmvp.radio_history_covariance(clock,ids);
B=cfg.ranging.device_persistent_bias_std_m^2*ones(n);
C=uavmvp.radio_history_covariance(online.history,ids);
relative=norm(C-W-K-B,'fro')/max(1,norm(C,'fro'));
assert(relative<1e-9,'uavstage:budget','Radio covariance components do not reconstruct the declared history.');
parts=struct('white',W,'clock',K,'device',B,'total',C,'relative_reconstruction_error',relative);
end

function history=append(history,cfg,event)
raw=event.raw;request=event.request;mode=request.mode;kappa=[];
if strcmp(mode,'warm'),kappa=history.kappa;end
sigma=cfg.ranging.range_white_std_m/299792458;
out=uavmvp.rtt_solve(raw.timestamps,1,cfg.sim.radio_hardware_delay_s,kappa,sigma^2*eye(numel(raw.timestamps)));
start=max(request.measurement_epoch_s,raw.timestamps(1,:)');
duration=out.kappa*(raw.timestamps(3,:)-raw.timestamps(2,:))';
out=uavmvp.rtt_clock_drift(out,[start,start+duration],raw.received_time_s,1,cfg,mode);
[history,~,~]=uavmvp.radio_history_append(history,out,raw.received_time_s,cfg,mode);
end
