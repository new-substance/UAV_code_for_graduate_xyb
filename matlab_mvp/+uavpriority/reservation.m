function r=reservation(cfg)
base=uavstage.reservation(cfg);bursts=min(cfg.priority.cold_services_per_position,cfg.priority.services_per_atomic_block);
% Each complete cold service has its own bounded attempts and identity poll.
r=base;r.bursts=bursts;r.atomic_service_cap=bursts;
r.spatial_service_target=cfg.priority.cold_services_per_position;
r.backoff_s=bursts*base.backoff_s;
r.atomic_radio_s=base.settle_s+bursts*base.attempt_count*(base.group_each_s+base.cold_each_max_s)+ ...
    r.backoff_s+base.margin_s;
r.radio_window_s=r.atomic_radio_s;
r.block_cap=cfg.priority.burst_recovery_blocks;
r.total_s=r.block_cap*(r.atomic_radio_s+base.post_radio_reference_s)+ ...
    base.extra_service_s+base.compute_s+base.stop_s;
end
