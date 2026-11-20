function timing = radio_timing(cfg,member_count)
%RADIO_TIMING Declared generic CSS block mapping, not vendor LoRa airtime.
if nargin<2,member_count=cfg.css.group_size;end
p=cfg.css; assert(member_count>=1 && member_count==floor(member_count));
timing.symbol_s=2^p.spreading_factor/p.bandwidth_hz;
names=fieldnames(p.frame_application_bytes);
for k=1:numel(names)
    timing.frames.(names{k})=frameDuration(p.frame_application_bytes.(names{k}),p);
end
timing.group_s=frameDuration(p.group_base_bytes+member_count*p.group_member_bytes,p);
timing.exchange_s=timing.frames.range_request+timing.frames.range_response+ ...
    timing.frames.range_metadata+p.response_processing_switch_s+ ...
    p.metadata_processing_switch_s+3*p.guard_s;
timing.reply_interval_s=timing.frames.range_request+p.response_processing_switch_s+p.guard_s;
timing.cold_success_s=max(timing.exchange_s,p.cold_request_min_separation_s)+timing.exchange_s;
timing.cold_max_s=max(p.exchange_deadline_s,p.cold_request_min_separation_s)+p.exchange_deadline_s;
timing.stop_success_s=timing.frames.stop+timing.frames.stop_ack+2*p.guard_s;
assert(timing.exchange_s<=p.exchange_deadline_s,'uavmvp:shortDeadline','Exchange exceeds deadline.');
assert(p.frame_application_bytes.range_metadata>=p.common_application_header_bytes+16+4+1+2);
assert(p.frame_application_bytes.stop_ack>=p.common_application_header_bytes+4+1);
end
function duration=frameDuration(bytes,p)
n=ceil(8*(bytes+p.crc_bytes)/(p.coding_efficiency*p.spreading_factor));
duration=(p.preamble_symbols+p.sync_delimiter_symbols+p.physical_header_symbols+n)* ...
    2^p.spreading_factor/p.bandwidth_hz;
end
