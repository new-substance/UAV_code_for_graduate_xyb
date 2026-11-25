function out=forest_radio(action,in,cfg)
%FOREST_RADIO H01/C01: pure unit conversion, paired RAW and airtime accounting.
switch action
    case 'raw'
        assert(strcmp(in.representation,'signed24_sign_extended_int32_counts'));
        x=double(in.value);
        assert(all(isfinite(x(:))) && all(x(:)==fix(x(:))) && ...
            all(x(:)>=-2^23 & x(:)<2^23),'forest:Raw','Invalid signed RAW.');
        assert(in.valid && strcmp(in.mode,'single_exchange_raw_not_internal_sliding_window'), ...
            'forest:Raw','Invalid or history-filtered result.');
        assert(in.bandwidth_hz>0 && isfinite(in.calibration_offset_m));
        out=x*150/(2^12*(in.bandwidth_hz/1e6))-in.calibration_offset_m;
    case 'pair'
        assert(numel(in.raw)==2 && numel(unique(string(in.exchange_ids)))==2);
        a=cfg.radio.pair_direction_weights(:);r=zeros(2,1);
        for k=1:2
            assert(in.raw(k).bandwidth_hz==cfg.radio.ranging_phy.bandwidth_hz,'forest:Profile','Pair mode differs from registered profile.');
            r(k)=uavmvp.forest_radio('raw',in.raw(k),cfg);
        end
        C=uavmvp.forest_covariance(in.direction_covariance_m2);
        assert(isequal(size(C),[2 2]) && abs(sum(a)-1)<1e-12);
        out=struct('range_m',a'*r,'variance_m2',a'*C*a, ...
            'exchange_ids',string(in.exchange_ids),'independent_pair_id',in.pair_id);
    case 'packet'
        p=cfg.radio.ordinary_phy;B=in;
        assert(isscalar(B) && B==fix(B) && B>=0 && B<=255);
        assert(p.sf>=7 && p.sf<=10 && ~p.long_interleaving && p.cr_index>=1 && p.cr_index<=4);
        symbols=p.preamble_symbols+4.25+8+(p.cr_index+4)* ...
            ceil(max(8*B+16*p.crc_enabled-4*p.sf+8+20*p.explicit_header,0)/(4*p.sf));
        out=symbols*2^p.sf/p.bandwidth_hz;
    case 'budget'
        s=cfg.service;p=cfg.radio.ranging_phy;
        re=2^p.sf/p.bandwidth_hz*(p.preamble_symbols+2*p.ranging_symbols+22.25);
        packet=@(B)uavmvp.forest_radio('packet',B,cfg);
        lower=3*packet(28)+packet(18)+2*re;
        K=s.responding_unknown_ids_per_wake;wake=packet(29+5*K);
        assert(s.effective_receive_window_s>=s.broadcast_start_spacing_s+wake,'forest:WakeWindow','Listen window too short.');
        count=ceil((s.listener_max_wait_s+s.effective_receive_window_s-wake)/s.broadcast_start_spacing_s)+1;
        E=s.schedule_error_base_bound_s+s.relative_clock_rate_bound_ppm*1e-6*s.listener_max_wait_s;
        slot=packet(26)+2*E+s.rx_gap_s;
        assert(s.response_slot_s>=slot,'forest:Slot','Response protection too short.');
        assert(cfg.radio.pair_whole_attempt_bound_s>=lower,'forest:PairDeadline','Deadline below radio lower bound.');
        out=struct('pair_radio_lower_s',lower,'pair_attempt_s',cfg.radio.pair_whole_attempt_bound_s, ...
            'wake_train_s',(count-1)*s.broadcast_start_spacing_s+wake, ...
            'wake_packets',count,'response_min_slot_s',slot,'ranging_exchange_s',re);
    case 'link'
        assert(isfinite(in.distance_m) && in.distance_m>=0);
        switch in.kind
            case 'control',p=cfg.radio.control_channel;
            case 'data',p=cfg.radio.data_channel;
            case 'range',p=cfg.radio.ranging_exchange_validity;
            otherwise,error('forest:Channel','Unknown channel kind.');
        end
        out=p.probability_at_short_distance/(1+exp((in.distance_m-p.half_probability_distance_m)/p.transition_scale_m));
    otherwise,error('forest:Action','Unknown radio action.');
end
end
