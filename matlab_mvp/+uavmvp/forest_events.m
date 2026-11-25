function [s,receipt]=forest_events(action,in,c)
%FOREST_EVENTS D01/N02/H01/S02: measured events only; never advances a world.
% Capture actual antenna epochs before asynchronous reports. Time advances by IMU only.
receipt=struct('accepted',true,'duplicate',false);
if strcmp(action,'init')
    ids=double(in.unknown_ids(:));refs=double(in.reference_ids(:));
    assert(numel(refs)==5 && numel(unique([ids;refs;in.uav_id]))==numel(ids)+6);
    assert(all([ids;refs;in.uav_id]>=0 & [ids;refs;in.uav_id]<65535));
    assert(all([ids;refs;in.uav_id]==fix([ids;refs;in.uav_id])));
    s=struct('navigation',uavmvp.forest_navigation('init',in.samples,c), ...
        'protocol',uavmvp.forest_protocol('init',struct('unknown_ids',string(ids)),c), ...
        'mission',in.mission,'session',in.session,'uav_id',in.uav_id, ...
        'reference_ids',refs,'pairs',{{}},'seen_keys',strings(0,1),'seen_bytes',{{}}, ...
        'notices',{{}},'reference_block',[],'events',0);
    return
end
assert(strcmp(action,'apply'),'forest:Action','Use init or apply for measured events.');
s=in.state;e=in.event;
assert(s.events<c.algorithm.max_protocol_records,'forest:EventBudget','Event budget exhausted.');
assert(isfinite(e.time_s) && e.time_s>=s.navigation.t,'forest:EventTime','Out-of-order event.');
s.events=s.events+1;
if strcmp(e.type,'imu')
    assert(e.time_s>s.navigation.t && e.time_s-s.navigation.t<=c.initialization.imu_sample_interval_s*(1+1e-6), ...
        'forest:IMUGap','Missing IMU interval; no command or truth interpolation allowed.');
    s.navigation=uavmvp.forest_navigation('predict',s.navigation,e.measurement,e.time_s-s.navigation.t,c);
    return
end
assert(abs(e.time_s-s.navigation.t)<1e-10,'forest:EventTime','Propagate actual IMU before processing radio events.');
switch e.type
    case 'capture'
        s.navigation=uavmvp.forest_navigation('clone',s.navigation,e.clone_id,c);
    case 'reference_block'
        assert(isempty(s.reference_block),'forest:ReferenceBlock','Reference block already active.');
        cost=uavmvp.forest_protocol('cost',struct('phase','REFERENCE'),c);
        s.reference_block=struct('id',string(e.block_id),'deadline',e.time_s+cost.seconds, ...
            'counts',zeros(5,1),'ids',strings(0,1));
    case 'pair_request'
        assert(~any(cellfun(@(x)~x.finished,s.pairs)),'forest:RadioBusy','One CSS radio cannot overlap pair transactions.');
        assert(~any(cellfun(@(x)strcmp(x.id,string(e.pair_id)),s.pairs)),'forest:Duplicate','Pair ID already used.');
        assert(e.transaction>=0 && e.transaction<65536 && e.transaction==fix(e.transaction));
        assert(~any(cellfun(@(x)~x.finished && x.peer==e.peer && x.transaction==e.transaction,s.pairs)), ...
            'forest:Transaction','Unclosed peer transaction.');
        if strcmp(e.role,'reference')
            assert(ismember(e.peer,s.reference_ids) && ~isempty(s.reference_block));
            assert(e.time_s+c.radio.pair_whole_attempt_bound_s<=s.reference_block.deadline,'forest:ReferenceBlock','No complete pair budget.');
        else
            assert(strcmp(e.role,'unknown'));
            k=find([s.protocol.nodes.id]==string(e.peer));assert(isscalar(k));n=s.protocol.nodes(k);
            assert(n.contact && ~n.verified && n.lease_until_s>=e.time_s+c.radio.pair_whole_attempt_bound_s, ...
                'forest:Lease','Unknown session expired or ranging stopped.');
        end
        assert(numel(s.pairs)<c.algorithm.max_protocol_records,'forest:EventBudget','Pair record cap.');
        p=struct('id',string(e.pair_id),'peer',e.peer,'role',e.role,'transaction',e.transaction, ...
            'deadline',e.time_s+c.radio.pair_whole_attempt_bound_s,'group',e.group, ...
            'calibration_ids',e.calibration_ids,'offsets_m',e.offsets_m, ...
            'raw',{{[],[]}},'clone_ids',strings(1,2),'finished',false,'range_m',NaN,'status','pending');
        assert(numel(p.calibration_ids)==2 && numel(p.offsets_m)==2 && all(isfinite(p.offsets_m)));
        s.pairs{end+1}=p;
    case 'direction_result'
        k=find(cellfun(@(x)strcmp(x.id,string(e.pair_id)),s.pairs));assert(isscalar(k));p=s.pairs{k};d=e.direction;
        assert(e.mission==s.mission && e.session==s.session,'forest:Session','Foreign ranging result.');
        assert(ismember(d,[1 2]));
        assert(e.peer==p.peer && e.transaction==p.transaction && e.calibration_id==p.calibration_ids(d),'forest:Transaction','Unmatched result.');
        if ~isempty(p.raw{d})
            assert(isequal(p.raw{d},e.raw) && p.clone_ids(d)==string(e.clone_id),'forest:ReplayConflict','Changed direction result.');
            receipt.duplicate=true;return
        end
        assert(~p.finished && e.time_s<=p.deadline,'forest:PairDeadline','Pair expired or closed.');
        ci=find(s.navigation.clone_ids==string(e.clone_id));assert(isscalar(ci),'forest:CloneID','Actual epoch was not captured.');
        assert(s.navigation.clone_times(ci)<=e.time_s && s.navigation.clone_times(ci)>=p.deadline-c.radio.pair_whole_attempt_bound_s, ...
            'forest:Epoch','Pose outside reserved transaction.');
        raw=e.raw;assert(raw.calibration_offset_m==p.offsets_m(d) && raw.bandwidth_hz==c.radio.ranging_phy.bandwidth_hz,'forest:Profile','Wrong calibration/profile.');
        uavmvp.forest_radio('raw',raw,c);
        p.raw{d}=raw;p.clone_ids(d)=string(e.clone_id);
        if all(~cellfun(@isempty,p.raw))
            sigma=c.radio.unknown_pair_white_std_m;
            if strcmp(p.role,'reference'),sigma=c.radio.reference_pair_white_std_m;end
            pair=struct('raw',[p.raw{1} p.raw{2}],'exchange_ids',p.id+["/AB" "/BA"], ...
                'pair_id',p.id,'direction_covariance_m2',2*sigma^2*eye(2));
            result=uavmvp.forest_radio('pair',pair,c);p.range_m=result.range_m;p.finished=true;p.status='received';
            if strcmp(p.role,'reference')
                b=struct('role','reference','raw_ids',p.id,'epoch_s',e.time_s, ...
                    'reference_indices',find(s.reference_ids==p.peer),'values_m',p.range_m, ...
                    'direction_clone_ids',p.clone_ids,'block_complete',false);
                s.navigation=uavmvp.forest_navigation('reference',s.navigation,b,c);
                s.protocol=reduce(s.protocol,struct('type','reference_version','version',s.navigation.version),c);
                j=find(s.reference_ids==p.peer);s.reference_block.counts(j)=s.reference_block.counts(j)+1;
                s.reference_block.ids(end+1,1)=p.id;
            else
                s.protocol=reduce(s.protocol,struct('type','range','id',string(p.peer),'raw_id',p.id,'valid',true,'range_m',p.range_m),c);
            end
        end
        s.pairs{k}=p;
    case 'pair_timeout'
        k=find(cellfun(@(x)strcmp(x.id,string(e.pair_id)),s.pairs));assert(isscalar(k));p=s.pairs{k};
        assert(~p.finished && e.time_s>=p.deadline,'forest:PairDeadline','Cannot timeout before deadline.');
        p.finished=true;p.status='timeout';s.pairs{k}=p;
    case 'reference_block_end'
        b=s.reference_block;assert(~isempty(b));gate=uavmvp.forest_navigation('gate',s.navigation,c);
        unfinished=false;
        for j=1:numel(s.pairs)
            p=s.pairs{j};
            if strcmp(p.role,'reference') && ~p.finished
                unfinished=true;p.finished=true;p.status='aborted_block_end';s.pairs{j}=p;
            end
        end
        accepted=~unfinished && e.time_s<=b.deadline && all(b.counts>=c.service.pairs_per_reference) && gate.ok;
        if accepted,s.navigation.last_reference_s=e.time_s;end
        receipt.accepted=accepted;s.reference_block=[];
    case 'release'
        mandatory=strings(0,1);
        for j=1:numel(s.pairs)
            p=s.pairs{j};needed=~p.finished;
            if strcmp(p.role,'unknown') && strcmp(p.status,'received')
                node=s.protocol.nodes([s.protocol.nodes.id]==string(p.peer));needed=~node.verified;
            end
            if needed,mandatory=[mandatory;p.clone_ids(p.clone_ids~="")'];end %#ok<AGROW>
        end
        assert(all(ismember(mandatory,string(e.keep_ids))),'forest:HistoryRetention','Unfinished service still needs these position blocks.');
        s.navigation=uavmvp.forest_navigation('prune',s.navigation,e.keep_ids);
    case 'notice_request'
        k=find([s.protocol.nodes.id]==string(e.peer));assert(isscalar(k));n=s.protocol.nodes(k);
        assert(e.result_seq==n.notice_result_sequence && e.seq>=0 && e.seq<65536);
        s.protocol=reduce(s.protocol,struct('type','notice_attempt','id',string(e.peer)),c);
        s.notices{end+1}=struct('peer',e.peer,'seq',e.seq,'result_seq',e.result_seq);
    case 'frame'
        assert(e.crc_valid,'forest:CRC','Invalid PHY frame.');f=uavmvp.forest_frame('decode',e.bytes);h=f.header;m=f.message;
        assert(h.mission==s.mission && h.session==s.session && h.target==s.uav_id,'forest:Session','Foreign/stale frame.');
        assert(any([s.protocol.nodes.id]==string(h.source)),'forest:NodeID','Unexpected source.');
        key=string(h.source)+"/"+string(h.seq);
        j=find(s.seen_keys==key);
        if ~isempty(j)
            assert(isequal(s.seen_bytes{j},e.bytes),'forest:ReplayConflict','Same sequence with changed bytes.');
            receipt.duplicate=true;return
        end
        switch h.type
            case 2
                v=struct('type','identity','id',string(h.source),'data_bytes',m.data_bytes,'lease_until_s',e.lease_until_s);
            case 3
                v=struct('type','data','id',string(h.source),'valid',true,'sequence',m.data_seq, ...
                    'fragment_count',m.fragment_count,'fragment_index',m.fragment_index+1,'bytes',numel(m.samples));
            case 6
                matches=cellfun(@(x)x.peer==h.source && x.seq==m.acked_seq,s.notices);
                j=find(matches,1,'last');assert(m.acked_type==7 && m.status==0 && ~isempty(j),'forest:ACK','ACK lacks issued COMPLETE.');
                v=struct('type','complete_ack','id',string(h.source),'valid',true,'result_seq',s.notices{j}.result_seq);
            otherwise,error('forest:FrameType','Use scheduled direction_result for hardware ranging reports.');
        end
        s.protocol=reduce(s.protocol,v,c);s.seen_keys(end+1,1)=key;s.seen_bytes{end+1,1}=e.bytes;
    otherwise,error('forest:Event','Unknown measured event.');
end
end

function s=reduce(s,e,c)
s=uavmvp.forest_protocol('event',struct('state',s,'event',e),c);
end
