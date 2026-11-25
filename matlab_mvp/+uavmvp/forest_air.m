function [s,out]=forest_air(action,in,c)
%FOREST_AIR C01/H01/S02: IRQ-driven logical airtime, no radio world.
% Only actual measured IMU/capture events advance the shared events state.
out=struct('kind','IDLE');
if strcmp(action,'init')
    s=struct('events',in.events,'current',[],'next_sequence',0,'next_command',0,'input_count',0,'last_rejection','');return
end
s=in.state;
if strcmp(action,'ingest')
    try
        [s,out]=uavmvp.forest_air('observe',in,c);
    catch problem
        s=in.state;s.input_count=min(6*c.algorithm.max_protocol_records,s.input_count+1);s.last_rejection=problem.identifier;
        out=struct('kind','REJECTED','reason',problem.identifier,'fatal',s.input_count>=6*c.algorithm.max_protocol_records);
    end
    return
end
switch action
    case 'sensor'
        assert(ismember(in.event.type,{'imu','capture'}));
        s.events=uavmvp.forest_events('apply',struct('state',s.events,'event',in.event),c);
    case {'start_pair','start_notice','start_data','start_wake'}
        assert(isempty(s.current) || s.current.closed,'forest:RadioBusy','Previous airtime transaction is open.');
        now=s.events.navigation.t;
        if strcmp(action,'start_pair')
            e=in.request;s.events=uavmvp.forest_events('apply',struct('state',s.events,'event',e),c);
            steps={'TX_CONTROL_A','RANGE_MASTER','TX_CONTROL_B','RANGE_SLAVE','RX_REPORT','TX_ACK'};
            budget=uavmvp.forest_radio('budget',[],c);pkt=@(B)uavmvp.forest_radio('packet',B,c);
            durations=[pkt(28),budget.ranging_exchange_s,pkt(28),budget.ranging_exchange_s,pkt(28),pkt(18)];
            deadline=now+c.radio.pair_whole_attempt_bound_s;peer=e.peer;kind='PAIR';
        elseif strcmp(action,'start_notice')
            assert(~any(cellfun(@(x)~x.finished,s.events.pairs)) && isempty(s.events.reference_block) && isempty(s.events.unknown_block), ...
                'forest:RadioBusy','Close ranging blocks before COMPLETE.');
            peer=in.peer;k=find([s.events.protocol.nodes.id]==string(peer));assert(isscalar(k));n=s.events.protocol.nodes(k);
            assert(n.verified && ~n.stop_ack,'forest:Stopped','COMPLETE only after verification.');
            deadline=now+c.service.data_attempt_bound_s;
            assert(n.lease_until_s>=deadline,'forest:Lease','Recontact before notice.');
            e=struct('type','notice_request','time_s',now,'peer',peer,'result_seq',n.notice_result_sequence,'seq',s.next_sequence);
            s.events=uavmvp.forest_events('apply',struct('state',s.events,'event',e),c);
            steps={'TX_COMPLETE','RX_COMPLETE_ACK'};
            durations=[uavmvp.forest_radio('packet',21,c),uavmvp.forest_radio('packet',18,c)];kind='NOTICE';
        elseif strcmp(action,'start_data')
            assert_radio_idle(s.events);
            peer=in.peer;k=find([s.events.protocol.nodes.id]==string(peer));assert(isscalar(k));n=s.events.protocol.nodes(k);
            cap=uavmvp.forest_protocol('data_capacity',[],c);N=ceil(n.expected_data_bytes/cap.payload_bytes);index=in.fragment_index;
            assert(N>=1 && N<=255 && index>=0 && index<N && index==fix(index),'forest:DataProfile','Unsupported fragment request.');
            assert(n.contact && ~n.verified && n.advertised_data_sequence>=0,'forest:DataSequence','No frozen sample/contact.');
            deadline=now+c.service.data_attempt_bound_s;assert(n.lease_until_s>=deadline,'forest:Lease','DATA lease too short.');
            count=min(cap.payload_bytes,n.expected_data_bytes-index*cap.payload_bytes);
            e=struct('type','data_attempt','id',n.id,'sequence',n.advertised_data_sequence,'fragment_index',index+1,'fragment_count',N);
            s.events.protocol=uavmvp.forest_protocol('event',struct('state',s.events.protocol,'event',e),c);
            e.bytes=count;e.guard_s=cap.response_guard_s;e.lease_until_s=n.lease_until_s;
            steps={'TX_DATA_GRANT','RX_DATA','TX_DATA_ACK'};
            durations=[uavmvp.forest_radio('packet',34,c),cap.response_guard_s+uavmvp.forest_radio('packet',24+count,c),uavmvp.forest_radio('packet',18,c)];kind='DATA';
        else
            assert_radio_idle(s.events);[s,e,steps,durations]=wake_request(s,in,c);
            deadline=e.deadline_s;peer=65535;kind='WAKE';
        end
        assert(sum(durations)<=deadline-now,'forest:AirBudget','Bare airtime exceeds transaction deadline.');
        s.current=struct('kind',kind,'request',e,'peer',peer,'steps',{steps},'durations',durations, ...
            'index',1,'issued',false,'closed',false,'deadline_s',deadline,'started_s',now, ...
            'command_id',-1,'stage_started_s',NaN,'stage_deadline_s',NaN,'history',{{}}, ...
            'slave_clone_id',"",'report_sequence',-1,'complete_sequence',-1, ...
            'ack_sent',false,'reason','pending','wake_tx_count',0,'wake_contacts',0,'data_frame_sequence',-1, ...
            'minimum_durations',durations,'data_grant_end_s',NaN);
        if strcmp(kind,'DATA')
            s.current.minimum_durations(2)=c.service.rx_gap_s+uavmvp.forest_radio('packet',24+e.bytes,c);
        end
        assert(s.next_command+numel(steps)<=6*c.algorithm.max_protocol_records,'forest:AirBudget','Complete transaction command budget unavailable.');
    case 'next'
        if isempty(s.current) || s.current.closed,out=outcome(s);return,end
        a=s.current;now=s.events.navigation.t;
        if a.issued
            if now>a.stage_deadline_s,[s,out]=close_failed(s,'radio_timeout');
            else,out=struct('kind','WAIT','deadline_s',a.stage_deadline_s);end
            return
        end
        if strcmp(a.kind,'WAKE')
            start=a.request.starts_s(a.index);finish=a.request.ends_s(a.index);
            if now<start,out=struct('kind','WAIT','not_before_s',start,'deadline_s',finish);return,end
            if strcmp(a.steps{a.index},'TX_WAKE') && now>finish-a.durations(a.index)+1e-12
                [s,out]=close_failed(s,'wake_train_start_late');return
            end
            if strcmp(a.steps{a.index},'RX_ID') && now+a.durations(a.index)>finish+1e-12
                a.index=a.index+1;
                if a.index>numel(a.steps),a.closed=true;a.reason='wake_windows_complete';end
                s.current=a;out=outcome(s);return
            end
        end
        remaining=sum(a.durations(a.index:end));
        if now+remaining>a.deadline_s+1e-12
            [s,out]=close_failed(s,'insufficient_remaining_airtime');return
        end
        assert(s.next_command<6*c.algorithm.max_protocol_records,'forest:AirBudget','Command count exhausted.');
        name=a.steps{a.index};bytes=uint8([]);
        if startsWith(name,'TX_')
            [s,bytes]=transmit_bytes(s,name,c);a=s.current;
        end
        a.command_id=s.next_command;s.next_command=s.next_command+1;a.issued=true;a.stage_started_s=now;
        a.stage_deadline_s=a.deadline_s-sum(a.durations(a.index+1:end));
        if strcmp(a.kind,'WAKE'),a.stage_deadline_s=a.request.ends_s(a.index);end
        if strcmp(name,'RX_DATA'),a.stage_deadline_s=min(a.stage_deadline_s,a.data_grant_end_s+a.durations(2));end
        if now+a.minimum_durations(a.index)>a.stage_deadline_s+1e-12
            [s,out]=close_failed(s,'response_window_missed');return
        end
        s.current=a;
        out=struct('kind','COMMAND','operation',name,'command_id',a.command_id,'peer',a.peer, ...
            'bytes',bytes,'not_before_s',now,'minimum_completion_s',now+a.minimum_durations(a.index), ...
            'deadline_s',a.stage_deadline_s);
    case 'observe'
        e=in.event;a=s.current;
        assert(~isempty(a),'forest:AirEvent','No transaction.');
        assert(s.input_count<6*c.algorithm.max_protocol_records,'forest:AirBudget','Callback budget exhausted.');s.input_count=s.input_count+1;
        assert(isfinite(e.completed_at_s) && e.completed_at_s<=s.events.navigation.t,'forest:AirEvent','Future callback.');
        for j=1:numel(a.history)
            old=a.history{j};
            if old.command_id==e.command_id
                assert(isequal(rmfield(old,'completed_at_s'),rmfield(e,'completed_at_s')),'forest:ReplayConflict','Changed completed callback.');
                out=struct('kind','DUPLICATE');return
            end
        end
        assert(~a.closed && a.issued && e.command_id==a.command_id,'forest:AirEvent','Unissued/out-of-order command callback.');
        now=s.events.navigation.t;
        assert(e.completed_at_s<=now && e.completed_at_s>=a.stage_started_s+a.minimum_durations(a.index)-1e-12, ...
            'forest:AirEvent','Callback precedes physical airtime or processing clock.');
        if strcmp(e.type,'timeout') || e.completed_at_s>a.stage_deadline_s
            assert(now>=a.stage_deadline_s,'forest:AirEvent','Premature timeout.');
            if strcmp(a.kind,'WAKE') && strcmp(a.steps{a.index},'RX_ID')
                a.history{end+1}=e;a.index=a.index+1;a.issued=false;
                if a.index>numel(a.steps),a.closed=true;a.reason='wake_windows_complete';end
                s.current=a;out=outcome(s);return
            end
            [s,out]=close_failed(s,'radio_timeout');return
        end
        name=a.steps{a.index};request=a.request;
        if startsWith(name,'TX_')
            assert(strcmp(e.type,'tx_done'),'forest:AirEvent','Expected TX completion.');
            if strcmp(name,'TX_ACK'),a.ack_sent=true;end
            if strcmp(name,'TX_DATA_ACK'),a.ack_sent=true;end
            if strcmp(name,'TX_WAKE'),a.wake_tx_count=a.wake_tx_count+1;end
            if strcmp(name,'TX_DATA_GRANT'),a.data_grant_end_s=e.completed_at_s;end
        elseif strcmp(name,'RANGE_MASTER')
            assert(strcmp(e.type,'range_done'),'forest:AirEvent','Expected master ranging IRQ.');
            s.events=direction(s.events,request,1,e.clone_id,e.raw,e.completed_at_s,c);
        elseif strcmp(name,'RANGE_SLAVE')
            assert(strcmp(e.type,'range_done') && any(s.events.navigation.clone_ids==string(e.clone_id)), ...
                'forest:AirEvent','Slave exchange needs a captured actual epoch.');
            a.slave_clone_id=string(e.clone_id); % Remote master RAW arrives in REPORT.
        else
            assert(strcmp(e.type,'rx_done') && e.crc_valid,'forest:AirEvent','Expected valid received frame.');
            f=uavmvp.forest_frame('decode',e.bytes);h=f.header;m=f.message;
            ctx=s.events;
            expectedPeer=a.peer;
            if strcmp(name,'RX_ID'),expectedPeer=request.members(a.index-request.packet_count);end
            assert(h.mission==ctx.mission && h.session==ctx.session && h.source==expectedPeer && h.target==ctx.uav_id, ...
                'forest:Session','Foreign transaction response.');
            if strcmp(name,'RX_REPORT')
                assert(h.type==5 && m.transaction==request.transaction && m.calibration_id==request.calibration_ids(2) && ...
                    bitand(uint16(m.quality_flags),uint16(1))~=0,'forest:Transaction','Unmatched/invalid RANGE_REPORT.');
                key=string(h.source)+"/"+string(h.seq);old=find(s.events.seen_keys==key);
                if isempty(old)
                    assert(numel(s.events.seen_keys)<c.algorithm.max_protocol_records,'forest:EventBudget','Received frame registry exhausted.');
                    s.events.seen_keys(end+1,1)=key;s.events.seen_bytes{end+1,1}=e.bytes;
                else
                    assert(isequal(s.events.seen_bytes{old},e.bytes),'forest:ReplayConflict','Peer reused a frame sequence with changed bytes.');
                end
                raw=struct('value',m.raw_result,'valid',true,'representation',c.radio.raw_representation, ...
                    'mode',c.radio.result_mode,'bandwidth_hz',c.radio.ranging_phy.bandwidth_hz,'calibration_offset_m',request.offsets_m(2));
                s.events=direction(s.events,request,2,a.slave_clone_id,raw,e.completed_at_s,c);a.report_sequence=h.seq;
            elseif strcmp(name,'RX_DATA')
                assert(h.type==3 && m.data_seq==request.sequence && m.fragment_index==request.fragment_index-1 && ...
                    m.fragment_count==request.fragment_count && numel(m.samples)==request.bytes, ...
                    'forest:DataSequence','DATA does not match issued grant.');
                received=struct('type','frame','time_s',now,'bytes',e.bytes,'crc_valid',true);
                s.events=uavmvp.forest_events('apply',struct('state',s.events,'event',received),c);a.data_frame_sequence=h.seq;
            elseif strcmp(name,'RX_ID')
                assert(h.type==2,'forest:FrameType','Identity response required in wake slot.');
                received=struct('type','frame','time_s',now,'bytes',e.bytes,'crc_valid',true,'lease_until_s',request.safe_lease_until_s);
                s.events=uavmvp.forest_events('apply',struct('state',s.events,'event',received),c);a.wake_contacts=a.wake_contacts+1;
            else
                assert(h.type==6 && m.acked_type==7 && m.acked_seq==a.complete_sequence && m.status==0, ...
                    'forest:ACK','ACK does not close the issued COMPLETE.');
                received=struct('type','frame','time_s',now,'bytes',e.bytes,'crc_valid',true);
                s.events=uavmvp.forest_events('apply',struct('state',s.events,'event',received),c);
            end
        end
        a.history{end+1}=e;a.index=a.index+1;a.issued=false;
        if a.index>numel(a.steps),a.closed=true;a.reason='transaction_complete';end
        s.current=a;out=outcome(s);
    otherwise,error('forest:Action','Unknown airtime action.');
end
end

function [s,bytes]=transmit_bytes(s,name,c)
a=s.current;ctx=s.events;seq=s.next_sequence;
assert(seq<65536,'forest:Sequence','No sequence wrap within mission.');s.next_sequence=seq+1;
h=struct('version',1,'type',0,'mission',ctx.mission,'session',ctx.session,'seq',seq,'source',ctx.uav_id,'target',a.peer,'flags',0);
if startsWith(name,'TX_CONTROL')
    d=1;if strcmp(name,'TX_CONTROL_B'),d=2;end
    role=2;if d==2,role=1;end % Application roles: 1 master, 2 slave.
    h.type=4;r=a.request;
    m=struct('transaction',r.transaction,'role',role,'count',1,'peer',ctx.uav_id,'sf',c.radio.ranging_phy.sf, ...
        'bw_code',1,'calibration_id',r.calibration_ids(d),'start_after_us',0); % Application BW code 1=1.625MHz, not ASIC register.
elseif strcmp(name,'TX_COMPLETE')
    h.type=7;m=struct('result_seq',a.request.result_seq,'action',1,'ack_window_us',floor((a.deadline_s-ctx.navigation.t-uavmvp.forest_radio('packet',21,c))*1e6));
    a.complete_sequence=seq;s.current=a;
elseif strcmp(name,'TX_WAKE') || strcmp(name,'TX_DATA_GRANT')
    h.type=1;r=a.request;
    if strcmp(name,'TX_DATA_GRANT')
        h.flags=1;h.target=a.peer;ids=a.peer;indices=r.fragment_index-1;index=r.sequence;
        pkt=uavmvp.forest_radio('packet',34,c);E=(r.guard_s-c.service.rx_gap_s)/2;
        remaining=E+c.service.rx_gap_s;slot=a.durations(2);lease=r.lease_until_s;
    else
        ids=r.members;indices=0:numel(ids)-1;index=a.index-1;pkt=a.durations(a.index);
        remaining=r.response_start_s-(ctx.navigation.t+pkt);slot=c.service.response_slot_s;lease=r.lease_until_s;
    end
    assert(remaining>=0,'forest:WakeWindow','Response start already passed.');
    members=repmat(struct('id',0,'role',2,'slot_index',0),1,numel(ids));
    for j=1:numel(ids),members(j).id=ids(j);members(j).slot_index=indices(j);end
    m=struct('train_index',index,'remaining_us',ceil(remaining*1e6),'lease_ms',floor((lease-ctx.navigation.t-pkt)*1000), ...
        'slot_us',floor(slot*1e6),'members',members);
elseif strcmp(name,'TX_DATA_ACK')
    h.type=6;m=struct('acked_type',3,'acked_seq',a.data_frame_sequence,'status',0);
else
    h.type=6;m=struct('acked_type',5,'acked_seq',a.report_sequence,'status',0);
end
bytes=uavmvp.forest_frame('encode',struct('header',h,'message',m));
end

function s=direction(s,r,d,clone,raw,completed,c)
e=struct('type','direction_result','time_s',s.navigation.t,'pair_id',r.pair_id,'peer',r.peer, ...
    'transaction',r.transaction,'direction',d,'clone_id',clone,'raw',raw,'calibration_id',r.calibration_ids(d), ...
    'mission',s.mission,'session',s.session,'result_completed_at_s',completed);
s=uavmvp.forest_events('apply',struct('state',s,'event',e),c);
end

function [s,out]=close_failed(s,reason)
a=s.current;
if strcmp(a.kind,'PAIR')
    k=find(cellfun(@(x)strcmp(x.id,a.request.pair_id),s.events.pairs));p=s.events.pairs{k};
    if ~p.finished
        % Explicit transaction abort can precede whole-pair deadline when a stage cannot fit.
        p.finished=true;p.status=reason;s.events.pairs{k}=p;
    end
end
a.closed=true;a.reason=reason;s.current=a;out=outcome(s);
end
function out=outcome(s)
if isempty(s.current),out=struct('kind','IDLE');return,end
a=s.current;observation=false;noticeAck=false;
if strcmp(a.kind,'PAIR')
    k=cellfun(@(x)strcmp(x.id,a.request.pair_id),s.events.pairs);assert(nnz(k)==1);observation=strcmp(s.events.pairs{k}.status,'received');
end
if strcmp(a.kind,'NOTICE')
    k=[s.events.protocol.nodes.id]==string(a.peer);noticeAck=s.events.protocol.nodes(k).stop_ack;
end
out=struct('kind','WAIT','closed',a.closed,'reason',a.reason,'pair_observation_available',observation,'ack_sent',a.ack_sent, ...
    'complete_ack_received',noticeAck,'physical_sleep_confirmed',false,'wake_tx_count',a.wake_tx_count,'wake_contacts',a.wake_contacts);
if a.closed,out.kind='CLOSED';end
end

function assert_radio_idle(e)
assert(~any(cellfun(@(x)~x.finished,e.pairs)) && isempty(e.reference_block) && isempty(e.unknown_block),'forest:RadioBusy','Close ranging blocks first.');
end
function [s,e,steps,durations]=wake_request(s,in,c)
ids=in.members(:)';K=numel(ids);now=s.events.navigation.t;
assert(K>=1 && K<=c.service.responding_unknown_ids_per_wake && numel(unique(ids))==K);
assert(in.lease_s>0 && isfinite(in.lease_s));
for peer=ids
    k=find([s.events.protocol.nodes.id]==string(peer));assert(isscalar(k));n=s.events.protocol.nodes(k);
    assert(~n.stop_ack && (~n.contact || ~n.discovered || n.lease_until_s<now),'forest:WakeWindow','No due wake/recontact demand.');
    if n.contact
        s.events.protocol=uavmvp.forest_protocol('event',struct('state',s.events.protocol, ...
            'event',struct('type','recontact_attempt','id',n.id)),c);
    end
end
s.events.protocol=uavmvp.forest_protocol('ticket',struct('state',s.events.protocol,'key',in.ticket),c);
pkt=uavmvp.forest_radio('packet',29+5*K,c);spacing=c.service.broadcast_start_spacing_s;
count=ceil((c.service.listener_max_wait_s+c.service.effective_receive_window_s-pkt)/spacing)+1;
slack=c.service.effective_receive_window_s-spacing-pkt;assert(slack>=0);
trainEnd=now+(count-1)*spacing+pkt;
E=c.service.schedule_error_base_bound_s+c.service.relative_clock_rate_bound_ppm*1e-6*(trainEnd-now+slack);
response=trainEnd+slack+E+c.service.rx_gap_s;lease=response+K*c.service.response_slot_s+in.lease_s;
deadline=trainEnd+K*c.service.response_slot_s+c.service.shared_control_prepare_s;
assert(response+K*c.service.response_slot_s<=deadline && lease<=c.service.task_resource_deadline_s,'forest:WakeWindow','Wake/lease budget cannot fit.');
safeLease=lease-c.service.schedule_error_base_bound_s-c.service.relative_clock_rate_bound_ppm*1e-6*(lease-now)-.001;
starts=[now+(0:count-1)*spacing,response-E+(0:K-1)*c.service.response_slot_s];
ends=[starts(1:count)+slack+pkt,starts(count+1:end)+c.service.response_slot_s];
e=struct('members',ids,'packet_count',count,'response_start_s',response,'lease_until_s',lease, ...
    'safe_lease_until_s',safeLease,'starts_s',starts,'ends_s',ends,'deadline_s',deadline);
steps=[repmat({'TX_WAKE'},1,count),repmat({'RX_ID'},1,K)];
durations=[repmat(pkt,1,count),repmat(uavmvp.forest_radio('packet',26,c),1,K)];
end
