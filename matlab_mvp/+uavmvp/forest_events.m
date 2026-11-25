function [s,receipt]=forest_events(action,in,c)
%FOREST_EVENTS D01/N02/H01/S02: measured events only; never advances a world.
% Capture actual antenna epochs before asynchronous reports. Time advances by IMU only.
receipt=struct('accepted',true,'duplicate',false);
if strcmp(action,'next_pair')
    s=next_pair(in,c);return
end
if strcmp(action,'init')
    ids=double(in.unknown_ids(:));refs=double(in.reference_ids(:));
    assert(numel(refs)==5 && numel(unique([ids;refs;in.uav_id]))==numel(ids)+6);
    assert(all([ids;refs;in.uav_id]>=0 & [ids;refs;in.uav_id]<65535));
    assert(all([ids;refs;in.uav_id]==fix([ids;refs;in.uav_id])));
    s=struct('navigation',uavmvp.forest_navigation('init',in.samples,c), ...
        'protocol',uavmvp.forest_protocol('init',struct('unknown_ids',string(ids)),c), ...
        'mission',in.mission,'session',in.session,'uav_id',in.uav_id, ...
        'reference_ids',refs,'pairs',{{}},'seen_keys',strings(0,1),'seen_bytes',{{}}, ...
        'notices',{{}},'reference_block',[],'unknown_block',[],'unknown_groups',{{}},'events',0,'imu_events',0, ...
        'rejected_events',0,'last_rejection','');
    return
end
if strcmp(action,'ingest')
    % Value semantics: rejection preserves the prior filter and consumes an input slot.
    try
        [s,receipt]=uavmvp.forest_events('apply',in,c);
        receipt.fatal=false;
    catch problem
        s=in.state;s.rejected_events=s.rejected_events+1;s.last_rejection=problem.identifier;
        s.events=min(c.algorithm.max_protocol_records,s.events+1);
        receipt=struct('accepted',false,'duplicate',false, ...
            'fatal',s.events>=c.algorithm.max_protocol_records,'reason',problem.identifier);
    end
    return
end
assert(strcmp(action,'apply'),'forest:Action','Use init or apply for measured events.');
s=in.state;e=in.event;
assert(isfinite(e.time_s) && e.time_s>=s.navigation.t,'forest:EventTime','Out-of-order event.');
if strcmp(e.type,'imu')
    assert(s.imu_events<ceil(c.service.task_resource_deadline_s/c.initialization.imu_sample_interval_s) && ...
        e.time_s<=c.service.task_resource_deadline_s,'forest:IMUBudget','IMU mission-time budget exhausted.');
    assert(e.time_s>s.navigation.t && e.time_s-s.navigation.t<=c.initialization.imu_sample_interval_s*(1+1e-6), ...
        'forest:IMUGap','Missing IMU interval; no command or truth interpolation allowed.');
    s.navigation=uavmvp.forest_navigation('predict',s.navigation,e.measurement,e.time_s-s.navigation.t,c);
    s.imu_events=s.imu_events+1;
    return
end
assert(s.events<c.algorithm.max_protocol_records,'forest:EventBudget','Event budget exhausted.');
s.events=s.events+1;
assert(abs(e.time_s-s.navigation.t)<1e-10,'forest:EventTime','Propagate actual IMU before processing radio events.');
switch e.type
    case 'capture'
        s.navigation=uavmvp.forest_navigation('clone',s.navigation,e.clone_id,c);
    case 'reference_block'
        assert(isempty(s.reference_block) && isempty(s.unknown_block),'forest:ReferenceBlock','Service block already active.');
        cost=uavmvp.forest_protocol('cost',struct('phase','REFERENCE'),c);
        s.reference_block=struct('id',string(e.block_id),'deadline',e.time_s+cost.seconds, ...
            'counts',zeros(5,1),'ids',strings(0,1), ...
            'ledger',ledger(5*c.service.pairs_per_reference,c.service.reference_block_shared_extra_attempts));
    case 'unknown_group'
        assert(isempty(s.reference_block) && isempty(s.unknown_block),'forest:ServiceBlock','Service block already active.');
        k=find([s.protocol.nodes.id]==string(e.peer));assert(isscalar(k));n=s.protocol.nodes(k);
        assert(n.contact && ~n.verified,'forest:Stopped','No new range block for uncontacted or verified node.');
        assert(isscalar(e.group) && isfinite(e.group) && e.group>=1 && e.group==fix(e.group));
        matches=cellfun(@(x)x.peer==e.peer && x.group==e.group,s.unknown_groups);
        j=find(matches);assert(numel(j)<=1);
        if isempty(j)
            l=ledger(c.service.pairs_per_unknown_group,c.service.unknown_group_shared_extra_attempts);
        else
            l=s.unknown_groups{j}.ledger;
            [point,~]=uavmvp.forest_navigation('antenna',s.navigation,c);
            for z=1:numel(s.pairs)
                p=s.pairs{z};
                if p.peer==e.peer && p.group==e.group && strcmp(p.status,'received')
                    [exists,ci]=ismember(p.clone_ids,s.navigation.clone_ids);
                    assert(all(exists) && all(vecnorm(s.navigation.clones(:,ci)-point,2,1)<=c.algorithm.group_motion_radius_m), ...
                        'forest:GroupMotion','Cannot complete an old group at a different site.');
                end
            end
        end
        seconds=(sum(~l.complete)+l.extra_limit-l.extra_used)*c.radio.pair_whole_attempt_bound_s;
        assert(n.lease_until_s>=e.time_s+seconds,'forest:Lease','Lease cannot contain remaining group.');
        s.unknown_block=struct('id',string(e.block_id),'peer',e.peer,'group',e.group, ...
            'deadline',e.time_s+seconds,'ledger',l);
    case 'pair_request'
        assert(~any(cellfun(@(x)~x.finished,s.pairs)),'forest:RadioBusy','One CSS radio cannot overlap pair transactions.');
        assert(~any(cellfun(@(x)strcmp(x.id,string(e.pair_id)),s.pairs)),'forest:Duplicate','Pair ID already used.');
        assert(e.transaction>=0 && e.transaction<65536 && e.transaction==fix(e.transaction));
        assert(~any(cellfun(@(x)~x.finished && x.peer==e.peer && x.transaction==e.transaction,s.pairs)), ...
            'forest:Transaction','Unclosed peer transaction.');
        if strcmp(e.role,'reference')
            assert(ismember(e.peer,s.reference_ids) && ~isempty(s.reference_block));
            assert(e.time_s+c.radio.pair_whole_attempt_bound_s<=s.reference_block.deadline,'forest:ReferenceBlock','No complete pair budget.');
            j=find(s.reference_ids==e.peer);job=(j-1)*c.service.pairs_per_reference+e.job_index;
            assert(e.job_index>=1 && e.job_index<=c.service.pairs_per_reference && e.job_index==fix(e.job_index));
            s.reference_block.ledger=attempt(s.reference_block.ledger,job,c);
        else
            assert(strcmp(e.role,'unknown'));
            k=find([s.protocol.nodes.id]==string(e.peer));assert(isscalar(k));n=s.protocol.nodes(k);
            assert(n.contact && ~n.verified && n.lease_until_s>=e.time_s+c.radio.pair_whole_attempt_bound_s, ...
                'forest:Lease','Unknown session expired or ranging stopped.');
            assert(~isempty(s.unknown_block) && s.unknown_block.peer==e.peer && s.unknown_block.group==e.group, ...
                'forest:ServiceBlock','Pair lacks a matching unknown group.');
            assert(e.time_s+c.radio.pair_whole_attempt_bound_s<=s.unknown_block.deadline,'forest:PairDeadline','Group cannot contain pair.');
            job=e.job_index;assert(job>=1 && job<=c.service.pairs_per_unknown_group && job==fix(job));
            s.unknown_block.ledger=attempt(s.unknown_block.ledger,job,c);
        end
        assert(numel(s.pairs)<c.algorithm.max_protocol_records,'forest:EventBudget','Pair record cap.');
        p=struct('id',string(e.pair_id),'peer',e.peer,'role',e.role,'transaction',e.transaction, ...
            'deadline',e.time_s+c.radio.pair_whole_attempt_bound_s,'group',e.group,'job',job, ...
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
                s.reference_block.ledger.complete(p.job)=true;
            else
                s.protocol=reduce(s.protocol,struct('type','range','id',string(p.peer),'raw_id',p.id,'valid',true,'range_m',p.range_m),c);
                s.unknown_block.ledger.complete(p.job)=true;
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
        accepted=~unfinished && e.time_s<=b.deadline && all(b.ledger.complete) && all(b.counts>=c.service.pairs_per_reference) && gate.ok;
        if accepted,s.navigation.last_reference_s=e.time_s;end
        receipt.accepted=accepted;s.reference_block=[];
    case 'unknown_group_end'
        b=s.unknown_block;assert(~isempty(b));unfinished=false;
        for j=1:numel(s.pairs)
            p=s.pairs{j};
            if strcmp(p.role,'unknown') && ~p.finished
                unfinished=true;p.finished=true;p.status='aborted_group_end';s.pairs{j}=p;
            end
        end
        receipt.accepted=~unfinished && e.time_s<=b.deadline && all(b.ledger.complete);
        j=find(cellfun(@(x)x.peer==b.peer && x.group==b.group,s.unknown_groups));
        if isempty(j),s.unknown_groups{end+1}=b;else,s.unknown_groups{j}=b;end
        s.unknown_block=[];
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

function l=ledger(jobs,extra)
l=struct('attempts',zeros(jobs,1),'complete',false(jobs,1),'extra_used',0,'extra_limit',extra);
end

function out=next_pair(in,c)
s=in.state;t=s.navigation.t;
if any(cellfun(@(x)~x.finished,s.pairs))
    out=struct('kind','WAIT_FOR_RESULT_OR_TIMEOUT');return
end
if ~isempty(s.reference_block)
    b=s.reference_block;role='reference';
elseif ~isempty(s.unknown_block)
    b=s.unknown_block;role='unknown';
else
    out=struct('kind','IDLE');return
end
if all(b.ledger.complete),out=struct('kind','BLOCK_COMPLETE');return,end
if t+c.radio.pair_whole_attempt_bound_s>b.deadline
    out=struct('kind','BLOCK_EXPIRED');return
end
l=b.ledger;eligible=~l.complete & l.attempts<c.service.max_attempts_per_pair & ...
    (l.attempts==0 | l.extra_used<l.extra_limit);
jobs=find(eligible);
if isempty(jobs),out=struct('kind','BLOCK_EXHAUSTED');return,end
[~,order]=sortrows([l.attempts(jobs),jobs],[1 2]);job=jobs(order(1));
if strcmp(role,'reference')
    peer=s.reference_ids(ceil(job/c.service.pairs_per_reference));
    index=mod(job-1,c.service.pairs_per_reference)+1;group=0;
else
    peer=b.peer;index=job;group=b.group;
end
k=find([in.calibration_profiles.peer]==peer);assert(isscalar(k),'forest:Profile','Missing or duplicate peer calibration.');
profile=in.calibration_profiles(k);number=numel(s.pairs)+1;
assert(number<=c.algorithm.max_protocol_records && number<=65536,'forest:Transaction','Transaction ID budget exhausted.');
e=struct('type','pair_request','time_s',t,'pair_id',string(s.mission)+"/"+string(s.session)+"/"+string(number), ...
    'peer',peer,'role',role,'transaction',number-1,'group',group,'job_index',index, ...
    'calibration_ids',profile.calibration_ids,'offsets_m',profile.offsets_m);
out=struct('kind','REQUEST','event',e,'reserved_attempt_s',c.radio.pair_whole_attempt_bound_s);
end
function l=attempt(l,job,c)
l=uavmvp.forest_protocol('attempt',struct('ledger',l,'job',job,'success',false),c);
end

function s=reduce(s,e,c)
s=uavmvp.forest_protocol('event',struct('state',s,'event',e),c);
end
