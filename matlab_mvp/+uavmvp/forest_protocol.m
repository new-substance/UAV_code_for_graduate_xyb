function out=forest_protocol(action,in,c)
%FOREST_PROTOCOL D01/S02/C01/C02: bounded event reducer and shared retry ledger.
switch action
    case 'init'
        ids=string(in.unknown_ids(:));assert(numel(unique(ids))==numel(ids) && ~isempty(ids));
        node=struct('id',"",'contact',false,'discovered',false,'data_delivered',false, ...
            'verified',false,'stop_ack',false,'lease_until_s',-Inf,'raw_ids',strings(0,1), ...
            'fragments',zeros(0,1),'fragment_count',0,'data_sequence',-1,'delivered_bytes',0,'expected_data_bytes',-1, ...
            'quality',false,'quality_version',-1,'planning_region_available',false,'complete_groups',0, ...
            'notice_attempts',0,'verification_rounds',0,'recontact_attempts',0, ...
            'fit_reselections',0,'pending_round',[],'fifth_job',[],'used_fifth_ids',strings(0,1), ...
            'published_xy_m',[],'published_version',-1,'notice_result_sequence',0,'failure_reason','');
        nodes=repmat(node,numel(ids),1);for j=1:numel(ids),nodes(j).id=ids(j);end
        out=struct('nodes',nodes,'wake_tickets',strings(0,1),'no_progress',0, ...
            'reference_recoveries',0,'insertions_on_edge',0,'extra_waypoints',0, ...
            'version',0,'closed',false,'event_count',0,'progress_signature',0);
    case 'ticket'
        out=in.state;key=string(in.key);
        assert(~ismember(key,out.wake_tickets),'forest:WakeTicket','Coverage wake ticket already consumed.');
        assert(numel(out.wake_tickets)<c.algorithm.max_protocol_records,'forest:ProtocolBudget','Wake ticket budget exhausted.');
        out.wake_tickets(end+1)=key;
    case 'phase'
        n=in;
        if n.verified
            if ~n.stop_ack && n.notice_attempts<c.finite_policy_limits.notice_attempts_per_node
                out='NOTICE';
            else
                out='NONE';
            end
        elseif n.verification_rounds>=c.finite_policy_limits.verification_rounds_per_node
            out='NONE';
        elseif n.fit_reselections>=c.finite_policy_limits.fit_reselection_attempts_per_node && ~n.quality
            out='NONE';
        elseif ~n.contact
            out='UNSEEN';
        elseif ~n.discovered
            out='INITIAL_FIT';
        elseif ~n.data_delivered
            out='DATA';
        elseif n.complete_groups<4 || ~n.planning_region_available
            out='INITIAL_FIT';
        elseif n.quality
            out='VERIFY';
        else
            out='IMPROVE';
        end
    case 'event'
        out=reduce_event(in.state,in.event,c);
    case 'cost'
        budget=uavmvp.forest_radio('budget',[],c);s=c.service;phase=in.phase;
        switch phase
            case 'REFERENCE'
                jobs=5*s.pairs_per_reference;extra=s.reference_block_shared_extra_attempts;
                seconds=(jobs+extra)*budget.pair_attempt_s;
            case {'INITIAL_FIT','IMPROVE','VERIFY'}
                jobs=s.pairs_per_unknown_group;extra=s.unknown_group_shared_extra_attempts;
                if isfield(in,'missing_pairs'),jobs=in.missing_pairs;end
                assert(jobs>=1 && jobs<=s.pairs_per_unknown_group);
                seconds=(jobs+extra)*budget.pair_attempt_s;
            case 'DATA'
                assert(in.payload_bytes>=0 && in.payload_bytes==fix(in.payload_bytes));
                capacity=0;
                for B=1:231
                    lower=uavmvp.forest_radio('packet',24+B,c)+uavmvp.forest_radio('packet',18,c);
                    if lower<=s.data_attempt_bound_s,capacity=B;end
                end
                assert(capacity>0,'forest:DataDeadline','DATA/ACK lower bound exceeds deadline.');
                fragments=max(1,ceil(in.payload_bytes/capacity));jobs=fragments;extra=0;
                seconds=fragments*s.data_attempts*s.data_attempt_bound_s;
            case 'NOTICE'
                jobs=1;extra=0;seconds=s.data_attempts*s.data_attempt_bound_s;
            otherwise,error('forest:Phase','Invalid service phase.');
        end
        % One shared pool, not max_attempts multiplied independently for all jobs.
        out=struct('seconds',seconds,'required_jobs',jobs,'shared_extra_attempts',extra, ...
            'max_attempts_per_job',s.max_attempts_per_pair);
    case 'attempt'
        out=in.ledger;j=in.job;assert(j>=1 && j<=numel(out.attempts));
        assert(~out.complete(j) && out.attempts(j)<c.service.max_attempts_per_pair,'forest:Retry','Job retry exhausted.');
        if out.attempts(j)>0
            assert(out.extra_used<out.extra_limit,'forest:Retry','Shared extra pool exhausted.');
            out.extra_used=out.extra_used+1;
        end
        out.attempts(j)=out.attempts(j)+1;out.complete(j)=in.success;
    otherwise,error('forest:Action','Unknown protocol action.');
end
end

function s=reduce_event(s,e,c)
assert(s.event_count<c.algorithm.max_protocol_records,'forest:ProtocolBudget','Protocol record budget exhausted.');
s.event_count=s.event_count+1;
if strcmp(e.type,'decision_result')
    signature=sum([s.nodes.discovered])+sum([s.nodes.delivered_bytes])+ ...
        sum([s.nodes.verified])+sum([s.nodes.stop_ack]);
    for j=1:numel(s.nodes),signature=signature+numel(s.nodes(j).raw_ids);end
    progressed=signature>s.progress_signature || e.coverage_advanced;
    if progressed,s.no_progress=0;else,s.no_progress=s.no_progress+1;end
    s.progress_signature=signature;
    if e.edge_advanced,s.insertions_on_edge=0;end
    if e.insertion_executed
        assert(s.insertions_on_edge<c.finite_policy_limits.max_insertions_per_coverage_edge && ...
            s.extra_waypoints<c.finite_policy_limits.max_active_extra_waypoints,'forest:InsertionLimit','Insertion budget exhausted.');
        s.insertions_on_edge=s.insertions_on_edge+1;s.extra_waypoints=s.extra_waypoints+1;
    end
    if e.reference_recovery
        assert(s.reference_recoveries<c.finite_policy_limits.reference_recovery_blocks,'forest:RecoveryLimit','Reference recovery exhausted.');
        s.reference_recoveries=s.reference_recoveries+1;
    end
    return
end
if strcmp(e.type,'reference_version')
    assert(e.version>s.version);s.version=e.version;
    for i=1:numel(s.nodes),if ~s.nodes(i).verified,s.nodes(i).quality=false;end,end
    % Tickets, attempts, rounds and no-progress are deliberately unchanged.
    return
end
k=find([s.nodes.id]==string(e.id));assert(isscalar(k),'forest:NodeID','Unregistered unknown ID.');n=s.nodes(k);
switch e.type
    case 'identity'
        assert(e.data_bytes>=1 && e.data_bytes<=65535 && e.data_bytes==fix(e.data_bytes));
        if n.expected_data_bytes<0,n.expected_data_bytes=e.data_bytes;
        else,assert(n.expected_data_bytes==e.data_bytes,'forest:DataSequence','Changed advertised sample size.');end
        n.contact=true;n.lease_until_s=e.lease_until_s;
    case 'range'
        assert(~n.verified,'forest:Stopped','Verified node cannot receive new RTT service.');
        assert(n.contact && e.valid && isfinite(e.range_m),'forest:Discovery','Identity and valid RTT required.');
        id=string(e.raw_id);
        if ~ismember(id,n.raw_ids),n.raw_ids(end+1,1)=id;n.discovered=true;end
    case 'data'
        assert(n.contact && e.valid && ~n.verified);
        assert(e.fragment_count>=1 && e.fragment_index>=1 && e.fragment_index<=e.fragment_count && e.bytes>=0);
        if n.data_sequence<0,n.data_sequence=e.sequence;n.fragment_count=e.fragment_count;end
        assert(n.data_sequence==e.sequence && n.fragment_count==e.fragment_count,'forest:DataSequence','Mixed sample or fragment count.');
        if ~ismember(e.fragment_index,n.fragments)
            n.fragments(end+1)=e.fragment_index;n.delivered_bytes=n.delivered_bytes+e.bytes;
        end
        assert(n.delivered_bytes<=n.expected_data_bytes,'forest:DataSequence','More bytes than advertised.');
        n.data_delivered=numel(n.fragments)==n.fragment_count && n.delivered_bytes==n.expected_data_bytes;
    case 'fit'
        assert(e.version==s.version && ~n.verified && e.complete_groups>=0 && e.complete_groups<=4);
        n.complete_groups=e.complete_groups;n.planning_region_available=e.region_available;
        n.quality=e.quality;n.quality_version=e.version;
    case 'reserve_verification'
        assert(n.data_delivered && n.quality && n.quality_version==s.version && isempty(n.pending_round) && ~isempty(n.fifth_job));
        ids=string(e.fifth_ids(:));assert(numel(ids)==c.service.pairs_per_unknown_group && numel(unique(ids))==numel(ids) && ...
            ~any(ismember(ids,[string(e.fit_ids(:));n.used_fifth_ids(:);n.fifth_job.raw_ids_before(:)])) && ...
            isequal(string(e.fit_ids(:)),n.fifth_job.fit_ids),'forest:HeldoutID','Reused or unreserved fifth pair.');
        assert(all(ismember([string(e.fit_ids(:));ids],n.raw_ids)),'forest:Snapshot','Snapshot must use actually received pairs.');
        assert(e.snapshot.version==s.version && isequal(e.snapshot.fit_raw_ids(:),string(e.fit_ids(:))) && ...
            isequal(e.snapshot.fifth_raw_ids(:),ids),'forest:Snapshot','Incoherent verification snapshot.');
        n.pending_round=struct('fifth_ids',ids,'version',s.version,'snapshot',e.snapshot);
        n.fifth_job=[];
    case 'request_fifth'
        assert(~n.verified,'forest:Stopped','Verified node cannot request a new heldout block.');
        assert(n.data_delivered && n.quality && n.quality_version==s.version && isempty(n.pending_round) && isempty(n.fifth_job));
        assert(n.verification_rounds<c.finite_policy_limits.verification_rounds_per_node,'forest:VerificationLimit','Verification rounds exhausted.');
        fitIds=string(e.fit_ids(:));assert(numel(fitIds)==4*c.service.pairs_per_unknown_group && ...
            numel(unique(fitIds))==numel(fitIds) && all(ismember(fitIds,n.raw_ids)),'forest:HeldoutID','Invalid frozen four groups.');
        n.fifth_job=struct('fit_ids',fitIds,'raw_ids_before',n.raw_ids);
        n.verification_rounds=n.verification_rounds+1;
    case 'fifth_acquisition_timeout'
        assert(~isempty(n.fifth_job));n.used_fifth_ids=[n.used_fifth_ids;setdiff(n.raw_ids,n.fifth_job.raw_ids_before,'stable')];n.fifth_job=[];
    case 'verification'
        r=n.pending_round;assert(~isempty(r),'forest:VerificationOnce','No unused verification round.');
        % Invalidate once on version change. Never retry the same values under another posterior.
        if r.version==s.version
            v=struct('snapshot',r.snapshot,'values_m',e.values_m,'raw_ids',e.raw_ids,'version',s.version);
            decision=uavmvp.forest_localization('verify',v,c);n.verified=decision.passed;
            if n.verified
                n.published_xy_m=decision.published_xy_m;n.published_version=decision.version;
                n.notice_result_sequence=n.verification_rounds;
            elseif n.verification_rounds>=c.finite_policy_limits.verification_rounds_per_node
                n.failure_reason='verification_round_limit';
            end
        end
        n.used_fifth_ids=[n.used_fifth_ids;r.fifth_ids];n.pending_round=[];
    case 'verification_timeout'
        assert(~isempty(n.pending_round));n.used_fifth_ids=[n.used_fifth_ids;n.pending_round.fifth_ids];n.pending_round=[];
    case 'notice_attempt'
        assert(n.verified && ~n.stop_ack && n.notice_attempts<c.finite_policy_limits.notice_attempts_per_node);
        n.notice_attempts=n.notice_attempts+1;
    case 'complete_ack'
        assert(n.verified && e.valid && n.notice_attempts>0 && e.result_seq==n.notice_result_sequence);n.stop_ack=true;
    case 'recontact_attempt'
        assert(n.recontact_attempts<c.finite_policy_limits.recontact_attempts_per_node);n.recontact_attempts=n.recontact_attempts+1;
    case 'fit_reselection'
        assert(~n.verified,'forest:Stopped','Published node cannot change its fitting selection.');
        assert(isempty(n.pending_round),'forest:VerificationOnce','Finish or timeout the immutable pending round first.');
        assert(n.fit_reselections<c.finite_policy_limits.fit_reselection_attempts_per_node);n.fit_reselections=n.fit_reselections+1;
        if ~isempty(n.fifth_job)
            n.used_fifth_ids=[n.used_fifth_ids;setdiff(n.raw_ids,n.fifth_job.raw_ids_before,'stable')];n.fifth_job=[];
        end
    otherwise,error('forest:Event','Unknown service event.');
end
s.nodes(k)=n;s.closed=all([s.nodes.discovered] & [s.nodes.data_delivered] & [s.nodes.verified] & [s.nodes.stop_ack]);
end
