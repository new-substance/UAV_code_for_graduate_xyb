classdef ProtocolBook < handle
    %PROTOCOLBOOK Online identity/session/admission ledger, no physical truth.
    % Transport executes the declared windows; this class does not advance time.
    properties (SetAccess=private)
        nodes
        mission_id
        sessions={}
    end
    properties (Access=private)
        cfg
        next_control=1
    end
    methods
        function obj=ProtocolBook(cfg,count,mission)
            assert(count>=1 && count==floor(count) && isfinite(mission));
            obj.cfg=cfg;obj.mission_id=mission;
            node=struct('identity_seen',false,'discovered',false,'completed',false, ...
                'last_contact_s',-Inf,'first_range_time_s',NaN,'range_records',{{}}, ...
                'completion_evidence',[],'stop_token',NaN,'stop_attempts',0,'stop_ack',false);
            obj.nodes=repmat(node,1,count);
        end
        function control=group_control(obj,ids,time)
            ids=sort(unique(ids(:)'));obj.check_ids(ids);
            ids=ids(~[obj.nodes(ids).completed]);
            if isempty(ids),control=[];return;end
            group=ceil(ids/obj.cfg.css.group_size);assert(all(group==group(1)));
            timing=uavmvp.radio_timing(obj.cfg,numel(ids));
            wake=~all([obj.nodes(ids).identity_seen]);ready=0;kind='GROUP_POLL';
            if wake,ready=obj.cfg.css.wake_ready_s;kind='GROUP_WAKE';end
            control=struct('mission_id',obj.mission_id,'kind',kind,'group_id',group(1), ...
                'active_members',ids,'control_id',obj.next_control,'frame_s',timing.group_s, ...
                'ready_s',ready,'reply_order',ids,'schedule', ...
                struct('address',{},'session_id',{},'sequence',{},'reply_offset_s',{},'slot_s',{}));
            obj.next_control=obj.next_control+1;
            slot=timing.frames.identity+obj.cfg.css.guard_s;
            for k=1:numel(ids)
                offset=ready+(k-1)*slot;
                ticket=obj.allocate(ids(k),'ID_REPLY',time+timing.group_s+offset+timing.frames.identity, ...
                    time+timing.group_s+offset+slot+obj.cfg.css.packet_lateness_limit_s,NaN,1);
                control.schedule(k)=struct('address',ids(k),'session_id',ticket.session_id, ...
                    'sequence',1,'reply_offset_s',offset,'slot_s',slot);
            end
            control.reserved_s=timing.group_s+ready+numel(ids)*slot;
        end
        function report=receive_identity(obj,packet,time)
            [ticket,report]=obj.validate(packet,time,'ID_REPLY');
            if ~report.accepted,return;end
            if obj.nodes(ticket.address).completed,report.accepted=false;report.reason='COMPLETED_TERMINAL';return;end
            id=ticket.address;obj.nodes(id).identity_seen=true;obj.nodes(id).last_contact_s=time;
            obj.consume(packet.session_id);report.reason='IDENTITY_ACCEPTED';
        end
        function request=begin_ranging(obj,id,time,nav_id,mode)
            obj.check_ids(id);assert(isscalar(id));node=obj.nodes(id);
            assert(~node.completed,'uavmvp:completedRanging','Completed terminal cannot receive new RTT.');
            assert(node.identity_seen && time>=node.last_contact_s && ...
                time-node.last_contact_s<=obj.cfg.css.contact_validity_s, ...
                'uavmvp:contactExpired','Need valid received identity/contact evidence.');
            assert(isfinite(nav_id) && nav_id>=1 && nav_id==floor(nav_id));
            timing=uavmvp.radio_timing(obj.cfg);count=1;reserved=obj.cfg.css.exchange_deadline_s;
            if strcmp(mode,'cold'),count=2;reserved=timing.cold_max_s;
            else,assert(strcmp(mode,'warm'));end
            ticket=obj.allocate(id,'RANGE_META',time,time+reserved+obj.cfg.css.packet_lateness_limit_s,nav_id,count);
            minimum=timing.exchange_s;if count==2,minimum=timing.cold_success_s;end
            ticket.earliest_receive=time+minimum;obj.sessions{ticket.session_id}=ticket;
            request=struct('mission_id',obj.mission_id,'address',id,'session_id',ticket.session_id, ...
                'sequences',1:count,'mode',mode,'nav_snapshot_id',nav_id,'measurement_epoch_s',time, ...
                'exchange_count',count,'reserved_s',reserved);
        end
        function report=receive_range(obj,packet,time)
            % A transport-assembled service result, only after all required META
            % frames. Two cold exchanges still create ONE spatial record.
            [ticket,report]=obj.validate(packet,time,'RANGE_META');
            if ~report.accepted,return;end
            id=ticket.address;
            if obj.nodes(id).completed,report.accepted=false;report.reason='COMPLETED_TERMINAL';return;end
            required={'range_m','reported_std_m','exchange_valid','exchange_sequences','nav_snapshot_id','measurement_epoch_s'};
            if ~all(isfield(packet,required)),report.accepted=false;report.reason='MISSING_RANGE_FIELDS';return;end
            paired=isequal(packet.exchange_sequences(:)',1:ticket.exchange_count) && ...
                numel(packet.exchange_valid)==ticket.exchange_count && all(packet.exchange_valid==true) && ...
                packet.nav_snapshot_id==ticket.nav_id && packet.measurement_epoch_s==ticket.measurement_time;
            quality=isscalar(packet.range_m) && isfinite(packet.range_m) && packet.range_m>0 && ...
                isscalar(packet.reported_std_m) && isfinite(packet.reported_std_m) && ...
                packet.reported_std_m>=0 && packet.reported_std_m<=obj.cfg.ranging.quality_reported_std_limit_m;
            if ~paired,report.accepted=false;report.reason='INVALID_EXCHANGE_PAIR';return;end
            if ~quality
                obj.consume(packet.session_id); % failed result cannot change quality on retransmission
                report.accepted=false;report.reason='RANGE_QUALITY';return;
            end
            obj.nodes(id).range_records{end+1}=packet;obj.consume(packet.session_id);
            report.first_discovery=~obj.nodes(id).discovered;
            if report.first_discovery
                obj.nodes(id).discovered=true;obj.nodes(id).first_range_time_s=ticket.measurement_time;
            end
            report.reason='RANGE_ACCEPTED';
        end
        function complete(obj,id,evidence)
            obj.check_ids(id);assert(isscalar(id));assert(obj.nodes(id).discovered);
            required={'passed','fit_ids','heldout_id','nav_version'};assert(all(isfield(evidence,required)));
            assert(evidence.passed && numel(evidence.fit_ids)==4 && ...
                numel(unique(evidence.fit_ids))==4 && ~ismember(evidence.heldout_id,evidence.fit_ids));
            ids=[evidence.fit_ids(:)',evidence.heldout_id];
            assert(all(isfinite(ids) & ids>=1 & ids==floor(ids) & ids<=numel(obj.nodes(id).range_records)), ...
                'uavmvp:missingEvidence','Completion must reference five received records.');
            % Geometry/versions must be supplied by the common localization backend.
            assert(isfinite(evidence.nav_version));
            obj.nodes(id).completed=true;obj.nodes(id).completion_evidence=evidence;
            obj.nodes(id).stop_token=obj.mission_id*65536+id;
        end
        function packet=stop_request(obj,id,time)
            obj.check_ids(id);assert(isscalar(id) && obj.nodes(id).completed);
            assert(obj.nodes(id).stop_attempts<obj.cfg.planner.stop_notice_attempt_cap, ...
                'uavmvp:stopAttemptLimit','Finite STOP attempts exhausted.');
            timing=uavmvp.radio_timing(obj.cfg);
            reserved=max(timing.stop_success_s,obj.cfg.css.exchange_deadline_s);
            ticket=obj.allocate(id,'STOP_ACK',time,time+reserved+ ...
                obj.cfg.css.packet_lateness_limit_s,NaN,1);
            ticket.earliest_receive=time+timing.stop_success_s;obj.sessions{ticket.session_id}=ticket;
            obj.nodes(id).stop_attempts=obj.nodes(id).stop_attempts+1;
            packet=struct('mission_id',obj.mission_id,'address',id,'kind','STOP', ...
                'session_id',ticket.session_id,'sequence',1,'crc_valid',true, ...
                'completion_token',obj.nodes(id).stop_token,'reserved_s',reserved);
        end
        function report=receive_stop_ack(obj,packet,time)
            [ticket,report]=obj.validate(packet,time,'STOP_ACK');
            if ~report.accepted,return;end
            if ~isfield(packet,'completion_token') || packet.completion_token~=obj.nodes(ticket.address).stop_token
                report.accepted=false;report.reason='STOP_TOKEN_MISMATCH';return;
            end
            obj.nodes(ticket.address).stop_ack=true;obj.consume(packet.session_id);report.reason='STOP_CONFIRMED';
        end
    end
    methods (Access=private)
        function check_ids(obj,ids)
            assert(all(isfinite(ids) & ids==floor(ids) & ids>=1 & ids<=numel(obj.nodes)));
        end
        function ticket=allocate(obj,id,kind,time,deadline,nav_id,count)
            ticket=struct('session_id',numel(obj.sessions)+1,'address',id,'kind',kind, ...
                'measurement_time',time,'deadline',deadline,'nav_id',nav_id, ...
                'exchange_count',count,'consumed',false,'earliest_receive',time);
            obj.sessions{end+1}=ticket;
        end
        function consume(obj,id)
            ticket=obj.sessions{id};ticket.consumed=true;obj.sessions{id}=ticket;
        end
        function [ticket,report]=validate(obj,packet,time,kind)
            ticket=[];report=struct('accepted',false,'reason','INVALID_HEADER','first_discovery',false);
            fields={'mission_id','address','kind','session_id','sequence','crc_valid'};
            if ~isstruct(packet) || ~all(isfield(packet,fields)),return;end
            if ~isscalar(packet.crc_valid) || ~packet.crc_valid || packet.mission_id~=obj.mission_id || ...
                    ~strcmp(packet.kind,kind) || ~isscalar(packet.address),return;end
            s=packet.session_id;
            if ~isscalar(s) || ~isfinite(s) || s~=floor(s) || s<1 || s>numel(obj.sessions)
                report.reason='UNKNOWN_SESSION';return;
            end
            ticket=obj.sessions{s};
            if ticket.address~=packet.address || ~strcmp(ticket.kind,kind) || packet.sequence~=1
                report.reason='SESSION_MISMATCH';return;
            end
            if ~isfinite(time) || time<ticket.earliest_receive-1e-12 || time>ticket.deadline
                report.reason='PACKET_TIME_WINDOW';return;
            end
            if ticket.consumed,report.reason='DUPLICATE';return;end
            report.accepted=true;report.reason='MATCHED';
        end
    end
end
