classdef TerminalTracker < handle
    %TERMINALTRACKER Fixed four-fit/new-heldout rounds with finite recovery.
    % Public inputs are admitted records, navigation history and radio models.
    properties (SetAccess=private)
        terminals
        computation_s=0
        event_log={}
        quality_log={}
    end
    properties (Access=private)
        cfg
        book
    end
    methods
        function obj=TerminalTracker(cfg,book)
            obj.cfg=cfg;obj.book=book;
            state=struct('phase','SEARCH','reason','AWAITING_FIRST_QUALIFIED_RANGE', ...
                'seen_records',0,'pool_ids',[],'ignored_ids',[],'fit_ids',[],'fit',[], ...
                'fit_version',-1,'fit_after_index',0,'verification_rounds',0,'used_heldout',[], ...
                'proof',[],'postcheck_version',-1);
            obj.terminals=repmat(state,1,numel(book.nodes));
        end
        function ingest(obj,id,nav,history)
            clock=tic;guard=onCleanup(@()obj.charge(toc(clock))); %#ok<NASGU>
            state=obj.terminals(id);records=obj.book.nodes(id).range_records;
            for index=state.seen_records+1:numel(records)
                assert(~ismember(state.phase,{'DONE','FAILED'}),'uavmvp:terminalEnded','No new terminal work after termination.');
                state.seen_records=index;
                if ~isempty(state.pool_ids)
                    data=uavmvp.record_bundle(records,nav,history,[state.pool_ids,index]);
                    pairs=[(1:numel(state.pool_ids))',repmat(numel(state.pool_ids)+1,numel(state.pool_ids),1)];
                    if ~uavmvp.distinct_positions(data.anchors,data.Pnav,obj.cfg,pairs)
                        state.ignored_ids(end+1)=index;state.reason='NO_NEW_DISTINCT_POSITION';
                        obj.log(id,index,state.reason,nav.version,[]);continue;
                    end
                end
                state.pool_ids(end+1)=index;
                if strcmp(state.phase,'SEARCH'),state.phase='LOCALIZE';end
                if strcmp(state.phase,'VERIFY')
                    assert(index>state.fit_after_index && ~ismember(index,state.used_heldout));
                    state.used_heldout(end+1)=index;state.verification_rounds=state.verification_rounds+1;
                    data=uavmvp.record_bundle(records,nav,history,[state.fit_ids,index]);
                    if state.fit_version~=nav.version
                        state.fit=obj.fit_four(data);state.fit_version=nav.version;
                    end
                    check=uavmvp.verify_position(state.fit,data.anchors,data.ranges,data.Pnav,data.Cradio,obj.cfg);
                    if check.passed
                        state.phase='DONE';state.reason='VERIFIED';
                        state.proof=struct('passed',true,'fit_ids',state.fit_ids,'heldout_id',index, ...
                            'nav_version',nav.version,'check',check);
                        state.postcheck_version=nav.version;obj.book.complete(id,state.proof);
                    else
                        state.phase='LOCALIZE';state.reason=check.reason;state.fit_after_index=index;
                        if state.verification_rounds>=obj.cfg.localization.verification_round_cap
                            state.phase='FAILED';state.reason='VERIFICATION_LIMIT';
                        end
                    end
                    obj.log(id,index,state.reason,nav.version,check);
                    obj.log_quality('VERIFY',id,index,nav,data,state.fit,check,'VERIFY',state.phase,state.reason);
                elseif numel(state.pool_ids)>=4
                    bundle=uavmvp.record_bundle(records,nav,history,state.pool_ids);
                    probe=[obj.cfg.scene.width_m/2;obj.cfg.scene.height_m/2];
                    if ~isempty(state.fit) && allFinite(state.fit.x),probe=state.fit.x;end
                    required=[];if state.fit_after_index>0,required=index;end
                    [chosen,selection]=uavmvp.select_four(bundle,required,obj.cfg,probe);
                    state.fit_after_index=index;
                    if isempty(chosen)
                        state.reason='NO_DISTINCT_FOUR_GROUP';
                    else
                        data=uavmvp.record_bundle(records,nav,history,chosen);
                        state.fit_ids=chosen;state.fit=obj.fit_four(data);state.fit_version=nav.version;
                        state.reason=state.fit.reason;
                        if state.fit.quality,state.phase='VERIFY';end
                        obj.log_quality('FIT',id,index,nav,data,state.fit,[],'LOCALIZE',state.phase,state.reason);
                    end
                    obj.log(id,index,state.reason,nav.version,selection);
                else
                    state.reason='NEED_FOUR_POSITIONS';
                end
                if ~strcmp(state.phase,'DONE') && numel(state.pool_ids)>=obj.cfg.localization.new_positions_per_terminal_cap
                    state.phase='FAILED';state.reason='POSITION_LIMIT';obj.log(id,index,state.reason,nav.version,[]);
                end
            end
            obj.terminals(id)=state;
        end
        function refresh_completed(obj,nav,histories)
            clock=tic;guard=onCleanup(@()obj.charge(toc(clock))); %#ok<NASGU>
            for id=1:numel(obj.terminals)
                state=obj.terminals(id);
                if ~strcmp(state.phase,'DONE') || state.postcheck_version==nav.version,continue;end
                proof=state.proof;data=uavmvp.record_bundle(obj.book.nodes(id).range_records,nav,histories{id}, ...
                    [proof.fit_ids,proof.heldout_id]);
                fit=obj.fit_four(data);check=uavmvp.verify_position(fit,data.anchors,data.ranges,data.Pnav,data.Cradio,obj.cfg);
                state.postcheck_version=nav.version;
                if check.passed
                    state.fit=fit;state.fit_version=nav.version;state.proof.nav_version=nav.version;state.proof.check=check;
                else
                    state.phase='FAILED';state.reason='POSTCHECK_INCONSISTENT';
                    % Protocol completed flag stays set: no reacquisition RTT.
                end
                obj.terminals(id)=state;obj.log(id,proof.heldout_id,state.reason,nav.version,check);
                obj.log_quality('POSTCHECK',id,proof.heldout_id,nav,data,fit,check,'DONE',state.phase,state.reason);
            end
        end
        function ids=needed_clones(obj)
            ids=[];
            for id=1:numel(obj.terminals)
                records=obj.book.nodes(id).range_records;pool=obj.terminals(id).pool_ids;
                if ~isempty(pool),selected=[records{pool}];ids=[ids,[selected.effective_nav_snapshot_id]];end %#ok<AGROW>
            end
            ids=unique(ids);
        end
    end
    methods (Access=private)
        function fit=fit_four(obj,data)
            fit=uavmvp.fit_position(data.anchors(:,1:4),data.ranges(1:4),data.Pnav(1:12,1:12),data.Cradio(1:4,1:4),obj.cfg);
        end
        function charge(obj,value)
            obj.computation_s=obj.computation_s+value;
        end
        function log(obj,id,index,reason,version,detail)
            obj.event_log{end+1}=struct('address',id,'record_index',index,'reason',reason,'nav_version',version,'detail',detail);
        end
        function log_quality(obj,kind,id,index,nav,bundle,fit,check,before,after,reason)
            % Immutable value snapshots; no truth, policy input or extra solve.
            % Keep the newly evaluated fit on a failed postcheck, rather than
            % the last accepted fit retained in the terminal's state.
            heldout=[];accepted=false;
            if ~isempty(check),heldout=bundle.record_ids(5);accepted=check.passed;end
            obj.quality_log{end+1}=struct('sequence',numel(obj.quality_log)+1, ...
                'kind',kind,'address',id,'record_index',index,'time_s',nav.t, ...
                'nav_version',nav.version,'fit_ids',bundle.record_ids(1:4), ...
                'heldout_id',heldout,'input',bundle,'fit',fit,'check',check, ...
                'accepted',accepted,'phase_before',before,'phase_after',after,'state_reason',reason);
        end
    end
end
function ok=allFinite(x)
ok=all(isfinite(x));
end
