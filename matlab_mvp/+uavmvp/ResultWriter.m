classdef ResultWriter < handle
    %RESULTWRITER Bounded UTF-8 events and one outcome per declared case (T01).
    % A reserved outcome slot survives an event-budget failure. No raw MAT,
    % navigation priors, per-step covariance or source snapshots are written.
    properties (SetAccess=private)
        directory
        bytes=0
        event_count=0
        outcomes=0
    end
    properties (Access=private)
        seeds
        limits
        active_before=0
        current=0
        case_bytes=0
        completed
        exhausted=false
        reserve_each=16384
        reserve_batch=8192
    end
    methods
        function obj=ResultWriter(directory,seeds,manifest,limits,active_before)
            if nargin<5,active_before=0;end
            assert(isvector(seeds) && ~isempty(seeds) && ...
                all(isfinite(seeds) & seeds>=0 & seeds==floor(seeds)) && ...
                numel(unique(seeds))==numel(seeds),'uavproject:InvalidSeeds', ...
                'Declare unique nonnegative integer seeds before writing.');
            names={'case_bytes','batch_bytes','active_bytes'};
            assert(all(isfield(limits,names)) && all(cellfun(@(n) ...
                isscalar(limits.(n)) && isfinite(limits.(n)) && ...
                limits.(n)==floor(limits.(n)) && limits.(n)>0,names)));
            assert(isscalar(active_before) && isfinite(active_before) && active_before>=0);
            assert(~isfolder(directory) && ~isfile(directory), ...
                'uavproject:OutputExists','Refusing to overwrite an existing batch.');
            obj.directory=char(directory);obj.seeds=seeds(:)';obj.limits=limits;
            obj.completed=false(size(obj.seeds));obj.active_before=active_before;
            obj.reserve_each=min(obj.reserve_each,floor(limits.case_bytes/2));
            assert(obj.reserve_each>=1024,'uavproject:BudgetTooSmall','Outcome reserve needs at least 1024 bytes.');
            payload=obj.encode(struct('seeds',obj.seeds,'limits',limits,'manifest',manifest));
            reserved=numel(seeds)*obj.reserve_each+obj.reserve_batch;
            assert(numel(payload)+reserved<=limits.batch_bytes && ...
                active_before+numel(payload)+reserved<=limits.active_bytes, ...
                'uavproject:BudgetTooSmall','Batch metadata and all outcome slots must fit before starting.');
            [ok,message]=mkdir(obj.directory);assert(ok,'uavproject:OutputCreate','%s',message);
            obj.write('manifest.json',payload,false);
        end
        function begin_case(obj,seed)
            assert(obj.current==0,'uavproject:CaseOpen','Finish the current outcome first.');
            k=find(obj.seeds==seed,1);
            assert(~isempty(k) && ~obj.completed(k),'uavproject:InvalidCase','Case must be declared and unfinished.');
            obj.current=k;obj.case_bytes=0;obj.exhausted=false;
        end
        function event(obj,kind,time_s,data)
            assert(obj.current>0,'uavproject:NoCase','Begin a case first.');
            assert(~obj.exhausted,'uavproject:StorageBudgetExceeded','Event budget already exhausted; stop the case.');
            assert(isscalar(time_s) && isfinite(time_s) && time_s>=0);
            payload=obj.encode(struct('seed',obj.seeds(obj.current),'kind',char(kind), ...
                'time_s',time_s,'data',data));
            n=numel(payload);reserved=sum(~obj.completed)*obj.reserve_each+obj.reserve_batch;
            fits=obj.case_bytes+n<=obj.limits.case_bytes-obj.reserve_each && ...
                obj.bytes+n+reserved<=obj.limits.batch_bytes && ...
                obj.active_before+obj.bytes+n+reserved<=obj.limits.active_bytes;
            if ~fits
                obj.exhausted=true;
                error('uavproject:StorageBudgetExceeded','Event not written; preserve the reserved outcome and return safely.');
            end
            obj.write(sprintf('events_%d.jsonl',obj.seeds(obj.current)),payload,true);
            obj.case_bytes=obj.case_bytes+n;obj.event_count=obj.event_count+1;
        end
        function finish_case(obj,outcome)
            assert(obj.current>0,'uavproject:NoCase','Begin a case first.');
            row=struct('seed',obj.seeds(obj.current),'event_budget_exhausted',obj.exhausted, ...
                'event_bytes',obj.case_bytes,'outcome',outcome);
            payload=obj.encode(row);
            assert(numel(payload)<=obj.reserve_each,'uavproject:OutcomeTooLarge', ...
                'Keep outcomes within their reserved slot; record compact scalars and bounded node rows.');
            obj.write('outcomes.jsonl',payload,true);
            obj.completed(obj.current)=true;obj.outcomes=obj.outcomes+1;obj.current=0;
        end
        function out=finish(obj)
            assert(obj.current==0 && all(obj.completed),'uavproject:MissingOutcome', ...
                'Every declared case, including skipped and failed cases, needs an outcome.');
            out=struct('cases',obj.outcomes,'events',obj.event_count,'bytes_before_summary',obj.bytes, ...
                'limits',obj.limits,'active_bytes_before_batch',obj.active_before);
            payload=obj.encode(out);assert(numel(payload)<=obj.reserve_batch);
            assert(~isfile(fullfile(obj.directory,'summary.json')), ...
                'uavproject:BatchFinished','Batch has already been finished.');
            obj.write('summary.json',payload,false);out.bytes=obj.bytes;
        end
    end
    methods (Access=private)
        function write(obj,name,payload,append)
            mode='wb';if append,mode='ab';end
            path=fullfile(obj.directory,name);fid=fopen(path,mode);
            assert(fid>=0,'uavproject:OutputOpen','Cannot open output.');
            close_file=onCleanup(@()fclose(fid)); %#ok<NASGU>
            n=fwrite(fid,payload,'uint8');
            assert(n==numel(payload),'uavproject:OutputWrite','Incomplete write (e.g. disk full).');
            obj.bytes=obj.bytes+n;
        end
        function bytes=encode(~,value)
            bytes=uint8(unicode2native([jsonencode(value),newline],'UTF-8'));
        end
    end
end
