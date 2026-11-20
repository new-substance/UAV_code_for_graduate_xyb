function report=run_managed(options)
%RUN_MANAGED T01 one lightweight batch, gated before any world is constructed.
review=uavmvp.review_gate();required={'seeds','count','rule','label'};
assert(isstruct(options) && all(isfield(options,required)) && all(ismember(fieldnames(options),required)));
assert(ismember(options.count,review.development_scope.terminal_counts) && ...
    ismember(options.rule,{'stage','cost'}));
seeds=options.seeds(:)';scope=review.development_scope;
assert(~isempty(seeds) && numel(seeds)<=scope.max_cases_per_batch && ...
    all(ismember(seeds,scope.allowed_seeds)),'uavproject:RunScope','Seeds/count exceed reviewed development scope.');
assert(~isempty(regexp(char(options.label),'^[A-Za-z0-9][A-Za-z0-9_-]{0,79}$','once')));
main=fileparts(fileparts(mfilename('fullpath')));repo=fileparts(main);
root=fullfile(repo,'results','current');if ~isfolder(root),mkdir(root);end
lock=fullfile(root,'.writer_lock');lock_file=java.io.File(lock);
assert(lock_file.mkdir(),'uavproject:WriterBusy','One managed batch may write active results at a time.');
cleanup=onCleanup(@()rmdir(lock)); %#ok<NASGU>
listing=dir(fullfile(root,'**','*'));listing=listing(~[listing.isdir]);active=sum([listing.bytes]);
c=load_project_config();c.scene.terminal_counts=options.count;c.sim.compact_recording=true;
identity=uavmvp.project_fingerprints();
limits=struct('case_bytes',review.budgets_bytes.default_case, ...
    'batch_bytes',review.budgets_bytes.development_batch,'active_bytes',review.budgets_bytes.active_new_results);
manifest=struct('purpose','development_not_independent_evaluation','identity',identity, ...
    'review_purpose',scope.purpose,'configuration_expanded',c,'options',options,'review',review.status);
w=uavmvp.ResultWriter(fullfile(root,char(options.label)),seeds,manifest,limits,active);
stop_batch=false;
for seed=seeds
    w.begin_case(seed);
    if stop_batch
        outcome=struct('reason','SKIPPED_AFTER_STORAGE_LIMIT','end_to_end_success',false);
    else
        try
            outcome=uavpriority.execute_task(c,seed,options.count,w,options.rule);
        catch problem
            message=problem.message;message=message(1:min(512,numel(message)));
            outcome=struct('reason','UNHANDLED_EXECUTOR_EXCEPTION','end_to_end_success',false, ...
                'identifier',problem.identifier,'message',message,'return_status','UNKNOWN');
        end
        if isfield(outcome,'exception') && ~isempty(outcome.exception)
            stop_batch=strcmp(outcome.exception.identifier,'uavproject:StorageBudgetExceeded');
        end
    end
    w.finish_case(outcome);
    fprintf('seed=%d reason=%s complete=%d\n',seed,outcome.reason,outcome.end_to_end_success);
end
report=w.finish();report.directory=w.directory;report.purpose='development_not_independent_evaluation';
end
