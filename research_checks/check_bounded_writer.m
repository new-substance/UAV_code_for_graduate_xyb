function report=check_bounded_writer()
%CHECK_BOUNDED_WRITER Real byte-limit/UTF-8 checks, no world or flight.
repo=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(repo,'matlab_mvp'));
setup_project();parent=fullfile(repo,'tmp');if ~isfolder(parent),mkdir(parent);end
test_root=tempname(parent);mkdir(test_root);
cleanup=onCleanup(@()remove_test_root(test_root,parent)); %#ok<NASGU>
limits=struct('case_bytes',4096,'batch_bytes',32768,'active_bytes',65536);
w=uavmvp.ResultWriter(fullfile(test_root,'batch'),[101,102],struct('kind','deterministic_writer_check'),limits,100);
w.begin_case(101);w.event('中文',0,struct('message','唤醒发现'));
before=w.bytes;
expect(@()w.event('OVERSIZE',1,struct('message',repmat('大',1,2000))), ...
    'uavproject:StorageBudgetExceeded');
assert(w.bytes==before); % no partial JSON or budget overrun on rejection
expect(@()w.event('AFTER_LIMIT',1,struct()),'uavproject:StorageBudgetExceeded');
w.finish_case(struct('status','STORAGE_LIMIT','returned',false));
w.begin_case(102);w.event('UNFINISHED',2,struct('reason','NO_NEW_POSITION'));
expect(@()w.finish(),'uavproject:MissingOutcome');
w.finish_case(struct('status','NOT_COMPLETED','exception','synthetic bounded failure'));
report=w.finish();listing=dir(fullfile(w.directory,'*'));listing=listing(~[listing.isdir]);
assert(sum([listing.bytes])==report.bytes && report.bytes<=limits.batch_bytes);
assert(report.cases==2 && report.events==2);
lines=splitlines(strtrim(fileread(fullfile(w.directory,'outcomes.jsonl'))));
first=jsondecode(lines{1});assert(first.event_budget_exhausted);
payload=fileread(fullfile(w.directory,'events_101.jsonl'));
decoded=jsondecode(payload);assert(strcmp(decoded.kind,'中文'));
expect(@()uavmvp.ResultWriter(w.directory,1,struct(),limits), 'uavproject:OutputExists');
expect(@()uavmvp.ResultWriter(fullfile(test_root,'no_space'),1,struct(),limits,limits.active_bytes), ...
    'uavproject:BudgetTooSmall');
assert(~isfolder(fullfile(test_root,'no_space')));
% Each global quota must also reject a live event while the case quota fits.
large=struct('case_bytes',1000000,'batch_bytes',30000,'active_bytes',100000);
wb=uavmvp.ResultWriter(fullfile(test_root,'batch_quota'),103,struct(),large);
wb.begin_case(103);
expect(@()wb.event('BATCH_LIMIT',0,repmat('x',1,8000)), ...
    'uavproject:StorageBudgetExceeded');
wb.finish_case(struct('status','STORAGE_LIMIT'));rb=wb.finish();
assert(rb.bytes<=large.batch_bytes);
large.batch_bytes=100000;large.active_bytes=30000;
wa=uavmvp.ResultWriter(fullfile(test_root,'active_quota'),104,struct(),large,1000);
wa.begin_case(104);
expect(@()wa.event('ACTIVE_LIMIT',0,repmat('x',1,8000)), ...
    'uavproject:StorageBudgetExceeded');
wa.finish_case(struct('status','STORAGE_LIMIT'));ra=wa.finish();
assert(1000+ra.bytes<=large.active_bytes);
report.all_passed=true;report.mission_count=0;
report.live_case_batch_and_active_limits_checked=true;
end
function expect(action,id)
try
    action();error('uavproject:ExpectedError','Expected %s.',id);
catch problem
    assert(strcmp(problem.identifier,id),'Unexpected error: %s (%s)',problem.message,problem.identifier);
end
end
function remove_test_root(path,parent)
assert(startsWith(path,[parent,filesep]) && isfolder(path));rmdir(path,'s');
end
