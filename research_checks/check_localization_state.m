function out=check_localization_state()
%CHECK_LOCALIZATION_STATE Fixed measurements through the 4+1 state interface.
repo=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(repo,'matlab_mvp'));
setup_project();c=load_project_config();
a=[10,10,15;50,10,15;10,50,15;50,50,15;30,10,15]';x=[30;30];
b=struct('anchors',a,'ranges',vecnorm(a-[x;0])'+[.1;-.1;.05;-.05;.02], ...
    'Pnav',.04*eye(15),'Cradio',eye(5)+9*ones(5),'record_ids',1:5, ...
    'nav_ids',1:5,'version',1,'model_bias_bounds_m',zeros(5,1), ...
    'aggregation_matrix',eye(5),'constituent_ids',{{1,2,3,4,5}});
b.Pnavigation=b.Pnav;b.Ptiming=zeros(15);s=[];
for k=1:4
    [s,~]=uavpriority.update_localization(s,uavpriority.subbundle(b,1:k),c);
    if k<4,assert(strcmp(s.phase,'LOCALIZE'));end
end
assert(strcmp(s.phase,'VERIFY') && isequal(s.fit_ids,1:4));
frozen=s;[s,d]=uavpriority.update_localization(s,b,c);
assert(strcmp(s.phase,'DONE') && s.proof.heldout_id==5 && d.check.passed);
assert(~ismember(5,s.proof.fit_ids) && s.verification_rounds==1);
% A corrupted future heldout cannot alter the frozen fit or be accepted.
bad=b;bad.ranges(5)=bad.ranges(5)+100;
[failed,detail]=uavpriority.update_localization(frozen,bad,c);
assert(~strcmp(failed.phase,'DONE') && ~detail.check.passed);
assert(isequal(detail.input.record_ids,1:5));
% Returning to a fitted location is not a fifth spatial position.
same=b;same.anchors(:,5)=same.anchors(:,1);
[waiting,ignored]=uavpriority.update_localization(frozen,same,c);
assert(strcmp(waiting.phase,'VERIFY') && waiting.verification_rounds==0 && ...
    strcmp(ignored.reason,'NOT_A_NEW_POSITION'));
% A later navigation version refreshes the same four, still holding out #5.
new=b;new.version=2;[changed,~]=uavpriority.update_localization(frozen,new,c);
assert(changed.proof.nav_version==2 && isequal(changed.proof.fit_ids,1:4));
overlap=b;overlap.constituent_ids{5}=1;
try
    uavpriority.update_localization(frozen,overlap,c);
    error('uavproject:ExpectedError','Repeated raw service was not rejected.');
catch problem
    assert(strcmp(problem.identifier,'uavproject:RepeatedConstituent'));
end
out=struct('all_passed',true,'mission_count',0,'four_plus_one',true, ...
    'bad_heldout_rejected',true,'same_position_rejected',true,'version_refresh',true, ...
    'raw_members_disjoint',true);
end
