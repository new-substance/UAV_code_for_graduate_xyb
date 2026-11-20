function out=check_position_feedback()
%CHECK_POSITION_FEEDBACK Saved-state/proposal contracts, no flight.
repo=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(repo,'matlab_mvp'));
setup_project();c=load_project_config();nr=size(c.scene.reference_centers_xy_m,1);
nav=struct('last_reference_id',1);attempts=zeros(2*nr,1);
f=struct('x',[20;25],'radius',15);
a=uavpriority.choose_service_point(nav,'VERIFY',f,[],attempts,c,'stage');
assert(a.available && a.eligible_count>0);
% A previously visited route vertex WITHOUT a qualified measurement is usable.
id=a.candidate_id;first=a.xy_m;
b=uavpriority.choose_service_point(nav,'VERIFY',f,first,attempts,c,'stage');
assert(b.available && norm(first-b.xy_m)>=c.localization.distinct_horizontal_separation_m+1);
assert(~logical(b.candidates(id,5)));
% A failed acquisition is an attempt, never a valid spatial observation.
attempts(id)=c.stage.navigation_recovery_cap;
d=uavpriority.choose_service_point(nav,'VERIFY',f,[],attempts,c,'stage');
assert(d.available && d.candidate_id~=id && ~logical(d.candidates(id,5)));
% All candidates bounded: deterministic no-action instead of an infinite loop.
attempts(:)=c.stage.navigation_recovery_cap;
none=uavpriority.choose_service_point(nav,'LOCALIZE',f,[],attempts,c,'stage');
assert(~none.available && none.eligible_count==0);
% Same candidate set/cost rule supports a fair future task-level comparator.
attempts(:)=0;
base=uavpriority.choose_service_point(nav,'LOCALIZE',f,[],attempts,c,'cost');
new=uavpriority.choose_service_point(nav,'LOCALIZE',f,[],attempts,c,'stage');
assert(isequal(base.candidates(:,1:9),new.candidates(:,1:9)));
f.x=[100;60];other=uavpriority.choose_service_point(nav,'VERIFY',f,[],attempts,c,'stage');
assert(other.available && norm(other.xy_m-a.xy_m)>c.localization.distinct_horizontal_separation_m);
out=struct('all_passed',true,'mission_count',0,'candidate_count',2*nr, ...
    'unmeasured_route_vertex_reusable',true,'bounded_no_action',true, ...
    'position_changes_next_point',true);
end
