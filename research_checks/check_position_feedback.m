function out=check_position_feedback()
%CHECK_POSITION_FEEDBACK Saved-state/proposal contracts, no flight.
repo=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(repo,'matlab_mvp'));
setup_project();c=load_project_config('legacy');nr=size(c.scene.reference_centers_xy_m,1);
[points,refs]=uavpriority.service_candidates(c);
assert(size(points,1)==3*nr && isequal(refs,repmat((1:nr)',3,1)));
assert(all(points>=0,'all') && all(points<=[c.scene.width_m,c.scene.height_m],'all'));
nav=struct('last_reference_id',1);attempts=zeros(size(points,1),1);
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
near=uavpriority.choose_service_point(nav,'VERIFY',f,[],attempts,c,'stage',[1,2]);
near_cost=uavpriority.choose_service_point(nav,'VERIFY',f,[],attempts,c,'cost',[1,2]);
assert(near.available && ismember(near.reference_id,[1,2]));
assert(all(ismember(near.candidates(logical(near.candidates(:,5)),2),[1,2])));
assert(isequal(near.candidates(:,1:9),near_cost.candidates(:,1:9)));
f.x=[100;60];other=uavpriority.choose_service_point(nav,'VERIFY',f,[],attempts,c,'stage');
assert(other.available && norm(other.xy_m-a.xy_m)>c.localization.distinct_horizontal_separation_m);
H=[1,0;.3,.7;-.5,.6;0,-1];s2=c.ranging.range_white_std_m^2/c.priority.cold_services_per_position;
covariance=s2*eye(4)+c.ranging.device_persistent_bias_std_m^2*ones(4);
assert(max(abs(uavpriority.ranging_information(H,c)-H'*(covariance\H)),[],'all')<1e-12);
assert(isequal(uavpriority.ranging_information(zeros(0,2),c),zeros(2)));
zero=c;zero.ranging.device_persistent_bias_std_m=0;
assert(max(abs(uavpriority.ranging_information(H,zero)-H'*H/s2),[],'all')<1e-12);
fixture=uavpriority.choose_service_point(nav,'LOCALIZE',struct('x',[66;75],'radius',18.6), ...
    [50,70;60,70;70,70;80,70],zeros(size(points,1),1),c,'stage');
out=struct('all_passed',true,'mission_count',0,'candidate_count',size(points,1), ...
    'unmeasured_route_vertex_reusable',true,'bounded_no_action',true, ...
    'position_changes_next_point',true,'information_proxy_fixture',fixture);
end
