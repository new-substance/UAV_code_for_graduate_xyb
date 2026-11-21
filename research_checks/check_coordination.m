function out=check_coordination()
%CHECK_COORDINATION P02 complete-block budget and cross-node dispatch checks.
repo=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(repo,'matlab_mvp'));
setup_project();c=load_project_config();
full=uavpriority.transaction_reservation(c,'VERIFY',4,1000,100);
assert(full.available && full.block_cap==4 && full.min_blocks==2);
near=uavpriority.transaction_reservation(c,'VERIFY',4,100+full.minimum_total_s+.001,100);
assert(near.available && near.block_cap==2 && near.total_s==near.minimum_total_s);
small=uavpriority.transaction_reservation(c,'VERIFY',4,100+full.minimum_total_s-.001,100);
assert(~small.available && small.block_cap==0);
unreachable=uavpriority.transaction_reservation(c,'VERIFY',4,Inf,Inf);
assert(~unreachable.available);
partial=uavpriority.transaction_reservation(c,'LOCALIZE',1,1000,100);
fit=uavpriority.transaction_reservation(c,'LOCALIZE',3,1000,100);
notice=uavpriority.transaction_reservation(c,'NOTICE',0,1000,100);
assert(partial.compute_s==.05 && partial.stop_s==0 && fit.compute_s==8 && fit.stop_s==0);
assert(notice.block_cap==0 && notice.total_s<partial.minimum_total_s && notice.atomic_radio_s==notice.stop_s);
delayed=c;delayed.stage.extra_service_delay_s=20;
before=uavpriority.transaction_reservation(delayed,'VERIFY',4,1000,100);
after=uavpriority.transaction_reservation(delayed,'VERIFY',4,1000,100,true);
assert(abs(before.total_s-after.total_s-20)<1e-10 && after.total_s==full.total_s);
% The first registered ID need not own the next trip; a nearby VERIFY wins.
a=proposal('VERIFY',50,0,0);b=proposal('VERIFY',5,0,0);z=proposal('LOCALIZE',1,1,0);
[id,~,alternatives]=uavpriority.select_service_owner({a,b,z},'stage');
assert(id==2 && size(alternatives,1)==3);
% VERIFY priority is shared with the cost comparator, with identical inputs.
assert(uavpriority.select_service_owner({a,b,z},'cost')==2);
a=proposal('LOCALIZE',2,.01,0);b=proposal('LOCALIZE',10,.03,0);
assert(uavpriority.select_service_owner({a,b},'stage')==2);
assert(uavpriority.select_service_owner({a,b},'cost')==1);
assert(uavpriority.select_service_owner({[],[]},'stage')==0);
nodes=repmat(struct('discovered',true,'completed',false),1,3);
states=repmat({struct('fit',struct('x',[30;40]))},1,3);
assert(uavpriority.service_phase_ready(nodes,states));
nodes(2).discovered=false;[ready,missing]=uavpriority.service_phase_ready(nodes,states);
assert(~ready && isequal(missing,2));nodes(2).discovered=true;
states{2}=[];[ready,missing]=uavpriority.service_phase_ready(nodes,states);
assert(ready && isequal(missing,2));
states{2}=struct('fit',struct('x',[NaN;40]));assert(uavpriority.service_phase_ready(nodes,states));
nodes(2).completed=true;states{2}=[];assert(uavpriority.service_phase_ready(nodes,states));
out=struct('all_passed',true,'mission_count',0,'minimum_complete_blocks',near.block_cap, ...
    'full_transaction_reserve_s',full.total_s,'minimum_transaction_reserve_s',near.total_s, ...
    'no_infinite_return_admission',true,'no_double_wait_charge',true, ...
    'cross_terminal_priority_checked',true,'identical_comparator_inputs',true, ...
    'all_ids_required_without_global_four_fit_barrier',true);
end
function c=proposal(stage,cost,gain,risk)
c=struct('available',true,'stage',stage,'candidate_id',1,'reference_id',1, ...
    'cost_proxy_s',cost,'geometry_gain',gain,'range_risk_m',risk);
end
