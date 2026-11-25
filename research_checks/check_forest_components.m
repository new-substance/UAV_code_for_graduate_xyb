function out=check_forest_components()
%CHECK_FOREST_COMPONENTS Saved-input component regressions only. No task/world.
repo=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(repo,'matlab_mvp'));setup_project();c=load_project_config();
assert(strcmp(c.kind,'forest_css_imu') && ~c.mission_enabled && ~isfield(c,'fixed_review') && ~isfield(c,'project_context'));
legacy=load_project_config('legacy');assert(isfield(legacy,'localization'));
must_fail(@()project('run',struct()),'uavproject:SimulationNotApproved'); %#ok<PRJET> project is this repo's entry.
P=uavmvp.forest_covariance([1 .2;.2 2]);assert(P(1,2)==.2);
must_fail(@()uavmvp.forest_covariance([1 2;2 1]),'forest:Covariance');
raw=struct('representation','signed24_sign_extended_int32_counts','value',-17, ...
    'valid',true,'mode','single_exchange_raw_not_internal_sliding_window','bandwidth_hz',1625000,'calibration_offset_m',.3);
r=uavmvp.forest_radio('raw',raw,c);assert(abs(r-(-17*150/(4096*1.625)-.3))<1e-12 && r<0);
bad=raw;bad.value=2^23;must_fail(@()uavmvp.forest_radio('raw',bad,c),'forest:Raw');
bad=raw;bad.mode='filtered';must_fail(@()uavmvp.forest_radio('raw',bad,c),'forest:Raw');
pair=struct('raw',[raw raw],'exchange_ids',["AB" "BA"],'direction_covariance_m2',[4 3;3 4],'pair_id',"p1");
pr=uavmvp.forest_radio('pair',pair,c);assert(abs(pr.variance_m2-3.5)<1e-12);
timing=uavmvp.forest_radio('budget',[],c);assert(abs(timing.pair_radio_lower_s-.1190990769230769)<1e-10);
assert(abs(timing.wake_train_s-(.55+89.25*256/1625000))<1e-12);
assert(abs(uavmvp.forest_policy('segment',5,c)-4.5)<1e-12);
ledger=struct('attempts',zeros(10,1),'complete',false(10,1),'extra_used',0,'extra_limit',2);
for k=1:10,ledger=uavmvp.forest_protocol('attempt',struct('ledger',ledger,'job',k,'success',false),c);end
for k=1:2,ledger=uavmvp.forest_protocol('attempt',struct('ledger',ledger,'job',k,'success',true),c);end
assert(sum(ledger.attempts)==12);must_fail(@()uavmvp.forest_protocol('attempt',struct('ledger',ledger,'job',3,'success',true),c),'forest:Retry');
refCost=uavmvp.forest_protocol('cost',struct('phase','REFERENCE'),c);assert(refCost.seconds==3);
dataCost=uavmvp.forest_protocol('cost',struct('phase','DATA','payload_bytes',231),c);assert(dataCost.required_jobs>1);
assert(uavmvp.forest_radio('link',struct('distance_m',100,'kind','control'),c)> ...
    uavmvp.forest_radio('link',struct('distance_m',100,'kind','range'),c));
h=struct('version',1,'type',5,'mission',3,'session',4,'seq',257,'source',2,'target',1,'flags',0);
message=struct('transaction',9,'raw_result',-17,'quality_flags',1,'calibration_id',7,'epoch_local',100);
bytes=uavmvp.forest_frame('encode',struct('header',h,'message',message));
decoded=uavmvp.forest_frame('decode',bytes);assert(numel(bytes)==28 && decoded.message.raw_result==-17 && isequal(decoded.header,h));
assert(isequal(bytes(7:8),uint8([1 1])) && all(bytes(17:19)==255));
must_fail(@()uavmvp.forest_frame('decode',bytes(1:end-1)),'forest:FrameSize');
h.type=1;member=struct('id',12,'role',2,'slot_index',0);
message=struct('train_index',2,'remaining_us',500000,'lease_ms',1000,'slot_us',20000,'members',repmat(member,1,4));
bytes=uavmvp.forest_frame('encode',struct('header',h,'message',message));
decoded=uavmvp.forest_frame('decode',bytes);assert(numel(bytes)==49 && numel(decoded.message.members)==4 && decoded.message.remaining_us==500000);

samples=struct('acc',repmat([0;0;9.80665],1,300),'gyro',zeros(3,300), ...
    'yaw_rad',0,'stationary_confirmed',true,'acc_bias_prior_mean_mps2',zeros(3,1));
n=uavmvp.forest_navigation('init',samples,c);
assert(norm(n.R-eye(3),'fro')<1e-12 && n.P(8,10)>0 && n.P(7,11)<0 && n.P(35,24)>0);
imu=struct('acc',[0;0;9.80665],'gyro',zeros(3,1));
p=uavmvp.forest_navigation('predict',n,imu,.01,c);
assert(norm(p.p)+norm(p.v)<1e-12 && p.t==.01 && p.P(1,1)>0 && p.version==n.version);
% Deliberately constructed current snapshot, not a flight or per-reference reset.
n.p=[0;0;17.7];n.P(1:3,1:3)=3.5^2*eye(3);
n=uavmvp.forest_navigation('clone',n,"history",c);before=n.P;
[antenna,~]=uavmvp.forest_navigation('antenna',n,c);
values=vecnorm(antenna-c.scene.known_reference_xyz_m',2,1)';
b=struct('role','reference','raw_ids',"ref"+string((1:5)'), ...
    'epoch_s',0,'reference_indices',(1:5)','values_m',values,'block_complete',true);
[posterior,info]=uavmvp.forest_navigation('reference',n,b,c);
assert(norm(info.residual_m)<1e-12 && posterior.version==1 && trace(posterior.P(1:3,1:3))<trace(before(1:3,1:3)));
assert(trace(posterior.P(38:40,38:40))<trace(before(38:40,38:40)) && norm(posterior.P(38:40,29))>0);
assert(rank(info.H(:,1:3))==3 && norm(info.H(:,26))>0);
shifted=b;shifted.values_m=shifted.values_m+[1;-1;-1;1;0];
[corrected,~]=uavmvp.forest_navigation('reference',n,shifted,c);
assert(norm(corrected.p-n.p)>1e-3 && norm(corrected.clones-n.clones,'fro')>1e-3);
must_fail(@()uavmvp.forest_navigation('reference',posterior,b,c),'forest:Duplicate');
bad=b;bad.role='unknown';must_fail(@()uavmvp.forest_navigation('reference',n,bad,c),'forest:Role');
pruned=uavmvp.forest_navigation('prune',posterior,strings(1,0));assert(isequal(pruned.P,posterior.P(1:37,1:37)));
% Correlated-noise update against a scalar Schur complement, including D.
nb=n;bb=b;bb.raw_ids="correlated"+string((1:5)');D=zeros(size(n.P,1),5);D(1,1)=.01;D(38,1)=.01;
bb.state_noise_cross=D;bb.noise_covariance_m2=eye(5);
[np,ii]=uavmvp.forest_navigation('reference',nb,bb,c);
H=ii.H;S=H*nb.P*H'+H*D+D'*H'+eye(5);B=nb.P*H'+D;
expected=nb.P-B*(S\B');assert(norm(np.P-expected,'fro')<1e-9);
assert(np.last_reference_s==np.t);
% Delayed paired result: two different antenna endpoints, not range at their mean.
nd=uavmvp.forest_navigation('init',samples,c);nd.p=[0;0;17.7];nd.P(1:3,1:3)=eye(3);
nd=uavmvp.forest_navigation('clone',nd,"early",c);nd.p(1)=.4;nd.t=.1;
nd=uavmvp.forest_navigation('clone',nd,"late",c);
dv=(vecnorm(nd.clones(:,1)-c.scene.known_reference_xyz_m',2,1)'+ ...
    vecnorm(nd.clones(:,2)-c.scene.known_reference_xyz_m',2,1)')/2;
db=b;db.epoch_s=.1;db.values_m=dv;db.direction_clone_ids=repmat(["early" "late"],5,1);
[~,di]=uavmvp.forest_navigation('reference',nd,db,c);assert(norm(di.residual_m)<1e-12);

% Five saved antenna means with a common translation covariance. No world exists.
fnav=uavmvp.forest_navigation('init',samples,c);fnav.version=2;
fnav.clones=[20 0 -20 0 20;0 20 0 -20 20;17.8 17.8 17.8 17.8 17.8];
fnav.clone_ids=["A" "B" "C" "D" "V"];fnav.clone_times=1:5;fnav.t=5;
fnav.P=blkdiag(fnav.P,9*kron(ones(5),eye(3))+.04*eye(15));
clones=repelem(fnav.clone_ids(1:4)',2);target=[5;5;-.2];
rf=repelem(vecnorm(fnav.clones(:,1:4)-target,2,1)',2)+3;
input=struct('navigation',fnav,'fit_values_m',rf,'fit_raw_ids',"F"+string((1:8)'), ...
    'fit_group_ids',repelem((1:4)',2),'fit_clone_ids',clones,'fifth_clone_ids',["V";"V"], ...
    'fifth_raw_ids',["V1";"V2"],'data_delivered',true,'execution_kind','fixed_input_review');
f=uavmvp.forest_localization('fit',input,c);assert(f.quality && f.complete_four_groups && f.spatial_valid);
deadlineInput=rmfield(input,'execution_kind');deadlineConfig=c;deadlineConfig.service.computation_bound_s=0;
must_fail(@()uavmvp.forest_localization('fit',deadlineInput,deadlineConfig),'forest:ComputationTimeout');
oldNav=fnav;oldWeights=f.weights;rv=repmat(norm(fnav.clones(:,5)-target)+3,2,1);
v=struct('snapshot',f,'values_m',rv,'raw_ids',["V1";"V2"],'version',2);
ok=uavmvp.forest_localization('verify',v,c);v.values_m=rv+[20;-20];badResult=uavmvp.forest_localization('verify',v,c);
assert(ok.passed && ~badResult.passed && abs(ok.mixed_cdf-badResult.mixed_cdf)<1e-12);
assert(isequal(f.weights,oldWeights) && isequal(fnav,oldNav));
v.version=3;must_fail(@()uavmvp.forest_localization('verify',v,c),'forest:VerificationPrerequisite');
repeat=input;repeat.fifth_raw_ids=["F1";"V2"];must_fail(@()uavmvp.forest_localization('fit',repeat,c),'forest:HeldoutID');
partial=input;keep=1:4;partial.fit_values_m=rf(keep);partial.fit_raw_ids=input.fit_raw_ids(keep);
partial.fit_clone_ids=clones(keep);partial.fit_group_ids=input.fit_group_ids(keep);
partial.fifth_clone_ids=strings(0,1);partial.fifth_raw_ids=strings(0,1);
coarse=uavmvp.forest_localization('fit',partial,c);assert(~coarse.quality && all(isfinite(coarse.cost)));
cond=uavmvp.forest_localization('condition',struct('joint_covariance',[4 2;2 3], ...
    'x_mean',0,'nuisance_mean',1,'x',2),c);assert(abs(cond.mean-2)<1e-12 && abs(cond.covariance-2)<1e-12);

s=uavmvp.forest_protocol('init',struct('unknown_ids',["U1";"U2"]),c);
s=event(s,struct('type','identity','id',"U1",'lease_until_s',20,'data_bytes',4),c);assert(s.nodes(1).contact && ~s.nodes(1).discovered);
for k=1:8
    ids=[input.fit_raw_ids;input.fifth_raw_ids];
    s=event(s,struct('type','range','id',"U1",'raw_id',ids(k),'valid',true,'range_m',30),c);
end
s=event(s,struct('type','reference_version','version',2),c);
assert(strcmp(uavmvp.forest_protocol('phase',s.nodes(1),c),'DATA'));
d=struct('type','data','id',"U1",'valid',true,'fragment_count',2,'fragment_index',1,'sequence',7,'bytes',2);
s=event(s,d,c);s=event(s,d,c);assert(s.nodes(1).delivered_bytes==2 && ~s.nodes(1).data_delivered);
d.fragment_index=2;s=event(s,d,c);assert(s.nodes(1).data_delivered);
s=event(s,struct('type','fit','id',"U1",'version',2,'complete_groups',4,'region_available',true,'quality',true),c);
s=event(s,struct('type','request_fifth','id',"U1",'fit_ids',input.fit_raw_ids),c);
for k=9:10
    ids=[input.fit_raw_ids;input.fifth_raw_ids];
    s=event(s,struct('type','range','id',"U1",'raw_id',ids(k),'valid',true,'range_m',30),c);
end
s=event(s,struct('type','reserve_verification','id',"U1",'fit_ids',input.fit_raw_ids,'fifth_ids',input.fifth_raw_ids,'snapshot',f),c);
s=event(s,struct('type','verification','id',"U1",'values_m',rv,'raw_ids',input.fifth_raw_ids),c);
assert(s.nodes(1).verified && ~s.nodes(1).stop_ack && ~s.closed);
must_fail(@()event(s,struct('type','verification','id',"U1",'values_m',rv,'raw_ids',input.fifth_raw_ids),c),'forest:VerificationOnce');
must_fail(@()event(s,struct('type','range','id',"U1",'raw_id',"new",'valid',true,'range_m',30),c),'forest:Stopped');
s=event(s,struct('type','notice_attempt','id',"U1"),c);
s=event(s,struct('type','complete_ack','id',"U1",'valid',true,'result_seq',1),c);assert(s.nodes(1).stop_ack && ~s.closed);
s=uavmvp.forest_protocol('ticket',struct('state',s,'key',"cell1/round1"),c);
s.no_progress=4;s=event(s,struct('type','reference_version','version',3),c);assert(s.no_progress==4);
must_fail(@()uavmvp.forest_protocol('ticket',struct('state',s,'key',"cell1/round1"),c),'forest:WakeTicket');
for k=1:8,s=event(s,struct('type','decision_result','coverage_advanced',false,'edge_advanced',false,'insertion_executed',false,'reference_recovery',false),c);end
assert(s.no_progress==7); % First decision accounts for genuine prior service progress.

graphInput=uavmvp.forest_policy('initial',[],c);assert(~isempty(graphInput.route_vertices) && size(graphInput.points_m,1)==170);
state=uavmvp.forest_protocol('init',struct('unknown_ids',"X"),c);
state.nodes(1).contact=true;state.nodes(1).discovered=true;
op=struct('id',"X",'phase','DATA','eligible',true,'payload_bytes',4);
action=struct('id',1,'target_m',[5 0 17.7],'path_m',[1 0 17.7;5 0 17.7], ...
    'move_and_settle_s',5.5,'shared_control_s',.1,'opportunities',op,'coverage_progress',false, ...
    'insertion',false,'adjacent_to_next_coverage',true,'reference',false,'reference_service_s',3, ...
    'corridor_valid',true,'model_valid',true,'prediction_valid',true,'navigation_ratio_upper',.9, ...
    'recovery_domain_valid',true,'geometry_gain',1,'fixed_order',1,'has_wake_ticket',false);
ii=struct('rule','B1','state',state,'now_s',0,'return_reservation_s',180, ...
    'next_reference_reservation_s',3,'reference_deadline_s',12,'actions',action, ...
    'coverage_protection',true,'reference_due',false,'wake_due',false);
choice=uavmvp.forest_policy('select',ii,c);assert(strcmp(choice.kind,'ACTION') && choice.next_waypoint_m(1)==1);
empty=ii;empty.actions=struct([]);choice=uavmvp.forest_policy('select',empty,c);assert(strcmp(choice.kind,'RETURN_OR_TERMINATE'));
ii.reference_deadline_s=8;choice=uavmvp.forest_policy('select',ii,c);assert(strcmp(choice.kind,'RETURN_OR_TERMINATE'));
ii.reference_deadline_s=12;ii.actions.navigation_ratio_upper=2;
choice=uavmvp.forest_policy('select',ii,c);assert(strcmp(choice.kind,'RETURN_OR_TERMINATE'));
ii.actions.reference=true;ii.actions.opportunities=struct([]);ii.reference_due=true;
choice=uavmvp.forest_policy('select',ii,c);assert(strcmp(choice.kind,'ACTION'));
% Same-capability B0/B1: baseline order versus actual service opportunities.
ii.actions=[action action];ii.actions(1).opportunities=struct([]);ii.actions(1).coverage_progress=true;
ii.actions(2).id=2;ii.actions(2).fixed_order=2;ii.rule='B0';b0=uavmvp.forest_policy('select',ii,c);
ii.rule='B1';b1=uavmvp.forest_policy('select',ii,c);assert(b0.selected.id==1 && b1.selected.id==2);

cmd=uavmvp.forest_motion('command',struct('navigation',n,'setpoint_m',n.p,'landing',false),c);
plant=struct('p',n.p,'v',zeros(3,1),'R',uavmvp.SO3.exp([.1;0;0]),'omega',zeros(3,1));
derivative=uavmvp.forest_motion('derivative',struct('plant',plant,'command',cmd),c);
assert(abs(derivative.v_dot(2))>.9); % Tilt must affect true thrust even if estimated hover is perfect.
land=uavmvp.forest_motion('landing',struct('navigation',posterior,'return_reservation_s',180, ...
    'next_reference_reservation_s',3,'reference_deadline_s',12),c);
assert(land.css_reference_required && ~land.physical_landing_confirmed);
ev=struct('estimated_terminal_xy_m',[0 0;25 0],'true_terminal_xy_m',[0 0;0 0], ...
    'final_true_position_m',[0;0;0],'true_home_m',[0;0;0],'physically_landed',false, ...
    'service_closed',true,'all_physically_asleep',true);
offline=uavmvp.forest_motion('evaluate',ev,c);assert(~offline.success && ~offline.actual_return && ~offline.all_terminal_accuracy);
ev.physically_landed=true;ev.estimated_terminal_xy_m=[0 0;19 0];offline=uavmvp.forest_motion('evaluate',ev,c);assert(offline.success);
out=struct('all_passed',true,'kind','forest_fixed_input_component_regression', ...
    'mission_count',0,'task_integrations',0,'radio_budget',timing,'grid_points',size(f.grid,1), ...
    'nominal_radius_m',f.nominal_radius_m,'nominal_fifth_passed',ok.passed, ...
    'opposite_residual_rejected',~badResult.passed,'reference_sigma_m',c.radio.reference_pair_white_std_m, ...
    'actual_flight_or_landing_validated',false);
end

function s=event(s,e,c)
s=uavmvp.forest_protocol('event',struct('state',s,'event',e),c);
end
function must_fail(f,id)
try
    f();
catch problem
    assert(strcmp(problem.identifier,id),'forest:WrongFailure','Expected %s, got %s',id,problem.identifier);
    return
end
error('forest:ExpectedFailure','Expected %s',id);
end
