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
forecast=forecast_checks(n,c);
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

graphInput=uavmvp.forest_policy('initial',[],c);assert(~isempty(graphInput.route_vertices) && size(graphInput.points_m,1)==173);
route=route_checks(n,c,graphInput);
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
service=service_checks(c,samples,fnav,input,rf,rv);
adapter=event_adapter_checks(c,samples,raw);
out=struct('route',route,'service',service,'event_adapter',adapter,'forecast',forecast,'all_passed',true,'kind','forest_fixed_input_component_regression', ...
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

function out=event_adapter_checks(c,samples,raw)
i=struct('samples',samples,'unknown_ids',7,'reference_ids',(100:104)','uav_id',1,'mission',3,'session',4);
s=uavmvp.forest_events('init',i,c);
h=struct('version',1,'type',2,'mission',3,'session',4,'seq',1,'source',7,'target',1,'flags',0);
m=struct('capability',1,'battery_mV',3000,'data_seq',2,'data_bytes',4,'sample_epoch_local',0);
e=struct('type','frame','time_s',0,'crc_valid',true,'lease_until_s',1, ...
    'bytes',uavmvp.forest_frame('encode',struct('header',h,'message',m)));
s=apply(s,e,c);assert(s.protocol.nodes.contact && ~s.protocol.nodes.discovered);
bad=e;bad.time_s=.01;must_fail(@()apply(s,bad,c),'forest:EventTime');
h.session=9;bad=e;bad.bytes=uavmvp.forest_frame('encode',struct('header',h,'message',m));
must_fail(@()apply(s,bad,c),'forest:Session');
s=apply(s,struct('type','capture','time_s',0,'clone_id',"a"),c);
s=apply(s,struct('type','capture','time_s',0,'clone_id',"b"),c);
p=struct('type','pair_request','time_s',0,'pair_id',"P1",'peer',7,'role','unknown', ...
    'transaction',9,'group',1,'job_index',1,'calibration_ids',[7 8],'offsets_m',[.3 .3]);
s=apply(s,struct('type','unknown_group','time_s',0,'peer',7,'block_id',"G1",'group',1),c);
s=apply(s,p,c);must_fail(@()apply(s,p,c),'forest:RadioBusy');before=s.navigation;
profile=struct('peer',7,'calibration_ids',[7 8],'offsets_m',[.3 .3]);
next=uavmvp.forest_events('next_pair',struct('state',s,'calibration_profiles',profile),c);
assert(strcmp(next.kind,'WAIT_FOR_RESULT_OR_TIMEOUT'));
d=struct('type','direction_result','time_s',0,'pair_id',"P1",'peer',7,'transaction',9, ...
    'direction',1,'clone_id',"a",'raw',raw,'calibration_id',7,'mission',3,'session',4);
s=apply(s,d,c);assert(~s.protocol.nodes.discovered);
d.direction=2;d.clone_id="b";d.calibration_id=8;s=apply(s,d,c);
assert(s.protocol.nodes.discovered && numel(s.protocol.nodes.raw_ids)==1 && isequal(s.navigation,before));
must_fail(@()apply(s,struct('type','release','time_s',0,'keep_ids',strings(0,1)),c),'forest:HistoryRetention');
[s,receipt]=uavmvp.forest_events('apply',struct('state',s,'event',d),c);assert(receipt.duplicate);
h.session=4;h.seq=2;h.type=3;
m=struct('data_seq',2,'fragment_index',0,'fragment_count',1,'sample_epoch_local',0,'sensor_flags',1,'samples',uint8([1 2 3 4]));
e.bytes=uavmvp.forest_frame('encode',struct('header',h,'message',m));s=apply(s,e,c);
assert(s.protocol.nodes.data_delivered && s.protocol.nodes.delivered_bytes==4);
[s,receipt]=uavmvp.forest_events('apply',struct('state',s,'event',e),c);
assert(receipt.duplicate && s.protocol.nodes.delivered_bytes==4);
retry=p;retry.pair_id="not_a_new_job";retry.transaction=10;
must_fail(@()apply(s,retry,c),'forest:Retry');
next=uavmvp.forest_events('next_pair',struct('state',s,'calibration_profiles',profile),c);
assert(strcmp(next.kind,'REQUEST') && next.event.peer==7 && next.event.job_index==2);
[ended,r]=uavmvp.forest_events('apply',struct('state',s,'event',struct('type','unknown_group_end','time_s',0)),c);
assert(~r.accepted && isempty(ended.unknown_block));
resumed=apply(ended,struct('type','unknown_group','time_s',0,'peer',7,'block_id',"G1_again",'group',1),c);
assert(resumed.unknown_block.ledger.complete(1) && resumed.unknown_block.ledger.attempts(1)==1);
must_fail(@()apply(resumed,retry,c),'forest:Retry');
% Fixed saved counter boundary: radio budget must not cap the 100 Hz IMU at 40.96 s.
boundary=ended;boundary.events=c.algorithm.max_protocol_records;
imu=struct('type','imu','time_s',.01,'measurement',struct('acc',[0;0;9.80665],'gyro',zeros(3,1)));
boundary=apply(boundary,imu,c);assert(boundary.imu_events==1 && boundary.events==c.algorithm.max_protocol_records);
bad=e;bad.crc_valid=false;
[rejected,r]=uavmvp.forest_events('ingest',struct('state',ended,'event',bad),c);
assert(~r.accepted && ~r.fatal && rejected.events==ended.events+1 && rejected.rejected_events==1 && isequal(rejected.navigation,ended.navigation));
[~,r]=uavmvp.forest_events('ingest',struct('state',boundary,'event',bad),c);assert(r.fatal);
ref=uavmvp.forest_events('init',i,c);
ref=apply(ref,struct('type','reference_block','time_s',0,'block_id',"R1"),c);
rp=struct('peer',100,'calibration_ids',[7 8],'offsets_m',[.3 .3]);
request=uavmvp.forest_events('next_pair',struct('state',ref,'calibration_profiles',rp),c);
assert(request.event.peer==100 && request.event.job_index==1);
ref=apply(ref,request.event,c);assert(ref.reference_block.ledger.attempts(1)==1);
[ref,r]=uavmvp.forest_events('apply',struct('state',ref,'event',struct('type','reference_block_end','time_s',0)),c);
assert(~r.accepted && isinf(ref.navigation.last_reference_s) && ref.pairs{1}.finished);
out=struct('all_passed',true,'world_steps',0,'unknown_range_does_not_change_navigation',true, ...
    'wire_fragment_zero_maps_to_business_fragment_one',true,'stale_session_and_future_receipt_rejected',true, ...
    'completed_job_not_reused',true,'partial_group_not_complete',true,'separate_imu_budget',true,'rejected_event_consumes_budget',true);
end
function s=apply(s,e,c)
s=uavmvp.forest_events('apply',struct('state',s,'event',e),c);
end

function out=forecast_checks(n,c)
% Pure matrix examples, not a nominal or actual trajectory rollout.
F=zeros(15);W=zeros(15);P=eye(15);P(3,6)=.9;P(6,3)=.9;F(3,6)=1;F(6,3)=-1;
b=uavmvp.forest_forecast('linear_window',struct('P',P,'F',F,'W',W,'duration_s',pi/2),c);
mid=expm(F*pi/4)*P*expm(F'*pi/4);
assert(abs(b.P_end(3,3)-1)<1e-12 && abs(mid(3,3)-1.9)<1e-12 && b.std_upper(2)>=sqrt(1.9));
zero=uavmvp.forest_forecast('linear_window',struct('P',P,'F',F,'W',W,'duration_s',0),c);
assert(norm(zero.P_end-P,'fro')<1e-12 && abs(zero.std_upper(2)-1)<1e-12);
a=struct('R',eye(3),'specific_force_body_mps2',[0;0;9.80665],'body_rate_radps',zeros(3,1),'duration_s',.01);
i=struct('navigation',n,'profile',a,'execution_kind','fixed_input_review');
old=n;f=uavmvp.forest_forecast('profile',i,c);
imu=struct('acc',[0;0;9.80665]+n.ba,'gyro',n.bg);
next=uavmvp.forest_navigation('predict',n,imu,.01,c);
assert(norm(f.predicted_covariance-next.P,'fro')<1e-11 && isequal(n,old) && f.future_reference_updates==0);
assert(f.horizon_s==.01 && isfinite(f.navigation_ratio_upper));
bad=i;bad.profile.body_rate_radps=[2;0;0];must_fail(@()uavmvp.forest_forecast('profile',bad,c),'forest:ForecastDomain');
expired=rmfield(i,'execution_kind');timed=c;timed.service.computation_bound_s=0;
must_fail(@()uavmvp.forest_forecast('profile',expired,timed),'forest:ComputationTimeout');
out=struct('all_passed',true,'world_steps',0,'trajectory_rollouts',0, ...
    'intersample_peak_variance',mid(3,3),'endpoint_variance',b.P_end(3,3), ...
    'interval_upper_std',b.std_upper(2),'one_step_matches_N01',true,'future_reference_updates',0);
end

function out=service_checks(c,samples,fnav,input,rf,rv)
i=struct('samples',samples,'unknown_ids',7,'reference_ids',(100:104)','uav_id',1,'mission',3,'session',4);
s=uavmvp.forest_events('init',i,c);s.navigation=fnav;
s.protocol=event(s.protocol,struct('type','reference_version','version',2),c);
s.protocol=event(s.protocol,struct('type','identity','id',"7",'lease_until_s',20,'data_bytes',4),c);
for j=1:8
 p=struct('id',input.fit_raw_ids(j),'peer',7,'role','unknown','status','received','group',ceil(j/2), ...
     'range_m',rf(j),'clone_ids',[input.fit_clone_ids(j) input.fit_clone_ids(j)]);
 s.pairs{end+1}=p;s.protocol=event(s.protocol,struct('type','range','id',"7",'raw_id',p.id,'valid',true,'range_m',p.range_m),c);
end
nav=s.navigation;request=struct('state',s,'peer',7,'execution_kind','fixed_input_review');
[s,r]=uavmvp.forest_service('refresh',request,c);assert(r.accepted && r.quality && isequal(nav,s.navigation));
s.protocol=event(s.protocol,struct('type','data','id',"7",'valid',true,'sequence',1,'fragment_index',1,'fragment_count',1,'bytes',4),c);
b=fnav;b.p=[20;0;17.7];q=struct('state',s,'peer',7,'belief',b);
[~,o]=uavmvp.forest_service('opportunity',q,c);assert(strcmp(o.phase,'VERIFY') && ~o.eligible);
b.p=[20;20;17.7];q.belief=b;[~,o]=uavmvp.forest_service('opportunity',q,c);assert(o.eligible);
[s,~]=uavmvp.forest_service('request_fifth',struct('state',s,'peer',7),c);
for j=1:2
 p=struct('id',input.fifth_raw_ids(j),'peer',7,'role','unknown','status','received','group',5, ...
     'range_m',rv(j),'clone_ids',["V" "V"]);
 s.pairs{end+1}=p;s.protocol=event(s.protocol,struct('type','range','id',"7",'raw_id',p.id,'valid',true,'range_m',p.range_m),c);
end
[s,r]=uavmvp.forest_service('verify_group',struct('state',s,'peer',7,'group',5,'execution_kind','fixed_input_review'),c);
assert(r.accepted && s.protocol.nodes.verified && isequal(nav,s.navigation));
must_fail(@()uavmvp.forest_service('request_fifth',struct('state',s,'peer',7),c),'forest:Stopped');
must_fail(@()uavmvp.forest_service('reselect',struct('state',s,'peer',7),c),'forest:Stopped');
% Rebuild a one-pair saved book: same-site service must request only its missing pair.
p=s.pairs{1};partial=uavmvp.forest_events('init',i,c);partial.navigation=fnav;partial.pairs={p};
partial.protocol=event(partial.protocol,struct('type','reference_version','version',2),c);
partial.protocol=event(partial.protocol,struct('type','identity','id',"7",'lease_until_s',20,'data_bytes',4),c);
partial.protocol=event(partial.protocol,struct('type','range','id',"7",'raw_id',p.id,'valid',true,'range_m',p.range_m),c);
partial.protocol=event(partial.protocol,struct('type','data','id',"7",'valid',true,'sequence',1,'fragment_index',1,'fragment_count',1,'bytes',4),c);
[partial,r]=uavmvp.forest_service('refresh',struct('state',partial,'peer',7,'execution_kind','fixed_input_review'),c);
assert(r.accepted && ~r.quality);
b.p=[20;0;17.7];[~,o]=uavmvp.forest_service('opportunity',struct('state',partial,'peer',7,'belief',b),c);
assert(o.eligible && o.missing_pairs==1 && o.partial_group==1);
p.id=input.fit_raw_ids(2);p.range_m=rf(2);partial.pairs{end+1}=p;
partial.protocol=event(partial.protocol,struct('type','range','id',"7",'raw_id',p.id,'valid',true,'range_m',p.range_m),c);
[partial,r]=uavmvp.forest_service('refresh',struct('state',partial,'peer',7,'execution_kind','fixed_input_review'),c);
assert(r.accepted && r.complete_groups==1 && numel(partial.fits{1}.fit_raw_ids)==2);
out=struct('all_passed',true,'world_steps',0,'fit_and_verification_bound_to_received_pair_book',true, ...
    'old_fit_point_not_new_fifth',true,'partial_group_requests_missing_pair_only',true,'navigation_unchanged',true);
end

function out=route_checks(n,c,map)
% No make_profile call: geometry and one saved-input nominal step only.
n.p=[0;0;17.7];n.v=zeros(3,1);n.R=eye(3);original=n;
a=struct('navigation',n,'setpoint_m',[5;0;17.7],'landing',false,'duration_s',.01,'body_rate_radps',zeros(3,1));
r=uavmvp.forest_motion('nominal_step',a,c);
command=r.command;expected=command.body_rate_rad_per_s*(1-exp(-.01/c.motion_model.body_rate_time_constant_s));
assert(norm(r.body_rate_radps-expected)<1e-12 && norm(r.navigation.R-eye(3),'fro')>0 && isequal(n,original));
assert(norm(r.profile.specific_force_body_mps2(1:2))==0);
assert(r.navigation.v(1)<.001); % Body attitude lag, no ideal world-acceleration injection.
imu=struct('acc',r.profile.specific_force_body_mps2+n.ba,'gyro',r.profile.body_rate_radps+n.bg);
[next,~,~]=uavmvp.nominal_step(n,imu,.01,[0;0;-9.80665]);
assert(norm(next.p-r.navigation.p)+norm(next.v-r.navigation.v)+norm(next.R-r.navigation.R,'fro')<1e-12);
[~,target]=min(vecnorm(map.points_m-[5 0 17.7],2,2));
p=uavmvp.forest_route('plan',struct('navigation',n,'map',map,'target_vertex',target),c);
assert(abs(p.move_and_settle_s-5.5)<1e-12);
n.p=[0;0;0];[distance,target]=min(vecnorm(map.points_m-[0 0 5],2,2));assert(distance<1e-12);
p=uavmvp.forest_route('plan',struct('navigation',n,'map',map,'target_vertex',target),c);
assert(abs(p.move_and_settle_s-7)<1e-12 && max(map.graph.Edges.Weight)<=5+1e-10);
expired=struct('navigation',n,'plan',p,'hold_s',.1,'body_rate_radps',zeros(3,1));expired.navigation.last_reference_s=-Inf;
must_fail(@()uavmvp.forest_route('profile',expired,c),'forest:ForecastBudget'); % Reject before any rollout.
n.p=[0;0;5];must_fail(@()uavmvp.forest_route('plan',struct('navigation',n,'map',map,'target_vertex',map.home_vertex),c),'forest:LandingRequired');
n.p=[0;0;0];
n.v=[.2;0;0];must_fail(@()uavmvp.forest_route('plan',struct('navigation',n,'map',map,'target_vertex',target),c),'forest:InitialMotion');
out=struct('all_passed',true,'world_steps',0,'trajectory_rollouts',0, ...
    'nominal_one_step_matches_N01_mean',true,'horizontal_five_m_budget_s',5.5, ...
    'vertical_five_m_budget_s',7,'ascent_split_into_reference_service_segments',true);
end
