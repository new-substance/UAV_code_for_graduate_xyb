function tests=test_radio_world
tests=functiontests(localfunctions);
end

function testCompositeCloneKeepsJointCovariance(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];
[nav,id1]=uavmvp.nav_clone(nav,cfg);nav.p=nav.p+[.2;0;0];nav.t=.2;
[nav,id2]=uavmvp.nav_clone(nav,cfg);[p,P]=uavmvp.nav_archive(nav,[id1,id2]);
J=[.5*eye(3),.5*eye(3)];
[nav,id,raw]=uavmvp.nav_combine_clones(nav,[id1,id2],[.5,.5]);
[actual,C]=uavmvp.nav_archive(nav,id);
verifyEqual(t,actual,mean(p,2),'AbsTol',1e-12);verifyEqual(t,C,J*P*J','AbsTol',1e-12);
verifyEqual(t,raw.epoch_s,.1,'AbsTol',1e-12);
nav=uavmvp.nav_prune(nav,id);[~,after]=uavmvp.nav_archive(nav,id);verifyEqual(t,after,C,'AbsTol',1e-12);
end
function testSharedTimingErrorDoesNotBecomeIndependentPerPosition(t)
cfg=uavmvp.config();cfg.navigation_candidate.timestamp_alignment_std_s=.1;
nav=uavmvp.nav_init(cfg);nav.P(:)=0;nav.v=[1;0;0];
[nav,first]=uavmvp.nav_clone(nav,cfg);nav.v=[-2;1;0];nav.t=.2;
[nav,second]=uavmvp.nav_clone(nav,cfg);[~,P]=uavmvp.nav_archive(nav,[first,second]);
v=[1;0;0;-2;1;0];verifyEqual(t,P,.01*(v*v'),'AbsTol',1e-15);
[nav,id]=uavmvp.nav_combine_clones(nav,[first,second],[.25,.75]);
[~,C]=uavmvp.nav_archive(nav,id);J=[.25*eye(3),.75*eye(3)];
verifyEqual(t,C,J*P*J','AbsTol',1e-15);
nav=uavmvp.nav_prune(nav,id);[~,again]=uavmvp.nav_archive(nav,id);verifyEqual(t,again,C,'AbsTol',0);
end

function testPhysicalColdThenWarmTimelineAndRange(t)
cfg=ideal();w=uavmvp.MotionWorld(cfg,8101);w.configure_radio(1,31,[20;10;0]);
uavmvp.fly_to(w,[10;10;15],cfg,40);w.hold(1);
book=uavmvp.ProtocolBook(cfg,1,31);control=book.group_control(1,w.t);start=w.t;
packets=w.transmit_group(control);verifyEqual(t,numel(packets),1);
verifyEqual(t,w.t-start,control.reserved_s,'AbsTol',1e-8);
book.receive_identity(packets{1},packets{1}.received_time_s);
h=uavmvp.radio_history_init(cfg);start=w.t;
[h,report,first]=uavmvp.execute_range_service(w,book,h,1,cfg);
verifyTrue(t,report.accepted,report.reason);verifyTrue(t,report.first_discovery);
timing=uavmvp.radio_timing(cfg);
verifyGreaterThanOrEqual(t,w.t-start,timing.cold_success_s-1e-9);
verifyLessThan(t,w.t-start,timing.cold_success_s+1e-4);
verifyEqual(t,first.mode,'cold');verifyEqual(t,numel(w.nav.clone_ids),1);
[p,~]=uavmvp.nav_archive(w.nav,first.effective_nav_snapshot_id);
verifyLessThan(t,abs(first.range_m-norm(p-[20;10;0])),.002);
start=w.t;[~,second_report,second]=uavmvp.execute_range_service(w,book,h,1,cfg);
verifyTrue(t,second_report.accepted);verifyFalse(t,second_report.first_discovery);
verifyEqual(t,second.mode,'warm');verifyLessThan(t,w.t-start,timing.exchange_s+1e-4);
verifyEqual(t,numel(book.nodes(1).range_records),2);
data=w.finish();verifyEqual(t,size(data.radio_truth.terminal_positions),[3,1]);
verifyEqual(t,numel(data.radio_events),3);
verifyError(t,@()w.transmit_group(control),'MATLAB:assertion:failed');
end

function testLostIdentityDoesNotPutDeviceToSleep(t)
cfg=ideal();cfg.ranging.baseline_stage_probabilities.identity_decode_given_control=0;
w=uavmvp.MotionWorld(cfg,8102);w.configure_radio(1,31,[20;10;0]);
book=uavmvp.ProtocolBook(cfg,1,31);control=book.group_control(1,w.t);
packets=w.transmit_group(control);verifyEmpty(t,packets);verifyFalse(t,book.nodes(1).identity_seen);
% Physical probe bypasses only the online ledger for this fault-injection test.
% It shows the device really remained awake despite losing its identity reply.
[id,~]=w.clone_antenna();request=struct('address',1,'mission_id',31, ...
    'measurement_epoch_s',w.t,'nav_snapshot_id',id,'exchange_count',2,'mode','cold');
raw=w.measure_service(request);verifyNotEmpty(t,raw);
verifyFalse(t,book.nodes(1).discovered);w.finish();
end

function testOutOfRangeAndTimeoutConsumeTimeWithoutObservation(t)
cfg=ideal();cfg.ranging.baseline_stage_probabilities.range_exchange_valid=0;
w=uavmvp.MotionWorld(cfg,8103);w.configure_radio(2,31,[20,110;10,70;0,0]);
book=uavmvp.ProtocolBook(cfg,2,31);control=book.group_control([1,2],w.t);
packets=w.transmit_group(control);verifyEqual(t,numel(packets),1);
book.receive_identity(packets{1},packets{1}.received_time_s);
start=w.t;[~,report,record]=uavmvp.execute_range_service(w,book,uavmvp.radio_history_init(cfg),1,cfg);
verifyFalse(t,report.accepted);verifyEmpty(t,record);verifyFalse(t,book.nodes(1).discovered);
verifyEqual(t,w.t-start,cfg.css.exchange_deadline_s,'AbsTol',1e-8);
verifyEmpty(t,w.nav.clone_ids);data=w.finish();
verifyEqual(t,data.radio_events{end}.reason,'NO_RANGE_RESPONSE');
end

function testStopDeliveryAndLostAckAreSeparatePhysicalEvents(t)
cfg=ideal();cfg.ranging.baseline_stage_probabilities.identity_decode_given_control=0;
w=uavmvp.MotionWorld(cfg,8104);w.configure_radio(1,31,[20;10;0]);
book=uavmvp.ProtocolBook(cfg,1,31);control=book.group_control(1,w.t);w.transmit_group(control);
command=struct('mission_id',31,'address',1,'kind','STOP','session_id',100,'sequence',1, ...
    'crc_valid',true,'completion_token',99,'reserved_s',.3);
start=w.t;packet=w.transmit_stop(command);verifyEmpty(t,packet);
verifyEqual(t,w.t-start,.3,'AbsTol',1e-8);
[id,~]=w.clone_antenna();request=struct('address',1,'mission_id',31, ...
    'measurement_epoch_s',w.t,'nav_snapshot_id',id,'exchange_count',2,'mode','cold');
verifyEmpty(t,w.measure_service(request));data=w.finish();
verifyTrue(t,data.radio_events{2}.physically_received);verifyFalse(t,data.radio_events{2}.ack_received);
end

function cfg=ideal()
cfg=uavmvp.config();n=cfg.navigation_candidate;
names={'initial_position_std_m','initial_velocity_std_mps','initial_roll_pitch_std_deg', ...
    'initial_yaw_std_deg','initial_acc_bias_std_mps2','initial_gyro_bias_std_radps', ...
    'survey_translation_std_m','survey_rotation_std_deg','acc_noise_density_mps2_per_sqrt_hz', ...
    'gyro_noise_density_radps_per_sqrt_hz','acc_bias_random_walk_mps2_per_sqrt_s', ...
    'gyro_bias_random_walk_radps_per_sqrt_s'};
for k=1:numel(names),n.(names{k})=0;end
cfg.navigation_candidate=n;
cfg.ranging.range_white_std_m=0;cfg.ranging.device_persistent_bias_std_m=0;
cfg.ranging.initial_clock_ratio_std_ppm=0;cfg.ranging.clock_ratio_random_walk_ppm_per_sqrt_s=0;
cfg.ranging.baseline_stage_probabilities=struct('wake_decode',1,'identity_decode_given_control',1,'range_exchange_valid',1);
end
