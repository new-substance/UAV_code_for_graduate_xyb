function tests=test_stage_foundations
tests=functiontests(localfunctions);
end
function setupOnce(~),setup_stage();end
function testDiagonalInitialDrawIsExactLegacy(t)
P=diag([.2,.1,.03].^2);
a=RandStream('mt19937ar','Seed',31);b=RandStream('mt19937ar','Seed',31);
verifyEqual(t,uavmvp.initial_error(P,a),sqrt(diag(P)).*randn(b,3,1));
end
function testJointPriorSamplesHaveCrossCovariance(t)
P=[2,1.2;1.2,3];r=RandStream('mt19937ar','Seed',51);X=zeros(2,12000);
for k=1:size(X,2),X(:,k)=uavmvp.initial_error(P,r);end
verifyLessThan(t,norm(cov(X')-P,'fro')/norm(P,'fro'),.04);
end
function testCommonSurveyTranslationIsNotRemovedByRelativePixels(t)
cfg=uavstage.config('common_survey');nav=uavmvp.nav_init(cfg);nav.p(3)=15;
[pixels,H]=uavmvp.camera_model(nav,1,cfg);
B=zeros(size(nav.P,1),2);B(1:2,:)=eye(2);
for j=1:size(nav.map_p,2),B(15+6*(j-1)+(1:2),:)=eye(2);end
verifyLessThan(t,norm(H*B,'fro'),1e-10);
obs=struct('marker_id',1,'detected',true,'pixels',pixels);
for k=1:10,[nav,r]=uavmvp.nav_update_reference(nav,obs,cfg);verifyTrue(t,r.accepted);end
verifyGreaterThanOrEqual(t,min(eig(nav.P(1:2,1:2))),9-1e-7);
end
function testDeepSleepDoesNotAcknowledgeDuplicates(t)
cfg=uavstage.config();book=uavmvp.ProtocolBook(cfg,1,17);
device=uavstage.TerminalDevice(1,17);control=book.group_control(1,0);
verifyNotEmpty(t,device.receive_group(control));
command=struct('mission_id',17,'address',1,'kind','STOP','crc_valid',true,'completion_token',1);
verifyNotEmpty(t,device.receive_stop(command));verifyEmpty(t,device.receive_stop(command));
verifyEmpty(t,device.receive_group(control));
verifyFalse(t,device.can_respond_range(struct('mission_id',17,'address',1)));
s=device.offline_state();verifyTrue(t,s.deep_sleep);verifyFalse(t,s.receiver_enabled);verifyFalse(t,s.environment_sampling);
end
function testReservationIncludesLongServiceAndCompleteColdRetries(t)
cfg=uavstage.config('long_service');r=uavstage.reservation(cfg);timing=uavmvp.radio_timing(cfg,1);
verifyGreaterThan(t,r.total_s,20);verifyEqual(t,r.extra_service_s,20);
verifyGreaterThanOrEqual(t,r.total_s,20+2*timing.cold_max_s+cfg.stage.compute_reserve_s);
verifyFalse(t,isfield(cfg.motion_and_budget,'stop_max_dwell_s'));
end
function testExactlyFourRequired(t)
cfg=uavstage.config();b=struct('anchors',zeros(3,3),'ranges',ones(3,1),'Pnav',eye(9),'Cradio',eye(3));
verifyError(t,@()uavstage.fit_position(b,cfg),'MATLAB:assertion:failed');
end
function testNoObservationCannotUpdateNavigation(t)
cfg=uavstage.config();nav=uavmvp.nav_init(cfg);prior=nav;
[nav,r]=uavmvp.nav_update_reference(nav,[],cfg);
verifyFalse(t,r.accepted);verifyEqual(t,nav,prior);
end
function testDefaultConfigHasNoNewOptionalBehavior(t)
cfg=uavmvp.config();verifyFalse(t,isfield(cfg.sim,'terminal_device_class'));
verifyFalse(t,isfield(cfg.navigation_candidate,'survey_common_translation_std_m'));
end
function testLongTransactionIsNotRejectedJustForElapsedAge(t)
cfg=uavstage.config();n=cfg.navigation_candidate;
names={'acc_noise_density_mps2_per_sqrt_hz','gyro_noise_density_radps_per_sqrt_hz', ...
    'acc_bias_random_walk_mps2_per_sqrt_s','gyro_bias_random_walk_radps_per_sqrt_s'};
for k=1:numel(names),n.(names{k})=0;end
cfg.navigation_candidate=n;nav=uavmvp.nav_init(cfg);nav.p=[40;20;15];
nav.P(:)=0;nav.P(1:3,1:3)=.01*eye(3);nav.last_reference_time_s=nav.t;
g=uavstage.navigation_gate(nav,40,cfg);
verifyTrue(t,g.accepted);verifyGreaterThan(t,g.predicted_reference_age_s,25);
end
function testDeclaredCandidatesHaveUncertaintyMarginBeyondFiveMetres(t)
cfg=uavstage.config();q=cfg.stage.route_xy_m;minimum=Inf;
for i=1:size(q,1)
    for j=1:i-1,minimum=min(minimum,norm(q(i,:)-q(j,:)));end
end
verifyGreaterThanOrEqual(t,minimum,7.5-1e-12);
end
function testPhysicalColdThenWarmKeepsOneSpaceCount(t)
cfg=uavstage.config();n=cfg.navigation_candidate;
names={'initial_position_std_m','initial_velocity_std_mps','initial_roll_pitch_std_deg','initial_yaw_std_deg', ...
    'initial_acc_bias_std_mps2','initial_gyro_bias_std_radps','survey_translation_std_m','survey_rotation_std_deg', ...
    'survey_common_translation_std_m','acc_noise_density_mps2_per_sqrt_hz','gyro_noise_density_radps_per_sqrt_hz', ...
    'acc_bias_random_walk_mps2_per_sqrt_s','gyro_bias_random_walk_radps_per_sqrt_s','timestamp_alignment_std_s'};
for k=1:numel(names),n.(names{k})=0;end
cfg.navigation_candidate=n;cfg.ranging.range_white_std_m=1;cfg.ranging.device_persistent_bias_std_m=1;
cfg.ranging.baseline_stage_probabilities=struct('wake_decode',1,'identity_decode_given_control',1,'range_exchange_valid',1);
w=uavmvp.MotionWorld(cfg,79001);w.configure_radio(1,17,[15;15;0]);
uavmvp.fly_to(w,[10;10;15],cfg,40);w.hold(1);
book=uavmvp.ProtocolBook(cfg,1,17);control=book.group_control(1,w.t);packets=w.transmit_group(control);
book.receive_identity(packets{1},packets{1}.received_time_s);h=uavmvp.radio_history_init(cfg);
[h,r1,~]=uavmvp.execute_range_service(w,book,h,1,cfg);
[h,r2,~]=uavmvp.execute_range_service(w,book,h,1,cfg);
verifyTrue(t,r1.accepted && r2.accepted);verifyEqual(t,h.kinds,{'cold','warm'});
b=uavstage.bundle(book.nodes(1).range_records,w.nav,h,[1,2],cfg);
verifyFalse(t,uavmvp.distinct_positions(b.anchors,b.Pnav,cfg));
raw=w.finish();services=raw.radio_events(cellfun(@(e)strcmp(e.kind,'RANGE_SERVICE'),raw.radio_events));
verifyEqual(t,services{1}.request.exchange_count,2);verifyEqual(t,services{2}.request.exchange_count,1);
end
