function tests=test_motion_world
tests=functiontests(localfunctions);
end
function testBrownianSegmentationAndQueryOrder(t)
a=uavmvp.NoisePath(2,.01,3,9001);b=uavmvp.NoisePath(2,.01,3,9001);
x=a.delta(.123,.279);y=a.delta(.279,.697);z=a.delta(.123,.697);
verifyEqual(t,x+y,z,'AbsTol',1e-14);
other=b.delta(.279,.697);first=b.delta(.123,.279);
verifyEqual(t,other,y,'AbsTol',0);verifyEqual(t,first,x,'AbsTol',0);
end
function testBrownianIncrementVariance(t)
samples=2000;values=zeros(samples,2);
for k=1:samples
    p=uavmvp.NoisePath(1,.1,1,9100+k);
    values(k,:)=[p.delta(.125,.275),p.delta(.275,.525)];
end
C=cov(values);expected=diag([.15,.25]);
verifyLessThan(t,max(abs((C-expected)./sqrt(diag(expected)*diag(expected)')),[],'all'),.09);
end
function testZeroNoisePhysicalAndImuMotion(t)
cfg=zeroNoise();w=uavmvp.MotionWorld(cfg,9201);p0=w.nav.p;
w.advance(2,[.2;-.1;.05]);data=w.finish();
verifyLessThan(t,norm(data.true_position-p0-[.4;-.2;.1]),1e-10);
verifyLessThan(t,data.navigation_error_m,1e-9);
verifyError(t,@()w.advance(.1,zeros(3,1)),'MATLAB:assertion:failed');
end
function testEstimatedFeedbackActualTakeoffAndReference(t)
cfg=zeroNoise();w=uavmvp.MotionWorld(cfg,9202);
target=[10;10;15];status=uavmvp.fly_to(w,target,cfg,35);
verifyTrue(t,status.arrived);verifyGreaterThan(t,w.t,15);
before=w.t;[obs,r]=w.observe_reference(1);
verifyNotEmpty(t,obs);verifyTrue(t,r.accepted);
verifyEqual(t,w.t-before,cfg.navigation_candidate.reference_observation_s,'AbsTol',1e-10);
data=w.finish();verifyLessThan(t,norm(data.true_position-target),.4);
end
function testInvisibleReferenceDoesNotUpdate(t)
cfg=zeroNoise();w=uavmvp.MotionWorld(cfg,9203);
before=w.nav.version;[obs,r]=w.observe_reference(4);
verifyEmpty(t,obs);verifyFalse(t,r.accepted);verifyEqual(t,w.nav.version,before);
verifyGreaterThan(t,w.t,0);w.finish();
end
function testPlantSpeedLimit(t)
cfg=zeroNoise();w=uavmvp.MotionWorld(cfg,9204);
w.advance(10,[10;10;10]);data=w.finish();
verifyLessThanOrEqual(t,norm(data.trajectory(end,8:9)),cfg.motion_and_budget.horizontal_max_speed_mps+1e-12);
verifyLessThanOrEqual(t,abs(data.trajectory(end,10)),cfg.motion_and_budget.vertical_speed_mps+1e-12);
verifyLessThan(t,data.navigation_error_m,1e-8);
end
function testGroundContactIsPhysicalAndDoesNotTeleportNavigation(t)
cfg=zeroNoise();w=uavmvp.MotionWorld(cfg,9205);home=w.nav.p;
w.advance(2,[0;0;.5]);w.advance(6,[0;0;-.5]);
data=w.finish();verifyTrue(t,data.physically_landed);
verifyGreaterThan(t,size(data.ground_contact_events,1),0);
verifyGreaterThanOrEqual(t,min(data.trajectory(:,4)),home(3)-1e-12);
verifyEqual(t,data.true_position(3),home(3),'AbsTol',1e-12);
verifyLessThan(t,data.navigation_error_m,.02);
end
function testDescentWithoutReferenceCannotClaimCompletion(t)
cfg=zeroNoise();w=uavmvp.MotionWorld(cfg,9206);
status=uavmvp.land_at_home(w,cfg,50);
verifyFalse(t,status.command_completed);verifyEqual(t,status.reason,'LANDING_FINAL_REFERENCE_MISSING');
data=w.finish();verifyFalse(t,data.physically_landed);
end
function cfg=zeroNoise()
cfg=uavmvp.config();n=cfg.navigation_candidate;
names={'initial_position_std_m','initial_velocity_std_mps','initial_roll_pitch_std_deg', ...
    'initial_yaw_std_deg','initial_acc_bias_std_mps2','initial_gyro_bias_std_radps', ...
    'survey_translation_std_m','survey_rotation_std_deg','acc_noise_density_mps2_per_sqrt_hz', ...
    'gyro_noise_density_radps_per_sqrt_hz','acc_bias_random_walk_mps2_per_sqrt_s', ...
    'gyro_bias_random_walk_radps_per_sqrt_s'};
for k=1:numel(names),n.(names{k})=0;end
n.marker_detect_probability_given_visibility=1;
cfg.navigation_candidate=n;
end
