function tests=test_stress_profile
tests=functiontests(localfunctions);
end
function testDisabledProfileIsExactIdentity(t)
cfg=uavmvp.config();verifyEqual(t,uavmvp.apply_stress_profile(cfg,'none'),cfg);
verifyError(t,@()uavmvp.apply_stress_profile(cfg,'unknown'),'uavmvp:stressProfile');
end
function testProfilesPreserveFixedMissionAndEstimatorRules(t)
cfg=uavmvp.config();names={'correlated_link','visual_dropout','visual_outage', ...
    'matched_ranging','mismatched_ranging','joint'};
for k=1:numel(names)
    actual=uavmvp.apply_stress_profile(cfg,names{k});
    verifyEqual(t,actual.scene,cfg.scene);verifyEqual(t,actual.motion_and_budget,cfg.motion_and_budget);
    verifyEqual(t,actual.localization,cfg.localization);verifyEqual(t,actual.css,cfg.css);
    verifyEqual(t,actual.planner,cfg.planner);
    verifyFalse(t,actual.navigation_candidate.unknown_terminal_rtt_updates_navigation);
    verifyEqual(t,actual.sim.stress_profile_label,names{k});
end
end
function testMatchedAndMismatchedNoiseAreDifferentModels(t)
cfg=uavmvp.config();matched=uavmvp.apply_stress_profile(cfg,'matched_ranging');
mismatch=uavmvp.apply_stress_profile(cfg,'mismatched_ranging');
verifyEqual(t,matched.ranging.range_white_std_m,10);verifyEqual(t,matched.ranging.device_persistent_bias_std_m,10);
verifyFalse(t,isfield(matched.sim,'radio_truth_range_white_std_m'));
verifyEqual(t,mismatch.ranging,cfg.ranging);verifyEqual(t,mismatch.sim.radio_truth_range_white_std_m,10);
verifyEqual(t,mismatch.sim.radio_truth_device_bias_std_m,10);
joint=uavmvp.apply_stress_profile(cfg,'joint');verifyTrue(t,joint.sim.radio_correlated_bad_states);
verifyEqual(t,joint.navigation_candidate.marker_detect_probability_given_visibility,.5);
verifyEqual(t,joint.navigation_candidate.timestamp_alignment_std_s,.01);
verifyEqual(t,joint.ranging.initial_clock_ratio_std_ppm,2);
verifyEqual(t,joint.ranging.clock_ratio_random_walk_ppm_per_sqrt_s,.2);
verifyEqual(t,joint.sim.reference_outage_intervals_s,[60,120]);
end
function testForcedMissingImageConsumesTimeWithoutUpdate(t)
cfg=uavmvp.config();cfg.navigation_candidate.marker_detect_probability_given_visibility=1;
normal=uavmvp.MotionWorld(cfg,81);blocked_cfg=cfg;
blocked_cfg.sim.reference_outage_intervals_s=[0,100];blocked=uavmvp.MotionWorld(blocked_cfg,81);
normal.advance(15,[0;0;1]);blocked.advance(15,[0;0;1]);
[image,~]=normal.observe_reference(1);verifyNotEmpty(t,image);
version=blocked.nav.version;before=blocked.t;[missing,report]=blocked.observe_reference(1);
verifyEmpty(t,missing);verifyFalse(t,report.accepted);verifyEqual(t,blocked.nav.version,version);
verifyEqual(t,blocked.t-before,cfg.navigation_candidate.reference_observation_s,'AbsTol',1e-10);
data=blocked.finish();verifyTrue(t,data.reference_log{1}.sensor_forced_outage);
verifyEmpty(t,data.reference_log{1}.observation);normal.finish();
end
function testPhysicalNoiseScalingKeepsDeploymentAndClockPaths(t)
cfg=uavmvp.config();mismatch=uavmvp.apply_stress_profile(cfg,'mismatched_ranging');
a=uavmvp.RadioWorld(cfg,81,2,71);b=uavmvp.RadioWorld(mismatch,81,2,71);
x=a.offline();y=b.offline();verifyEqual(t,x.terminal_positions,y.terminal_positions);
verifyEqual(t,y.persistent_range_bias_m,x.persistent_range_bias_m*(10/3),'AbsTol',1e-13);
verifyEqual(t,x.bad_intervals,y.bad_intervals);
zero=cfg;zero.sim.radio_truth_range_white_std_m=0;
base=uavmvp.RadioWorld(zero,81,2,71);times=[1;1.0000002;1.02;1.0200002];
s0=base.timestamp(1,times);s5=a.timestamp(1,times);s10=b.timestamp(1,times);
verifyEqual(t,s10-s0,2*(s5-s0),'AbsTol',2e-15);
end
