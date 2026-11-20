function tests=test_terminal_tracker
tests=functiontests(localfunctions);
end
function testFourFitRequiresNewFifthAndPostcheckDoesNotResumeRtt(t)
[cfg,book,tracker,nav,h,a,x]=fixture();
for k=1:4,[nav,h]=add(book,tracker,nav,h,a(:,k),x,k,cfg,0);end
state=tracker.terminals(1);verifyEqual(t,state.phase,'VERIFY',state.reason);
verifyFalse(t,book.nodes(1).completed);verifyEqual(t,state.fit_ids,1:4);
[nav,h]=add(book,tracker,nav,h,a(:,5),x,5,cfg,0);
verifyEqual(t,tracker.terminals(1).phase,'DONE');verifyTrue(t,book.nodes(1).completed);
verifyLessThan(t,norm(tracker.terminals(1).fit.x-x),.01);
verifyEqual(t,tracker.terminals(1).proof.heldout_id,5);
nav.clones(:,5)=nav.clones(:,5)+[25;0;0];nav.version=nav.version+1;
tracker.refresh_completed(nav,{h});
verifyEqual(t,tracker.terminals(1).reason,'POSTCHECK_INCONSISTENT');
verifyError(t,@()book.begin_ranging(1,6,6,'cold'),'uavmvp:completedRanging');
end
function testFailedHeldoutNeedsNewFitPositionAndNewHeldout(t)
[cfg,book,tracker,nav,h,a,x]=fixture();
for k=1:4,[nav,h]=add(book,tracker,nav,h,a(:,k),x,k,cfg,0);end
[nav,h]=add(book,tracker,nav,h,a(:,5),x,5,cfg,25);
state=tracker.terminals(1);verifyEqual(t,state.phase,'LOCALIZE');
verifyEqual(t,state.used_heldout,5);verifyEqual(t,state.verification_rounds,1);
tracker.ingest(1,nav,h);verifyEqual(t,tracker.terminals(1),state);
[~,~]=add(book,tracker,nav,h,a(:,6),x,6,cfg,0);
verifyTrue(t,ismember(6,tracker.terminals(1).fit_ids));
verifyEqual(t,tracker.terminals(1).verification_rounds,1);
verifyFalse(t,book.nodes(1).completed);
end
function testNavigationRevisionBeforeHeldoutKeepsOriginalFour(t)
[cfg,book,tracker,nav,h,a,x]=fixture();
for k=1:4,[nav,h]=add(book,tracker,nav,h,a(:,k),x,k,cfg,0);end
shift=[1.5;-.5;0];nav.clones=nav.clones+shift;nav.version=1;
[~,~]=add(book,tracker,nav,h,a(:,5),x,5,cfg,0,shift);
state=tracker.terminals(1);verifyEqual(t,state.phase,'DONE',state.reason);
verifyEqual(t,state.fit_ids,1:4);verifyEqual(t,state.fit_version,1);
verifyLessThan(t,norm(state.fit.x-x-shift(1:2)),.01);
verifyEqual(t,state.proof.heldout_id,5);
end
function testDuplicateLocationDoesNotBecomeFourthOrHeldout(t)
[cfg,book,tracker,nav,h,a,x]=fixture();
for k=1:3,[nav,h]=add(book,tracker,nav,h,a(:,k),x,k,cfg,0);end
[~,~]=add(book,tracker,nav,h,a(:,3),x,4,cfg,0);
verifyEqual(t,tracker.terminals(1).phase,'LOCALIZE');
verifyEqual(t,tracker.terminals(1).pool_ids,1:3);verifyEqual(t,tracker.terminals(1).ignored_ids,4);
end
function testGeometrySelectorCannotUseRangeResiduals(t)
[cfg,book,tracker,nav,h,a,x]=fixture();
for k=1:4,[nav,h]=add(book,tracker,nav,h,a(:,k),x,k,cfg,0);end
data=uavmvp.record_bundle(book.nodes(1).range_records,nav,h,1:4);
[first,score]=uavmvp.select_four(data,[],cfg,x);data.ranges=data.ranges+[100;-100;50;-50];
[second,other]=uavmvp.select_four(data,[],cfg,x);
verifyEqual(t,first,second);verifyEqual(t,score,other);
end
function testHistoricalQualitySnapshotsReplayAndRemainImmutable(t)
[cfg,book,tracker,nav,h,a,x]=fixture();
for k=1:5,[nav,h]=add(book,tracker,nav,h,a(:,k),x,k,cfg,0);end
saved=tracker.quality_log;verifyNumElements(t,saved,2);
verifyEqual(t,saved{1}.kind,'FIT');verifyEqual(t,saved{2}.kind,'VERIFY');
nav.clones=nav.clones+[1;0;0];
nav.clones(:,5)=nav.clones(:,5)+[25;0;0];nav.version=nav.version+1;
tracker.refresh_completed(nav,{h});
verifyEqual(t,tracker.quality_log(1:2),saved);verifyNumElements(t,tracker.quality_log,3);
verifyFalse(t,tracker.quality_log{3}.accepted);verifyEqual(t,tracker.quality_log{3}.kind,'POSTCHECK');
verifyGreaterThan(t,norm(tracker.quality_log{3}.fit.x-tracker.terminals(1).fit.x),.9);
data=offline_data(book,tracker,nav,x);data=jsondecode(jsonencode(data));
report=audit_quality_history(data,cfg);
verifyEqual(t,report.replayed,3);verifyEqual(t,report.statistics.initial_accepted,1);
verifyEqual(t,report.statistics.postcheck_rejections,1);
verifyEqual(t,report.statistics.initial_accepted_outside,0);
verifyTrue(t,book.nodes(1).completed);verifyEqual(t,tracker.terminals(1).phase,'FAILED');
end
function testCommonShiftFalseAcceptanceSurvivesLaterRevocation(t)
[cfg,book,tracker,nav,h,a,x]=fixture();shift=[21;0;0];
for k=1:5,[nav,h]=add(book,tracker,nav,h,a(:,k),x,k,cfg,0,shift);end
verifyEqual(t,tracker.terminals(1).phase,'DONE');
verifyGreaterThan(t,norm(tracker.terminals(1).fit.x-x),20);
nav.clones(:,5)=nav.clones(:,5)+[25;0;0];nav.version=nav.version+1;
tracker.refresh_completed(nav,{h});verifyEqual(t,tracker.terminals(1).phase,'FAILED');
report=audit_quality_history(offline_data(book,tracker,nav,x),cfg);s=report.statistics;
verifyEqual(t,s.initial_accepted_outside,1);verifyEqual(t,s.initial_false_accept_fraction,1);
verifyEqual(t,s.outside_initial_accept_fraction,1);verifyEqual(t,s.revoked_terminal_ids,1);
verifyTrue(t,s.task_has_observed_false_completion);verifyEqual(t,s.ever_false_accepted_terminal_ids,1);
verifyError(t,@()book.begin_ranging(1,6,6,'cold'),'uavmvp:completedRanging');
end
function testLaterAcceptedVersionCanBeWrongAfterCorrectFirstAccept(t)
[cfg,book,tracker,nav,h,a,x]=fixture();
for k=1:5,[nav,h]=add(book,tracker,nav,h,a(:,k),x,k,cfg,0);end
nav.clones=nav.clones+[21;0;0];nav.version=nav.version+1;
tracker.refresh_completed(nav,{h});verifyEqual(t,tracker.terminals(1).phase,'DONE');
report=audit_quality_history(offline_data(book,tracker,nav,x),cfg);s=report.statistics;
verifyEqual(t,s.initial_accepted_outside,0);verifyEqual(t,s.accepted_versions,2);
verifyEqual(t,s.accepted_versions_outside,1);verifyEqual(t,s.published_version_false_accept_fraction,.5);
verifyTrue(t,s.task_has_observed_false_completion);verifyEmpty(t,s.revoked_terminal_ids);
end
function testNoAcceptanceLeavesRiskDenominatorUndefined(t)
[cfg,book,tracker,nav,h,a,x]=fixture();
for k=1:4,[nav,h]=add(book,tracker,nav,h,a(:,k),x,k,cfg,0);end
[nav,~]=add(book,tracker,nav,h,a(:,5),x,5,cfg,25);
report=audit_quality_history(offline_data(book,tracker,nav,x),cfg);s=report.statistics;
verifyEqual(t,s.initial_checks,1);verifyEqual(t,s.initial_accepted,0);
verifyTrue(t,isnan(s.initial_false_accept_fraction));verifyTrue(t,isnan(s.outside_initial_accept_fraction));
verifyEqual(t,s.initial_rejected_inside,1);verifyFalse(t,s.task_has_observed_false_completion);
end
function testQualityAuditRejectsAlteredAndMissingEvidence(t)
[cfg,book,tracker,nav,h,a,x]=fixture();
for k=1:5,[nav,h]=add(book,tracker,nav,h,a(:,k),x,k,cfg,0);end
original=offline_data(book,tracker,nav,x);changed=original;
changed.quality_decisions{2}.check.variance=changed.quality_decisions{2}.check.variance+1;
verifyError(t,@()audit_quality_history(changed,cfg),'uavmvp:qualityReplay');
changed=original;changed.quality_decisions=changed.quality_decisions(1);
verifyError(t,@()audit_quality_history(changed,cfg),'uavmvp:incompleteQualityHistory');
end
function data=offline_data(book,tracker,nav,x)
% Synthetic terminated-record fixture; truth is never passed to the tracker.
data=struct('quality_decisions',{tracker.quality_log},'tracker_events',{tracker.event_log}, ...
    'protocol_nodes',book.nodes,'terminals',tracker.terminals,'final_navigation',nav, ...
    'radio_truth',struct('terminal_positions',[x;0]));
end
function [cfg,book,tracker,nav,h,a,x]=fixture()
cfg=uavmvp.config();cfg.scene.width_m=40;cfg.scene.height_m=30;
cfg.ranging.device_persistent_bias_std_m=.3;cfg.ranging.clock_ratio_random_walk_ppm_per_sqrt_s=0;
book=uavmvp.ProtocolBook(cfg,1,17);device=uavmvp.TerminalDevice(1,17);
control=book.group_control(1,0);packet=device.receive_group(control);timing=uavmvp.radio_timing(cfg);
book.receive_identity(packet,control.frame_s+packet.reply_offset_s+timing.frames.identity);
tracker=uavmvp.TerminalTracker(cfg,book);nav=uavmvp.nav_init(cfg);nav.P(:)=0;
h=uavmvp.radio_history_init(cfg);a=[2,38,38,2,20,20;2,2,28,28,1,29;15,15,15,15,15,15];x=[17.3;13.2];
end
function [nav,h]=add(book,tracker,nav,h,anchor,x,k,cfg,error,nav_shift)
if nargin<10,nav_shift=zeros(3,1);end
nav.p=anchor-cfg.navigation_candidate.antenna_lever_arm_body_m(:)+nav_shift;nav.t=k;
[nav,id]=uavmvp.nav_clone(nav,cfg);request=book.begin_ranging(1,k,id,'cold');
range=norm(anchor-[x;0])+error;out=struct('range_m',range,'kappa',1,'joint_covariance',diag([1,0]),'exchange_count',2);
[h,radio_id,q]=uavmvp.radio_history_append(h,out,k+.4,cfg,'cold');
packet=struct('mission_id',17,'address',1,'kind','RANGE_META','session_id',request.session_id, ...
    'sequence',1,'crc_valid',true,'range_m',range,'reported_std_m',q.reported_std_m, ...
    'exchange_valid',[true,true],'exchange_sequences',[1,2],'nav_snapshot_id',id, ...
    'measurement_epoch_s',k,'effective_nav_snapshot_id',id,'radio_history_id',radio_id);
report=book.receive_range(packet,k+.4);assert(report.accepted);tracker.ingest(1,nav,h);
end
