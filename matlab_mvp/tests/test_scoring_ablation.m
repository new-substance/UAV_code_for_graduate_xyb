function tests=test_scoring_ablation
tests=functiontests(localfunctions);
end
function testFullScoreRetainsDeclaredCostEquation(t)
cfg=uavmvp.config();need=[2,1];Q=[.8,.9;.7,.1;.2,.8];d=[10;12;14];xy=[0,0;10,0;20,0];
reduction=[10,30];reserve=65;
[score,tail]=uavmvp.stage_candidate_score(need,Q,d,xy,1,reduction,reserve,cfg,1);
expected=uavmvp.shared_tail_cost(need,Q,d,xy,1,cfg,1);
verifyEqual(t,tail,expected);
verifyEqual(t,score,10+expected.time_proxy_s+65-12*(.5+1),'AbsTol',1e-12);
end
function testNoStageIgnoresRemainingStageCounts(t)
cfg=uavmvp.config();Q=[1,1;1,0;0,1];d=10*ones(3,1);xy=[0,0;10,0;20,0];
a=uavmvp.stage_candidate_score([1,4],Q,d,xy,1,[40,0],65,cfg,0);
b=uavmvp.stage_candidate_score([4,1],Q,d,xy,1,[40,0],65,cfg,0);
verifyNotEqual(t,a,b);
cfg.planner.scoring_ablation='no_stage';
a=uavmvp.stage_candidate_score([1,4],Q,d,xy,1,[40,0],65,cfg,0);
b=uavmvp.stage_candidate_score([4,1],Q,d,xy,1,[40,0],65,cfg,0);
verifyEqual(t,a,b);
end
function testNoSharingUsesSingleTargetAndPreservesOneTargetCase(t)
cfg=uavmvp.config();Q=ones(3,2);d=10*ones(3,1);xy=[0,0;10,0;20,0];
shared=uavmvp.stage_candidate_score([2,2],Q,d,xy,1,[0,0],65,cfg,1);
single=uavmvp.stage_candidate_score([2,0],Q,d,xy,1,[0,0],65,cfg,1);
cfg.planner.scoring_ablation='no_sharing';
[separate,tail]=uavmvp.stage_candidate_score([2,2],Q,d,xy,1,[0,0],65,cfg,1);
one=uavmvp.stage_candidate_score([2,0],Q,d,xy,1,[0,0],65,cfg,1);
verifyGreaterThan(t,separate,shared);verifyEqual(t,one,single);
verifyEqual(t,tail.focus_terminal,1);verifyEqual(t,tail.unresolved,[0,2]);
verifyLessThanOrEqual(t,numel(tail.sequence),1);
end
function testLocalCostDoesNotCreditTailOrReturnInRanking(t)
cfg=uavmvp.config();cfg.planner.scoring_ablation='local_cost';
Q=ones(2,1);d=[10;20];xy=[0,0;10,0];
[a,tail]=uavmvp.stage_candidate_score(5,Q,d,xy,1,0,65,cfg,0);
b=uavmvp.stage_candidate_score(5,Q,d,xy,1,0,200,cfg,1);
verifyEqual(t,a,10);verifyEqual(t,a,b);verifyEmpty(t,tail);
end
function testAblationsPreserveCandidatesForecastAndActualState(t)
[cfg,nav,tracker,book,h]=fixture();original=nav;original_book=book;original_tracker=tracker;
cfg.planner.prediction_model='conditional_reference';
base=uavmvp.local_service_candidates(nav,6,tracker,book,h,cfg,600,'stage_shared',2);
verifyTrue(t,base.accepted);verifyGreaterThan(t,numel(base.candidates),1);
for mode={'no_stage','no_sharing','local_cost'}
    cfg.planner.scoring_ablation=mode{1};
    choice=uavmvp.local_service_candidates(nav,6,tracker,book,h,cfg,600,'stage_shared',2);
    verifyEqual(t,choice.accepted,base.accepted);
    verifyEqual(t,rmfield(choice.candidates,{'score','tail'}),rmfield(base.candidates,{'score','tail'}));
    verifyEqual(t,choice.needs,base.needs);verifyEqual(t,choice.remaining_local_actions,2);
end
verifyEqual(t,nav,original);verifyEqual(t,book,original_book);verifyEqual(t,tracker,original_tracker);
end
function [cfg,nav,tracker,book,h]=fixture()
cfg=reference_layout(12);n=cfg.navigation_candidate;
n.acc_noise_density_mps2_per_sqrt_hz=0;n.gyro_noise_density_radps_per_sqrt_hz=0;
n.acc_bias_random_walk_mps2_per_sqrt_s=0;n.gyro_bias_random_walk_radps_per_sqrt_s=0;
cfg.navigation_candidate=n;nav=uavmvp.nav_init(cfg);nav.P(:)=0;nav.P(1:3,1:3)=.01*eye(3);
nav.p=[40;10;15];nav.v(:)=0;nav.t=1;[nav,id]=uavmvp.nav_clone(nav,cfg);
h0=uavmvp.radio_history_init(cfg);[history,rid]=uavmvp.predict_radio_record(h0,1,cfg);h={history};
record=struct('effective_nav_snapshot_id',id,'radio_history_id',rid,'range_m',sqrt(20^2+20^2+15^2));
book=struct('nodes',struct('discovered',true,'range_records',{{record}},'last_contact_s',1));
tracker=struct('terminals',struct('phase','LOCALIZE','pool_ids',1,'fit',[]));
nav.p=[60;10;15];nav.t=10;
end
