function tests=test_service_stop
tests=functiontests(localfunctions);
end
function testFirstRoundBudgetCountsPotentialRepliesNotOnlyReceivedOnes(t)
cfg=uavmvp.config();timing=uavmvp.radio_timing(cfg,3);
expected=cfg.motion_and_budget.settle_s+timing.group_s+cfg.css.wake_ready_s+ ...
    3*(timing.frames.identity+cfg.css.guard_s+timing.cold_max_s+cfg.motion_and_budget.calculation_model_s_per_decision);
verifyEqual(t,uavmvp.first_round_budget(cfg,1:3),expected,'AbsTol',1e-12);
verifyGreaterThan(t,uavmvp.first_round_budget(cfg,1:10),uavmvp.first_round_budget(cfg,1:5));
verifyEqual(t,uavmvp.first_round_budget(cfg,[]),0);
end
function testFirstRoundPrecedesRetriesAndAllTimeoutsAreCharged(t)
cfg=uavmvp.config();cfg.ranging.baseline_stage_probabilities=struct('wake_decode',1,'identity_decode_given_control',1,'range_exchange_valid',0);
w=uavmvp.MotionWorld(cfg,8301);w.configure_radio(10,44,repmat([20;10;0],1,10));
book=uavmvp.ProtocolBook(cfg,10,44);tracker=uavmvp.TerminalTracker(cfg,book);
histories=repmat({uavmvp.radio_history_init(cfg)},1,10);
[~,visit]=uavmvp.service_stop(w,book,tracker,histories,cfg,1,600);
verifyTrue(t,visit.opportunity_complete);verifyLessThanOrEqual(t,visit.end_s-visit.start_s,10+1e-10);
types=cellfun(@(x)x.kind,visit.actions,'UniformOutput',false);
first_rtt=find(strcmp(types,'RTT'),1);verifyEqual(t,first_rtt,5); % four groups first
rtt=[visit.actions{strcmp(types,'RTT')}];
verifyEqual(t,[rtt(1:10).address],1:10);verifyEqual(t,[rtt(1:10).round],ones(1,10));
verifyEqual(t,visit.attempts,2*ones(1,10));verifyEqual(t,visit.contact_counts,ones(1,10));
verifyFalse(t,any([book.nodes.discovered]));
verifyEqual(t,[rtt.end_s]-[rtt.start_s],.35*ones(1,20),'AbsTol',1e-8);
data=w.finish();verifyGreaterThan(t,data.time_s,8);
end
function testInsufficientVisitBudgetDoesNotClaimCoverageOpportunity(t)
cfg=uavmvp.config();w=uavmvp.MotionWorld(cfg,8302);w.configure_radio(10,44);
book=uavmvp.ProtocolBook(cfg,10,44);tracker=uavmvp.TerminalTracker(cfg,book);
histories=repmat({uavmvp.radio_history_init(cfg)},1,10);
[~,visit]=uavmvp.service_stop(w,book,tracker,histories,cfg,1,2);
verifyFalse(t,visit.opportunity_complete);verifyEqual(t,visit.reason,'FIRST_ROUND_BUDGET_UNAVAILABLE');
verifyEmpty(t,visit.actions);verifyEqual(t,w.t,0);w.finish();
end
