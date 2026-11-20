function tests=test_fixed_coverage
tests=functiontests(localfunctions);
end
function testTwoCyclicSweepsHaveFixedOrderAndFiniteExit(t)
cfg=reference_layout(12);order=[1,5,6,7,2,8,3,9,10,11,4,12];
visits=zeros(12,1);cursor=1;current=1;sequence=[];
for k=1:24
    sequence(end+1)=current;visits(current)=visits(current)+1; %#ok<AGROW>
    [choice,cursor]=uavmvp.next_coverage_reference(order,cursor,visits,current,cfg);
    if k<24
        verifyTrue(t,choice.accepted);
        verifyEqual(t,choice.selected.path.ids([1,end]),[current,choice.selected.reference_id]);
        current=choice.selected.reference_id;
    else
        verifyFalse(t,choice.accepted);verifyEqual(t,choice.reason,'COVERAGE_ATTEMPT_LIMIT');
    end
end
verifyEqual(t,sequence,[order,order]);verifyEqual(t,visits,2*ones(12,1));
end
function testExhaustedStationsAreSkippedAndDisconnectedMapFails(t)
cfg=reference_layout(12);order=[1,5,6,7,2,8,3,9,10,11,4,12];
visits=2*ones(12,1);visits(7)=1;
[choice,cursor]=uavmvp.next_coverage_reference(order,1,visits,1,cfg);
verifyTrue(t,choice.accepted);verifyEqual(t,cursor,4);
verifyEqual(t,choice.scanned_ids,[5,6,7]);verifyEqual(t,choice.selected.reference_id,7);
cfg.sim.reference_graph_edge_limit_m=1;
[blocked,~]=uavmvp.next_coverage_reference(order,1,visits,1,cfg);
verifyFalse(t,blocked.accepted);verifyEqual(t,blocked.reason,'REFERENCE_GRAPH_DISCONNECTED');
end
function testFixedCoveragePreservesActionsAndFixedRank(t)
[cfg,nav,tracker,book,h]=fixture();
fixed=uavmvp.local_service_candidates(nav,6,tracker,book,h,cfg,600,'fixed_coverage',1);
legacy=uavmvp.local_service_candidates(nav,6,tracker,book,h,cfg,600,'fixed',1);
information=uavmvp.local_service_candidates(nav,6,tracker,book,h,cfg,600,'information',1);
verifyTrue(t,fixed.accepted);verifyGreaterThan(t,numel(fixed.candidates),1);
verifyEqual(t,fixed.candidates,legacy.candidates);
verifyEqual(t,fixed.selected.target,fixed.candidates(1).target);
verifyEqual(t,rmfield(fixed.candidates,'score'),rmfield(information.candidates,'score'));
verifyEqual(t,[fixed.candidates.score],1:numel(fixed.candidates));
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
