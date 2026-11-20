function tests=test_information_lookahead
tests=functiontests(localfunctions);
end
function testBlockMatchesIndependentFullJointSolve(t)
cfg=uavmvp.config();x=[50;35];a=[10,70,70,10,40,60;10,10,60,60,10,60;15,15,15,15,15,15];
P=kron(ones(6),diag([9,4,1]))+.04*eye(18);C=25*eye(6)+9*ones(6);
s=uavmvp.block_information(x,a,P,C,4,cfg);
[~,H,V]=uavmvp.range_model(x,a,P,C,0);J0=diag(12./[120,80].^2);
J1=J0+H(1:4,:)'*(V(1:4,1:4)\H(1:4,:));J2=J0+H'*(V\H);
verifyEqual(t,s.before,inv(J1),'AbsTol',1e-9);
verifyEqual(t,s.after,inv(J2),'AbsTol',1e-9);
verifyEqual(t,s.information_gain,log(det(J2)/det(J1)),'AbsTol',1e-11);
% Full joint and successive conditional updates agree; two independent
% prospective increments conditioned only on the old rows do not.
first=uavmvp.conditional_information(x,a(:,1:5),P(1:15,1:15),C(1:5,1:5),cfg);
last=uavmvp.conditional_information(x,a,P,C,cfg);
verifyEqual(t,s.information_gain,first.information_gain+last.information_gain,'AbsTol',1e-11);
idx=[1:12,16:18];cols=[1:4,6];
other=uavmvp.conditional_information(x,a(:,cols),P(idx,idx),C(cols,cols),cfg);
verifyGreaterThan(t,abs(s.information_gain-first.information_gain-other.information_gain),1e-3);
end
function testBlockSingleFutureMatchesCommonInformation(t)
cfg=uavmvp.config();a=[10,70,70,10;10,10,60,60;15,15,15,15];P=kron(ones(4),eye(3));C=25*eye(4)+9*ones(4);
a1=uavmvp.conditional_information([40;30],a,P,C,cfg);
a2=uavmvp.block_information([40;30],a,P,C,3,cfg);
verifyEqual(t,a1.after,a2.after,'AbsTol',1e-10);
verifyEqual(t,a1.information_gain,a2.information_gain,'AbsTol',1e-12);
end
function testBothSuccessUsesJointCorrelatedRadioAndNav(t)
[cfg,tracker,book,h,beliefs,nav,first,second]=fixture();original=nav;
result=uavmvp.two_step_information(tracker,book,h,beliefs,nav,first,second,cfg);
[h1,r1]=uavmvp.predict_radio_record(h{1},10,cfg);[h2,r2]=uavmvp.predict_radio_record(h1,20,cfg);
[a,P]=uavmvp.nav_archive(nav,[first,second]);C=uavmvp.radio_history_covariance(h2,[r1,r2]);
[~,H,V]=uavmvp.range_model(beliefs{1}.samples',a,P,C,0);prior=diag(12./[120,80].^2);
expected=log(det(prior+H'*(V\H))/det(prior));
verifyEqual(t,result.total,expected,'AbsTol',1e-11);
verifyNotEqual(t,C(1,2),0);verifyGreaterThan(t,norm(P(1:3,4:6)),0);
verifyEqual(t,nav,original);verifyEqual(t,h{1},uavmvp.radio_history_init(cfg));
end
function testFailedFirstDoesNotGiveSecondWarmCalibration(t)
[cfg,tracker,book,h,beliefs,nav,first,second]=fixture();
nav.clones(:,1)=[90;30;15];beliefs{1}.samples=[10,30];
result=uavmvp.two_step_information(tracker,book,h,beliefs,nav,first,second,cfg);
[hf,id]=uavmvp.predict_radio_record(h{1},20,cfg);[a,P]=uavmvp.nav_archive(nav,second);
C=uavmvp.radio_history_covariance(hf,id);[~,H,V]=uavmvp.range_model([10;30],a,P,C,0);
prior=diag(12./[120,80].^2);expected=log(det(prior+H'*(V\H))/det(prior));
verifyEqual(t,result.modes,{'cold','warm','cold'});
verifyEqual(t,result.total,expected,'AbsTol',1e-11);
end
function testHeldoutAndCoincidentSuccessDoNotAddSecondRtt(t)
[cfg,tracker,book,h,beliefs,nav,first,second]=fixture();
beliefs{1}.phase='VERIFY';
result=uavmvp.two_step_information(tracker,book,h,beliefs,nav,first,second,cfg);
[hf,id]=uavmvp.predict_radio_record(h{1},10,cfg);[a,P]=uavmvp.nav_archive(nav,first);
C=uavmvp.radio_history_covariance(hf,id);one=uavmvp.block_information(beliefs{1}.samples',a,P,C,0,cfg);
verifyEqual(t,result.total,one.information_gain,'AbsTol',1e-11);
beliefs{1}.phase='LOCALIZE';nav.clones(:,2)=nav.clones(:,1);
result=uavmvp.two_step_information(tracker,book,h,beliefs,nav,first,second,cfg);
verifyFalse(t,result.pair_distinct);verifyEqual(t,result.total,one.information_gain,'AbsTol',1e-11);
end
function testFutureRefinementCannotChangeFirstEligibility(t)
[cfg,tracker,book,h,beliefs,nav,first,second]=fixture();
result=uavmvp.two_step_information(tracker,book,h,beliefs,nav,first,second,cfg,false);
[hf,id]=uavmvp.predict_radio_record(h{1},20,cfg);[a,P]=uavmvp.nav_archive(nav,second);
C=uavmvp.radio_history_covariance(hf,id);one=uavmvp.block_information(beliefs{1}.samples',a,P,C,0,cfg);
verifyEqual(t,result.first_total,0);verifyEqual(t,result.additional_total,result.total,'AbsTol',1e-12);
verifyEqual(t,result.total,one.information_gain,'AbsTol',1e-11);
end
function testOneRemainingActionIsExactlyCommonMyopicScore(t)
cfg=uavmvp.config();rows=repmat(struct('information',[1,2],'duration_proxy_s',10),2,1);
rows(2).information=[2,4];rows(2).duration_proxy_s=15;
scored=uavmvp.information_lookahead(rows,{},[],1,[],[],{}, {},cfg,600,1);
verifyEqual(t,[scored.score],[-.3,-.4],'AbsTol',1e-12);
verifyEqual(t,scored(1).horizon.sequence,1);verifyEqual(t,scored(2).horizon.sequence,2);
horizons=[scored.horizon];verifyEqual(t,[horizons.evaluated_pairs],[0,0]);
end
function testOutcomeWeightsMatchIndependentBranchEnumeration(t)
[cfg,tracker,book,h,beliefs,nav,first,second]=fixture();
cfg.ranging.baseline_stage_probabilities=struct('wake_decode',.8,'identity_decode_given_control',.7,'range_exchange_valid',.6);
result=uavmvp.two_step_information(tracker,book,h,beliefs,nav,first,second,cfg);
[h1,r1]=uavmvp.predict_radio_record(h{1},10,cfg);
[h11,r11]=uavmvp.predict_radio_record(h1,20,cfg);[h01,r01]=uavmvp.predict_radio_record(h{1},20,cfg);
[a,P]=uavmvp.nav_archive(nav,[first,second]);prior=diag(12./[120,80].^2);x=beliefs{1}.samples';
C1=uavmvp.radio_history_covariance(h1,r1);C11=uavmvp.radio_history_covariance(h11,[r1,r11]);
C01=uavmvp.radio_history_covariance(h01,r01);
[~,H1,V1]=uavmvp.range_model(x,a(:,1),P(1:3,1:3),C1,0);
[~,H11,V11]=uavmvp.range_model(x,a,P,C11,0);
[~,H01,V01]=uavmvp.range_model(x,a(:,2),P(4:6,4:6),C01,0);
g1=log(det(prior+H1'*(V1\H1))/det(prior));g11=log(det(prior+H11'*(V11\H11))/det(prior));
g01=log(det(prior+H01'*(V01\H01))/det(prior));p1=.8*.7*.6^2;p11=.8*.7*.6;p01=p1;
expected=p1*(1-p11)*g1+p1*p11*g11+(1-p1)*p01*g01;
verifyEqual(t,result.total,expected,'AbsTol',1e-11);
end
function testPairForecastPropagatesWithoutChangingActualNav(t)
cfg=reference_layout(12);n=cfg.navigation_candidate;
n.acc_noise_density_mps2_per_sqrt_hz=0;n.gyro_noise_density_radps_per_sqrt_hz=0;
n.acc_bias_random_walk_mps2_per_sqrt_s=0;n.gyro_bias_random_walk_radps_per_sqrt_s=0;
cfg.navigation_candidate=n;nav=uavmvp.nav_init(cfg);nav.P(:)=0;nav.P(1:3,1:3)=.01*eye(3);
nav.p=[60;10;15];nav.v(:)=0;nav.t=10;original=nav;
tracker=struct('terminals',struct('pool_ids',[]));book=struct('nodes',struct('discovered',true));
h={uavmvp.radio_history_init(cfg)};beliefs={struct('samples',[60,25],'phase','LOCALIZE')};
rows=repmat(struct('information',1,'duration_proxy_s',20,'target',[], ...
    'window_s',1,'conditional_on_actual_reference',true),2,1);forecasts=cell(1,2);
for k=1:2
    rows(k).target=[60+(2*k-3)*6.5;10;15];
    travel=uavmvp.predict_leg(nav,rows(k).target,cfg,60);forecasts{k}=travel.nav;
end
scored=uavmvp.information_lookahead(rows,forecasts,nav,6,tracker,book,h,beliefs,cfg,600,2);
horizons=[scored.horizon];verifyGreaterThan(t,sum([horizons.feasible_pairs]),0);
verifyEqual(t,[horizons.evaluated_pairs],[1,1]);verifyEqual(t,nav,original);
for k=1:2,verifyLessThanOrEqual(t,numel(scored(k).horizon.sequence),2);end
verifyEqual(t,vertcat(scored.target),vertcat(rows.target));
end
function [cfg,tracker,book,h,beliefs,nav,first,second]=fixture()
cfg=uavmvp.config();pr=cfg.ranging.baseline_stage_probabilities;
pr.wake_decode=1;pr.identity_decode_given_control=1;pr.range_exchange_valid=1;
cfg.ranging.baseline_stage_probabilities=pr;
tracker=struct('terminals',struct('pool_ids',[]));
book=struct('nodes',struct('discovered',true));h={uavmvp.radio_history_init(cfg)};
beliefs={struct('samples',[38,35],'phase','LOCALIZE')};
nav=uavmvp.nav_init(cfg);nav.P(:)=0;nav.P(1:3,1:3)=diag([4,4,1]);nav.v(:)=0;
nav.p=[30;30;15];nav.t=10;[nav,first]=uavmvp.nav_clone(nav,cfg);
nav.p=[45;40;15];nav.t=20;[nav,second]=uavmvp.nav_clone(nav,cfg);
end
