function tests=test_common_prediction
tests=functiontests(localfunctions);
end
function testPoliciesShareForecastAndOneStepScore(t)
[cfg,nav,tracker,book,h]=fixture();original=nav;original_book=book;original_tracker=tracker;
legacy=uavmvp.local_service_candidates(nav,6,tracker,book,h,cfg,600,'information',1);
cfg.planner.prediction_model='conditional_reference';
policies={'stage_shared','information','lookahead'};choices=cell(size(policies));
for k=1:numel(policies)
    choices{k}=uavmvp.local_service_candidates(nav,6,tracker,book,h,cfg,600,policies{k},1);
    verifyTrue(t,choices{k}.accepted);verifyEqual(t,numel(choices{k}.candidates),numel(legacy.candidates));
    fields={'target','conditional_on_actual_reference','window_s','duration_proxy_s','return_reserve_s', ...
        'probability','information','radius_reduction'};
    for j=1:numel(legacy.candidates)
        for f=1:numel(fields)
            verifyEqual(t,choices{k}.candidates(j).(fields{f}),legacy.candidates(j).(fields{f}));
        end
    end
end
for j=1:numel(legacy.candidates)
    verifyEqual(t,choices{1}.candidates(j).prediction,choices{2}.candidates(j).prediction);
    verifyEqual(t,choices{2}.candidates(j).prediction,choices{3}.candidates(j).prediction);
    verifyEqual(t,choices{2}.candidates(j).score,choices{3}.candidates(j).score,'AbsTol',1e-14);
end
verifyEqual(t,choices{2}.selected.target,choices{3}.selected.target);
verifyEqual(t,nav,original);verifyEqual(t,book,original_book);verifyEqual(t,tracker,original_tracker);
end
function testLegacyReferenceAliasEqualsCommonConditionalHorizon(t)
[cfg,nav,tracker,book,h]=fixture();
legacy=uavmvp.local_service_candidates(nav,6,tracker,book,h,cfg,600,'lookahead_reference',2);
cfg.planner.prediction_model='conditional_reference';
shared=uavmvp.local_service_candidates(nav,6,tracker,book,h,cfg,600,'lookahead',2);
verifyTrue(t,shared.accepted);verifyEqual(t,shared.candidates,legacy.candidates);
verifyEqual(t,shared.selected.target,legacy.selected.target);
horizons=[shared.candidates.horizon];verifyGreaterThan(t,sum([horizons.feasible_pairs]),0);
end
function testZeroImageProbabilityHasNoServiceBenefit(t)
[cfg,nav,tracker,book,h]=fixture();original=nav;
cfg.navigation_candidate.marker_detect_probability_given_visibility=0;
beliefs=uavmvp.planning_belief(tracker,book,h,nav,cfg);
p=uavmvp.reference_service_preview(nav,true,1,nav.t,6,tracker,book,h,beliefs,cfg,600);
verifyEqual(t,p.summary.reference_weight,0);verifyFalse(t,p.summary.admissible);
verifyEqual(t,p.summary.probability,0);verifyEqual(t,p.summary.information,0);
verifyEqual(t,p.summary.radius_reduction,0);
verifyEqual(t,p.summary.service_epoch_s,nav.t+1+cfg.motion_and_budget.calculation_model_s_per_decision,'AbsTol',1e-12);
prop=uavmvp.predict_hold(nav,1+cfg.motion_and_budget.calculation_model_s_per_decision,cfg);
verifyEqual(t,p.nav.P,prop.nav.P,'AbsTol',1e-11);verifyEqual(t,nav,original);
end
function testServiceEpochAndScalarPosteriorHaveIndependentOracle(t)
[cfg,nav,tracker,book,h]=fixture();nav.P(:)=0;nav.P(1,1)=.16;
beliefs=uavmvp.planning_belief(tracker,book,h,nav,cfg);original=nav;
p=cfg.navigation_candidate.marker_detect_probability_given_visibility*gammainc(cfg.sim.reference_nis_gate/2,4);
[~,H]=uavmvp.camera_model(nav,6,cfg);
precision=H(:,1)'*H(:,1)/(cfg.navigation_candidate.camera.pixel_std*cfg.sim.camera_noise_scale)^2;
q=1-(1-p)^2;variance=(2*p*(1-p)/(.16^-1+precision)+p^2/(.16^-1+2*precision))/q;
preview=uavmvp.reference_service_preview(nav,true,1,nav.t,6,tracker,book,h,beliefs,cfg,600);
verifyEqual(t,preview.summary.reference_weight,q,'AbsTol',1e-13);
verifyEqual(t,preview.nav.P(1,1),variance,'AbsTol',1e-12);
verifyEqual(t,preview.summary.service_epoch_s,nav.t+1.05,'AbsTol',1e-12);
verifyEqual(t,preview.nav.version,nav.version);verifyEqual(t,preview.nav.last_reference_id,nav.last_reference_id);
verifyEqual(t,preview.nav.last_reference_time_s,nav.last_reference_time_s);verifyEqual(t,nav,original);
end
function testDeadlineSuppressesForecastBenefitWithoutNewEvidence(t)
[cfg,nav,tracker,book,h]=fixture();beliefs=uavmvp.planning_belief(tracker,book,h,nav,cfg);original=nav;
enough=uavmvp.reference_service_preview(nav,false,1,nav.t,6,tracker,book,h,beliefs,cfg,600);
late=uavmvp.reference_service_preview(nav,false,1,nav.t,6,tracker,book,h,beliefs,cfg,nav.t);
verifyTrue(t,enough.summary.admissible);verifyGreaterThan(t,sum(enough.summary.information),0);
verifyFalse(t,late.summary.admissible);verifyEqual(t,late.summary.information,0);
verifyEqual(t,enough.nav,late.nav);verifyEqual(t,late.nav.t,nav.t+.05,'AbsTol',1e-12);
verifyEqual(t,nav,original);
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
