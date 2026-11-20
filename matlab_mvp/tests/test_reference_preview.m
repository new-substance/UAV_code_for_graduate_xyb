function tests=test_reference_preview
tests=functiontests(localfunctions);
end
function testTwoFramesMatchIndependentScalarBayesMixture(t)
[cfg,nav]=fixture();nav.P(:)=0;nav.P(1,1)=.16;
[nav,clone]=uavmvp.nav_clone(nav,cfg);original=nav;
p=cfg.navigation_candidate.marker_detect_probability_given_visibility*gammainc(cfg.sim.reference_nis_gate/2,4);
[~,H]=uavmvp.camera_model(nav,6,cfg);precision=H(:,1)'*H(:,1)/cfg.navigation_candidate.camera.pixel_std^2;
q=1-(1-p)^2;expected=(2*p*(1-p)/(.16^-1+precision)+p^2/(.16^-1+2*precision))/q;
preview=uavmvp.reference_belief_preview(nav,6,cfg,2);
verifyEqual(t,preview.success_probability_proxy,q,'AbsTol',1e-13);
verifyEqual(t,preview.conditional_nav.P(1,1),expected,'AbsTol',1e-12);
[~,P]=uavmvp.nav_archive(preview.conditional_nav,clone);
verifyEqual(t,P(1,1),expected,'AbsTol',1e-12);
verifyEqual(t,nav,original);verifyEqual(t,preview.conditional_nav.version,nav.version);
verifyEqual(t,preview.conditional_nav.last_reference_id,nav.last_reference_id);
verifyEqual(t,preview.conditional_nav.last_reference_time_s,nav.last_reference_time_s);
verifyEqual(t,preview.conditional_nav.p,nav.p,'AbsTol',1e-12);
verifyEqual(t,preview.conditional_nav.t,nav.t+1,'AbsTol',1e-12);
verifyGreaterThanOrEqual(t,min(eig(preview.conditional_nav.P)),-1e-12);
end
function testCommonMapTranslationRemainsUnobservable(t)
[cfg,nav]=fixture();v=zeros(size(nav.P,1),1);v(1)=1;
for id=1:size(nav.map_p,2),v(15+6*(id-1)+1)=1;end
nav.P=.16*(v*v');[nav,~]=uavmvp.nav_clone(nav,cfg);original=nav;
preview=uavmvp.reference_belief_preview(nav,6,cfg,2);
verifyGreaterThan(t,preview.success_probability_proxy,0);
verifyEqual(t,preview.conditional_nav.P,nav.P,'AbsTol',1e-11);
verifyEqual(t,nav,original);
end
function testMissingAndInvisibleReferenceCannotReduceCovariance(t)
[cfg,nav]=fixture();nav.P(:)=0;nav.P(1,1)=.16;nav.P(4,4)=.0001;
cfg.navigation_candidate.marker_detect_probability_given_visibility=0;
preview=uavmvp.reference_belief_preview(nav,6,cfg,2);prop=uavmvp.predict_hold(nav,1,cfg);
verifyEqual(t,preview.success_probability_proxy,0);verifyEqual(t,preview.conditional_nav.P,prop.nav.P,'AbsTol',1e-11);
cfg.navigation_candidate.marker_detect_probability_given_visibility=1;
preview=uavmvp.reference_belief_preview(nav,1,cfg,2);
verifyEqual(t,preview.success_probability_proxy,0);verifyEqual(t,preview.conditional_nav.P,prop.nav.P,'AbsTol',1e-11);
end
function testConditionalPreviewLeavesActualCandidateSetUnchanged(t)
[cfg,nav]=fixture();nav.P(:)=0;nav.P(1:3,1:3)=.01*eye(3);
nav.p=[40;10;15];nav.t=1;[nav,id]=uavmvp.nav_clone(nav,cfg);
h0=uavmvp.radio_history_init(cfg);[h,rid]=uavmvp.predict_radio_record(h0,1,cfg);
record=struct('effective_nav_snapshot_id',id,'radio_history_id',rid,'range_m',sqrt(20^2+20^2+15^2));
book=struct('nodes',struct('discovered',true,'range_records',{{record}},'last_contact_s',1));
tracker=struct('terminals',struct('phase','LOCALIZE','pool_ids',1,'fit',[]));
nav.p=[60;10;15];nav.t=10;original=nav;
plain=uavmvp.local_service_candidates(nav,6,tracker,book,{h},cfg,600,'information',2);
preview=uavmvp.local_service_candidates(nav,6,tracker,book,{h},cfg,600,'lookahead_reference',2);
verifyTrue(t,plain.accepted);verifyTrue(t,preview.accepted);verifyEqual(t,nav,original);
fields={'target','conditional_on_actual_reference','window_s','duration_proxy_s','return_reserve_s','probability','information','radius_reduction'};
verifyEqual(t,numel(plain.candidates),numel(preview.candidates));
for j=1:numel(plain.candidates)
    for k=1:numel(fields),verifyEqual(t,plain.candidates(j).(fields{k}),preview.candidates(j).(fields{k}));end
end
horizons=[preview.candidates.horizon];verifyGreaterThan(t,sum([horizons.feasible_pairs]),0);
verifyTrue(t,all([horizons.first_reference_weight]<=1));
for j=1:numel(horizons),verifyLessThanOrEqual(t,numel(horizons(j).sequence),2);end
end
function [cfg,nav]=fixture()
cfg=reference_layout(12);n=cfg.navigation_candidate;
n.acc_noise_density_mps2_per_sqrt_hz=0;n.gyro_noise_density_radps_per_sqrt_hz=0;
n.acc_bias_random_walk_mps2_per_sqrt_s=0;n.gyro_bias_random_walk_radps_per_sqrt_s=0;
cfg.navigation_candidate=n;nav=uavmvp.nav_init(cfg);nav.p=[60;10;15];nav.v(:)=0;nav.t=10;
end
