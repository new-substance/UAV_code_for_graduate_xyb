function tests=test_reference_pose_oracle
tests=functiontests(localfunctions);
end
function testIndependentProjectionAndConditionalLift(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[12;9;15];
rs=RandStream('mt19937ar','Seed',82191);n=size(nav.P,1);B=randn(rs,n);
scale=sqrt(diag(nav.P));nav.P=diag(scale)*(B*B'/n+eye(n))*diag(scale);
[nav,~]=uavmvp.nav_clone(nav,cfg); % singular full prior, retained correlations
obs=struct('marker_id',1,'pixels',uavmvp.camera_model(nav,1,cfg));
model=reference_pose_oracle_model(nav,obs,cfg,'map');
verifyEqual(t,model.basis*model.basis',nav.P(model.indices,model.indices),'AbsTol',1e-12);
verifyEqual(t,model.conditional_lift(model.indices,:),model.basis,'AbsTol',1e-11);
for k=1:5
    eta=.4*randn(rs,model.rank,1);dx=model.conditional_lift*eta;
    full=uavmvp.nav_inject(nav,dx);expected=uavmvp.camera_model(full,1,cfg);
    [cost,detail]=reference_pose_objective(eta,model);
    verifyEqual(t,detail.pixels,expected,'AbsTol',1e-10);
    verifyEqual(t,cost,eta'*eta+sum((obs.pixels-expected).^2),'AbsTol',1e-8);
    % Independent minimum prior cost, including a singular history clone.
    verifyEqual(t,dx'*pinv(nav.P)*dx,eta'*eta,'AbsTol',1e-7);
end
end
function testRankOneMAPMatchesIndependentScalarPosterior(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];nav.P(:)=0;nav.P(1,1)=4;
truth=nav;truth.p(1)=10.6;z=uavmvp.camera_model(truth,1,cfg);
obs=struct('marker_id',1,'detected',true,'pixels',z);model=reference_pose_oracle_model(nav,obs,cfg);
verifyEqual(t,model.rank,1);result=reference_pose_multistart(model,[0,-2,2]);
verifyEqual(t,result.converged_count,3);
[~,H]=uavmvp.camera_model(nav,1,cfg);prediction=uavmvp.camera_model(nav,1,cfg);
delta=(1/4+H(:,1)'*H(:,1))\(H(:,1)'*(z-prediction));
for k=1:3,verifyEqual(t,result.runs(k).pose.position(1),10+delta,'AbsTol',2e-6);end
end
function testMAPOracleIncludesPoseAndMapUncertainty(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[12;9;15];
obs=struct('marker_id',1,'detected',true,'pixels',uavmvp.camera_model(nav,1,cfg)+[.3;-.2;.1;.4;-.3;-.2;-.1;.2]);
model=reference_pose_oracle_model(nav,obs,cfg);verifyEqual(t,model.rank,12);
[updated,report]=uavmvp.nav_update_reference(nav,obs,cfg);verifyTrue(t,report.accepted);
oracle=reference_pose_multistart(model,zeros(12,1));verifyEqual(t,oracle.converged_count,1);
best=oracle.runs(oracle.best_index);
verifyTrue(t,best.refinement.stationary);
verifyGreaterThan(t,min(best.refinement.hessian_eigenvalues),0);
verifyEqual(t,best.cost,report.final_cost,'AbsTol',1e-7);
verifyLessThan(t,norm(best.pose.position-updated.p),2e-5);
verifyLessThan(t,norm(best.pose.marker_position-updated.map_p(:,1)),2e-5);
end
function testCollapsedSimplexRequiresStationarity(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[14;13;15];
obs=struct('marker_id',1,'pixels',uavmvp.camera_model(nav,1,cfg)+[1;-2;.5;1;-.8;.2;-1;1]);
model=reference_pose_oracle_model(nav,obs,cfg);initial=ones(model.rank,1)*.2;
[x,value,check]=reference_pose_refine(initial,model);
verifyTrue(t,check.qualified);verifyLessThan(t,value,reference_pose_objective(initial,model));
% A distinct short second run must keep the stationary solution unchanged.
[again,cost,second]=reference_pose_refine(x,model);
verifyTrue(t,second.qualified);verifyEqual(t,again,x,'AbsTol',1e-9);
verifyEqual(t,cost,value,'AbsTol',1e-10);
end
