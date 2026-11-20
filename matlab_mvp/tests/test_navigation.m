function tests = test_navigation
tests = functiontests(localfunctions);
end

function testRotationAndRightJacobian(t)
rs=RandStream('mt19937ar','Seed',4101);
for k=1:30
    d=randn(rs,3,1); d=d/norm(d)*(2.8*k/30);
    R=uavmvp.SO3.exp(d);
    verifyLessThan(t,norm(R'*R-eye(3),'fro'),2e-14);
    verifyLessThan(t,norm(uavmvp.SO3.log(R)-d),2e-13);
    J=zeros(3); h=1e-7;
    for j=1:3
        e=zeros(3,1);e(j)=h;
        J(:,j)=(uavmvp.SO3.log(R'*uavmvp.SO3.exp(d+e))- ...
            uavmvp.SO3.log(R'*uavmvp.SO3.exp(d-e)))/(2*h);
    end
    verifyLessThan(t,max(abs(J-uavmvp.SO3.rightJacobian(d)),[],'all'),1e-8);
end
end

function testNearZeroAndPi(t)
axis=[1;2;-3];axis=axis/norm(axis);
for angle=[0,1e-10,pi-1e-7,pi]
    R=uavmvp.SO3.exp(axis*angle);
    verifyLessThan(t,norm(uavmvp.SO3.exp(uavmvp.SO3.log(R))-R,'fro'),1e-8);
end
end

function testStationaryGravity(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);
nav.R=uavmvp.SO3.exp([0.12;-0.08;0.4]);
initial=nav; imu.acc=-nav.R'*cfg.sim.gravity;imu.gyro=zeros(3,1);
for k=1:1000
    nav=uavmvp.nav_predict(nav,imu,0.01,cfg);
end
verifyLessThan(t,norm(nav.p-initial.p),1e-10);
verifyLessThan(t,norm(nav.v),1e-11);
verifyGreaterThan(t,trace(nav.P(1:3,1:3)),trace(initial.P(1:3,1:3)));
verifyLessThan(t,abs(nav.t-10),1e-10);
end

function testConstantAcceleration(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);
nav.R=uavmvp.SO3.exp([0;0;.7]);p0=nav.p;
a=[.3;-.2;.1];imu.acc=nav.R'*(a-cfg.sim.gravity);imu.gyro=zeros(3,1);
for k=1:200
    nav=uavmvp.nominal_step(nav,imu,0.01,cfg.sim.gravity);
end
verifyLessThan(t,norm(nav.p-p0-.5*a*4),1e-10);
verifyLessThan(t,norm(nav.v-2*a),1e-11);
end

function testDiscreteImuJacobian(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);
nav.R=uavmvp.SO3.exp([.13;-.21;.4]);nav.v=[1;-.5;.2];
nav.ba=[.01;-.02;.03];nav.bg=[.002;-.003;.001];
imu.acc=[.6;-.3;9.7];imu.gyro=[.2;-.3;.5];dt=.03;
[reference,Phi]=uavmvp.nominal_step(nav,imu,dt,cfg.sim.gravity);
J=zeros(15);h=1e-6;
for k=1:15
    d=zeros(size(nav.P,1),1);d(k)=h;
    plus=uavmvp.nav_inject(nav,d);minus=uavmvp.nav_inject(nav,-d);
    plus=uavmvp.nominal_step(plus,imu,dt,cfg.sim.gravity);
    minus=uavmvp.nominal_step(minus,imu,dt,cfg.sim.gravity);
    J(:,k)=(stateError(reference,plus)-stateError(reference,minus))/(2*h);
end
verifyLessThan(t,max(abs(J-Phi),[],'all'),2e-8);
end

function testProcessCovarianceAgainstVanLoan(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);
nav.R=uavmvp.SO3.exp([.1;-.2;.3]);
imu.acc=[1;2;9];imu.gyro=[.3;-.2;.1];dt=.01;
[~,~,F]=uavmvp.nominal_step(nav,imu,dt,cfg.sim.gravity);
n=cfg.navigation_candidate;
Qc=diag([zeros(1,3),repmat(n.acc_noise_density_mps2_per_sqrt_hz^2,1,3), ...
    repmat(n.gyro_noise_density_radps_per_sqrt_hz^2,1,3), ...
    repmat(n.acc_bias_random_walk_mps2_per_sqrt_s^2,1,3), ...
    repmat(n.gyro_bias_random_walk_radps_per_sqrt_s^2,1,3)]);
E=expm([F,Qc;zeros(15),-F']*dt);
exact=E(1:15,16:30)*E(1:15,1:15)';
actual=uavmvp.process_noise(F,dt,cfg);
verifyLessThan(t,norm(actual-exact,'fro')/norm(exact,'fro'),1e-7);
verifyGreaterThanOrEqual(t,min(eig(actual)),-1e-18);
verifyGreaterThan(t,actual(1,1),0);
end

function testCameraJacobian(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);
nav.p=[11;9;15];nav.R=uavmvp.SO3.exp([.02;-.03;.1]);
nav.map_R(:,:,1)=uavmvp.SO3.exp([.01;.02;-.04]);
[~,H,visible]=uavmvp.camera_model(nav,1,cfg);
verifyTrue(t,visible);J=zeros(size(H));h=1e-6;
for k=1:size(nav.P,1)
    d=zeros(size(nav.P,1),1);d(k)=h;
    plus=uavmvp.nav_inject(nav,d);minus=uavmvp.nav_inject(nav,-d);
    zp=uavmvp.camera_model(plus,1,cfg);zm=uavmvp.camera_model(minus,1,cfg);
    J(:,k)=(zp-zm)/(2*h);
end
verifyLessThan(t,max(abs(J-H),[],'all'),2e-6);
end

function testReferenceVisibility(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];
[~,~,visible]=uavmvp.camera_model(nav,1,cfg);verifyTrue(t,visible);
nav.p=[60;40;15];[~,~,visible]=uavmvp.camera_model(nav,1,cfg);verifyFalse(t,visible);
nav.p=[10;10;40];[~,~,visible]=uavmvp.camera_model(nav,1,cfg);verifyFalse(t,visible);
nav.p=[10;10;15];nav.R=uavmvp.SO3.exp([pi;0;0]);
[~,~,visible]=uavmvp.camera_model(nav,1,cfg);verifyFalse(t,visible);
end

function testNoObservationAndOutlierDoNotUpdate(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];
[out,report]=uavmvp.nav_update_reference(nav,[],cfg);
verifyEqual(t,out,nav);verifyFalse(t,report.accepted);
obs=struct('marker_id',1,'detected',true, ...
    'pixels',uavmvp.camera_model(nav,1,cfg)+10000);
[out,report]=uavmvp.nav_update_reference(nav,obs,cfg);
verifyEqual(t,out,nav);verifyEqual(t,report.reason,'INNOVATION_GATE');
obs.detected=false;[out,report]=uavmvp.nav_update_reference(nav,obs,cfg);
verifyEqual(t,out,nav);verifyFalse(t,report.accepted);
end

function testAntennaCloneJacobian(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);
nav.R=uavmvp.SO3.exp([.2;.3;-.4]);
l=cfg.navigation_candidate.antenna_lever_arm_body_m(:);
J=zeros(3,size(nav.P,1));h=1e-6;
for k=1:size(nav.P,1)
    d=zeros(size(nav.P,1),1);d(k)=h;
    plus=uavmvp.nav_inject(nav,d);minus=uavmvp.nav_inject(nav,-d);
    J(:,k)=((plus.p+plus.R*l)-(minus.p+minus.R*l))/(2*h);
end
[out,id,raw]=uavmvp.nav_clone(nav,cfg);
verifyEqual(t,id,1);verifyLessThan(t,norm(raw.covariance-J*nav.P*J','fro'),1e-10);
verifyLessThan(t,norm(out.P(1:end-3,end-2:end)-nav.P*J','fro'),1e-10);
verifyGreaterThan(t,min(eig(out.P)),-1e-12);
end

function testReferenceUpdatesHistoryAndVersion(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];
[nav,id,raw]=uavmvp.nav_clone(nav,cfg);
truth=nav;truth.p(1)=truth.p(1)+.10;
obs=struct('marker_id',1,'detected',true,'pixels',uavmvp.camera_model(truth,1,cfg));
[updated,report]=uavmvp.nav_update_reference(nav,obs,cfg);
verifyTrue(t,report.accepted);
[position,P,version]=uavmvp.nav_archive(updated,id);
verifyEqual(t,version,1);verifyEqual(t,raw.version,0);
verifyGreaterThan(t,norm(position-raw.position),0);
verifyLessThan(t,trace(P),trace(raw.covariance));
verifyGreaterThan(t,min(eig(updated.P)),-1e-12);
end

function testPersistentMapUncertaintyFloor(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];
obs=struct('marker_id',1,'detected',true,'pixels',uavmvp.camera_model(nav,1,cfg));
lower=1/(1/cfg.navigation_candidate.initial_position_std_m^2+ ...
    1/cfg.navigation_candidate.survey_translation_std_m^2);
for k=1:150
    [nav,r]=uavmvp.nav_update_reference(nav,obs,cfg);
    verifyTrue(t,r.accepted);
end
verifyGreaterThanOrEqual(t,nav.P(1,1),lower*(1-1e-8));
verifyGreaterThanOrEqual(t,nav.P(2,2),lower*(1-1e-8));
end

function testPruningIsMarginalization(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);
[nav,id1]=uavmvp.nav_clone(nav,cfg);nav.p(1)=nav.p(1)+1;
[nav,id2]=uavmvp.nav_clone(nav,cfg);
[p,P]=uavmvp.nav_archive(nav,id2);oldCount=size(nav.P,1);
nav=uavmvp.nav_prune(nav,id2);[after,Q]=uavmvp.nav_archive(nav,id2);
verifyEqual(t,p,after);verifyEqual(t,P,Q);
verifyEqual(t,size(nav.P,1),oldCount-3);
verifyFalse(t,ismember(id1,nav.clone_ids));
end

function testResetJacobian(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);d=zeros(size(nav.P,1),1);
d(7:9)=[.1;-.2;.3];d(19:21)=[-.2;.1;.05];
[ref,G]=uavmvp.nav_inject(nav,d);
% Map coordinates reach 110 m; 1e-7 m differences amplify subtraction roundoff.
h=1e-5;J=zeros(size(G));
for k=1:numel(d)
    e=zeros(size(d));e(k)=h;
    plus=uavmvp.nav_inject(nav,d+e);minus=uavmvp.nav_inject(nav,d-e);
    J(:,k)=(fullError(ref,plus)-fullError(ref,minus))/(2*h);
end
verifyLessThan(t,max(abs(J-G),[],'all'),2e-8);
end

function e=stateError(a,b)
e=[b.p-a.p;b.v-a.v;uavmvp.SO3.log(a.R'*b.R);b.ba-a.ba;b.bg-a.bg];
end

function e=fullError(a,b)
e=stateError(a,b);
for j=1:size(a.map_p,2)
    e=[e;b.map_p(:,j)-a.map_p(:,j); ...
        uavmvp.SO3.log(a.map_R(:,:,j)'*b.map_R(:,:,j))]; %#ok<AGROW>
end
e=[e;reshape(b.clones-a.clones,[],1)];
end
