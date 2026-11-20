function tests=test_priority_aggregation
tests=functiontests(localfunctions);
end
function setupOnce(~),setup_priority();end
function testWhiteAveragesButSharedDeviceAndNavigationDoNot(t)
[records,nav,history,macros,cfg]=fixture();
b=uavpriority.bundle(records,nav,history,macros,2:2:10,cfg);
verifyEqual(t,b.Cradio,2*eye(5)+9*ones(5),'AbsTol',1e-12);
verifyEqual(t,b.Pnavigation(1:3,1:3),b.source_bundle.Pnavigation(1:3,1:3),'AbsTol',1e-12);
verifyEqual(t,sum(b.aggregation_matrix,2),ones(5,1));
end
function testFourServicesRestoreWhiteBudgetWithoutDilutingBias(t)
[records,nav,history,macros,cfg]=fixture(4);
b=uavpriority.bundle(records,nav,history,macros,4:4:20,cfg);
verifyEqual(t,b.Cradio,eye(5)+9*ones(5),'AbsTol',1e-12);
verifyEqual(t,b.Pnavigation(1:3,1:3),b.source_bundle.Pnavigation(1:3,1:3),'AbsTol',1e-12);
end
function testAllHeldoutMembersRemainOutOfFourFit(t)
[records,nav,history,macros,cfg]=fixture();
b=uavpriority.bundle(records,nav,history,macros,2:2:10,cfg);
f=uavpriority.fit_position(uavpriority.subbundle(b,1:4),cfg);
verifyTrue(t,f.quality);check=uavpriority.verify_position(f,b,cfg);verifyTrue(t,check.passed);
old=f;records{9}.range_m=records{9}.range_m+200;
changed=uavpriority.bundle(records,nav,history,macros,2:2:10,cfg);
again=uavpriority.fit_position(uavpriority.subbundle(changed,1:4),cfg);
verifyEqual(t,again.x,old.x,'AbsTol',1e-10);
c=uavpriority.verify_position(old,changed,cfg);verifyFalse(t,c.passed);
verifyEmpty(t,intersect([b.constituent_ids{1:4}],b.constituent_ids{5}));
end
function testDatumAllowanceDoesNotChangeLikelihoodEstimator(t)
[records,nav,history,macros,cfg]=fixture();
b=uavpriority.bundle(records,nav,history,macros,2:2:8,cfg);
f=uavpriority.fit_position(b,cfg);bare=cfg;bare.priority.datum_translation_bound_m=0;
g=uavpriority.fit_position(b,bare);
verifyEqual(t,f.x,g.x,'AbsTol',1e-10);
verifyEqual(t,f.radius-g.radius,2,'AbsTol',1e-8);
end
function testAggregateMembersCannotOverlap(t)
[records,nav,history,macros,cfg]=fixture();macros{2}.constituent_ids=[2,3];
verifyError(t,@()uavpriority.bundle(records,nav,history,macros,[2,4],cfg),'uavpriority:overlap');
end
function testDeclaredTimestampEnvelopeIsPositiveSemidefinite(t)
c=uavpriority.config();times=[0,.2];tau=[times;times+1e-7;times+.0100001;times+.0100002];
a=uavmvp.rtt_solve(tau,1,0,[],(5/299792458)^2*eye(8));
b=uavmvp.rtt_solve(tau,1,0,[],(10/299792458)^2*eye(8));
verifyEqual(t,b.joint_covariance,4*a.joint_covariance,'RelTol',1e-12);
verifyGreaterThanOrEqual(t,min(eig(b.joint_covariance-a.joint_covariance)),-1e-15);
verifyLessThan(t,sqrt(b.joint_covariance(1,1)+c.ranging.device_persistent_bias_std_m^2),8);
end
function testFitPoolReusesAnExistingEstimate(t)
[records,nav,history,macros,cfg]=fixture();
[f,ids,~]=uavpriority.fit_pool(records,nav,history,macros,2:2:10,10,cfg,struct('x',[30;30]));
verifyEqual(t,numel(ids),4);verifyTrue(t,ismember(10,ids));
verifyTrue(t,isfinite(f.objective));verifyEqual(t,f.nav_version,nav.version);
end
function [records,nav,history,macros,cfg]=fixture(repeats)
if nargin<1,repeats=2;end
cfg=uavpriority.config();cfg.priority.cold_services_per_position=repeats;
nav=uavmvp.nav_init(cfg);nav.v(:)=0;
centers=[30,10;10,30;50,10;10,50;10,10];records={};macros={};
for k=1:5
    children=[];
    for repetition=1:repeats
        nav.p=[centers(k,:)';15];[nav,id,raw]=uavmvp.nav_clone(nav,cfg);
        index=numel(records)+1;children(end+1)=index; %#ok<AGROW>
        records{index}=struct('effective_nav_snapshot_id',id,'radio_history_id',index, ...
            'range_m',norm(raw.position-[30;30;0]),'short_stop_bias_bound_m',0); %#ok<AGROW>
    end
    macros{k}=struct('id',children(end),'constituent_ids',children);
end
n=5*repeats;history=uavmvp.radio_history_init(cfg);history.last_id=n;
history.P=zeros(n+2);history.P(3:end,3:end)=4*eye(n)+9*ones(n);
end
