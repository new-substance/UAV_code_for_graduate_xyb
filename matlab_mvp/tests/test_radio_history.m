function tests=test_radio_history
tests=functiontests(localfunctions);
end

function testColdWarmDriftCrossCovarianceAgainstLatentMatrix(t)
cfg=uavmvp.config();h=uavmvp.radio_history_init(cfg);
L=[3,0;1e-8,1e-7];cold=sampleOut('cold');cold.joint_covariance=L*L';
[h,id1]=uavmvp.radio_history_append(h,cold,1,cfg,'cold');
warm=sampleOut('warm');warm.kappa=cold.kappa;
[h,id2]=uavmvp.radio_history_append(h,warm,6,cfg,'warm');
[h,id3]=uavmvp.radio_history_append(h,warm,11,cfg,'warm');
% Independent latent errors: bias, cold noise 1/2, drift 1/2, warm noise 1/2.
b=cfg.ranging.device_persistent_bias_std_m;
q=cfg.ranging.clock_ratio_random_walk_ppm_per_sqrt_s*1e-6*sqrt(5);
s=warm.kappa_sensitivity;w=sqrt(warm.joint_covariance(1,1));
A=[b,L(1,:),0,0,0,0; b,s*L(2,:),-s*q,0,w,0; b,s*L(2,:),-s*q,-s*q,0,w];
C=uavmvp.radio_history_covariance(h,[id1,id2,id3]);
verifyLessThan(t,norm(C-A*A','fro'),1e-10);
rs=RandStream('mt19937ar','Seed',7401);samples=A*randn(rs,7,30000);
empirical=cov(samples');scale=sqrt(diag(C));
verifyLessThan(t,max(abs((empirical-C)./(scale*scale')),[],'all'),.04);
end

function testFreshCalibrationRetainsOldRangesButResetsClock(t)
cfg=uavmvp.config();h=uavmvp.radio_history_init(cfg);cold=sampleOut('cold');
cold.joint_covariance=[9,1e-7;1e-7,2e-14];
[h,id1]=uavmvp.radio_history_append(h,cold,1,cfg,'cold');
warm=sampleOut('warm');[h,id2]=uavmvp.radio_history_append(h,warm,6,cfg,'warm');
old=uavmvp.radio_history_covariance(h,[id1,id2]);
[h,id3]=uavmvp.radio_history_append(h,cold,40,cfg,'cold');
C=uavmvp.radio_history_covariance(h,[id1,id2,id3]);
verifyEqual(t,C(1:2,1:2),old,'AbsTol',1e-12);
verifyEqual(t,C(1:2,3),cfg.ranging.device_persistent_bias_std_m^2*ones(2,1),'AbsTol',1e-12);
verifyEqual(t,h.P(2,3:4),[0,0],'AbsTol',1e-20);
verifyGreaterThanOrEqual(t,min(eig(h.P)),-1e-10);
end

function testExpiredClockIsRejected(t)
cfg=uavmvp.config();h=uavmvp.radio_history_init(cfg);
[h,~]=uavmvp.radio_history_append(h,sampleOut('cold'),1,cfg,'cold');
verifyError(t,@()uavmvp.radio_history_append(h,sampleOut('warm'),32,cfg,'warm'), ...
    'uavmvp:staleCalibration');
end

function out=sampleOut(kind)
out=struct('range_m',30,'kappa',1,'kappa_sensitivity',-9e6, ...
    'joint_covariance',diag([25,1e-14]),'exchange_count',1);
if strcmp(kind,'cold'),out.exchange_count=2;end
end
