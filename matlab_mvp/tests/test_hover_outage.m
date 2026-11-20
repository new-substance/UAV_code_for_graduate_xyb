function tests=test_hover_outage
tests=functiontests(localfunctions);
end
function testClosedFormVariancePowers(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);n=cfg.navigation_candidate;g=norm(cfg.sim.gravity);
for T=[0,1,15,30,60]
    q=hover_outage_covariance(nav,T,cfg);
    vertical=n.initial_position_std_m^2+T^2*n.initial_velocity_std_mps^2+ ...
        T^4*n.initial_acc_bias_std_mps2^2/4+ ...
        n.acc_noise_density_mps2_per_sqrt_hz^2*T^3/3+ ...
        n.acc_bias_random_walk_mps2_per_sqrt_s^2*T^5/20;
    horizontal=vertical+g^2*(deg2rad(n.initial_roll_pitch_std_deg)^2*T^4/4+ ...
        n.initial_gyro_bias_std_radps^2*T^6/36+ ...
        n.gyro_noise_density_radps_per_sqrt_hz^2*T^5/20+ ...
        n.gyro_bias_random_walk_radps_per_sqrt_s^2*T^7/252);
    verifyEqual(t,diag(q.P(1:3,1:3)),[horizontal;horizontal;vertical], ...
        'AbsTol',1e-10*max(1,horizontal));
end
end
function testFullCorrelatedCovarianceAndSingularClone(t)
cfg=uavmvp.config();nav=uavmvp.nav_init(cfg);nav.p=[10;10;15];
nav.R=uavmvp.SO3.exp([.2;-.1;.7]);rs=RandStream('mt19937ar','Seed',84200);
B=randn(rs,size(nav.P,1));s=sqrt(diag(nav.P));nav.P=diag(s)*(B*B'/size(B,1)+eye(size(B)))*diag(s);
[nav,~]=uavmvp.nav_clone(nav,cfg);T=7;q=hover_outage_covariance(nav,T,cfg);
% Independent continuous integral via the Van Loan block exponential.
M=expm([q.F,q.Qc;zeros(15),-q.F']*T);Phi=M(1:15,1:15);Q=M(1:15,16:30)*Phi';
E=eye(size(nav.P));E(1:15,1:15)=Phi;expected=E*nav.P*E';expected(1:15,1:15)=expected(1:15,1:15)+Q;
verifyEqual(t,q.P,expected,'AbsTol',1e-10);
verifyEqual(t,q.P(16:end,16:end),nav.P(16:end,16:end),'AbsTol',1e-13);
end
function testDiscreteMidpointGyroBiasTerm(t)
cfg=uavmvp.config();cfg.navigation_candidate.acc_noise_density_mps2_per_sqrt_hz=0;
cfg.navigation_candidate.gyro_noise_density_radps_per_sqrt_hz=0;
cfg.navigation_candidate.acc_bias_random_walk_mps2_per_sqrt_s=0;
cfg.navigation_candidate.gyro_bias_random_walk_radps_per_sqrt_s=0;
initial=uavmvp.nav_init(cfg);initial.P(:)=0;initial.P(14,14)=1;initial.p=[10;10;15];
T=5;ref=hover_outage_covariance(initial,T,cfg);A=ref.F(4:6,7:9);
imu=struct('acc',-cfg.sim.gravity,'gyro',zeros(3,1));
for h=[.1,.05,.01]
    nav=initial;for k=1:round(T/h),nav=uavmvp.nav_predict(nav,imu,h,cfg);end
    % Exact n-step derivative of this midpoint integrator differs from the
    % continuous transition by -A*T*h^2/12 in its position/gyro-bias block.
    Phi=ref.Phi;Phi(1:3,13:15)=Phi(1:3,13:15)-A*T*h^2/12;
    expected=Phi*initial.P(1:15,1:15)*Phi';
    verifyEqual(t,nav.P(1:15,1:15),expected,'AbsTol',1e-8);
end
end
