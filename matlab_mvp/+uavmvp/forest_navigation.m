function varargout=forest_navigation(action,varargin)
%FOREST_NAVIGATION I01/N01/N02/N03. Joint RF reference state, no unknown update.
% Layout: [p v right-theta ba bg; survey(13); device; links(5); home(3); clones].
switch action
    case 'init'
        [varargout{1:nargout}]=initialize(varargin{:});
    case 'predict'
        n=varargin{1};imu=varargin{2};dt=varargin{3};c=varargin{4};
        [next,~,F]=uavmvp.nominal_step(n,imu,dt,[0;0;-9.80665]);
        Rm=n.R*uavmvp.SO3.exp((imu.gyro(:)-n.bg)*dt/2);
        qa=(c.imu.white_acc_asd_micro_g_per_sqrt_hz(:)*1e-6*9.80665).^2/2;
        qg=deg2rad(c.imu.white_gyro_asd_deg_per_s_per_sqrt_hz)^2/2;
        W=blkdiag(zeros(3),Rm*diag(qa)*Rm',qg*eye(3), ...
            c.imu.acc_bias_random_walk_m_per_s2_per_sqrt_s^2*eye(3), ...
            deg2rad(c.imu.gyro_bias_random_walk_deg_per_s_per_sqrt_s)^2*eye(3));
        E=expm([F,W;zeros(15),-F']*dt);Phi=E(1:15,1:15);Q=E(1:15,16:30)*Phi';
        Q=uavmvp.forest_covariance((Q+Q')/2);
        next.P=n.P;next.P(1:15,1:15)=Phi*n.P(1:15,1:15)*Phi'+Q;
        next.P(1:15,16:end)=Phi*n.P(1:15,16:end);
        next.P(16:end,1:15)=next.P(1:15,16:end)';next.P=(next.P+next.P')/2;
        % Initial/update/clone P is validated; this PSD map preserves the static block.
        assert(all(isfinite(next.P(:))),'forest:Covariance','Nonfinite propagation.');varargout={next};
    case 'reference'
        [varargout{1:nargout}]=reference_update(varargin{:});
    case 'antenna'
        n=varargin{1};c=varargin{2};J=zeros(3,size(n.P,1));
        J(:,1:3)=eye(3);J(:,7:9)=-n.R*uavmvp.SO3.skew(c.scene.uav_antenna_lever_body_m);
        varargout={n.p+n.R*c.scene.uav_antenna_lever_body_m(:),J};
    case 'clone'
        n=varargin{1};id=string(varargin{2});c=varargin{3};
        assert(~any(n.clone_ids==id),'forest:CloneID','Duplicate clone epoch ID.');
        assert(numel(n.clone_ids)<c.algorithm.max_position_clones,'forest:CloneBudget','Prune unused clones before adding.');
        [p,J]=uavmvp.forest_navigation('antenna',n,c);B=n.P*J';
        n.P=uavmvp.forest_covariance([n.P B;B' J*B]);n.clones(:,end+1)=p;
        n.clone_ids(end+1)=id;n.clone_times(end+1)=n.t;varargout={n};
    case 'prune'
        n=varargin{1};keep=string(varargin{2});
        assert(all(ismember(keep,n.clone_ids)) && numel(unique(keep))==numel(keep));
        [~,k]=ismember(keep,n.clone_ids);idx=[1:n.base_dimension,reshape(n.base_dimension+3*k(:)'-2+(0:2)',1,[])];
        n.P=n.P(idx,idx);n.clones=n.clones(:,k);n.clone_ids=keep(:)';n.clone_times=n.clone_times(k);varargout={n};
    case 'gate'
        n=varargin{1};c=varargin{2};P=uavmvp.forest_covariance(n.P);a=c.navigation_admission;
        ratios=[sqrt(max(eig(P(1:2,1:2))))/a.horizontal_max_std_m, ...
            sqrt(max(0,P(3,3)))/a.vertical_std_m, ...
            sqrt(max(eig(P(4:6,4:6))))/a.velocity_max_std_m_per_s, ...
            sqrt(max(eig(P(7:9,7:9))))/deg2rad(a.attitude_max_std_deg)];
        varargout={struct('ok',all(ratios<=1),'ratio',max(ratios),'version',n.version)};
    otherwise,error('forest:Action','Unknown navigation action.');
end
end

function n=initialize(samples,c)
% I01: stationary interval averages; no independent tilt/bias reset.
assert(size(samples.acc,1)==3 && isequal(size(samples.acc),size(samples.gyro)) && ...
    all(isfinite([samples.acc(:);samples.gyro(:)])) && samples.stationary_confirmed);
N=size(samples.acc,2);T=N*c.initialization.imu_sample_interval_s;
assert(T>=c.initialization.stationary_duration_s && isfinite(samples.yaw_rad));
ba=samples.acc_bias_prior_mean_mps2(:);f=mean(samples.acc,2)-ba;
r=hypot(f(2),f(3));F=norm(f);assert(F>1 && r/F>.05,'forest:Alignment','Degenerate alignment.');
phi=atan2(f(2),f(3));theta=atan2(-f(1),r);
R=uavmvp.SO3.exp([0;0;samples.yaw_rad])*uavmvp.SO3.exp([0;theta;0])*uavmvp.SO3.exp([phi;0;0]);
qa=(c.imu.white_acc_asd_micro_g_per_sqrt_hz(:)*1e-6*9.80665).^2/2;
qg=deg2rad(c.imu.white_gyro_asd_deg_per_s_per_sqrt_hz)^2/2;
% Primitives: ba error, average accel noise, average gyro noise, yaw, velocity.
S=blkdiag(c.initialization.acc_calibration_prior_std_m_per_s2^2*eye(3), ...
    diag(qa)/T,qg/T*eye(3),deg2rad(c.initialization.external_yaw_std_deg)^2, ...
    c.initialization.initial_relative_velocity_std_m_per_s^2*eye(3));
Jang=[0,f(3)/r^2,-f(2)/r^2;-r/F^2,f(1)*f(2)/(r*F^2),f(1)*f(3)/(r*F^2)];
B=[1 0 -sin(theta);0 cos(phi) sin(phi)*cos(theta);0 -sin(phi) cos(phi)*cos(theta)];
J=zeros(15,13);J(7:9,1:3)=-B(:,1:2)*Jang;J(7:9,4:6)=J(7:9,1:3);
J(7:9,10)=B(:,3);J(10:12,1:3)=eye(3);J(13:15,7:9)=-eye(3);J(4:6,11:13)=eye(3);
s=c.scene;sig=[repmat(s.outer_reference_local_xy_std_m,1,8), ...
    repmat(s.home_local_xy_std_m,1,2),s.common_terminal_plane_relative_height_std_m, ...
    repmat(s.home_reference_relative_xy_std_m,1,2)];
P=blkdiag(J*S*J',diag(sig.^2),c.radio.shared_device_range_bias_std_m^2, ...
    c.radio.reference_persistent_link_std_m^2*eye(5), ...
    diag(s.common_map_translation_std_m^2+[s.home_local_xy_std_m^2,s.home_local_xy_std_m^2,0]));
% Home local XY also occurs in outer-reference relative registration.
P(35:36,24:25)=s.home_local_xy_std_m^2*eye(2);P(24:25,35:36)=P(35:36,24:25)';
n=struct('p',zeros(3,1),'v',zeros(3,1),'R',R,'ba',ba,'bg',mean(samples.gyro,2), ...
    't',0,'P',uavmvp.forest_covariance(P),'base_dimension',37,'survey',zeros(13,1), ...
    'device_bias_m',0,'link_bias_m',zeros(5,1),'home',s.home_imu_xyz_m(:), ...
    'clones',zeros(3,0),'clone_ids',strings(1,0),'clone_times',zeros(1,0), ...
    'reference_ids',strings(1,0),'version',0,'last_reference_s',-Inf,'model_id',c.model_id);
end

function [n,info]=reference_update(n,b,c)
assert(strcmp(b.role,'reference'),'forest:Role','Unknown ranges must not update navigation.');
ids=string(b.raw_ids(:));assert(numel(unique(ids))==numel(ids) && ~any(ismember(ids,n.reference_ids)),'forest:Duplicate','Repeated reference pair.');
assert(numel(ids)+numel(n.reference_ids)<=c.algorithm.max_protocol_records,'forest:ProtocolBudget','Reference ID budget exhausted.');
delayed=isfield(b,'direction_clone_ids');
assert(b.epoch_s<=n.t && all(b.reference_indices>=1 & b.reference_indices<=5));
if ~delayed,assert(b.epoch_s==n.t,'forest:Epoch','Noncurrent reference requires actual antenna clones.');end
M=numel(b.values_m);assert(numel(ids)==M && numel(b.reference_indices)==M && all(isfinite(b.values_m(:))));
H=zeros(M,size(n.P,1));h=zeros(M,1);[p,J]=uavmvp.forest_navigation('antenna',n,c);
if delayed
    assert(isequal(size(b.direction_clone_ids),[M 2]));
    [found,cloneIndex]=ismember(string(b.direction_clone_ids),n.clone_ids);
    assert(all(found(:)) && all(n.clone_times(cloneIndex(:))<=b.epoch_s),'forest:Epoch','Missing or future reference pose.');
    times=reshape(n.clone_times(cloneIndex(:)),M,2);
    assert(all(abs(times(:,1)-times(:,2))<=c.radio.pair_whole_attempt_bound_s),'forest:Epoch','Reference pair exceeded attempt bound.');
end
for k=1:M
    j=b.reference_indices(k);A=zeros(3,13);A(3,11)=1;
    if j<=4,A(1,2*j-1)=1;A(2,2*j)=1;A(1,9)=-1;A(2,10)=-1;
    else,A(1,12)=1;A(2,13)=1;end
    ref=c.scene.known_reference_xyz_m(j,:)'-c.scene.home_imu_xyz_m(:)+A*n.survey;
    for direction=1:2
        if delayed
            ci=cloneIndex(k,direction);point=n.clones(:,ci);Jdir=zeros(3,size(n.P,1));Jdir(:,37+3*ci-2+(0:2))=eye(3);
        else
            point=p;Jdir=J;
        end
        v=point-ref;d=norm(v);assert(d>1e-6,'forest:Geometry','Coincident antenna/reference.');u=v'/d;
        w=c.radio.pair_direction_weights(direction);
        H(k,:)=H(k,:)+w*u*Jdir;H(k,16:28)=H(k,16:28)-w*u*A;h(k)=h(k)+w*d;
    end
    H(k,29)=1;H(k,29+j)=1;
    h(k)=h(k)+n.device_bias_m+n.link_bias_m(j)+c.radio.reference_persistent_link_mean_m;
end
Rn=c.radio.reference_pair_white_std_m^2*eye(M);D=zeros(size(n.P,1),M);
if isfield(b,'noise_covariance_m2'),Rn=uavmvp.forest_covariance(b.noise_covariance_m2);end
if isfield(b,'state_noise_cross'),D=b.state_noise_cross;end
assert(isequal(size(Rn),[M M]) && isequal(size(D),[size(n.P,1) M]));
uavmvp.forest_covariance([n.P D;D' Rn]);
S=H*n.P*H'+H*D+D'*H'+Rn;L=chol((S+S')/2,'lower');
B=n.P*H'+D;K=(B/L')/L;dx=K*(b.values_m(:)-h);
P=n.P-K*B';[n,G]=inject(n,dx);n.P=uavmvp.forest_covariance(G*P*G');
n.reference_ids=[n.reference_ids ids'];n.version=n.version+1;
% A single range is not a completed navigation-service block.
gate=uavmvp.forest_navigation('gate',n,c);
if b.block_complete && gate.ok,n.last_reference_s=n.t;end
info=struct('residual_m',b.values_m(:)-h,'H',H,'gain',K,'block_accepted',b.block_complete && gate.ok);
end

function [n,G]=inject(n,dx)
G=eye(numel(dx));n.p=n.p+dx(1:3);n.v=n.v+dx(4:6);n.R=n.R*uavmvp.SO3.exp(dx(7:9));
G(7:9,7:9)=uavmvp.SO3.rightJacobian(dx(7:9));n.ba=n.ba+dx(10:12);n.bg=n.bg+dx(13:15);
n.survey=n.survey+dx(16:28);n.device_bias_m=n.device_bias_m+dx(29);n.link_bias_m=n.link_bias_m+dx(30:34);n.home=n.home+dx(35:37);
if numel(dx)>37,n.clones=n.clones+reshape(dx(38:end),3,[]);end
end
