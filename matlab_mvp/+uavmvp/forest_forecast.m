function out=forest_forecast(action,in,c)
%FOREST_FORECAST N01/I01-e: no future reference updates, truth or packet draws.
% profile is a finite declared nominal model, not future measured IMU.
switch action
    case 'linear_window'
        out=linear_window(in.P,in.F,in.W,in.duration_s);
    case 'profile'
        n=in.navigation;assert(strcmp(n.model_id,c.model_id));P=n.P;
        clock=tic;enforce=true;
        if isfield(in,'execution_kind')
            assert(strcmp(in.execution_kind,'fixed_input_review'));enforce=false;
        end
        intervalLimit=ceil(c.service.reference_max_age_s/c.initialization.imu_sample_interval_s)+c.algorithm.max_candidate_actions+1;
        assert(~isempty(in.profile) && numel(in.profile)<=intervalLimit);
        upper=zeros(1,4);duration=0;
        for k=1:numel(in.profile)
            if enforce && toc(clock)>c.service.computation_bound_s
                error('forest:ComputationTimeout','Navigation forecast exceeded its declared computation budget.');
            end
            a=in.profile(k);R=a.R;f=a.specific_force_body_mps2(:);w=a.body_rate_radps(:);dt=a.duration_s;
            assert(isequal(size(R),[3 3]) && all(isfinite(R(:))) && norm(R'*R-eye(3),'fro')<1e-8 && abs(det(R)-1)<1e-8);
            assert(all(isfinite([f;w])) && numel(f)==3 && numel(w)==3);
            assert(dt>0 && dt<=c.initialization.imu_sample_interval_s*(1+1e-6));
            assert(norm(w)<=c.motion_model.body_rate_limit_rad_per_s*(1+1e-8) && ...
                norm(f)<=c.motion_model.maximum_normalized_thrust_m_per_s2*(1+1e-8),'forest:ForecastDomain','Nominal model exceeds declared actuation.');
            duration=duration+dt;
            assert(duration<=c.service.reference_max_age_s+1e-8,'forest:ForecastBudget','Forecast horizon exceeds registered reference window.');
            F=zeros(15);F(1:3,4:6)=eye(3);F(4:6,7:9)=-R*uavmvp.SO3.skew(f);
            F(4:6,10:12)=-R;F(7:9,7:9)=-uavmvp.SO3.skew(w);F(7:9,13:15)=-eye(3);
            qa=(c.imu.white_acc_asd_micro_g_per_sqrt_hz(:)*1e-6*9.80665).^2/2;
            qg=deg2rad(c.imu.white_gyro_asd_deg_per_s_per_sqrt_hz)^2/2;
            W=blkdiag(zeros(3),R*diag(qa)*R',qg*eye(3), ...
                c.imu.acc_bias_random_walk_m_per_s2_per_sqrt_s^2*eye(3), ...
                deg2rad(c.imu.gyro_bias_random_walk_deg_per_s_per_sqrt_s)^2*eye(3));
            b=linear_window(P(1:15,1:15),F,W,dt);upper=max(upper,b.std_upper);
            P(1:15,16:end)=b.Phi*P(1:15,16:end);P(16:end,1:15)=P(1:15,16:end)';
            P(1:15,1:15)=b.P_end;
        end
        cap=c.navigation_admission;
        if enforce && toc(clock)>c.service.computation_bound_s
            error('forest:ComputationTimeout','Navigation forecast exceeded its declared computation budget.');
        end
        ratios=upper./[cap.horizontal_max_std_m,cap.vertical_std_m,cap.velocity_max_std_m_per_s,deg2rad(cap.attitude_max_std_deg)];
        out=struct('version',n.version,'start_time_s',n.t,'horizon_s',duration,'std_upper',upper, ...
            'navigation_ratio_upper',max(ratios),'navigation_ok',all(ratios<=1), ...
            'predicted_covariance',P,'future_reference_updates',0, ...
            'scope','piecewise_frozen_N01_nominal_profile_not_actual_flight_guarantee');
    otherwise,error('forest:Action','Use profile or linear_window.');
end
end

function out=linear_window(P,F,W,t)
P=uavmvp.forest_covariance(P);W=uavmvp.forest_covariance(W);
assert(isequal(size(P),[15 15]) && isequal(size(F),[15 15]) && isequal(size(W),[15 15]));
assert(isfinite(t) && t>=0 && all(isfinite(F(:))));
E=expm([F W;zeros(15) -F']*t);Phi=E(1:15,1:15);Q=E(1:15,16:30)*Phi';
Pend=uavmvp.forest_covariance(Phi*P*Phi'+(Q+Q')/2);
% For 0<=s<=t: |exp(Fs)-I| <= exp(|F|t)-I, entrywise.
% The nonnegative comparison integral also bounds |Q(s)| entrywise.
M=abs(F);E=expm([M abs(W);zeros(15) -M']*t);
A=E(1:15,1:15);B=E(1:15,16:30)*A';B=(B+B')/2;
assert(min(B(:))>=-1e-9*max(1,norm(B,'fro')),'forest:ForecastNumerics','Invalid comparison integral.');
B=max(B,0);[V,D]=eig(P);root=V*diag(sqrt(max(0,diag(D))))*V';
change=max(A-eye(15),0)*abs(root);
blocks={1:2,3,4:6,7:9};upper=zeros(1,4);
for k=1:4
    j=blocks{k};base=sqrt(max(0,max(eig(P(j,j)))));
    drift=norm(change(j,:),2);noise=max(sum(B(j,j),2));
    upper(k)=sqrt((base+drift)^2+noise);
end
out=struct('P_end',Pend,'Phi',Phi,'std_upper',upper,'scope','whole_frozen_linear_interval');
end
