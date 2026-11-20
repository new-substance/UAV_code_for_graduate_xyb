function [nav,report] = nav_update_reference(nav,obs,cfg)
%NAV_UPDATE_REFERENCE Iterated prior-coordinate MAP; one covariance update.
% Whitened coordinates support PSD priors, including exact history clones.
report = struct('accepted',false,'reason','NO_OBSERVATION','nis',NaN, ...
    'iterations',0,'initial_cost',NaN,'final_cost',NaN,'pixel_residual_norm',NaN);
if isempty(obs), return; end
if ~isfield(obs,'detected') || ~obs.detected, return; end
[pred,H,~,~] = uavmvp.camera_model(nav,obs.marker_id,cfg);
% Availability is decided by physical sensor, not a true-pose navigation gate.
if any(~isfinite(pred)) || numel(obs.pixels)~=8 || any(~isfinite(obs.pixels))
    report.reason='INVALID_PIXELS'; return;
end
V = (cfg.navigation_candidate.camera.pixel_std*cfg.sim.camera_noise_scale)^2*eye(8);
S = H*nav.P*H'+V; S = (S+S')/2;
[L,flag] = chol(S,'lower');
if flag~=0, report.reason='INVALID_INNOVATION_COVARIANCE'; return; end
r = obs.pixels(:)-pred; y=L\r; report.nis=y'*y;
if report.nis>cfg.sim.reference_nis_gate
    report.reason='INNOVATION_GATE'; return;
end
prior=nav;P=(nav.P+nav.P')/2;
[U,D]=eig(P,'vector');
if min(D)<-1e-10*max(1,max(D))
    report.reason='INVALID_PRIOR_COVARIANCE';return;
end
W=U*diag(sqrt(max(0,D))); % P=W W', no inverse of a singular prior
eta=zeros(size(P,1),1);value=r'*(V\r);report.initial_cost=value;
converged=false;
for iteration=1:cfg.sim.reference_max_iterations
    [current,G]=uavmvp.nav_inject(prior,W*eta);
    [pred,H]=uavmvp.camera_model(current,obs.marker_id,cfg);
    J=H*G;B=J*W;r=obs.pixels(:)-pred;
    % Gauss-Newton solution in the SAME original prior coordinates.
    candidate=B'/(B*B'+V)*(r+B*eta);
    step=candidate-eta;
    report.iterations=iteration;
    if norm(step)<=cfg.sim.reference_step_tolerance*(1+norm(eta))
        converged=true;break;
    end
    gradient=eta-B'*(V\r);alpha=1;accepted=false;
    for backtrack=1:16
        trial=eta+alpha*step;
        [trial_nav,~]=uavmvp.nav_inject(prior,W*trial);
        [prediction,~,~,detail]=uavmvp.camera_model(trial_nav,obs.marker_id,cfg);
        residual=obs.pixels(:)-prediction;
        trial_value=trial'*trial+residual'*(V\residual);
        if all(detail.depth>cfg.sim.reference_min_depth_m) && isfinite(trial_value) && ...
                trial_value<=value+1e-4*alpha*2*gradient'*step+1e-12
            eta=trial;value=trial_value;accepted=true;break;
        end
        alpha=alpha/2;
    end
    if ~accepted,break;end
end
report.final_cost=value;
if ~converged
    report.reason='REFERENCE_SOLVER_NOT_CONVERGED';return;
end
[updated,G]=uavmvp.nav_inject(prior,W*eta);
[pred,H]=uavmvp.camera_model(updated,obs.marker_id,cfg);
J=H*G;B=J*W;K=W*B'/(B*B'+V);
A=eye(size(P))-K*J;
post=A*P*A'+K*V*K'; % one observation contributes once, after iteration
nav=updated;
nav.P = G*post*G'; nav.P=(nav.P+nav.P')/2;
nav.version = nav.version+1;
nav.last_reference_id=obs.marker_id;nav.last_reference_time_s=nav.t;
report.accepted=true; report.reason='UPDATED';
report.pixel_residual_norm=norm(obs.pixels(:)-pred);
end
