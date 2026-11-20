function Q = process_noise(F, dt, cfg)
%PROCESS_NOISE Positive-semidefinite Simpson quadrature of continuous Qd.
% Continuous isotropic body sensor densities rotate to isotropic world noise.
n = cfg.navigation_candidate;
d = [zeros(1,3),repmat(n.acc_noise_density_mps2_per_sqrt_hz^2,1,3), ...
    repmat(n.gyro_noise_density_radps_per_sqrt_hz^2,1,3), ...
    repmat(n.acc_bias_random_walk_mps2_per_sqrt_s^2,1,3), ...
    repmat(n.gyro_bias_random_walk_radps_per_sqrt_s^2,1,3)];
Qc = diag(d);
I = eye(15); X = F*dt;
A = I+X+X*X/2+X*X*X/6;
B = I+X/2+X*X/8+X*X*X/48;
Q = dt/6*(Qc+4*B*Qc*B'+A*Qc*A');
Q = (Q+Q')/2;
end
