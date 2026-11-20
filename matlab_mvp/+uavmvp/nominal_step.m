function [next,Phi,F] = nominal_step(nav, imu, dt, gravity)
%NOMINAL_STEP Midpoint rotation, piecewise-constant specific force/rate.
% Phi is the exact derivative of THIS discrete integrator, not Euler F*dt.
assert(dt>0 && isfinite(dt));
f = imu.acc(:)-nav.ba; w = imu.gyro(:)-nav.bg;
q = w*dt; Eh = uavmvp.SO3.exp(q/2); E = uavmvp.SO3.exp(q);
Rm = nav.R*Eh; acc = Rm*f+gravity(:);
next = nav;
next.p = nav.p+nav.v*dt+0.5*acc*dt^2;
next.v = nav.v+acc*dt; next.R = nav.R*E;
next.t = nav.t+dt;
At = -nav.R*uavmvp.SO3.skew(Eh*f);
Abg = Rm*uavmvp.SO3.skew(f)*uavmvp.SO3.rightJacobian(q/2)*(dt/2);
Phi = eye(15);
Phi(1:3,4:6) = eye(3)*dt;
Phi(1:3,7:9) = 0.5*dt^2*At;
Phi(1:3,10:12) = -0.5*dt^2*Rm;
Phi(1:3,13:15) = 0.5*dt^2*Abg;
Phi(4:6,7:9) = dt*At;
Phi(4:6,10:12) = -dt*Rm;
Phi(4:6,13:15) = dt*Abg;
Phi(7:9,7:9) = E';
Phi(7:9,13:15) = -uavmvp.SO3.rightJacobian(q)*dt;
F = zeros(15);
F(1:3,4:6) = eye(3);
F(4:6,7:9) = -Rm*uavmvp.SO3.skew(f);
F(4:6,10:12) = -Rm;
F(7:9,7:9) = -uavmvp.SO3.skew(w);
F(7:9,13:15) = -eye(3);
end
