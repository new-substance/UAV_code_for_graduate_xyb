function [nav,id,raw] = nav_clone(nav,cfg)
%NAV_CLONE Clone antenna position at measurement epoch, including lever arm.
l = cfg.navigation_candidate.antenna_lever_arm_body_m(:);
p = nav.p+nav.R*l;
J = zeros(3,size(nav.P,1)); J(:,1:3)=eye(3);
J(:,7:9)=-nav.R*uavmvp.SO3.skew(l);
C = nav.P*J';
raw = struct('epoch_s',nav.t,'position',p,'covariance',J*C,'version',nav.version);
nav.P = [nav.P,C;C',J*C]; nav.P=(nav.P+nav.P')/2;
id=nav.next_clone_id; nav.next_clone_id=id+1;
nav.clones(:,end+1)=p; nav.clone_ids(end+1)=id;
nav.clone_times(end+1)=nav.t;
nav.clone_velocities(:,end+1)=nav.v;
raw.covariance=raw.covariance+nav.radio_imu_timing_std_s^2*(nav.v*nav.v');
raw.timing_model='Persistent radio/IMU offset, first-order constant-attitude antenna velocity';
end
