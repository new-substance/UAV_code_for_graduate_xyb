function out=check_noise_horizon()
%CHECK_NOISE_HORIZON Larger time allowance cannot suppress/change old noise.
repo=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(repo,'matlab_mvp'));setup_project;
c=load_project_config();seed=mod(2026112302+314159,2^32-1);
short=uavmvp.NoisePath(2400,c.sim.dt,12,seed);
long=uavmvp.NoisePath(3000,c.sim.dt,12,seed);difference=0;
for time=[0,31.234,999.5,1799.99,2399.9]
    difference=max(difference,max(abs(short.at(time)-long.at(time))));
end
assert(difference<1e-12);
assert(all(isfinite(long.delta(2399.9,2800.123))));
out=struct('all_passed',true,'mission_count',0,'old_horizon_s',2400,'new_horizon_s',3000, ...
    'max_prefix_difference',difference,'future_noise_not_clamped',true);
end
