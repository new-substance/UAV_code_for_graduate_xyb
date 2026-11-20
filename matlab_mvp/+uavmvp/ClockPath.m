classdef ClockPath
    %CLOCKPATH Physical oscillator: fixed-grid Brownian rate, integrated phase.
    % Uses a piecewise linear 100 Hz approximation, independent of query order.
    % This object belongs to the private radio world, never to the planner.
    properties (Access=private)
        rates
        integrals
        offset
        dt
        horizon
    end
    methods
        function obj=ClockPath(cfg,horizon,seed)
            obj.dt=cfg.sim.dt;count=ceil(horizon/obj.dt);obj.horizon=count*obj.dt;
            rs=RandStream('mt19937ar','Seed',seed);
            initial=1+cfg.ranging.initial_clock_ratio_std_ppm*1e-6*randn(rs);
            obj.offset=randn(rs);
            density=cfg.ranging.clock_ratio_random_walk_ppm_per_sqrt_s*1e-6;
            obj.rates=initial+[0,cumsum(density*sqrt(obj.dt)*randn(rs,1,count))];
            obj.integrals=[0,cumsum((obj.rates(1:end-1)+obj.rates(2:end))*obj.dt/2)];
            assert(min(obj.rates)>0);
        end
        function [stamp,rate]=sample(obj,time)
            assert(all(time>=0 & time<=obj.horizon,'all'));
            shape=size(time);time=time(:)';
            index=min(floor(time/obj.dt)+1,numel(obj.rates)-1);
            u=time-(index-1)*obj.dt;a=obj.rates(index);b=obj.rates(index+1);
            rate=a+(b-a).*u/obj.dt;
            stamp=obj.offset+obj.integrals(index)+a.*u+(b-a).*u.^2/(2*obj.dt);
            stamp=reshape(stamp,shape);rate=reshape(rate,shape);
        end
    end
end
