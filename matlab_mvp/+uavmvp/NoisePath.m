classdef NoisePath < handle
    %NOISEPATH Reproducible Brownian path, independent of query segmentation.
    % Coarse increments are stored; dyadic Brownian bridges resolve partial
    % IMU cells down to dt/2^20. The private world owns this object.
    properties (Access=private)
        coarse
        step
        dimensions
        seed
        cache
    end
    methods
        function obj=NoisePath(duration,dt,dimensions,seed)
            obj.step=dt;obj.dimensions=dimensions;obj.seed=seed;
            rs=RandStream('mt19937ar','Seed',seed);
            n=ceil(duration/dt);
            obj.coarse=[zeros(dimensions,1),cumsum(sqrt(dt)*randn(rs,dimensions,n),2)];
            obj.cache=containers.Map('KeyType','char','ValueType','any');
        end
        function value=at(obj,time)
            assert(time>=0 && time<=(size(obj.coarse,2)-1)*obj.step+1e-10);
            q=time/obj.step;nearest=round(q);
            if abs(q-nearest)<1e-9
                value=obj.coarse(:,nearest+1);return;
            end
            cell_id=floor(q);fraction=q-cell_id;
            left=obj.coarse(:,cell_id+1);right=obj.coarse(:,cell_id+2);
            lo=0;hi=1;node=0;
            for level=1:20
                key=sprintf('%d_%d_%d',cell_id,level,node);
                if isKey(obj.cache,key)
                    z=obj.cache(key);
                else
                    seed=mod(double(obj.seed)*131071+cell_id*524287+level*8191+node*65537,4294967291);
                    rs=RandStream('mt19937ar','Seed',seed);z=randn(rs,obj.dimensions,1);
                    obj.cache(key)=z;
                end
                mid=(lo+hi)/2;middle=(left+right)/2+sqrt(obj.step*(hi-lo)/4)*z;
                if abs(fraction-mid)<1e-12,value=middle;return;end
                if fraction<mid
                    right=middle;hi=mid;node=2*node;
                else
                    left=middle;lo=mid;node=2*node+1;
                end
            end
            alpha=(fraction-lo)/(hi-lo);value=(1-alpha)*left+alpha*right;
        end
        function increment=delta(obj,a,b)
            assert(b>=a);increment=obj.at(b)-obj.at(a);
        end
    end
end
