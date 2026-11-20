classdef SO3
    %SO3 Rotation matrices; right local perturbations R_true=R*Exp(delta).
    methods (Static)
        function X = skew(x)
            x = x(:);
            X = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
        end
        function R = exp(x)
            x = x(:); t2 = x' * x; X = uavmvp.SO3.skew(x);
            if t2 < 1e-12
                a = 1-t2/6+t2*t2/120;
                b = 1/2-t2/24+t2*t2/720;
            else
                t = sqrt(t2); a = sin(t)/t; b = (1-cos(t))/t2;
            end
            R = eye(3) + a*X + b*(X*X);
        end
        function x = log(R)
            c = min(1,max(-1,(trace(R)-1)/2));
            v = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)]/2;
            t = atan2(norm(v), c);
            if t < 1e-7
                x = (1+t*t/6)*v;
            elseif pi-t < 1e-5
                [V,D] = eig((R+R')/2);
                [~,k] = max(diag(D)); a = V(:,k);
                if a'*v < 0, a = -a; end
                x = t*a;
            else
                x = (t/sin(t))*v;
            end
        end
        function J = rightJacobian(x)
            x = x(:); t2 = x'*x; X = uavmvp.SO3.skew(x);
            if t2 < 1e-12
                a = 1/2-t2/24+t2*t2/720;
                b = 1/6-t2/120+t2*t2/5040;
            else
                t = sqrt(t2); a = (1-cos(t))/t2;
                b = (t-sin(t))/(t2*t);
            end
            J = eye(3)-a*X+b*X*X;
        end
    end
end
