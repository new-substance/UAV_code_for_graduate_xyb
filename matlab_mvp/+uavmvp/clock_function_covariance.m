function C=clock_function_covariance(intervals,weights,initial_variance,density)
%CLOCK_FUNCTION_COVARIANCE Linear functionals of a Brownian clock rate.
% [a,b], b>a: integral_a^b delta_alpha(t) dt; [t,t]: delta_alpha(t).
% Kernel integration uses positive Simpson weights and avoids subtracting
% nearly equal O(t^3) absolute-time covariance terms.
assert(size(intervals,2)==2 && size(weights,2)==size(intervals,1));
assert(all(isfinite(intervals),'all') && all(intervals(:,1)>=0) && ...
    all(intervals(:,2)>=intervals(:,1)) && initial_variance>=0 && density>=0);
point=intervals(:,1)==intervals(:,2);constant=diff(intervals,1,2);constant(point)=1;
v=weights*constant;C=initial_variance*(v*v');
edges=unique([0;intervals(:)]);
for j=1:numel(edges)-1
    left=edges(j);right=edges(j+1);mid=(left+right)/2;
    W=zeros(size(intervals,1),3);
    for k=1:size(intervals,1)
        a=intervals(k,1);b=intervals(k,2);
        if point(k),W(k,:)=double(mid<b);
        elseif mid<a,W(k,:)=b-a;
        elseif mid<b,W(k,:)=b-[left,mid,right];end
    end
    B=weights*W;
    C=C+density^2*(right-left)/6*(B(:,1)*B(:,1)'+4*B(:,2)*B(:,2)'+B(:,3)*B(:,3)');
end
C=(C+C')/2;
end
