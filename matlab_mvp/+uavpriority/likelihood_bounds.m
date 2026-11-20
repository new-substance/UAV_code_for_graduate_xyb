function [lower,upper,nominal,a]=likelihood_bounds(points,b,height)
%LIKELIHOOD_BOUNDS Q01 triangle bounds for a four-dimensional error box.
assert(numel(b.ranges)==4 && numel(b.model_bias_bounds_m)==4);
bias=b.model_bias_bounds_m(:);assert(all(isfinite(bias) & bias>=0));
[nominal,chi,p]=uavmvp.range_nll_batch(points,b.anchors,b.ranges,b.Pnav,b.Cradio,height);
n=size(points,1);a=zeros(n,1);signs=2*(dec2bin(0:15,4)-'0')-1;
for corner=1:16
    delta=signs(corner,:)'.*bias;whitened=zeros(n,4);
    for i=1:4
        value=repmat(delta(i),n,1);
        for j=1:i-1,value=value-p.L(:,i,j).*whitened(:,j);end
        whitened(:,i)=value./p.L(:,i,i);
    end
    a=max(a,sqrt(sum(whitened.^2,2)));
end
lower=max(0,sqrt(chi)-a).^2+p.logdet;
upper=(sqrt(chi)+a).^2+p.logdet;
lower(~p.valid)=Inf;upper(~p.valid)=Inf;
end
