function [distinct,lower_separation] = distinct_positions(anchors,P,cfg,pairs)
%DISTINCT_POSITIONS Conservative horizontal separation from relative errors.
m=size(anchors,2);
if nargin<4,pairs=nchoosek(1:m,2);end
lower_separation=zeros(size(pairs,1),1);
for k=1:size(pairs,1)
    i=pairs(k,1);j=pairs(k,2);a=3*i-2:3*i-1;b=3*j-2:3*j-1;
    D=P(a,a)+P(b,b)-P(a,b)-P(b,a);D=(D+D')/2;
    variance=max(eig(D));
    if variance < -1e-10
        lower_separation(k)=-Inf;continue;
    end
    lower_separation(k)=norm(anchors(1:2,i)-anchors(1:2,j))- ...
        sqrt(cfg.localization.region_chi2_df2_level*max(variance,0));
end
distinct=all(lower_separation>=cfg.localization.distinct_horizontal_separation_m);
end
