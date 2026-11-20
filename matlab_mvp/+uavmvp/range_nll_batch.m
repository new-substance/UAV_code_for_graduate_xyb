function [F,chi] = range_nll_batch(points,anchors,ranges,Pnav,Cradio,height)
%RANGE_NLL_BATCH Vectorized small-matrix Cholesky over position candidates.
% F = r' C(x)^(-1) r + log det C(x); no covariance inverse is formed.
n=size(points,1);m=size(anchors,2);
assert(size(points,2)==2 && numel(ranges)==m);
U=zeros(n,m,3);U(:,:,1)=anchors(1,:)-points(:,1);
U(:,:,2)=anchors(2,:)-points(:,2);U(:,:,3)=repmat(anchors(3,:)-height,n,1);
d=sqrt(sum(U.^2,3));valid=all(d>0,2);U=U./max(d,realmin);
C=zeros(n,m,m);
for i=1:m
    for j=1:i
        value=repmat(Cradio(i,j),n,1);
        for a=1:3
            for b=1:3
                value=value+U(:,i,a).*U(:,j,b)*Pnav(3*i-3+a,3*j-3+b);
            end
        end
        C(:,i,j)=value;C(:,j,i)=value;
    end
end
L=zeros(n,m,m);y=zeros(n,m);logdet=zeros(n,1);
residual=ranges(:)'-d;
for i=1:m
    for j=1:i
        value=C(:,i,j);
        for k=1:j-1,value=value-L(:,i,k).*L(:,j,k);end
        if i==j
            valid=valid & isfinite(value) & value>0;
            value(~valid)=1; % only to finish arithmetic; result marked Inf below
            L(:,i,i)=sqrt(value);logdet=logdet+log(value);
        else
            L(:,i,j)=value./L(:,j,j);
        end
    end
    z=residual(:,i);
    for j=1:i-1,z=z-L(:,i,j).*y(:,j);end
    y(:,i)=z./L(:,i,i);
end
chi=sum(y.^2,2);F=chi+logdet;
F(~valid)=Inf;chi(~valid)=Inf;
end
