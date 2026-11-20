function [distance,H,C,G] = range_model(x,anchors,Pnav,Cradio,height)
%RANGE_MODEL Marginal distance covariance with FULL joint antenna uncertainty.
% Nav error is true minus estimate, so residual includes +G*delta_antenna.
m=size(anchors,2);target=[x(:);height];d=anchors-target;
distance=sqrt(sum(d.^2,1))';assert(all(distance>0));
U=d./distance';H=-U(1:2,:)';G=zeros(m,3*m);
for k=1:m,G(k,3*k-2:3*k)=U(:,k)';end
C=Cradio+G*Pnav*G';C=(C+C')/2;
end
