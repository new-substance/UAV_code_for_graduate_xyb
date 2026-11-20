function [nav,G] = nav_inject(nav,dx)
%NAV_INJECT Error-state retraction and its reset Jacobian at the correction.
assert(numel(dx)==size(nav.P,1));
dx = dx(:); G = eye(numel(dx));
nav.p = nav.p+dx(1:3); nav.v = nav.v+dx(4:6);
nav.R = nav.R*uavmvp.SO3.exp(dx(7:9));
nav.ba = nav.ba+dx(10:12); nav.bg = nav.bg+dx(13:15);
G(7:9,7:9) = uavmvp.SO3.rightJacobian(dx(7:9));
m = size(nav.map_p,2);
for j=1:m
    k = 15+(j-1)*6;
    nav.map_p(:,j) = nav.map_p(:,j)+dx(k+(1:3));
    nav.map_R(:,:,j) = nav.map_R(:,:,j)*uavmvp.SO3.exp(dx(k+(4:6)));
    G(k+(4:6),k+(4:6)) = uavmvp.SO3.rightJacobian(dx(k+(4:6)));
end
if ~isempty(nav.clones)
    nav.clones = nav.clones+reshape(dx(15+6*m+1:end),3,[]);
end
end
