function [z,H,visible,details] = camera_model(nav,marker_id,cfg)
%CAMERA_MODEL Four measured corners in calibrated camera, actual geometry.
% Pixel ordering: u1,v1,u2,v2,...; right attitude errors in body/marker frames.
n = cfg.navigation_candidate; cam = n.camera;
half = n.marker_side_m/2;
corners = half*[-1,1,1,-1; -1,-1,1,1; 0,0,0,0];
Rbc = n.camera_to_body_rotation;
lc = n.camera_lever_arm_body_m(:);
Rm = nav.map_R(:,:,marker_id); pm = nav.map_p(:,marker_id);
z = zeros(8,1); H = zeros(8,size(nav.P,1));
depth = zeros(1,4); image = zeros(2,4);
k = 15+(marker_id-1)*6;
for j=1:4
    a = pm+Rm*corners(:,j);
    s = nav.R'*(a-nav.p); q = Rbc'*(s-lc);
    depth(j) = q(3);
    if abs(q(3)) < eps
        z(:)=NaN; H(:)=NaN; visible=false;
        details=struct('depth',depth,'slant_m',Inf,'min_edge_px',0); return;
    end
    image(:,j) = [cam.fx_px*q(1)/q(3)+cam.cx_px; cam.fy_px*q(2)/q(3)+cam.cy_px];
    J = [cam.fx_px/q(3),0,-cam.fx_px*q(1)/q(3)^2; ...
        0,cam.fy_px/q(3),-cam.fy_px*q(2)/q(3)^2];
    rows = 2*j-1:2*j; z(rows) = image(:,j);
    H(rows,1:3) = -J*Rbc'*nav.R';
    H(rows,7:9) = J*Rbc'*uavmvp.SO3.skew(s);
    H(rows,k+(1:3)) = J*Rbc'*nav.R';
    H(rows,k+(4:6)) = -J*Rbc'*nav.R'*Rm*uavmvp.SO3.skew(corners(:,j));
end
edges = vecnorm(image-image(:,[2,3,4,1]));
pc = nav.p+nav.R*lc;
slant = norm(pm-pc);
front = nav.map_R(:,3,marker_id)'*(pc-pm)>0;
visible = all(depth>cfg.sim.reference_min_depth_m) && front && ...
    all(image(1,:)>=0 & image(1,:)<cam.width_px) && ...
    all(image(2,:)>=0 & image(2,:)<cam.height_px) && ...
    min(edges)>=n.marker_min_projected_edge_px && slant<=n.marker_max_slant_m;
details = struct('depth',depth,'slant_m',slant,'min_edge_px',min(edges));
end
