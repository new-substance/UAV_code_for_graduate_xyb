function points=service_candidate_points(reference_id,cfg)
%SERVICE_CANDIDATE_POINTS Shared nominal geometry; feasibility is checked later.
% These points use only the surveyed reference and area, never terminal data.
base=cfg.scene.reference_centers_xy_m(reference_id,:)';
switch cfg.planner.local_candidate_pattern
    case 'circle'
        inward=[cfg.scene.width_m;cfg.scene.height_m]/2-base;
        if norm(inward)<eps,inward=[1;0];else,inward=inward/norm(inward);end
        offsets=zeros(2,numel(cfg.planner.local_candidate_angles_deg));
        for k=1:numel(cfg.planner.local_candidate_angles_deg)
            angle=deg2rad(cfg.planner.local_candidate_angles_deg(k));
            R=[cos(angle),-sin(angle);sin(angle),cos(angle)];
            offsets(:,k)=cfg.planner.local_candidate_radius_m*R*inward;
        end
    case 'rectangle'
        halfwidth=cfg.planner.local_candidate_halfwidth_xy_m(:);
        assert(numel(halfwidth)==2 && all(isfinite(halfwidth)) && all(halfwidth>0));
        offsets=halfwidth.*[1,-1,-1,1;1,1,-1,-1];
    case 'footprint'
        % Nominal level/downward-camera footprint only. It proposes points;
        % actual attitude, map errors, covariance and flight are checked later.
        camera=cfg.navigation_candidate.camera;
        depth=cfg.scene.flight_height_m+cfg.navigation_candidate.camera_lever_arm_body_m(3);
        half=depth*[camera.width_px/(2*camera.fx_px);camera.height_px/(2*camera.fy_px)]- ...
            cfg.navigation_candidate.marker_side_m/2;
        half=.5*floor(2*cfg.planner.footprint_candidate_fraction*half);
        assert(all(half>0));
        [x,y]=meshgrid([-half(1),0,half(1)],[-half(2),0,half(2)]);
        offsets=[x(:)';y(:)'];offsets(:,all(offsets==0,1))=[];
        R=cfg.navigation_candidate.camera_to_body_rotation;
        offsets=R(1:2,1:2)*offsets;
    otherwise
        error('uavmvp:candidatePattern','Unknown local candidate geometry.');
end
xy=[base,base+offsets]';
points=[xy,repmat(cfg.scene.flight_height_m,size(xy,1),1)];
end
