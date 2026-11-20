function tests=test_priority_geometry
tests=functiontests(localfunctions);
end
function setupOnce(~),setup_priority();end
function testGlobalGeometryKeepsInformationThatLocalRankMisses(t)
c=uavpriority.config();c.localization.region_chi2_df2_level=c.stage.region_gamma;
xy=[80,10;90,10;100,10;110,10;110,20;110,30;110,40];n=size(xy,1);
b=struct('anchors',[xy,15*ones(n,1)]','ranges',zeros(n,1), ...
    'Pnav',.04*eye(3*n),'Cradio',27*eye(n)+9*ones(n), ...
    'record_ids',1:n,'nav_ids',1:n,'version',1);
probe=[115;5];local=c;local.localization.group_geometry_rule='local_information';
global_cfg=c;global_cfg.localization.group_geometry_rule='global_geometry';
[a,~]=uavmvp.select_four(b,7,local,probe);[z,~]=uavmvp.select_four(b,7,global_cfg,probe);
ia=reshape(3*(a-1)+(1:3)',1,[]);iz=reshape(3*(z-1)+(1:3)',1,[]);
pa=uavmvp.geometry_region_proxy(probe,b.anchors(:,a),b.Pnav(ia,ia),b.Cradio(a,a),c);
pz=uavmvp.geometry_region_proxy(probe,b.anchors(:,z),b.Pnav(iz,iz),b.Cradio(z,z),c);
verifyLessThan(t,pz.radius,pa.radius-10);
verifyTrue(t,any(ismember(z,[4,5]))); % near-corner constraints remove distant aliases
verifyTrue(t,ismember(7,z));
end
function testFreshInsetIsFixedAndChosenWithoutFutureRange(t)
c=uavpriority.config();nav=uavmvp.nav_init(c);nav.last_reference_id=1;
fit=struct('x',[60;40]);visited=c.scene.reference_centers_xy_m;
[q,d]=uavpriority.next_extra(nav,fit,visited,c);
verifyTrue(t,d.available);verifyGreaterThanOrEqual(t,min(vecnorm(visited-q,2,2)),6);
verifyEqual(t,norm(q-c.scene.reference_centers_xy_m(d.reference_id,:)),7.5,'AbsTol',1e-10);
verifyLessThanOrEqual(t,norm([q';15]-[fit.x;0]),40);
[r,e]=uavpriority.next_extra(nav,fit,[visited;q],c);
verifyTrue(t,e.available);verifyGreaterThan(t,norm(r-q),6);
end
function testDegenerateLineMinimumIsRejectedWithoutException(t)
c=uavpriority.config();a=[40,10,15;50,10,15;60,10,15;70,10,15]';
b=struct('anchors',a,'ranges',vecnorm(a-[55;10;0])'-10, ...
    'Pnav',zeros(12),'Cradio',eye(4),'record_ids',1:4,'nav_ids',1:4,'version',1, ...
    'model_bias_bounds_m',zeros(4,1));
f=uavpriority.fit_position(b,c);
verifyFalse(t,f.quality);verifyTrue(t,isinf(f.radius) || f.radius>20);
end
