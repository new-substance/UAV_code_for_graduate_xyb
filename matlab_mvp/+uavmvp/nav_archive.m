function [positions,P,version] = nav_archive(nav,ids)
%NAV_ARCHIVE All requested epochs in one current navigation version.
[ok,k]=ismember(ids,nav.clone_ids);
assert(all(ok),'uavmvp:missingClone','Unknown/deleted measurement clone.');
positions=nav.clones(:,k);
base=15+6*size(nav.map_p,2);
ix=reshape(base+3*(k(:)'-1)+(1:3)',1,[]);
P=nav.P(ix,ix); version=nav.version;
if isfield(nav,'radio_imu_timing_std_s')
    velocity=reshape(nav.clone_velocities(:,k),[],1);
    P=P+nav.radio_imu_timing_std_s^2*(velocity*velocity');
end
end
