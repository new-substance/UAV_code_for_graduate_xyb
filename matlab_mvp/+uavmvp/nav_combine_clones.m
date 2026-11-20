function [nav,id,raw]=nav_combine_clones(nav,ids,weights)
%NAV_COMBINE_CLONES Linear weighted antenna average, retaining ALL covariance.
% This represents one short-stop observation, not several spatial locations.
weights=weights(:);assert(numel(weights)==numel(ids) && all(weights>=0) && abs(sum(weights)-1)<1e-12);
[ok,index]=ismember(ids,nav.clone_ids);assert(all(ok));
base=15+6*size(nav.map_p,2);J=zeros(3,size(nav.P,1));
for k=1:numel(ids),J(:,base+3*(index(k)-1)+(1:3))=J(:,base+3*(index(k)-1)+(1:3))+weights(k)*eye(3);end
p=nav.clones(:,index)*weights;epoch=nav.clone_times(index)*weights;
velocity=nav.clone_velocities(:,index)*weights;
cross=nav.P*J';covariance=J*cross;
raw=struct('epoch_s',epoch,'position',p,'covariance',covariance,'version',nav.version, ...
    'support_ids',ids,'weights',weights');
nav.P=[nav.P,cross;cross',covariance];nav.P=(nav.P+nav.P')/2;
id=nav.next_clone_id;nav.next_clone_id=id+1;
nav.clones(:,end+1)=p;nav.clone_ids(end+1)=id;nav.clone_times(end+1)=epoch;
nav.clone_velocities(:,end+1)=velocity;
raw.covariance=raw.covariance+nav.radio_imu_timing_std_s^2*(velocity*velocity');
end
