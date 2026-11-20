function nav = nav_prune(nav,keep_ids)
%NAV_PRUNE Marginalize by submatrix, NEVER a Schur complement conditioning.
[ok,k]=ismember(keep_ids,nav.clone_ids);
assert(all(ok) && numel(unique(keep_ids))==numel(keep_ids));
base=15+6*size(nav.map_p,2);
ix=[1:base,reshape(base+3*(k(:)'-1)+(1:3)',1,[])];
nav.P=nav.P(ix,ix); nav.clones=nav.clones(:,k);
nav.clone_ids=nav.clone_ids(k); nav.clone_times=nav.clone_times(k);
nav.clone_velocities=nav.clone_velocities(:,k);
end
