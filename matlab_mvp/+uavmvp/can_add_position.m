function ok=can_add_position(tracker,book,id,nav,cfg)
%CAN_ADD_POSITION Preview geometry from the current online navigation state.
state=tracker.terminals(id);ok=false;
if ismember(state.phase,{'DONE','FAILED'}),return;end
if isempty(state.pool_ids),ok=true;return;end
records=book.nodes(id).range_records;selected=[records{state.pool_ids}];
[preview,new]=uavmvp.nav_clone(nav,cfg);
[positions,P]=uavmvp.nav_archive(preview,[[selected.effective_nav_snapshot_id],new]);
m=numel(state.pool_ids);pairs=[(1:m)',repmat(m+1,m,1)];
ok=uavmvp.distinct_positions(positions,P,cfg,pairs);
end
