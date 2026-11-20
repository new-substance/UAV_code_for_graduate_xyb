function bundle=record_bundle(records,nav,history,ids)
%RECORD_BUNDLE One consistent navigation version and full radio covariance.
selected=[records{ids}];nav_ids=[selected.effective_nav_snapshot_id];
[anchors,P,version]=uavmvp.nav_archive(nav,nav_ids);
C=uavmvp.radio_history_covariance(history,[selected.radio_history_id]);
bundle=struct('anchors',anchors,'Pnav',P,'Cradio',C,'ranges',[selected.range_m]', ...
    'version',version,'record_ids',ids,'nav_ids',nav_ids);
end
