function nav=recover_version(raw,cfg,version)
%RECOVER_VERSION Offline only, from an actual accepted pixel observation.
for k=1:numel(raw.reference_log)
    r=raw.reference_log{k};
    if r.report.accepted && r.prior.version+1==version
        [nav,report]=uavmvp.nav_update_reference(r.prior,r.observation,cfg);
        assert(report.accepted && nav.version==version);return;
    end
end
error('uavpriority:missingVersion','Missing actual pixel version.');
end
