function manifest=snapshot_source(output_dir)
% Old full-copy snapshots are retired; use the root Git version reference.
error('uavproject:LegacySnapshotDisabled','Full source copying is disabled. Use the managed project entry.');
end
