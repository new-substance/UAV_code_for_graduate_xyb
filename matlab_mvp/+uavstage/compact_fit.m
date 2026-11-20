function small=compact_fit(fit)
small=rmfield(fit,intersect(fieldnames(fit),{'grid_xy','grid_nll','region_mask'}));
end
