function out=subbundle(b,rows)
out=uavstage.subbundle(b,rows);
out.model_bias_bounds_m=b.model_bias_bounds_m(rows);
out.aggregation_matrix=b.aggregation_matrix(rows,:);
out.constituent_ids=b.constituent_ids(rows);
end
