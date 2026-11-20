function C=radio_history_covariance(history,ids)
%RADIO_HISTORY_COVARIANCE Same-device measurements, including all shared errors.
assert(all(ids>=1 & ids<=history.last_id & ids==floor(ids)));
C=history.P(2+ids,2+ids);
end
