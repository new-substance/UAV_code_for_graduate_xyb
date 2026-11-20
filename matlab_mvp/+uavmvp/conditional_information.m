function score=conditional_information(x,anchors,Pnav,Cradio,cfg)
%CONDITIONAL_INFORMATION Prospective last row given correlated existing rows.
% Existing observations are not re-counted as independent after adding a point.
[~,H,C]=uavmvp.range_model(x,anchors,Pnav,Cradio,cfg.scene.terminal_height_m);
prior=diag(12./[cfg.scene.width_m,cfg.scene.height_m].^2);
m=size(H,1)-1;I=prior;
if m>0
    L=chol(C(1:m,1:m),'lower');Z=L\H(1:m,:);I=I+Z'*Z;
    cross=(L'\(L\C(1:m,end)))';
    h=H(end,:)-cross*H(1:m,:);variance=C(end,end)-cross*C(1:m,end);
else,h=H;variance=C;end
assert(isfinite(variance) && variance>0,'uavmvp:planningCovariance','Invalid conditional innovation variance.');
updated=I+h'*h/variance;
before=I\eye(2);after=updated\eye(2);
score=struct('before',before,'after',after,'innovation_variance',variance, ...
    'information_gain',log(det(updated))-log(det(I)), ...
    'radius_before',sqrt(cfg.localization.region_chi2_df2_level*max(eig(before))), ...
    'radius_after',sqrt(cfg.localization.region_chi2_df2_level*max(eig(after))));
end
