function result=block_information(x,anchors,Pnav,Cradio,past_count,cfg)
%BLOCK_INFORMATION Joint prospective mean information, retaining correlations.
% This is a planning proxy with covariance frozen at x, not a fit/acceptance.
[~,H,C]=uavmvp.range_model(x,anchors,Pnav,Cradio,cfg.scene.terminal_height_m);
n=size(H,1);assert(past_count>=0 && past_count<n && past_count==floor(past_count));
future=past_count+1:n;prior=diag(12./[cfg.scene.width_m,cfg.scene.height_m].^2);
J=prior;
if past_count>0
    past=1:past_count;L=chol(C(past,past),'lower');Z=L\H(past,:);J=J+Z'*Z;
    A=(L'\(L\C(past,future)))';
    S=C(future,future)-A*C(past,future);D=H(future,:)-A*H(past,:);
else
    S=C;D=H;
end
S=(S+S')/2;L=chol(S,'lower');Z=L\D;updated=J+Z'*Z;
before=J\eye(2);after=updated\eye(2);
result=struct('before',before,'after',after,'conditional_covariance',S, ...
    'information_gain',2*sum(log(diag(chol(updated))))-2*sum(log(diag(chol(J)))));
end
