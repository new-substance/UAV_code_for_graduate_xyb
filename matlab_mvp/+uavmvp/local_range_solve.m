function [x,value,flag] = local_range_solve(seed,anchors,ranges,Pnav,Cradio,cfg)
%LOCAL_RANGE_SOLVE Projected Newton NLL with a checked box-KKT residual.
% Numerical derivatives use the smooth distance model across a box edge;
% all accepted iterates remain feasible. No penalty discontinuity at edges.
upper=[cfg.scene.width_m;cfg.scene.height_m];clip=@(q)min(upper,max(zeros(2,1),q));
x=clip(seed(:));value=objective(x);flag=0;
for iteration=1:cfg.localization.max_local_search_iterations
    h=max(1e-3,1e-5*abs(x));g=zeros(2,1);H=zeros(2);
    for j=1:2
        e=zeros(2,1);e(j)=h(j);fp=objective(x+e);fm=objective(x-e);
        g(j)=(fp-fm)/(2*h(j));H(j,j)=(fp-2*value+fm)/h(j)^2;
    end
    e=[h(1);0];d=[0;h(2)];
    H(1,2)=(objective(x+e+d)-objective(x+e-d)-objective(x-e+d)+objective(x-e-d))/(4*prod(h));
    H(2,1)=H(1,2);
    if any(~isfinite([g;H(:)])),return;end
    projected=x-clip(x-g);
    if norm(projected,Inf)<=2e-7,flag=1;return;end
    active=(x<=1e-10 & g>0) | (x>=upper-1e-10 & g<0);free=find(~active);
    step=zeros(2,1);
    if ~isempty(free)
        [U,D]=eig(H(free,free),'vector');floor_value=1e-5*max(1,max(abs(D)));
        B=U*diag(max(D,floor_value))*U';step(free)=-B\g(free);
    end
    delta=clip(x+step)-x;
    if g'*delta>=0 || norm(delta)<1e-12,step=-g;end
    accepted=false;alpha=1;
    for backtrack=1:24
        candidate=clip(x+alpha*step);delta=candidate-x;next=objective(candidate);
        if isfinite(next) && next<=value+1e-4*g'*delta+1e-12
            x=candidate;value=next;accepted=true;break;
        end
        alpha=alpha/2;
    end
    if ~accepted,return;end
end
    function f=objective(q)
        f=uavmvp.range_nll_batch(q(:)',anchors,ranges,Pnav,Cradio,cfg.scene.terminal_height_m);
    end
end
