function error=initial_error(P,stream)
%INITIAL_ERROR Sample a PSD JOINT prior, preserving the legacy diagonal draw.
if isequal(P,diag(diag(P)))
    error=sqrt(diag(P)).*randn(stream,size(P,1),1);return;
end
[U,d]=eig((P+P')/2,'vector');
assert(min(d)>=-1e-10*max(1,max(d)),'uavmvp:initialPSD','Invalid joint initial prior.');
error=U*diag(sqrt(max(0,d)))*randn(stream,size(P,1),1);
end
