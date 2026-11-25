function P=forest_covariance(P)
%FOREST_COVARIANCE N02/N03: validate PSD without silently adding jitter.
assert(ismatrix(P) && size(P,1)==size(P,2) && all(isfinite(P(:))), ...
    'forest:Covariance','Covariance must be a finite square matrix.');
scale=max(1,norm(P,2));
assert(norm(P-P','fro')<=1e-9*scale,'forest:Covariance','Asymmetric covariance.');
P=(P+P')/2;
assert(min(eig(P))>=-1e-10*scale,'forest:Covariance','Indefinite covariance.');
end
