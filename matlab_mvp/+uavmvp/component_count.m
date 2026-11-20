function [count,labels] = component_count(mask)
%COMPONENT_COUNT Eight-connected sampled likelihood-region components.
[nr,nc]=size(mask);labels=zeros(nr,nc,'uint32');count=0;
queue=zeros(nnz(mask),1);[rows,cols]=find(mask);
for i=1:numel(rows)
    r=rows(i);c=cols(i);if labels(r,c)>0,continue;end
    count=count+1;head=1;tail=1;queue(1)=sub2ind([nr,nc],r,c);labels(r,c)=count;
    while head<=tail
        [a,b]=ind2sub([nr,nc],queue(head));head=head+1;
        for da=-1:1
            for db=-1:1
                aa=a+da;bb=b+db;
                if aa>=1 && aa<=nr && bb>=1 && bb<=nc && mask(aa,bb) && labels(aa,bb)==0
                    tail=tail+1;queue(tail)=sub2ind([nr,nc],aa,bb);labels(aa,bb)=count;
                end
            end
        end
    end
end
end
