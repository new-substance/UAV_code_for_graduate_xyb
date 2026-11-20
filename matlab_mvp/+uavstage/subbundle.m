function out=subbundle(b,rows)
out=b;ix=reshape(3*(rows(:)'-1)+(1:3)',1,[]);
out.anchors=b.anchors(:,rows);out.ranges=b.ranges(rows);out.ranges=out.ranges(:);
out.Pnav=b.Pnav(ix,ix);out.Pnavigation=b.Pnavigation(ix,ix);out.Ptiming=b.Ptiming(ix,ix);
out.Cradio=b.Cradio(rows,rows);out.record_ids=reshape(b.record_ids(rows),1,[]);
out.nav_ids=reshape(b.nav_ids(rows),1,[]);
end
