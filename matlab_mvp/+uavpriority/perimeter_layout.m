function [xy,route]=perimeter_layout
%PERIMETER_LAYOUT Peripheral 10 m spacing, all service points stay peripheral.
xy=[(10:10:110)',10*ones(11,1);110*ones(5,1),(20:10:60)'; ...
    (110:-10:10)',70*ones(11,1);10*ones(5,1),(60:-10:20)'];
route=xy; % fixed clockwise coverage order, node positions are not inputs
end
