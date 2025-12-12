function occ_inflated = thicken_obstacles(occ, radius)
    % Here occ: 1 = free, 0 = occupied
    occ = logical(occ);
    se = strel('disk', radius);

    % Work in occupied space: 1 = occupied, 0 = free
    occ_occ = ~occ;              % now 1 = occupied
    occ_occ_dil = imdilate(occ_occ, se);  % grow obstacles
    occ_inflated = ~occ_occ_dil;          % convert back: 1 = free, 0 = occupied
end
