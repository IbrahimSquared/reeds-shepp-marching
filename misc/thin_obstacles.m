function occ_thinned = thin_obstacles(occ, radius)
    % 1 = free, 0 = occupied
    occ = logical(occ);
    se = strel('disk', radius);

    % Erode obstacles (in occupied space) -> they shrink
    occ_occ = ~occ;
    occ_occ_ero = imerode(occ_occ, se);
    occ_thinned = ~occ_occ_ero;
end
