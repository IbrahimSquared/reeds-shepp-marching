function occ_noisy = add_cell_flip_noise(occ, p_free_to_occ, p_occ_to_free)
% ADD_CELL_FLIP_NOISE Randomly flip free/occupied cells of an occupancy grid.
%
%   occ              : logical or 0/1 occupancy grid (1 = occupied, 0 = free)
%   p_free_to_occ    : probability of flipping free -> occupied
%   p_occ_to_free    : probability of flipping occupied -> free
%
%   occ_noisy        : noisy occupancy grid

    occ = logical(occ);

    free_mask = (occ == 0);
    occ_mask  = (occ == 1);

    % Uniform random numbers
    r = rand(size(occ));

    flip_free = (r < p_free_to_occ) & free_mask;
    flip_occ  = (r < p_occ_to_free) & occ_mask;

    occ_noisy = occ;
    occ_noisy(flip_free) = 1;
    occ_noisy(flip_occ)  = 0;
end