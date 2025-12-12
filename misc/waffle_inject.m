clc; clear;

%----------------------------
% load base map
%----------------------------
occ_img = imread('images/waffle_inflated.jpeg');

if size(occ_img,3) > 1
    occ_gray = rgb2gray(occ_img);
else
    occ_gray = occ_img;
end

threshold = graythresh(occ_gray);
occ_base = imbinarize(occ_gray, threshold);
% occ_base = ~occ_base;  % uncomment if your convention is inverted

occ_base = logical(occ_base);

%----------------------------
% prepare output folder
%----------------------------
out_dir = 'images/noisy_waffle';
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

%----------------------------
% define noise levels
%----------------------------
noise_levels = [0, 0.01, 0.05];       % 0 = clean, 1% flips, 5% flips
radii_dilate = [0, 1, 2];             % structural noise (disk radius)
num_realizations = 5;                 % different random maps per level

rng(0); % for reproducibility

for i = 1:numel(noise_levels)
    p = noise_levels(i);
    rad = radii_dilate(i);
    
    for k = 1:num_realizations
        occ_noisy = occ_base;
        
        % 1) optional structural noise (thicken obstacles)
        if rad > 0
            occ_noisy = thicken_obstacles(occ_noisy, rad);
        end
        
        % 2) random cell flip noise
        if p > 0
            occ_noisy = add_cell_flip_noise(occ_noisy, p, p);
        end
        
        % save as image (0/255)
        img_to_save = uint8(occ_noisy) * 255;
        
        filename = sprintf('waffle_noise_rad%d_p%.3f_seed%02d.png', ...
                           rad, p, k);
        imwrite(img_to_save, fullfile(out_dir, filename));
    end
end

disp('Noisy maps generated and saved.');
