% generate_scaled_maps.m
clc; clear; close all;

% Base map (100x100) used by RS_marching
baseMapPath = '/home/ibrahim/projects/RS_marching/maps/base_100x100.png';

img = imread(baseMapPath);
if size(img,3) > 1
    imgGray = rgb2gray(img);
else
    imgGray = img;
end

% Binary occupancy: assume white = free, black = obstacle
% -> 1 = free, 0 = occupied (as requested)
occ_base = imbinarize(imgGray);   % white -> 1, black -> 0
[n0x, n0y] = size(occ_base);      % should be 100x100

fprintf('Base map size: %d x %d\n', n0x, n0y);

% Target sizes (adjust if you like)
target_sizes = 100:100:2000;  % 100, 200, ..., 2000

scaledDir = '/home/ibrahim/projects/RS_marching/scaled_maps';
if ~exist(scaledDir, 'dir')
    mkdir(scaledDir);
end

for N = target_sizes
    scaleFactor = N / n0x;
    fprintf('Generating scaled map: %d x %d (scale %.2f)\n', N, N, scaleFactor);
    
    % Nearest-neighbor to preserve occupancy pattern
    occ_scaled = imresize(occ_base, [N N], 'nearest');
    
    % Save as PNG: free=white (255), occupied=black (0)
    img_out = uint8(occ_scaled) * 255;  % 1->255 (free), 0->0 (occupied)
    outPath = fullfile(scaledDir, sprintf('map_%dx%d.png', N, N));
    imwrite(img_out, outPath);
end

disp('All scaled maps written to scaled_maps/.');
