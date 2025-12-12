% load image images/waffle.jpeg, and display it
clc; clear;

img = imread('images/waffle.jpeg');
% filter the image such that randomly occupied pixels/noise are removed
% and the image is smoothened
% use a 3x3 filter
img = medfilt2(img, [10 10]);
img(650:668, 348:349) = 255;
img(750:751,292:310) = 255;

% save image as images/waffle_filtered.jpeg
% imwrite(img, 'images/waffle_filtered.jpeg');
imshow(img);

% addpath(genpath('~/expfig/'))
% export_fig('samples/test_waffle_4.png', '-r300', '-transparent', '-png')
