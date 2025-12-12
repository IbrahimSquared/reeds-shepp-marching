clc; clear;

bw0 = imread('images/waffle.jpeg'); 
bw0 = imbinarize(bw0); 
bw = bwmorph(bw0, 'thicken', 4); 
bw = imdilate(bw, strel('square', 5)); 
bw = imerode(bw, strel('square', 12)); 

% overlay the two images as if blueprint on top of the original image
image = imread('images/waffle.jpeg');
inflatedImage = imoverlay(image, bw, [0 0 1]);

figure(9);
clf
imshow(inflatedImage);
figure(10)
imshow(bw);

% figure(9);
% clf
% subplot(1, 2, 1);
% imshow(bw0);
% title('Original Image');
% subplot(1, 2, 2);
% imshow(bw);
% title('Bounding Box Image');

% figure(9);
% clf
% subplot(1, 2, 1);
% imshow(image);
% title('Original Image');
% subplot(1, 2, 2);
% imshow(inflatedImage);
% title('Inflated Image');

imwrite(bw, 'images/waffle_inflated.jpeg');
