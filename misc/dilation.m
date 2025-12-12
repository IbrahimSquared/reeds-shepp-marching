clc; clear;

image = imread('images/lommel.jpeg');
image = im2bw(image, 0.5);

dilatedImage = image;
s = 5;
for i = 1:size(image, 1)
    for j = 1:size(image, 2)
        if image(i, j) < 1
            for k = -s:s
                for l = -s:s
                    if i + k > 0 && i + k <= size(image, 1) && j + l > 0 && j + l <= size(image, 2)
                        dilatedImage(i + k, j + l) = 0;
                    end
                end
            end
        end
    end
end

se = strel('square', 3);
dilatedImage = imdilate(dilatedImage, se);
% dilatedImage(280:330,482:492) = 0;
% dilatedImage(403:420,481:695) = 0;
% dilatedImage(:,110:130) = 0;

figure(9);
clf
subplot(1, 2, 1);
imshow(image);
title('Original Image');
subplot(1, 2, 2);
imshow(dilatedImage);
title('Dilated Image');

imwrite(dilatedImage, 'images/lommel_dilated_1.jpeg');
