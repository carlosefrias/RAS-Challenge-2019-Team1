close all

imgRGB = imread('hammer.png');

figure,
imshow(imgRGB);

imgHSV = rgb2hsv(imgRGB);

figure,
imshow(imgHSV);

[hue,saturation,value] = imsplit(imgHSV);

BW = imbinarize(value);

figure,
imshow(BW);

CC = bwconncomp(BW);
stats = regionprops(CC,'area', 'BoundingBox', 'centroid');

centroids = [stats.Centroid];

areas = [stats.Area];
boundingBoxes = [stats.BoundingBox];
[~,idx] = find( areas == max(areas));

rect = boundingBoxes(4*idx - 3:4*idx);
centroid = centroids(2*idx - 1:2*idx);
rightBox = [154.5100 84.5100 112.9800 140.9800];
croppedImg = imcrop(imgRGB, rightBox);

figure,
imshow(croppedImg);
disp('end');