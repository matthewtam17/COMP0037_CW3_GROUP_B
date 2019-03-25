% This small script creates the markup for the obstacles
baseMap = imread('factory.png');

% Aisle B obstacle
baseMap(246:251, 179:249, 1:3) = 30;

% Aisle C obstacle
baseMap(120:125, 257:331, 1:3) = 60;

% Aisle D obstacle
baseMap(17:50, 339:406, 1:3) = 90;

% Aisle E obstacle
baseMap(55:270, 414:485, 1:3) = 120;


figure(1)
imagesc(baseMap(:,:,1))

figure(2)
imshow(baseMap)

imwrite(baseMap, 'factory_with_obstacles.png');
