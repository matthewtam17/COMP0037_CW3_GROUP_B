% Dumb script to create a map; takes in a simple grid and generates two
% output maps: a "shrunk" map which is loaded by the map_server, and an
% "expanded" map which is taken by the turtlesim

mapX = 40;
mapY = 40;

% The map resolution - metres per pixel
mapResolution = 0.5;

map = 255 * ones(mapX, mapY);

map(5:13, 5:35) = 0;
map(17:25, 7:35) = 0;
map(29:32, 5:30) = 0;

map(35:37, 5:35) = 0;

% Expressed in grid cells; converted into (x,y) coordinates on write out
goals = [1 1;4 15;1 1;16 30;38 2;40 40;20 36;1 1];

mapNameRoot = 's02_factory';

writeOutFiles