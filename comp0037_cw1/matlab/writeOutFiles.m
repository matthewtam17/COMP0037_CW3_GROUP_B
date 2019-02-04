I=mat2gray(map', [0 255]);
imshow(I)

% Write out the map server files
yamlFileName = [mapNameRoot, '.yaml'];
mapServerImageFileName = [mapNameRoot, '.png'];
mapServerTextFileName = [mapNameRoot, '.txt'];

yamlFileFD = fopen(yamlFileName, 'w');
fprintf(yamlFileFD, 'image: %s\n', mapServerImageFileName);
fprintf(yamlFileFD, 'file: %s\n', mapServerTextFileName);
fprintf(yamlFileFD, 'resolution: %f\n', mapResolution);
fprintf(yamlFileFD, 'origin: [0.0, 0.0, 0.0]\noccupied_thresh: 0.99\nfree_thresh: 0.01\nnegate: 0\n');
fclose(yamlFileFD);

% Write out the map server image file
imwrite(I, mapServerImageFileName)

% Write out the text version of the file
fID = fopen(mapServerTextFileName, 'w');
fprintf(fID,'%d %d\n', mapX, mapY);
for h = 1 : mapY
    fprintf(fID,'%d', map(1, h));
    for w = 2 : mapX
        fprintf(fID,' %d', map(w,h));
    end
    fprintf(fID, '\n');
end
fclose(fID);

% Write out the waypoints file; the -0.5 comes from the fact that we -1 to
% zero index and +0.5 to move to the middle of the cell.
scaledGoals = mapResolution * (goals - 0.5);
goalFileName = [mapNameRoot, '_goals.txt'];
save (goalFileName, '-ascii', 'scaledGoals');


% Now create the turtlesim file;

% pixels per m
turtleSimMapResolution = 80;

% length of side of grid square in pixels

gridSquareLength = floor(turtleSimMapResolution * mapResolution);

I2 = zeros(gridSquareLength * mapX, gridSquareLength * mapY);

% Now assign each cell 
for x = 1 : mapX,
    for y = 1 : mapY,
        idxX = (1:gridSquareLength-1)+((x-1)*gridSquareLength);
        idxY = (1:gridSquareLength-1)+((y-1)*gridSquareLength);
        I2(idxX, idxY) = map(x, y);
    end
end

% Create the turtle file name
turtleSimImageFileName = [mapNameRoot, '_turtle.png'];

imwrite(I2', turtleSimImageFileName)
