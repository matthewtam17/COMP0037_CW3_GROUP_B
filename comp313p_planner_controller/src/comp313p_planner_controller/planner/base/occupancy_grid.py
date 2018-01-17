from helpers import clamp

# This class stores the occupancy grid. This is a "chessboard-like"
# representation of the environment. The environment is represented by
# a set of square cells. Each cell encodes whether that bit of the
# environment is free, or whether it is blocked. A "0" says that a
# cell is free and so the robot can travel over it. A "1" means that
# it is blocked and the robot cannot travel over it.

class OccupancyGrid(object):

    # Construct a new occupancy grid with a given width and
    # height. The resolution says the lenght of the side of each cell
    # in metres. By default, all the cells are set to "0" which means
    # that there are no obstacles.
    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.data = [[0 for x in range(width)] for y in range(height)]

    # Set the data from the array received from the map server. The
    # memory layout is different, so we have to flip it here. The map
    # server also scales 100 to mean free and 0 to mean blocked. We
    # use 0 for free and 1 for blocked.
    def setFromDataArrayFromMapServer(self, data):
        for x in range(self.width):
            for y in range(self.height):
                if (data[len(data)-(self.height-y-1)-self.width*x-1] == 100):
                    self.data[x][y] = 1
                else:
                    self.data[x][y] = 0

    # The width of the occupancy map in cells                
    def getWidth(self):
        return self.width

    # The height of the occupancy map in cells                
    def getHeight(self):
        return self.height

    # The resolution of each cell (the length of its side in metres)
    def getResolution(self):
        return self.resolution

    # Get the status of a cell.
    def getCell(self, x, y):
        return self.data[y][x]

    # Set the status of a cell.
    def setCell(self, x, y, c):
        self.data[y][x] = c
    
    # Take a position in world coordinates (i.e., m) and turn it into
    # cell coordinates. Clamp the value so that it always falls within
    # the grid. The conversion uses integer rounding.
    def getCellCoordinatesFromWorldCoordinates(self, worldCoords):

        cellCoords = (clamp(int(worldCoords[0] / self.resolution), 0, self.width - 1), \
                      clamp(int(worldCoords[1] / self.resolution), 0, self.height - 1))
        
        return cellCoords
    
    # Convert a position in cell coordinates to world coordinates. The
    # conversion uses the centre of a cell, hence the mysterious 0.5
    # addition. No clamping is currently done.
    def getWorldCoordinatesFromCellCoordinates(self, cellCoords):

        worldCoords = ((cellCoords[0] + 0.5) * self.resolution, \
                      (cellCoords[1] + 0.5) * self.resolution)

        return worldCoords
