# -*- coding: utf-8 -*-

import math
 
def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))

# This class stores the occupancy grid. This is a "chessboard-like"
# representation of the environment. The environment is represented by
# a set of square cells. Each cell encodes whether that bit of the
# environment is free, or whether it is blocked. A "0" says that a
# cell is free and so the robot can travel over it. A "1" means that
# it is blocked and the robot cannot travel over it.

class OccupancyGrid(object):

    # Construct a new occupancy grid with a given width and
    # height. The resolution says the length of the side of each cell
    # in metres. By default, all the cells are set to "0" which means
    # that there are no obstacles.
    def __init__(self, widthInCells, heightInCells, resolution):
        self.widthInCells = widthInCells
        self.heightInCells = heightInCells
        self.extentInCells = (self.widthInCells, self.heightInCells)

        self.resolution = resolution

        self.width = widthInCells * self.resolution
        self.height = heightInCells * self.resolution
        self.extent = (self.width, self.height)

        self.grid = [[0 for y in range(self.heightInCells)] for y in range(self.widthInCells)]

    # Set the data from the array received from the map server. The
    # memory layout is different, so we have to flip it here. The map
    # server also scales 100 to mean free and 0 to mean blocked. We
    # use 0 for free and 1 for blocked.
    def setFromDataArrayFromMapServer(self, data):
        print len(data)
        print str(self.resolution)
        print self.width
        print self.height
        print self.widthInCells
        print self.heightInCells
        
        for x in range(self.widthInCells):
            for y in range(self.heightInCells):
                if (data[len(data)-(self.widthInCells-x-1)-self.heightInCells*y-1] == 100):
                    self.grid[x][self.heightInCells-y-1] = 1
                else:
                    self.grid[x][self.heightInCells-y-1] = 0

    # The width of the occupancy map in cells                
    def getWidth(self):
        return self.width

    # The height of the occupancy map in cells                
    def getHeight(self):
        return self.height

    # The resolution of each cell (the length of its side in metres)
    def getResolution(self):
        return self.resolution
    
    # Take a position in world coordinates (i.e., m) and turn it into
    # cell coordinates. Clamp the value so that it always falls within
    # the grid. The conversion uses integer rounding.
    def getCellCoordinatesFromWorldCoordinates(self, worldCoords):

        cellCoords = (clamp(int(worldCoords[0] / self.resolution), 0, self.widthInCells - 1), \
                      clamp(int(worldCoords[1] / self.resolution), 0, self.heightInCells - 1))
        
        return cellCoords
    
    # Convert a position in cell coordinates to world coordinates. The
    # conversion uses the centre of a cell, hence the mysterious 0.5
    # addition. No clamping is currently done.
    def getWorldCoordinatesFromCellCoordinates(self, cellCoords):

        worldCoords = ((cellCoords[0] + 0.5) * self.resolution, \
                      (cellCoords[1] + 0.5) * self.resolution)

        return worldCoords
 
    # The width of the occupancy map in cells                
    def getWidthInCells(self):
        return self.widthInCells

    # The height of the occupancy map in cells                
    def getHeightInCells(self):
        return self.heightInCells

    def getExtentInCells(self):
        return self.extentInCells

    # The resolution of each cell (the length of its side in metres)
    def getResolution(self):
        return self.resolution

    # Get the status of a cell.
    def getCell(self, x, y):
        return self.grid[x][y]

    # Set the status of a cell.
    def setCell(self, x, y, c):
        self.grid[x][y] = c
    
    # Take a position in workspace coordinates (i.e., m) and turn it into
    # cell coordinates. Clamp the value so that it always falls within
    # the grid. The conversion uses integer rounding.
    def getCellCoordinatesFromWorldCoordinates(self, worldCoords):

        cellCoords = (clamp(int(worldCoords[0] / self.resolution), 0, self.extentInCells[0]), \
                      clamp(int(worldCoords[1] / self.resolution), 0, self.extentInCells[1]))
        
        return cellCoords
    
    # Convert a position in cell coordinates to workspace coordinates. The
    # conversion uses the centre of a cell, hence the mysterious 0.5
    # addition. No clamping is currently done.
    def getWorldCoordinatesFromCellCoordinates(self, cellCoords):

        worldCoords = ((cellCoords[0] + 0.5) * self.resolution, \
                      (cellCoords[1] + 0.5) * self.resolution)

        return worldCoords
