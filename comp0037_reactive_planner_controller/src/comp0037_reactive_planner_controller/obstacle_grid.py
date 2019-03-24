
import math
import rospy
import copy
from comp0037_reactive_planner_controller.planner_controller_base import OccupancyGrid
 
# This class  extends the OccupancyGrid to support a parallel grid where the resulting grid has an OR relationship
# stores the occupancy grid. This is a "chessboard-like"
# representation of the environment. The environment is represented by
# a set of square cells. Each cell encodes whether that bit of the
# environment is free, or whether it is blocked. A "0" says that a
# cell is free and so the robot can travel over it. A "1" means that
# it is blocked and the robot cannot travel over it.

class ObstacleGrid(OccupancyGrid):

    # Construct a new occupancy grid with a given width and
    # height. The resolution says the length of the side of each cell
    # in metres. By default, all the cells are set to "0" which means
    # that there are no obstacles.
    def __init__(self, widthInCells, heightInCells, resolution, initialValue = 0.0):
        super(ObstacleGrid, self).__init__(widthInCells, heightInCells, resolution, initialValue)        
        self.obstacle_grid = [[initialValue for y in range(self.heightInCells)] for x in range(self.widthInCells)]   
           
    def scaleEmptyMap(self):
        # redo based on self.grid
        pass

    def clearMap(self, initialValue=0):
        # ADD DELETING OF
        pass
            
                         
    def scaleMap(self):
        # make into static and apply to both obstacle map and 
        pass

    # Get the status of a cell.
    def getCell(self, x, y):        
        return int(self.grid[x][y] or self.obstacle_grid[x][y])

    # Set the status of a cell.
    def setCell(self, x, y, c):        
        self.grid[x][y] = c
    