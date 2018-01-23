from comp313p_planner_controller.cell import Cell

class SearchGrid(object):

    # This class stores the state of a search grid to illustrate forward search

    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution

    # Construct the class using an occupancy grid object
    @classmethod
    def fromOccupancyGrid(cls, occupancyGrid):

        self = cls(occupancyGrid.width, occupancyGrid.height, occupancyGrid.resolution);

        # Populate the search grid from the occupancy grid
        self.setFromOccupancyGrid(occupancyGrid)
        
        return self

    # Reset the state of the search grid to the value of the occupancy grid
    def setFromOccupancyGrid(self, occupancyGrid):
        self.grid = [[Cell((x, y), occupancyGrid.getCell(x,y)) for y in range(self.height)] \
                     for x in range(self.width)]

    def getCellFromCoords(self, coords):
        return self.grid[coords[0]][coords[1]]

    def getWidth(self):
        return self.width

    def getHeight(self):
        return self.height
