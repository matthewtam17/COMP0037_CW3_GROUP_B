from cell import Cell

class SearchGrid(object):

    # This class stores the state of a search grid to illustrate forward search

    def __init__(this, width, height, resolution):
        this.width = width
        this.height = height
        this.resolution = resolution

    # Construct the class using an occupancy grid object
    @classmethod
    def fromOccupancyGrid(cls, occupancyGrid):

        this = cls(occupancyGrid.width, occupancyGrid.height, occupancyGrid.resolution);

        # Populate the search grid from the occupancy grid
        this.setFromOccupancyGrid(occupancyGrid)
        
        return this

    # Reset the state of the search grid to the value of the occupancy grid
    def setFromOccupancyGrid(this, occupancyGrid):
        this.grid = [[Cell((x, y), occupancyGrid.getCell(x,y)) for y in range(this.height)] \
                     for x in range(this.width)]

    def getCellFromCoords(this, coords):
        return this.grid[coords[0]][coords[1]]

    def getWidth(this):
        return this.width

    def getHeight(this):
        return this.height
