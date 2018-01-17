from graphics import *
from search_grid import SearchGrid
from cell import CellLabel

class GridDrawer(object):

    def __init__(this, searchGrid, maximumGridDrawerWindowHeightInPixels):

        this.searchGrid = searchGrid;
        width = searchGrid.getWidth();
        height = searchGrid.getHeight();

        # Make sure that the height of the window is less than the specified maximum
        cellSize = min(20, maximumGridDrawerWindowHeightInPixels / height)

        # Create the window
        this.win = GraphWin("Graphics", width * cellSize , height * cellSize, autoflush = False)
        
        # Allocate the cells
        this.rectangles = [[Rectangle(Point(i * cellSize, (height - j - 1) * cellSize), \
                                      Point((i+1)*cellSize, (height - j)*cellSize)) \
                            for i in range(width)] \
                           for j in range(height)]

        for i in range(width):
            for j in range(height):
                this.rectangles[j][i].draw(this.win)
                
    def update(this):

        ### Figure out the width and height
        width = this.searchGrid.getWidth();
        height = this.searchGrid.getHeight();

        for i in range(width):
            for j in range(height):
                cellLabel = this.searchGrid.getCellFromCoords((i, j)).label
                if cellLabel == CellLabel.OBSTRUCTED:
                    color = 'purple'
                elif cellLabel == CellLabel.START:
                    color = 'green'
                elif cellLabel == CellLabel.GOAL:
                    color = 'blue'
                elif cellLabel == CellLabel.UNVISITED:
                    color = 'gray'
                elif cellLabel == CellLabel.DEAD:
                    color = 'black'
                else:
                    color = 'white'
                this.rectangles[j][i].setFill(color);

        # Flush the drawing right at the very end for speed
        this.win.flush()

    # Draw the path
    def drawPath(this, path):
        color = 'yellow'
        for p in path.waypoints:
            this.rectangles[p.coords[1]][p.coords[0]].setFill(color)
        this.win.flush()
                    
    def waitForKeyPress(this):
        # This always hangs for me:
        #this.win.getKey()
        try:
            input("Press enter to continue...")
        except SyntaxError:
            pass
        
