from graphics import *
from search_grid import SearchGrid
from cell import CellLabel

class GridDrawer(object):

    def __init__(self, searchGrid, maximumGridDrawerWindowHeightInPixels):

        self.searchGrid = searchGrid;
        width = searchGrid.getWidth();
        height = searchGrid.getHeight();

        # Make sure that the height of the window is less than the specified maximum
        cellSize = min(20, maximumGridDrawerWindowHeightInPixels / height)

        # Create the window
        self.win = GraphWin("Graphics", width * cellSize , height * cellSize, autoflush = False)
        
        # Allocate the cells
        self.rectangles = [[Rectangle(Point(i * cellSize, (height - j - 1) * cellSize), \
                                      Point((i+1)*cellSize, (height - j)*cellSize)) \
                            for i in range(width)] \
                           for j in range(height)]

        for i in range(width):
            for j in range(height):
                self.rectangles[j][i].draw(self.win)
                
    def update(self):

        ### Figure out the width and height
        width = self.searchGrid.getWidth();
        height = self.searchGrid.getHeight();

        for i in range(width):
            for j in range(height):
                cellLabel = self.searchGrid.getCellFromCoords((i, j)).label
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
                self.rectangles[j][i].setFill(color);

        # Flush the drawing right at the very end for speed
        self.win.flush()

    # Draw the path
    def drawPath(self, path):
        color = 'yellow'
        for p in path.waypoints:
            self.rectangles[p.coords[1]][p.coords[0]].setFill(color)
        self.win.flush()
                    
    def waitForKeyPress(self):
        # This always hangs for me:
        #self.win.getKey()
        try:
            input("Press enter to continue...")
        except SyntaxError:
            pass
        
