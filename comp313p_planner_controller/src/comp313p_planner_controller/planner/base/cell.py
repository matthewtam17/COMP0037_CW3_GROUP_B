from enum import Enum

# Enumeration to give the cell label

class CellLabel(Enum):
    OBSTRUCTED=-3
    START=-2
    GOAL=-1
    UNVISITED=0
    DEAD=1
    ALIVE=2

# This class stores information about each cell - its coordinates in the grid,
# its label, and the path cost to reach it

class Cell(object):

    def __init__(this, coords, isOccupied):

        # Set coordinates
        this.coords = coords

        # Label the cell. If it is known to be obstructed, mark it as
        # such. Otherwise, assume it is free and mark as unvisited.
        if (isOccupied > 0):
            this.label = CellLabel.OBSTRUCTED;
        else:
            this.label = CellLabel.UNVISITED

        # Initially the cell has no parents.
        this.parent = None

        # The initial path cost is infinite. For algorithms that need
        # it, this is the necessary initial condition.
        this.pathCost = float("inf")
    
