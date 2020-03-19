from cell_based_forward_search import CellBasedForwardSearch
from Queue import PriorityQueue
import math

# This class implements A* with the octile heuristic.

class AStarOctilePlanner(CellBasedForwardSearch):
    
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.priorityQueue = PriorityQueue()

    # Put the cell on the queue, using the path cost as the key to
    # determine the search order
    def pushCellOntoQueue(self, cell):
    
        if (cell.parent is not None):
            # Work out the cost of the action from the parent to this cell
            d = self.computeLStageAdditiveCost(cell.parent, cell)
            cell.pathCost = cell.parent.pathCost + d
        else:
            cell.pathCost = 0

        # Octile heuristic (hard coded)
        dX = abs(cell.coords[0] - self.goal.coords[0])
        dY = abs(cell.coords[1] - self.goal.coords[1])
        octileDistance = max(dX, dY) +(math.sqrt(2) -1) * min(dX, dY)
        
        key = cell.pathCost + octileDistance
            
        self.priorityQueue.put((key, cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.priorityQueue.empty()

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        tuple = self.priorityQueue.get()
        return tuple[1]

    def resolveDuplicate(self, cell, parentCell):

        # See if the cost from the parent cell to this cell is shorter
        # than the existing path. If so, use it instead.
        d = self.computeLStageAdditiveCost(cell.parent, cell)
        pathCostThroughNewParent = parentCell.pathCost + d
        if (pathCostThroughNewParent < cell.pathCost):
            cell.parent = parentCell
            cell.pathCost = pathCostThroughNewParent
            self.reorderPriorityQueue()

    # Reorder the queue. I don't see a clean way to do this.  Here I
    # just blindly create a new queue and copy over.  Another approach
    # is to transform into a list, heapify and transform back. People
    # have also used lambdas and sort functions.
    def reorderPriorityQueue(self):
        newQueue = PriorityQueue()

        while self.priorityQueue.empty() is False:
            tuple = self.priorityQueue.get()
            newQueue.put(tuple)
             
        self.priorityQueue = newQueue
