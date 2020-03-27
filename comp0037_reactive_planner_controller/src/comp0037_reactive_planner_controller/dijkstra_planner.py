from cell_based_forward_search import CellBasedForwardSearch
from Queue import PriorityQueue
import math

# This class implements Dijkstra's forward search algorithm

class DijkstraPlanner(CellBasedForwardSearch):
    
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.priorityQueue = PriorityQueue()

    # Update the cost to self cell and sort according to self cumulative cost
    def pushCellOntoQueue(self, cell):
    
        if (cell.parent is not None):
            # Work out the cost of the action from the parent to self cell
            d = self.computeLStageAdditiveCost(cell.parent, cell)
            cell.pathCost = cell.parent.pathCost + d
        else:
            cell.pathCost = 0

        key = self.computePriorityQueueKey(cell)
            
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
        # than the existing path. If so, use it instead. NOTE: This should
        d = self.computeLStageAdditiveCost(parentCell, cell)
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
            cell = tuple[1]
            key = self.computePriorityQueueKey(cell)
            newQueue.put((key, cell))
             
        self.priorityQueue = newQueue
            
    def computePriorityQueueKey(self, cell):
        
        # Compute the cost using weighted A*
        key = cell.pathCost

        return key
