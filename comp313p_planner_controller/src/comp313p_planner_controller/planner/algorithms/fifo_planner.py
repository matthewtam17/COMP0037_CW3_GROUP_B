from planner_base import PlannerBase
from collections import deque

# This class implements the FIFO - or breadth first search - planning
# algorithm. It works by using a double ended queue: cells are pushed
# onto the back of the queue, and are popped from the front of the
# queue.

class FIFOPlanner(PlannerBase):

    # This implements a simple FIFO search algorithm
    
    def __init__(this, occupancyGrid):
        PlannerBase.__init__(this, occupancyGrid)
        this.fifoQueue = deque()

    # Simply put on the end of the queue
    def pushCellOntoQueue(this, cell):
        this.fifoQueue.append(cell)

    # Check the queue size is zero
    def isQueueEmpty(this):
        return not this.fifoQueue

    # Simply pull from the front of the list
    def popCellFromQueue(this):
        cell = this.fifoQueue.popleft()
        return cell

    def resolveDuplicate(this, cell, parentCell):
        # Nothing to do in this case
        pass
