from planner_base import PlannerBase

class LIFOPlanner(PlannerBase):

    # This implements a simple LIFO search algorithm
    
    def __init__(this, occupancyGrid):
        PlannerBase.__init__(this, occupancyGrid)
        this.lifoQueue = list()

    # Simply put on the end of the queue
    def pushCellOntoQueue(this, cell):
        this.lifoQueue.append(cell)

    # Check the queue size is zero
    def isQueueEmpty(this):
        return not this.lifoQueue

    # Simply pull from the front of the list
    def popCellFromQueue(this):
        cell = this.lifoQueue.pop()
        return cell

    def resolveDuplicate(this, cell, parentCell):
        # Nothing to do in this case
        pass
