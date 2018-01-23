from comp313p_planner_controller.planner_base import PlannerBase

class LIFOPlanner(PlannerBase):

    # This implements a simple LIFO search algorithm
    
    def __init__(self, occupancyGrid):
        PlannerBase.__init__(self, occupancyGrid)
        self.lifoQueue = list()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self.lifoQueue.append(cell)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.lifoQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.lifoQueue.pop()
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in this case
        pass
