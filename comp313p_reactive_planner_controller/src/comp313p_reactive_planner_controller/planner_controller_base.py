# This class handles the passive planner and controller.

class PlannerControllerBase(object):

    def __init__(self, occupancyGrid, planner, controller):
        self.occupancyGrid = occupancyGrid
        self.planner = planner
        self.controller = controller
    
    def driveToGoal(self, goal):
        raise NotImplementedError()
