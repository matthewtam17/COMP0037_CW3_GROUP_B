# This class handles the passive planner and controller.

import rospy

from planner_controller_base import PlannerControllerBase

class PassivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)
    
    def driveToGoal(self, goal):

        # Exit if we need to
        if rospy.is_shutdown() is True:
            return False

        # Get the coal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Set the start conditions to the current position of the robot
        pose = self.controller.getCurrentPose()
        start = (pose.x, pose.y)
        startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

        # Plan a path using the current occupancy grid
        pathToGoalFound = self.planner.search(startCellCoords, goalCellCoords)

        # If we can't reach the goal, give up and return
        if pathToGoalFound is False:
            rospy.logwarn("Could not reach the goal at (%d, %d); moving to next goal", \
                          goalCellCoords[0], goalCellCoords[1])
            return False
            
        # Extract the path
        path = self.planner.extractPathToGoal()

        # Drive the path to the goal.
        goalNotReached = self.controller.drivePathToGoal(path, goal.theta, self.planner.getPlannerDrawer())

        return goalNotReached is False
