# This class manages the key logic for the reactive planner and controller

import rospy
from planner_controller_base import PlannerControllerBase

class ReactivePlannerController(PlannerControllerBase):

    def driveToGoal(self, goal):

        # Get the coal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.

        goalNotReached = True
        
        while goalNotReached is True:

            # If ROS is shutting down, break the loop and exit
            if rospy.is_shutdown() is True:
                return False

            # Set the start conditions to the current position of the robot
            pose = self.robotController.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

            # Plan a path using the current occupancy grid
            pathToGoalFound = self.planner.search(startCellCoords, goalCellCoords)

            # If we can't reach the goal, give up and return
            if pathToGoalFound is False:
                rospy.logwarn("Could not find a path to the goal at (%d, %d)", \
                              goalCellCoords[0], goalCellCoords[1])
                return False
            
            # Extract the path
            path = self.planner.extractPathToGoal()

            # Drive along the path the goal
            goalNotReached = self.robotController.drivePathToGoal(path, goal.theta, self.planner.getPlannerDrawer())
            
            
