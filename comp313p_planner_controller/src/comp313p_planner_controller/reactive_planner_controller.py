# This class manages the key logic for the reactive planner and controller

import rospy

class ReactivePlannerController(object):

    def driveToGoal(self, goal):

        # Get the coal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Reactive planner main loop

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
                rospy.logwarn("Could not reach the goal at (%d, %d); moving to next goal", \
                              goalCellCoords[0], goalCellCoords[1])
                return False
            
            # Extract the path
            path = self.planner.extractPathToGoal()

            # Drive the path to the goal.
            goalNotReached = self.robotController.drivePathToGoal(path, goal.theta, self.planner.getPlannerDrawer())
            
            
