# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot mtion.

import rospy
from planner_controller_base import PlannerControllerBase

class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)

    def handleMapUpdate(self, mapUpdateMessage):

        # Update the occupancy grid
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()

        # Get the new search grid. This has been updated to account
        # for the occupancy grid and robot size.
        currentSearchGrid = self.planner.getSearchGrid()

        # Check if the current planned path is good
        
        # If not, 

        pass
    
    def driveToGoal(self, goal):

        # Get the coal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.

        goalNotReached = True
        
        while (goalNotReached is True) & (rospy.is_shutdown() is False):

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
            self.currentPlannedPath = self.planner.extractPathToGoal()

            # Drive along the path the goal
            goalNotReached = self.robotController.drivePathToGoal(self.currentPlannedPath, goal.theta, self.planner.getPlannerDrawer())
            
            
