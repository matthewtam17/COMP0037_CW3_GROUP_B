#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.srv import GetMap
from comp313p_planner_controller.planner.base.occupancy_grid import OccupancyGrid
from comp313p_planner_controller.planner.graphics.grid_drawer import GridDrawer
from comp313p_planner_controller.planner.algorithms.fifo_planner import FIFOPlanner
from comp313p_planner_controller.controller.move2goal_controller import Move2GoalController
from comp313p_planner_controller.srv import *
import threading

# Self class interfaces with the planner and the controller
class PlannerControllerNode(object):

    def __init__(self):
        rospy.init_node('planner_controller', anonymous=True)
        
        self.waitForGoal =  threading.Condition()
        self.waitForDriveCompleted =  threading.Condition()
        self.goal = None
        pass
    
    def createOccupancyGridFromMapServer(self):
        # Get the map service
        rospy.loginfo('Waiting for static_map to become available.')
        rospy.wait_for_service('static_map') 
        self.mapServer = rospy.ServiceProxy('static_map', GetMap)
        rospy.loginfo('Found static_map; requesting map data')

        # Query the map status
        response = self.mapServer()
        map = response.map
        rospy.loginfo('Got map data')

        # Allocate the occupancy grid and set the data from the array sent back by the map server
        self.occupancyGrid = OccupancyGrid(map.info.width, map.info.height, map.info.resolution)
        self.occupancyGrid.setFromDataArrayFromMapServer(map.data)

    def createPlanner(self):
        self.planner = FIFOPlanner(self.occupancyGrid)
        self.planner.setPauseTime(0)
        self.planner.maximumGridDrawerWindowHeightInPixels = rospy.get_param('maximum_window_height_in_pixels', 700)
        
    def createRobotController(self):
        self.robotController = Move2GoalController(self.occupancyGrid)

    def handleDriveToGoal(self, goal):
        # Report to the main loop that we have a new goal
        self.waitForGoal.acquire()
        self.goal = goal
        self.waitForGoal.notify()
        self.waitForGoal.release()

        # Wait until the robot has finished driving
        self.waitForDriveCompleted.acquire()
        self.waitForDriveCompleted.wait()
        self.waitForDriveCompleted.release()

        return GoalResponse(True)
        

    def driveToGoal(self, goal):

        # Get the current pose of the robot
        pose = self.robotController.getCurrentPose()
        start = (pose.x, pose.y)

        print "start = " + str(start)
        print "goal = " + str(goal)
        
        # Call the planner
        startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        print "startCellCoords = " + str(startCellCoords)
        print "goalCellCoords = " + str(goalCellCoords)

        # Get the plan
        goalReached = self.planner.plan(startCellCoords, goalCellCoords)

        # If we can't reach the goal, give up and return
        if (goalReached == False):
            rospy.logwarn("Could not reach the goal at (%d, %d); moving to next goal", \
                          goalCellCoords[0], goalCellCoords[1])
            self.planner.gridDrawer.waitForKeyPress()
            return False
        
        # Extract the path
        path = self.planner.extractPathToGoal()

        # Now drive it
        self.robotController.drivePathToGoal(path)

        return True
    
    def run(self):

        # First set up the occupancy grid
        self.createOccupancyGridFromMapServer()

        # Create the planner
        self.createPlanner()
        
        # Set up the robot controller
        self.createRobotController()

        # Set up the wait for the service. Note that we can't directly
        # handle all the driving operations in the service
        # handler. The reason is that the planner can create a GUI,
        # and self MUST run in the main thread. The result is pretty
        # ugly logic and can lead to deadlocking.
        service = rospy.Service('drive_to_goal', Goal, self.handleDriveToGoal)

        print 'Spinning to service goal requests'
        
        while not rospy.is_shutdown():

            # Wait for a new goal. Allow at most 0.1s, which gives
            # time to check if we are shutting down
            self.waitForGoal.acquire()
            self.waitForGoal.wait(0.1)
            self.waitForGoal.release()

            # If no goal has been allocated, cycle around
            if (self.goal is None):
                continue

            self.driveToGoal(self.goal)
            self.goal = None

            # Signal back to the service handler that we are done
            self.waitForDriveCompleted.acquire()
            self.waitForDriveCompleted.notify()
            self.waitForDriveCompleted.release()

if __name__ == '__main__':
    try:
        plannerController = PlannerControllerNode()
        plannerController.run()
    except rospy.ROSInterruptException:
        pass
