#!/usr/bin/env python
import rospy
import threading
import math

# NOTE: THERE ARE SOME VERY, VERY BAD HACKS HERE WITH THREADING AT THE
# MOMENT. WE'LL HAVE TO SEE IF WE CAN GET AWAY WITH IT IN PYTHON. THE
# MAIN PROBLEM IS THAT WE HAVE TO RUN THE GUI IN THE MAIN THREAD, BUT
# ROS PUTS ALL OF THE MESSAGE HANDLERS IN A SEPARATE THREAD. WHAT'S
# DONE RIGHT NOW COULD BE GENEROUSLY DESCRIBED AS A CHAOTIC MESS.

# Robot pose
from geometry_msgs.msg import Pose

# Response we get from the map server when we ask for the map
from nav_msgs.srv import GetMap

from comp0037_mapper.srv import RequestMapUpdate

# The service messages this node sends. This is actually a report
# that the robot has reached its goal
from comp0037_reactive_planner_controller.srv import *

# The occupancy grid, used to store our representation of the map
from comp0037_reactive_planner_controller.occupancy_grid import OccupancyGrid

from comp0037_reactive_planner_controller.passive_planner_controller import PassivePlannerController
from comp0037_reactive_planner_controller.reactive_planner_controller import ReactivePlannerController
from comp0037_reactive_planner_controller.a_star_planner import AStarPlanner
from comp0037_reactive_planner_controller.move2goal_controller import Move2GoalController

# This class is the main node and orchestrates everything else

class PlannerControllerNode(object):

    def __init__(self):
        rospy.init_node('planner_controller', anonymous=True)
        self.waitForGoal =  threading.Condition()
        self.waitForDriveCompleted =  threading.Condition()
        self.goal = None
        self.goalReached = False

    
    def createOccupancyGridFromMapServer(self):
        
        # Get the map service
        #rospy.loginfo('Waiting for static_map to become available.')
        #rospy.wait_for_service('static_map') 
        #self.mapServer = rospy.ServiceProxy('static_map', GetMap)
        #rospy.loginfo('Found static_map service')

        # Wait for the 
        rospy.loginfo('Waiting for the request_map_update service')
        rospy.wait_for_service('request_map_update')
        rospy.loginfo('request_map_update service registered')
        
        # Now pull the rest of the data from the mapping node
        mapRequestService = rospy.ServiceProxy('request_map_update', RequestMapUpdate)
        mapUpdate = mapRequestService(False)

        self.occupancyGrid = OccupancyGrid.fromMapUpdateMessage(mapUpdate.initialMapUpdate)
        #self.occupancyGrid.setScale(rospy.get_param('plan_scale', 5))

        # Debug code
        if False:
            uniqueCellValues1 = []
            for x in range(self.occupancyGrid.widthInCells):
                for y in range(self.occupancyGrid.heightInCells):
                    if self.occupancyGrid.grid[x][y] not in uniqueCellValues1:
                        uniqueCellValues1.append(self.occupancyGrid.grid[x][y])

            print '**************************************************************************************'
            print str(uniqueCellValues1)
            print '**************************************************************************************'


    def mapUpdateCallback(self, msg):
        rospy.loginfo("******************************** map update received")
        self.plannerController.handleMapUpdateMessage(msg)
        
    def createPlanner(self):
        self.planner = AStarPlanner('A*', self.occupancyGrid)
        self.planner.setPauseTime(0)
        self.planner.windowHeightInPixels = rospy.get_param('maximum_window_height_in_pixels', 700)

        removeGoalCellFromPathIfOccupied = rospy.get_param('remove_goal_cell_from_path_if_occupied', False)
        self.planner.setRemoveGoalCellFromPathIfOccupied(removeGoalCellFromPathIfOccupied)
        
    def createRobotController(self):
        self.robotController = Move2GoalController(self.occupancyGrid)

    def createPlannerController(self):
        if rospy.get_param('use_reactive_planner_controller', False) is True:
            self.plannerController = ReactivePlannerController(self.occupancyGrid, self.planner, self.robotController)
        else:
            self.plannerController = PassivePlannerController(self.occupancyGrid, self.planner, self.robotController)

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

        return GoalResponse(self.goalReached)

    def waitForObstacleToClear(self):
        return True
    
    def run(self):

        # First set up the occupancy grid
        self.createOccupancyGridFromMapServer()

        # Create the planner
        self.createPlanner()
        
        # Set up the robot controller
        self.createRobotController()

        # Set up the planner controller, which puts the two together
        self.createPlannerController()

        # Set up the wait for the service. Note that we can't directly
        # handle all the driving operations in the service
        # handler. The reason is that the planner can create a GUI,
        # and self MUST run in the main thread. The result is pretty
        # ugly logic and can lead to deadlocking.
        service = rospy.Service('drive_to_goal', Goal, self.handleDriveToGoal)

        rospy.loginfo('Spinning to service goal requests')
        
        while not rospy.is_shutdown():

            # Wait for a new goal. Allow at most 0.1s, which gives
            # time to check if we are shutting down
            self.waitForGoal.acquire()
            self.waitForGoal.wait(0.1)
            self.waitForGoal.release()

            # If no goal has been allocated, cycle around
            if (self.goal is None):
                continue

            # Try to drive down this aisle to the final goal
            self.goalReached = self.plannerController.driveToGoal(self.goal)
                
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
