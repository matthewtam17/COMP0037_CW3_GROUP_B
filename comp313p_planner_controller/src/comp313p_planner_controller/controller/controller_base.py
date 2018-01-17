#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from planned_path import PlannedPath
import time

# Self class defines a possible base of what the robot controller
# could do.

class ControllerBase(object):

    def __init__(self, occupancyGrid):

        # Set up the node
        rospy.init_node('stdr_controller', anonymous=True)

        # Wait for the service to start (which shows that STDR is now
        # running) and then sleep for 1.0s for good measure
#        rospy.wait_for_service('/robot0/replace')
#        rospy.sleep(1.0)
        rospy.wait_for_message('/robot0/odom', Odometry)

        # Create the node, publishers and subscriber
        self.velocityPublisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)
        self.pose = Pose()

        self.distanceTolerance = 0.01
        self.angleTolerance = 5

        # Store the occupany grid - used to transform from cell
        # coordinates to world driving coordinates
        self.occupancyGrid = occupancyGrid
        
        # Set up variables to store the pose from the robot
        self.rate = rospy.Rate(10)

    # Get the pose of the robot
    def odometryCallback(self, odometry):
        self.pose = odometry.pose.pose

    # Return the most up-to-date pose of the robot
    def getCurrentPose(self):
        return self.pose

    # Handle the logic of driving the robot to the next waypoint
    def driveToWaypoint(self, waypoint):
        raise NotImplementedError()

    # Drive to each waypoint in turn
    def drivePathToGoal(self, path):

        self.distance_tolerance = 1e-1

        for waypointNumber in range(0, len(path.waypoints)):
            cell = path.waypoints[waypointNumber]
            waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)
            rospy.loginfo("Driving to waypoint (%f, %f)", waypoint[0], waypoint[1])
            self.driveToWaypoint(waypoint)
            # Handle ^C
            if (rospy.is_shutdown()):
                break
 
