#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose
from math import pow,atan2,sqrt
from comp313p_planner_controller.planned_path import PlannedPath
from comp313p_planner_controller.controller_base import ControllerBase
import math
import angles

# This class defines a possible base of what the robot controller
# could do.

class Move2GoalController(ControllerBase):

    def __init__(self, occupancyGrid):
        ControllerBase.__init__(self, occupancyGrid)
        
        # Get the proportional gain settings
        self.distanceErrorGain = rospy.get_param('distance_error_gain', 0.01)
        self.angleErrorGain = rospy.get_param('distance_error_gain', 4)

    def driveToWaypoint(self, waypoint):
        vel_msg = Twist()

        dX = waypoint[0] - self.pose.x
        dY = waypoint[1] - self.pose.y
        distanceError = sqrt(dX * dX + dY * dY)
        angleError = angles.shortest_angular_distance(self.pose.theta, atan2(dY, dX))
       
        while ((distanceError >= self.distance_tolerance) & (not rospy.is_shutdown())):
            print str(distanceError) + "," + str(angleError)
            #Proportional Controller
            #linear velocity in the x-axis: only switch on when the angular error is sufficiently small
            if (math.fabs(angleError) < 1e-3):
                vel_msg.linear.x = self.distanceErrorGain * distanceError
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angleErrorGain * angleError

            #Publishing our vel_msg
            print str(vel_msg)
            self.velocityPublisher.publish(vel_msg)
            self.rate.sleep()

            # Now get the latest pose again
            dX = waypoint[0] - self.pose.x
            dY = waypoint[1] - self.pose.y
            distanceError = sqrt(dX * dX + dY * dY)
            angleError = angles.shortest_angular_distance(self.pose.theta, atan2(dY, dX))

        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocityPublisher.publish(vel_msg)
