#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose
from math import pow,atan2,sqrt
from comp313p_planner_controller.planned_path import PlannedPath
from comp313p_planner_controller.controller_base import ControllerBase

# This class defines a possible base of what the robot controller
# could do.

class Move2GoalController(ControllerBase):

    def __init__(self, occupancyGrid):
        ControllerBase.__init__(self, occupancyGrid)

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def shortestAngularDistance(self, fromAngle, toAngle):
        delta = toAngle - fromAngle
        if (delta < -math.pi):
            delta = delta + 2.0*math.pi
        elif(delta > math.pi):
            delta = delta - 2.0*math.pi
        return delta

    def driveToWaypoint(self, waypoint):
        vel_msg = Twist()

        distanceError = sqrt(pow((waypoint[0] - self.pose.x), 2) + pow((waypoint[1] - self.pose.y), 2))
        angleError = self.shortestAngularDistance(self.pose.theta, atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))
       
        while ((distanceError >= self.distance_tolerance) & (not rospy.is_shutdown())):
            #Proportional Controller
            #linear velocity in the x-axis: only switch on when the angular error is sufficiently small
            if (math.fabs(angleError) < 1e-3):
                vel_msg.linear.x = distanceError
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 4 * angleError

            #Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            self.rate.sleep()
            distanceError = sqrt(pow((waypoint[0] - self.pose.x), 2) + pow((waypoint[1] - self.pose.y), 2))
            angleError = self.shortestAngularDistance(self.pose.theta, atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))

        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocityPublisher.publish(vel_msg)
