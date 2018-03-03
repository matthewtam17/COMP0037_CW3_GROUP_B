#!/usr/bin/env python

import sys
import rospy
import math
import tf
import copy

import numpy as np
from nav_msgs.srv import GetMap
from comp313p_reactive_planner_controller.occupancy_grid import OccupancyGrid
from comp313p_reactive_planner_controller.grid_drawer import OccupancyGridDrawer
from comp313p_reactive_planner_controller.cell import CellLabel
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from threading import Lock
from geometry_msgs.msg  import Twist

# This class implements basic mapping capabilities. Given knowledge
# about the robot's position and orientation, it processes laser scans
# to produce a new occupancy grid. If this grid differs from the
# previous one, a new grid is created and broadcast.
#


class MapperNode(object):

    def __init__(self):

        rospy.init_node('mapper_node', anonymous=True)
	rospy.wait_for_service('static_map')
	rospy.loginfo('------> 0')
        self.mapServer = rospy.ServiceProxy('static_map', GetMap)
	rospy.loginfo('------> 1')
        resp = self.mapServer()
        self.occupancyGrid = OccupancyGrid(resp.map.info.width, resp.map.info.height, resp.map.info.resolution, 0.5)
	self.occupancyGrid.setScale(rospy.get_param('plan_scale',5))
	self.occupancyGrid.scaleEmptyMap()
                         
        self.deltaOccupancyGrid = OccupancyGrid(resp.map.info.width, resp.map.info.height, resp.map.info.resolution, 0)
	self.deltaOccupancyGrid.setScale(rospy.get_param('plan_scale',5))
	self.deltaOccupancyGrid.scaleEmptyMap()

	rospy.loginfo('------> 2')
	rospy.loginfo('------> 3')
        self.occupancyGridDrawer = OccupancyGridDrawer('Mapper Node Occupancy Grid',\
                                                       self.occupancyGrid,rospy.get_param('maximum_window_height_in_pixels', 700))
	self.occupancyGridDrawer.open()
        self.deltaOccupancyGridDrawer = OccupancyGridDrawer('Mapper Node Delta Occupancy Grid',\
                                                            self.deltaOccupancyGrid,rospy.get_param('maximum_window_height_in_pixels', 700))
	self.deltaOccupancyGridDrawer.open()
	rospy.loginfo('------> 4')

        # Set up the subscribers. These track the robot position, speed and laser scans.
        self.mostRecentOdometry = Odometry()
        self.odometrySubscriber = rospy.Subscriber("robot0/odom", Odometry, self.odometryCallback, queue_size=1)
        self.mostRecentTwist = Twist();
        self.twistSubscriber = rospy.Subscriber('/robot0/cmd_vel', Twist, self.twistCallback, queue_size=1)
        self.laseSubscriber = rospy.Subscriber("robot0/laser_0", LaserScan, self.parseScanCallback, queue_size=1)
        
        # Set up the lock to ensure thread safety
        self.dataCopyLock = Lock()

        rospy.loginfo('------> Initialised')

    def odometryCallback(self, msg):
        self.dataCopyLock.acquire()
        self.mostRecentOdometry = msg
        self.dataCopyLock.release()

    def twistCallback(self, msg):
        self.dataCopyLock.acquire()
        self.mostRecentVelocity = msg
        self.dataCopyLock.release()

    # Predict the pose of the robot to the current time. This is to
    # hopefully make the pose of the robot a bit more accurate. The
    # equation is: currentPose = lastPose + dT * lastTwist. Note this
    # isn't quite right. e.g. a more proper implementation would store
    # a history of velocities and interpolate over them.
    
    def predictPose(self, predictTime):

        # Copy the last odometry and velocity
        self.dataCopyLock.acquire()
        currentPose = copy.deepcopy(self.mostRecentOdometry.pose.pose)
        currentPoseTime = self.mostRecentOdometry.header.stamp.to_sec()
        currentTwist = copy.deepcopy(self.mostRecentTwist)
        self.dataCopyLock.release()

        dT = predictTime - currentPoseTime

        quaternion = (currentPose.orientation.x, currentPose.orientation.y,
                      currentPose.orientation.z, currentPose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        theta = euler[2]
        
        # These are the "ideal motion model" prediction equations from
        # stdr which attempt to accurately describe the trajectory of
        # the robot if it turns as it moves. The equations are precise
        # if the angular and linear velocity is constant over the
        # prediction interval.

        if (abs(currentTwist.angular.z) < 1e-6):
            x = currentPose.position.x + dT * currentTwist.linear.x * math.cos(theta)
            y = currentPose.position.y + dT * currentTwist.linear.x * math.sin(theta)
        else:
            x = currentPose.position.x - currentTwist.linear.x / currentTwist.angular.z * sin(theta) + \
                currentTwist.linear.x / currentTwist.angular.z * sin(theta + dT * currentTwist.angular.z)

            y = currentPose.position.y - currentTwist.linear.x / currentTwist.angular.z * cos(theta) + \
                currentTwist.linear.x / currentTwist.angular.z * cos(theta + dT * currentTwist.angular.z);

        theta = theta + currentTwist.angular.z * dT

        return x, y, theta

        
    def parseScanCallback(self, msg):

        # Predict the robot pose to the time the scan was taken
        x, y, theta = self.predictPose(msg.header.stamp.to_sec())
        
        gridHasChanged = False

        # For each ray, check the range is good. If so, check all the
        # cells along the ray and mark cells as either open or
        # blocked. To get around numerical issues, we trace along each
        # ray in turn and terminate when we hit the first obstacle or at the end of the ray.
        for ii in range(int(math.floor((msg.angle_max - msg.angle_min) / msg.angle_increment))):
            # rospy.loginfo("{} {} {}".format(msg.ranges[ii],msg.angle_min,msg.angle_max))

            detectedRange = msg.ranges[ii]
            
            # If the detection is below the minimum range, assume this ray is busted and continue
            if (detectedRange < msg.range_min):
                continue

            rayEndsOnObject = True

            # If the detection range is beyond the end of the sensor,
            # this is the mark which says that nothing was detected
            if detectedRange >= msg.range_max:
                rayEndsOnObject = False
                detectedRange = msg.range_max

            # Get the angle of this ray
            angle = msg.angle_min + msg.angle_increment * ii + theta

            # Get the list of cells which sit on this ray. The ray is
            # scaled so that the last sell is at the detected range
            # from the sensor.
            between = self.ray_trace(detectedRange, x, y, angle, msg)

            # Traverse along the ray and set cells. We can only change
            # cells from unknown (0.5) to free. If we encounter a
            # blocked cell, terminate. Sometimes the ray can slightly
            # extend through a blocked cell due to numerical rounding
            # issues.
            traversedToEnd = True
            for point in between:
                try:
                    if self.occupancyGrid.getCell(point[0], point[1]) > 0.5:
                        traversedToEnd = False
                        break
                    
                    if self.occupancyGrid.getCell(point[0], point[1]) == 0.5:
                        self.occupancyGrid.setCell(point[0], point[1], 0)
                        self.deltaOccupancyGrid.setCell(point[0], point[1], 1.0)
                        gridHasChanged = True
                except IndexError as e:
                    print(e)
                    print "between: " + str(point[0]) + ", " + str(point[1])

            # If we got to the end okay, see if we have to mark the
            # state of the end cell to occupied or not. To do this, we 

            # Note that we can change a cell
            # from unknown and free to occupied, but we cannot change
            # the state from occupied back to anything else. This gets
            # around the issue that there can be "blinking" between
            # whether a cell is occupied or not.
            if (traversedToEnd is True) & (rayEndsOnObject is True):
                lastPoint = between[-1]
                if self.occupancyGrid.getCell(lastPoint[0], lastPoint[1]) < 1.0:
                    self.occupancyGrid.setCell(point[0], point[1], 1)
                    self.deltaOccupancyGrid.setCell(point[0], point[1], 1.0)

    def ray_trace(self, dist, x, y, angle, scanmsg):
        """
        Function to get a list of points between two points
        :param origin: position of the origin in world coordinates
        :param dist: distance to end point
        :param angle: angle from robot
        :param scanmsg: Laser Scan message
        :return: list of points in between the origin and end point
        """
        points = []

        space = np.linspace(scanmsg.range_min, dist, scanmsg.range_max * 5)
        for a in space:
            point_world_coo = [math.cos(angle) * a + x,
                               math.sin(angle) * a + y]
            points.append(self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(point_world_coo))
        return points
        
    def update_visualisation(self):	 
	self.occupancyGridDrawer.update()
	self.deltaOccupancyGridDrawer.update()
        self.deltaOccupancyGrid.clearMap(0)
	
    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(1)
            self.update_visualisation()
        
  

  
