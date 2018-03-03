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
        #self.deltaOccupancyGridDrawer = OccupancyGridDrawer('Mapper Node Delta Occupancy Grid',\
#                                                            self.deltaOccupancyGrid,rospy.get_param('maximum_window_height_in_pixels', 700))
	#self.deltaOccupancyGridDrawer.open()
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
        print 'odometry: ' + str(msg.header.stamp)
        self.dataCopyLock.release()

    def twistCallback(self, msg):
        self.dataCopyLock.acquire()
        self.mostRecentVelocity = msg
        print 'twist: ' + str(rospy.Time().now())
        self.dataCopyLock.release()


    def predictPose(self, predictTime):

        # Copy the last odometry and velocity
        self.dataCopyLock.acquire()
        currentPose = copy.deepcopy(self.mostRecentOdometry.pose.pose)
        currentPoseTime = self.mostRecentOdometry.header.stamp.to_sec()
        currentTwist = copy.deepcopy(self.mostRecentTwist)
        self.dataCopyLock.release()

        dT = predictTime - currentPoseTime

        print str(dT)

        quaternion = (currentPose.orientation.x, currentPose.orientation.y,
                      currentPose.orientation.z, currentPose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        theta = euler[2]
        
        # Motion model adapted from stdr
        # if (abs(mostRecentVelocity.angular.z) < 1e-6):
        x = currentPose.position.x + dT * currentTwist.linear.x * math.cos(theta)
        y = currentPose.position.y + dT * currentTwist.linear.x * math.sin(theta)
        #else:
        # #   currentPose.position.x = currentPose.position.x - currentTwist.linear.x / currentTwist.angular.z * \
        # #                            sin(theta) + currentTwist.linear.x / currentTwist.angular.z * \
        # #                            sinf(theta + dT * currentTwist.angular.z);

        #    currentPose.position.y = currentPose.position.y - currentTwist.linear.x / currentTwist.angular.z * \
        #                             sin(theta) + currentTwist.linear.x / currentTwist.angular.z * \
        #                             sinf(theta + dT * currentTwist.angular.z);

        #_pose.y -= - _currentTwist.linear.x / _currentTwist.angular.z * 
        # cosf(_pose.theta) + 
        #_currentTwist.linear.x / _currentTwist.angular.z * 
        #cosf(_pose.theta + dt.to_sec() * _currentTwist.angular.z);
        #    }
        theta = theta + currentTwist.angular.z * dT;

        return x, y, theta

        
    def parseScanCallback(self, msg):

        print 'scan: ' + str(msg.header.stamp)

        # Prediction interval; note we do not handle the velocity properly
        x, y, theta = self.predictPose(msg.header.stamp.to_sec())
        
        occupied_points = []
        gridHasChanged = False

        for ii in range(int(math.floor((msg.angle_max - msg.angle_min) / msg.angle_increment))):
            # rospy.loginfo("{} {} {}".format(msg.ranges[ii],msg.angle_min,msg.angle_max))
            valid = (msg.ranges[ii] > msg.range_min) and (msg.ranges[ii] < msg.range_max)

            angle = msg.angle_min + msg.angle_increment * ii + theta

            # If the range is valid find the end point add it to occupied points and set the distance for ray tracing
            if valid:
                point_world_coo = [math.cos(angle) * msg.ranges[ii] + x,
                                   math.sin(angle) * msg.ranges[ii] + y]

                occupied_points.append(self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(point_world_coo))
                dist = msg.ranges[ii]
            else:
                dist = msg.range_max
            between = self.ray_trace(dist- 0.1, x, y, angle, msg)

            for point in between:
                try:
                    if self.occupancyGrid.getCell(point[0], point[1]) != 0.0:
                        self.occupancyGrid.setCell(point[0], point[1], 0)
                        self.deltaOccupancyGrid.setCell(point[0], point[1], 1.0)
                        gridHasChanged = True
                except IndexError as e:
                    print(e)
                    print "between: " + str(point[0]) + ", " + str(point[1])

        for point in occupied_points:
            try:
                if self.occupancyGrid.getCell(point[0], point[1]) != 1.0:
                    self.occupancyGrid.setCell(point[0], point[1], 1.0)
                    self.deltaOccupancyGrid.setCell(point[0], point[1], 1.0)
                    gridHasChanged = True
            except IndexError as e:
                print(e)
                print "occupied_points: " + str(point[0]) + ", " + str(point[1])

#        if gridHasChanged is True:
#            print "grid has changed"

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
	#self.deltaOccupancyGridDrawer.update()
        self.deltaOccupancyGrid.clearMap(0)
	
    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(1)
            self.update_visualisation()
        
  

  
