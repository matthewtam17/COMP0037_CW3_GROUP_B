#!/usr/bin/env python

import sys
import rospy
import math
import tf
import copy

import numpy as np
from nav_msgs.srv import GetMap
from comp313p_reactive_planner_controller.occupancy_grid import OccupancyGrid
from comp313p_reactive_planner_controller.search_grid import SearchGrid
from comp313p_reactive_planner_controller.grid_drawer import OccupancyGridDrawer
from comp313p_reactive_planner_controller.grid_drawer import SearchGridDrawer
from comp313p_reactive_planner_controller.cell import CellLabel
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

#This class creates a simulation of SLAM data:
# uses True data to create a map that is initialised from robot starting location and publishes robot odometry
#  that is initialised at starting location

class Mock_Slam_Node(object):

    def __init__(self):
        rospy.init_node('mock_slam_node', anonymous=True)
	rospy.wait_for_service('static_map')
	rospy.loginfo('------> 0')
        self.mapServer = rospy.ServiceProxy('static_map', GetMap)
	rospy.loginfo('------> 1')
        resp = self.mapServer()
        self.occupancyGrid = OccupancyGrid(resp.map.info.width, resp.map.info.height, resp.map.info.resolution, 0.5)
        
	self.occupancyGrid.setScale(rospy.get_param('plan_scale',5))
	self.occupancyGrid.scaleEmptyMap()
	rospy.loginfo('------> 2')
        self.searchGrid = SearchGrid.fromOccupancyGrid(self.occupancyGrid)
	rospy.loginfo('------> 3')
        self.searchGridDrawer = SearchGridDrawer('Map', self.searchGrid, 600)
        self.searchGridDrawer.open()
        self.gridDrawer = OccupancyGridDrawer('Map',self.occupancyGrid,600)
	self.gridDrawer.open()
	rospy.loginfo('------> 4')
        self.local_odometry = Odometry()
        self.laser_sub= rospy.Subscriber("robot0/laser_0", LaserScan, self.parse_scan, queue_size=1)
        self.odom_sub = rospy.Subscriber("robot0/odom", Odometry, self.update_odometry)
        #rospy.Timer(rospy.Duration(1),self.update_visualisation)
	rospy.loginfo('------> Initialised')

    def update_odometry(self,msg):
        self.local_odometry=msg

        
    def parse_scan(self, msg):
        occupied_points = []
        current_pose = copy.deepcopy(self.local_odometry.pose.pose)
        for ii in range(int(math.floor((msg.angle_max - msg.angle_min) / msg.angle_increment))):
            # rospy.loginfo("{} {} {}".format(msg.ranges[ii],msg.angle_min,msg.angle_max))
            valid = (msg.ranges[ii] > msg.range_min) and (msg.ranges[ii] < msg.range_max)

            quaternion = (current_pose.orientation.x, current_pose.orientation.y,
                          current_pose.orientation.z, current_pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            angle = msg.angle_min + msg.angle_increment * ii + euler[2]

            # If the range is valid find the end point add it to occupied points and set the distance for ray tracing
            if valid:
                point_world_coo = [math.cos(angle) * msg.ranges[ii] + current_pose.position.x,
                                   math.sin(angle) * msg.ranges[ii] + current_pose.position.y]

                occupied_points.append(self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(point_world_coo))
                dist = msg.ranges[ii]
            else:
                dist = msg.range_max
            between = self.ray_trace(dist- 0.1, angle, msg)

            for point in between:
                try:
                    if self.occupancyGrid.getCell(point[0], point[1]) != 1.0:
                        self.occupancyGrid.setCell(point[0], point[1], 0)
                except IndexError as e:
                    print(e)

        for point in occupied_points:
            try:
                # if self.occupancyGrid.getCell(point[0], point[1]) is not 1:
                self.occupancyGrid.setCell(point[0], point[1], 1.0)
            except IndexError as e:
                print(e)

        self.searchGrid.updateFromOccupancyGrid()
    def update_odometry(self, msg):
        self.local_odometry = msg

    def parse_scan(self, msg):
        occupied_points = []
        current_pose = copy.deepcopy(self.local_odometry.pose.pose)
        for ii in range(int(math.floor((msg.angle_max - msg.angle_min) / msg.angle_increment))):
            # rospy.loginfo("{} {} {}".format(msg.ranges[ii],msg.angle_min,msg.angle_max))
            valid = (msg.ranges[ii] > msg.range_min) and (msg.ranges[ii] < msg.range_max)

            quaternion = (current_pose.orientation.x, current_pose.orientation.y,
                          current_pose.orientation.z, current_pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            angle = msg.angle_min + msg.angle_increment * ii + euler[2]

            # If the range is valid find the end point add it to occupied points and set the distance for ray tracing
            if valid:
                point_world_coo = [math.cos(angle) * msg.ranges[ii] + current_pose.position.x,
                                   math.sin(angle) * msg.ranges[ii] + current_pose.position.y]

                occupied_points.append(self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(point_world_coo))
                dist = msg.ranges[ii]
            else:
                dist = msg.range_max
            between = self.ray_trace(dist- 0.1, angle, msg)

            for point in between:
                try:
                    if self.occupancyGrid.getCell(point[0], point[1]) != 1.0:
                        self.occupancyGrid.setCell(point[0], point[1], 0)
                except IndexError as e:
                    print(e)

        for point in occupied_points:
            try:
                # if self.occupancyGrid.getCell(point[0], point[1]) is not 1:
                self.occupancyGrid.setCell(point[0], point[1], 1.0)
            except IndexError as e:
                print(e)

        self.searchGrid.updateFromOccupancyGrid()

    def ray_trace(self, dist, angle, scanmsg):
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
            point_world_coo = [math.cos(angle) * a + self.local_odometry.pose.pose.position.x,
                               math.sin(angle) * a + self.local_odometry.pose.pose.position.y]
            points.append(self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(point_world_coo))
        return points
        
    def update_visualisation(self):	 
	self.gridDrawer.update()
	self.searchGridDrawer.update()
	

if __name__ == '__main__':
    mock = Mock_Slam_Node()

    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
	mock.update_visualisation()
	rate.sleep()
	
    
  

  
