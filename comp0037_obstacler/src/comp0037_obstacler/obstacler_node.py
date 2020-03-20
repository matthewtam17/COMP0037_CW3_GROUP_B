#!/usr/bin/env python

import rospy
from threading import Lock
from math import pow,atan2,sqrt,pi
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from comp0037_obstacler.obstacle import *

# This script runs "the obstacler". This is a very simple
# simulation engine which causes obstacles to appear and
# disappear. Its actions are instigated by "the boss" node.

class ObstaclerNode(object):

    def __init__(self):
        rospy.init_node('obstacler')

        rospy.wait_for_service('request_map_update')
        
        # Create a set of obstacles
        self.obstacles = []
       
        # Set up the lock to ensure thread safety
        self.obstacleLock = Lock()
        self.noOdometryReceived = True
        self.mostRecentOdometry = Odometry()

        # Register the subscriber to get the robot's state. This is mostly used just to get regular updates
        # to ensure that we properly handle the obstacle sleep stuff.
        self.odometrySubscriber = rospy.Subscriber("/robot0/odom", Odometry, self.odometryCallback, queue_size=1)

        # Register the subscriber to get the laser scan. This is used to detect if an obstacle has been detected
        self.laserSubscriber = rospy.Subscriber("robot0/laser_0", LaserScan, self.laserScanCallback, queue_size=1)

        # If the obstacle file parameter was specified, load. Otherwise, create defaults
        if rospy.has_param('obstacles_file') is True:
            self.loadObstacleDetailsFromFile(rospy.get_param('obstacles_file'))
        else:
            self.createTestObstacles()
            
        # Set all the objects up. Note that this makes the objects which are present detectable at the start
        for obstacle in self.obstacles:
            obstacle.reset()


    def loadObstacleDetailsFromFile(self, obstacleFileName):
        rospy.loginfo('Loading goals from file {}'.format(obstacleFileName))
        fp = open(obstacleFileName)
        lines = fp.readlines()
        for line in lines:
            nextLine = ([float(v) for v in line.split()])
            self.obstacles.append(Obstacle(nextLine[0], nextLine[1], nextLine[2]))
            
        rospy.loginfo("Read {} obstacles".format(len(self.obstacles)))
            
    def createTestObstacles(self):
        # Add test
        self.obstacles.append(Obstacle(52, 20, 1))
        self.obstacles.append(Obstacle(64, 30, 1))
        self.obstacles.append(Obstacle(76, 40, 1))
        self.obstacles.append(Obstacle(88, 50, 1))
 
    def odometryCallback(self, msg):
        self.obstacleLock.acquire()
        for obstacle in self.obstacles:
            obstacle.update()
        self.obstacleLock.release()

    # Go through the laser scan and find all the unique intensity values. From these, identify
    # the set of detected objects and signal appropriately.
    def laserScanCallback(self, msg):

        uniqueIntensities = []
        for intensity in msg.intensities:
            if intensity not in uniqueIntensities:
                uniqueIntensities.append(intensity)

        # rospy.loginfo('Unique intensities ' + str(uniqueIntensities))

        # Now go through all obstacles and if we find one, ping it
        self.obstacleLock.acquire()
        for obstacle in self.obstacles:
            for intensity in uniqueIntensities:
                if obstacle.sameID(intensity) is True:
                    obstacle.detectedByLaser()
        self.obstacleLock.release()
        
    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        
  
