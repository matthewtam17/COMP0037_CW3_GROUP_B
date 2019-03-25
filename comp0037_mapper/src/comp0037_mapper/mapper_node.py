#!/usr/bin/env python

import sys
import rospy
import math
import tf
import copy
from std_msgs.msg import Int32

import numpy as np
from nav_msgs.srv import GetMap
from comp0037_reactive_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_reactive_planner_controller.grid_drawer import OccupancyGridDrawer
from comp0037_reactive_planner_controller.cell import CellLabel
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from threading import Lock
from geometry_msgs.msg  import Twist
from comp0037_mapper.msg import MapUpdate
from comp0037_mapper.srv import *
from bresenhamalgorithm import bresenham

# This class implements basic mapping capabilities. Given knowledge
# about the robot's position and orientation, it processes laser scans
# to produce a new occupancy grid. If this grid differs from the
# previous one, a new grid is created and broadcast.

class MapperNode(object):

    def __init__(self):

        rospy.init_node('mapper_node', anonymous=True)
 
        # Check if we use the ground truth map from stdr; if this is enabled, the mapper will ignore
        # the laser scans
        self.useGroundTruthMap = True

         # Set up the lock to ensure thread safety
        self.dataCopyLock = Lock()

        self.noOdometryReceived = True
        self.noTwistReceived = True
        self.noLaserScanReceived = True

        self.enableMapping = True

        # List of active obstacles
        self.activatedObstacles = []
        self.disactivatedObstacles = []

        self.activeObstacles = []

        # Get the ground truth map from stdr; we use this to figure out the dimensions of the map
        rospy.loginfo('waiting for the static_map service')
        rospy.wait_for_service('static_map')
        self.mapServer = rospy.ServiceProxy('static_map', GetMap)
        resp = self.mapServer()

        # Drawing options
        self.showOccupancyGrid = rospy.get_param('show_mapper_occupancy_grid', True)
        self.showDeltaOccupancyGrid = rospy.get_param('show_mapper_delta_occupancy_grid', True)

        # Flag set to true if graphics can be updated
        self.visualisationUpdateRequired = False
        
        # Create the publisher for the regular map messages
        self.mapUpdatePublisher = rospy.Publisher('updated_map', MapUpdate, queue_size = 1)

        # Get the map scale
        mapScale = rospy.get_param('plan_scale',5)

        # Create the occupancy grid
        self.occupancyGrid = OccupancyGrid(resp.map.info.width, resp.map.info.height, resp.map.info.resolution, 0.5)
        self.occupancyGrid.setScale(mapScale)
        self.occupancyGrid.scaleEmptyMap()
        
        # Create the delta occupancy grid map. This stores the difference since the last time the map was sent out
        self.deltaOccupancyGrid = OccupancyGrid(resp.map.info.width, resp.map.info.height, resp.map.info.resolution, 0)
        self.deltaOccupancyGrid.setScale(mapScale)
        self.deltaOccupancyGrid.scaleEmptyMap()

        # If we are using ground truth, create a ground truth occupancy map (used for reference) and set its value to the
        # ground truth map.
        if self.useGroundTruthMap is True:
            self.groundTruthOccupancyGrid = OccupancyGrid(resp.map.info.width, resp.map.info.height, resp.map.info.resolution, 0.5)
            self.groundTruthOccupancyGrid.setScale(mapScale)
            self.groundTruthOccupancyGrid.setFromDataArrayFromMapServer(resp.map.data)
            self.updateOccupancyGridFromGroundTruthAndObstacles()
            self.visualisationUpdateRequired = True
            self.noLaserScanReceived = False

        # Debug code
        if False:
            uniqueCellValues1 = []
            for x in range(self.occupancyGrid.widthInCells):
                for y in range(self.occupancyGrid.heightInCells):
                    if self.groundTruthOccupancyGrid.grid[x][y] not in uniqueCellValues1:
                        uniqueCellValues1.append(self.occupancyGrid.grid[x][y])

            uniqueCellValues = []
            print str(len(resp.map.data))
            for x in range(len(resp.map.data)):
                g = resp.map.data[x]
                if g not in uniqueCellValues:
                    uniqueCellValues.append(g)

            print '**************************************************************************************'
            print str(uniqueCellValues)
            print str(uniqueCellValues1)
            print '**************************************************************************************'

        windowHeight = rospy.get_param('maximum_window_height_in_pixels', 600)

        if self.showOccupancyGrid is True:        
            self.occupancyGridDrawer = OccupancyGridDrawer('Mapper Node Occupancy Grid',\
                                                           self.occupancyGrid, windowHeight)
	    self.occupancyGridDrawer.open()

        if self.showDeltaOccupancyGrid is True:
            self.deltaOccupancyGridForShow = OccupancyGrid(resp.map.info.width, resp.map.info.height, resp.map.info.resolution, 0)
	    self.deltaOccupancyGridForShow.setScale(mapScale)
	    self.deltaOccupancyGridForShow.scaleEmptyMap()
            self.deltaOccupancyGridDrawer = OccupancyGridDrawer('Mapper Node Delta Occupancy Grid',\
                                                                self.deltaOccupancyGridForShow, windowHeight)
	    self.deltaOccupancyGridDrawer.open()


        # Set up the subscribers. These track the robot position, speed, threshold and laser scans.
        self.mostRecentOdometry = Odometry()
        self.odometrySubscriber = rospy.Subscriber("robot0/odom", Odometry, self.odometryCallback, queue_size=1)
        self.mostRecentTwist = Twist()
        self.twistSubscriber = rospy.Subscriber('/robot0/cmd_vel', Twist, self.twistCallback, queue_size=1)
        self.laserSubscriber = rospy.Subscriber("robot0/laser_0", LaserScan, self.laserScanCallback, queue_size=1)
        self.addObstacleToMapSubscriber = \
            rospy.Subscriber("/add_obstacle_to_map", Int32, self.addObstacleToMapCallback, queue_size=1)
        self.removeObstacleFromMapSubscriber = \
            rospy.Subscriber("/remove_obstacle_from_map", Int32, self.removeObstacleFromMapCallback, queue_size=1)
                 
        # Register the supported services
        self.changeMapperStateService = rospy.Service('change_mapper_state', ChangeMapperState, self.mappingStateService)
        self.requestMapUpdateService = rospy.Service('request_map_update', RequestMapUpdate, self.requestMapUpdateService)

    def odometryCallback(self, msg):
        self.dataCopyLock.acquire()
        self.mostRecentOdometry = msg
        self.noOdometryReceived = False
        self.dataCopyLock.release()

    def twistCallback(self, msg):
        self.dataCopyLock.acquire()
        self.mostRecentVelocity = msg
        self.noTwistReceived = False
        self.dataCopyLock.release()

    def mappingStateService(self, changeMapperState):
        self.enableMapping = changeMapperState.enableMapping
        # rospy.loginfo('Changing the enableMapping state to %d', self.enableMapping)
        return ChangeMapperStateResponse()

    def requestMapUpdateService(self, request):
        rospy.loginfo('requestMapUpdateService with deltaOccupancyGridRequired %d', request.deltaOccupancyGridRequired)
        mapUpdateMessage = self.constructMapUpdateMessage(request.deltaOccupancyGridRequired)
        return RequestMapUpdateResponse(mapUpdateMessage)

    # Handle the laser scan callback. First process the scans and update the various maps
    
    def laserScanCallback(self, msg):

        if self.useGroundTruthMap is True:
            return

        # Can't process anything until stuff is enabled
        if self.enableMapping is False:
            return

        # Can't process anything until we have the first scan
        if (self.noOdometryReceived is True) or (self.noTwistReceived is True):
            return

        # Process the scan
        gridHasChanged = self.parseScan(msg)

        # If nothing has changed, return
        if gridHasChanged is False:
            return

        # Mark that there is a pending update to the visualisation
        self.visualisationUpdateRequired = True

        # Mark that a laser scan has been received
        self.noLaserScanReceived = False

        # Construct the map update message and send it out
        mapUpdateMessage = self.constructMapUpdateMessage(True)
	rospy.loginfo('publishing map update message')
        self.mapUpdatePublisher.publish(mapUpdateMessage)

    def predictPose(self, predictTime):
        
        self.dataCopyLock.acquire()
        currentPose = copy.deepcopy(self.mostRecentOdometry.pose.pose)
        currentPoseTime = self.mostRecentOdometry.header.stamp.to_sec()
        currentTwist = copy.deepcopy(self.mostRecentTwist)
        self.dataCopyLock.release()

        dT = 0#predictTime - currentPoseTime

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
                currentTwist.linear.x / currentTwist.angular.z * cos(theta + dT * currentTwist.angular.z)

        theta = theta + currentTwist.angular.z * dT
        
        return x, y, theta

    # This method notes the obstacle ID with the groundtruth list.
    def addObstacleToMapCallback(self, obstacleMsg):

        id = obstacleMsg.data

        rospy.loginfo('addObstacleToMapCallback: adding obstacle {}'.format(id))

        if id not in self.activeObstacles:
            self.activeObstacles.append(id)

        rospy.loginfo('addObstacleToGroundtruthListCallback: activeObstacles = {}'.format(self.activeObstacles))
        self.updateOccupancyGridFromGroundTruthAndObstacles()
        self.visualisationUpdateRequired = True

    def removeObstacleFromMapCallback(self, obstacleMsg):

        id = obstacleMsg.data

        rospy.loginfo('removeObstacleFromGroundtruthListCallback: removing obstacle {}'.format(id))

        if id in self.activeObstacles:
            self.activeObstacles.remove(id)

        rospy.loginfo('removeObstacleFromGroundtruthListCallback: activateObstacles = {}'.format(self.activeObstacles))

        self.updateOccupancyGridFromGroundTruthAndObstacles()
        self.visualisationUpdateRequired = True

    def parseScan(self, msg):

        # Predict the robot pose to the time the scan was taken
        x, y, theta = self.predictPose(msg.header.stamp.to_sec())
        
        # Clear the flag which shows that the map has changed
        gridHasChanged = False

        # Clear the delta map, to imply that no changes have been detected
        self.deltaOccupancyGrid.clearMap(0)
        
        # For each ray, check the range is good. If so, check all the
        # cells along the ray and mark cells as either open or
        # blocked. To get around numerical issues, we trace along each
        # ray in turn and terminate when we hit the first obstacle or at the end of the ray.
        for ii in range(int(math.floor((msg.angle_max - msg.angle_min) / msg.angle_increment))):
            # rospy.loginfo("{} {} {}".format(msg.ranges[ii],msg.angle_min,msg.angle_max))

            detectedRange = msg.ranges[ii]
            
            # If the detection is below the minimum range, assume this ray is busted and continue
            if (detectedRange <= msg.range_min):
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

            # If between is empty, something went wrong with the ray cast, so skip
            if len(between) == 0:
                continue

            # Traverse along the ray and set cells. We can only change
            # cells from unknown (0.5) to free. If we encounter a
            # blocked cell, terminate. Sometimes the ray can slightly
            # extend through a blocked cell due to numerical rounding
            # issues.
            traversedToEnd = True
            for point in between:
                try:
                    # if self.occupancyGrid.getCell(point[0], point[1]) == 0.5:
                    self.occupancyGrid.setCell(point[0], point[1], 0)
                    self.deltaOccupancyGrid.setCell(point[0], point[1], 1.0)
                    if self.showDeltaOccupancyGrid is True:
                        self.deltaOccupancyGridForShow.setCell(point[0], point[1], 1.0)
                    gridHasChanged = True
                except IndexError as e:
                    print(e)
                    print "between: " + str(point[0]) + ", " + str(point[1])
                    print "extent: " + str(self.occupancyGrid.getExtent())

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
                    self.occupancyGrid.setCell(lastPoint[0], lastPoint[1], 1)
                    self.deltaOccupancyGrid.setCell(lastPoint[0], lastPoint[1], 1.0)
                    if self.showDeltaOccupancyGrid is True:
                        self.deltaOccupancyGridForShow.setCell(lastPoint[0], lastPoint[1], 1.0)
                    gridHasChanged = True
            

        return gridHasChanged

    def ray_trace(self, dist, x, y, angle, scanmsg):
        """
        Function to get a list of points between two points
        :param origin: position of the origin in world coordinates
        :param dist: distance to end point
        :param angle: angle from robot
        :param scanmsg: Laser Scan message
        :return: list of points in between the origin and end point
        """
        startPoint = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates([math.cos(angle) * scanmsg.range_min + x, \
                                                                              math.sin(angle) * scanmsg.range_min + y])
        endPoint = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates([math.cos(angle) * dist + x, \
                                                                              math.sin(angle) * dist + y])

        points = bresenham(endPoint, startPoint)

        return points.path

    def updateOccupancyGridFromGroundTruthAndObstacles(self):

        # Loop through the map and change the occupancy grid so that only
        # visible objects are shown. Note that we turn the cell occupancy back
        # into an integer to avoid rounding issues in comparisons. 
        self.occupancyGrid.grid = copy.deepcopy(self.groundTruthOccupancyGrid.grid)
        for x in range(self.occupancyGrid.widthInCells):
            for y in range(self.occupancyGrid.heightInCells):
                cell = int(self.occupancyGrid.getCell(x, y)*100)
                if cell > 0 and cell < 100:
                    if cell in self.activeObstacles:
                        self.occupancyGrid.setCell(x, y, 1)
                    else:
                        self.occupancyGrid.setCell(x, y, 0)

        mapUpdateMessage = self.constructMapUpdateMessage(True)
	rospy.loginfo('publishing map update message')
        self.mapUpdatePublisher.publish(mapUpdateMessage)


    def updateVisualisation(self):

        # If anything has changed the state of the occupancy grids,
        # visualisationUpdateRequired is set to true. Therefore, the
        # graphics are updated. If set to false, we flush anyway to
        # make sure that the windows are properly (re)drawn in VNC.
        
        if self.visualisationUpdateRequired is True:

            if self.showOccupancyGrid is True:        
	        self.occupancyGridDrawer.update()

            if self.showDeltaOccupancyGrid is True:
	        self.deltaOccupancyGridDrawer.update()
                self.deltaOccupancyGridForShow.clearMap(0)

            self.visualisationUpdateRequired = False

        else:

            if self.showOccupancyGrid is True:        
	        self.occupancyGridDrawer.flushAndUpdateWindow()

            if self.showDeltaOccupancyGrid is True:
	        self.deltaOccupancyGridDrawer.flushAndUpdateWindow()

    def constructMapUpdateMessage(self, deltaMapRequired):
        # Construct the map update message
        mapUpdateMessage = MapUpdate()

	#rospy.loginfo('constructMapUpdateMessage: invoked')

        mapUpdateMessage.header.stamp = rospy.Time().now()
        mapUpdateMessage.isPriorMap = self.noLaserScanReceived
        mapUpdateMessage.scale = self.occupancyGrid.getScale()
        mapUpdateMessage.resolution = self.occupancyGrid.getResolution()
        mapUpdateMessage.extentInCells = self.occupancyGrid.getExtentInCells()
        mapUpdateMessage.occupancyGrid = self.occupancyGrid.getGridAsVector()

        if deltaMapRequired is True:
            mapUpdateMessage.deltaOccupancyGrid = self.deltaOccupancyGrid.getGridAsVector()

        return mapUpdateMessage

        
    def run(self):
        while not rospy.is_shutdown():
            self.updateVisualisation()
            rospy.sleep(0.1)
        
  

  
