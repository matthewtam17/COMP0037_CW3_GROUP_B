#!/usr/bin/env python

# Self script runs "the boss". Self sends out a set of goals to the robot in turn.

import rospy
import sys
from comp313p_planner_controller.srv import *
from nav_msgs.srv import GetMap


from random import randint

# The boss tells the robot where to go. Self is either a set of
# goals from a file, or randomly pulled from the map.

class BossNode(object):

    def __init__(self, argv):
        rospy.init_node('the_boss')

        # Get the teleport robot service
        rospy.loginfo('Waiting for service teleport_absolute')
        rospy.wait_for_service('/turtle1/teleport_absolute')
        self.teleportAbsoluteService = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        rospy.loginfo('Got the teleport_absolute service')
        
        # Get the drive robot service
        rospy.loginfo('Waiting for service drive_to_goal')
        rospy.wait_for_service('drive_to_goal')
        self.driveToGoalService = rospy.ServiceProxy('drive_to_goal', Goal)
        rospy.loginfo('Got the drive_to_goal service')

        # Set up the goal array
        self.goals = []

        # If a command line argument was provided, assume it includes
        # a list of goals, so load it. If not, create the goals at
        # random.
        if (len(argv) > 1):
            self.loadGoalsFromFile(argv[1])
        else:
            self.createRandomGoals()

    # Read the goal from the file
    def loadGoalsFromFile(self, goalFileName):
        rospy.loginfo('Loading goals from file %s', str(goalFileName))
        fp = open(goalFileName)
        lines = fp.readlines()
        for line in lines:
            self.goals.append([float(v) for v in line.split()])
        rospy.loginfo("Read %d goals", len(self.goals))

    # If no goal file is specified, create a bunch of random goals
    def createRandomGoals(self):
        rospy.loginfo('No goal file specified; finding map and generating a random set of goals')
        rospy.loginfo('Waiting for static_map to become available.')
        rospy.wait_for_service('static_map') 
        self.mapServer = rospy.ServiceProxy('static_map', GetMap)
        rospy.loginfo('Found static_map; requesting map data')

        # Query the map status
        response = self.mapServer()
        map = response.map
        rospy.loginfo('Got map data')

        # Iterate over the goals and create them
        for g in range(0, 15):
            goalX = randint(0, map.info.width - 1) * map.info.resolution
            goalY = randint(0, map.info.height - 1) * map.info.resolution
            self.goals.append((goalX, goalY))
        

    def sendGoalToRobot(self, goal):
        rospy.logwarn("Sending the robot to " + str(goal))
        try:
            response = self.driveToGoalService(goal[0], goal[1])
            return response.reachedGoal
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

    def bossTheRobotAround(self):

        # Teleport to the first waypoint
        self.teleportAbsoluteService(self.goals[0][0], self.goals[0][1], 0)
        
        # Now iterate through the rest of the waypoints and drive to them
        for g in range(1, len(self.goals)):
            bossNode.sendGoalToRobot(self.goals[g])
            if (rospy.is_shutdown()):
                break

if __name__ == '__main__':
    try:

        # Strip out all the ROS-specific command line arguments
        myargv = rospy.myargv(argv=sys.argv)
        
        bossNode = BossNode(myargv)

        bossNode.bossTheRobotAround()
        
    except rospy.ROSInterruptException:
        pass

    finally:
        pass
