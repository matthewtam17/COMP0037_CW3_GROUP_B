#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32
from scipy.stats import poisson
from scipy.stats import norm
import random

# This class represents the workspace, which is the representation of the actual environment.

class Obstacle(object):

    def __init__(self, mu, sigma, val):
        self.mu = mu
        self.sigma = sigma
        self.val = val   
        self.timer=rospy.Timer(rospy.Duration(0.5), self.roll_event)
        self.t0=rospy.Time.now()
        self.inSimulation = True
        # Starting all obstacles as set
        addObstacleToSimulationPublisher.publish(self.val)
    
    def roll_event(self, event):
        t=(rospy.Time.now()-self.t0).to_sec()
        if random.random() > (1-norm.cdf(t,self.mu,self.sigma)):
            if self.inSimulation is True:
                removeObstacleFromSimulationPublisher.publish(self.val)
                self.inSimulation = False
                rospy.loginfo('Removing obstacle {} from the simulation'.format(self.val))
            else:
                addObstacleToSimulationPublisher.publish(self.val)
                self.inSimulation = True
                rospy.loginfo('Adding obstacle {} to the simulation'.format(self.val))
            self.t0=rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node('obstacle_manager')
    addObstacleToSimulationPublisher=rospy.addObstacleToSimulationPublisher = \
        rospy.Publisher("/add_obstacle_to_simulation", Int32, queue_size=1)
    removeObstacleFromSimulationPublisher=rospy.addObstacleToSimulationPublisher = \
        rospy.Publisher("/remove_obstacle_from_simulation", Int32, queue_size=1)
    top_ob=Obstacle(1,1,52)
    mid_ob=Obstacle(2,1,64)
    bot_ob=Obstacle(3,1,76)
    bot_ob2=Obstacle(3,1,88)
    rospy.spin()
