#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt
from PyKDL import Rotation

class stdr_controller():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('stdr_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.current_pose_subscriber = rospy.Subscriber('/robot0/odom', Odometry, self.current_callback)
        self.current_pose = Odometry()

    def current_callback(self, data):
        self.current_pose = data

    def run(self):
        while not rospy.is_shutdown():
            distance_tolerance = 0.01
            vel_msg = Twist()
	    
            rospy.loginfo('Current position, x: {}, y:{}'.format(self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y))
            lin_velocity = float(raw_input('Enter desired linear velocity: '))
            rot_velocity = float(raw_input('Enter desired rotational velocity: '))
            duration = float(raw_input('Enter desired duration: '))
            vel_msg.linear.x = lin_velocity
            vel_msg.angular.z = rot_velocity
            self.velocity_publisher.publish(vel_msg)
            rospy.sleep(duration)
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)

   
if __name__ == '__main__':
    try:
        #Testing our function
        x = stdr_controller()
        x.run()

    except rospy.ROSInterruptException: pass
