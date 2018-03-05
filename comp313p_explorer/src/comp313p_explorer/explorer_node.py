import rospy
from comp313p_mapper.msg import *

class ExplorerNode(object):

    def __init__(self):
        rospy.init_node('explorer')

        # Subscribe to get the map update messages
        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)

    def mapUpdateCallback(self, msg):
        rospy.loginfo("map update received")
    
    def run(self):
        rospy.spin()
