import rospy
from comp313p_mapper.msg import *
from comp313p_reactive_planner_controller.srv import *

class ExplorerNodeBase(object):

    def __init__(self):
        rospy.init_node('explorer')

        # Get the drive robot service
        rospy.loginfo('Waiting for service drive_to_goal')
        rospy.wait_for_service('drive_to_goal')
        self.driveToGoalService = rospy.ServiceProxy('drive_to_goal', Goal)
        rospy.loginfo('Got the drive_to_goal service')

        # Subscribe to get the map update messages
        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)

        # Clear the map variables
        self.occupancyGrid = None
        self.deltaOccupancyGrid = None

    def mapUpdateCallback(self, msg):
        rospy.loginfo("map update received")

        # If the occupancy grids do not exist, create them
        if self.occupancyGrid is None:
            self.occupancyGrid = OccupancyGrid.fromMapUpdateMessage(msg)
            self.deltaOccupancyGrid = OccupancyGrid.fromMapUpdateMessage(msg)

        # Update the grids
        self.occupancyGrid.updateGridFromVector(msg.occupancyGrid)
        self.deltaOccupancyGrid.updateGridFromVector(msg.deltaOccupancyGrid)

        # Update the frontiers
        #anyFrontiersRemaining = self.updateFrontiers()

        # Create a new robot waypoint if required
        #chooseNewDestination, newDestination = self.chooseNewDestination()

        # If a new destination is required, send it out


    # Simple helper to see if a cell is on a frontier or not
    def isFrontierCell(self, coords):
        return False

    # You should provide your own implementation of this method which
    # maintains and updates the frontiers.  The method should return
    # True if any outstanding frontiers exist. If the method returns
    # False, it is assumed that the map is completely explored and the
    # explorer will exit.
    def updateFrontiers(self):
        pass
    
    def run(self):
        rospy.spin()
