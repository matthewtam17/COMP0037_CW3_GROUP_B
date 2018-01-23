from collections import deque

# This class stores the planned path

class PlannedPath(object):

    # Construct a new planner object and set defaults.
    def __init__(self):

        # Does the path actually reach the goal or not?
        self.goalReached = False
        
        # The list of waypoint cells, from start to finish, which make
        # up the path.
        self.waypoints = deque()

        # Performance information - number of waypoints, and the
        # travel cost of the path.
        self.numberOfWaypoints = 0
        self.pathTravelCost = 0
