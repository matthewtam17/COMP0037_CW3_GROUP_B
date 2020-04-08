# -*- coding: utf-8 -*-

# The planned path the robot will take. This consists of a set of waypoints.
from collections import deque

class PlannedPath(object):

    # Construct a new planner object and set defaults.
    def __init__(self):

        # Does the path actually reach the goal or not?
        self.goalReached = False
        
        # The list of waypoints, from start to finish, which make up the path.
        # The type of data stored here depends on the 
        self.waypoints = deque()

        # Performance information - number of waypoints, and the
        # travel cost of the path.
        self.numberOfWaypoints = 0
        self.travelCost = 0

    # This method appends another planned path to the end of this
    # path. The end of this path must be the same cell as the
    # start of the other path
    def addToEnd(self, otherPlannedPath):
       # Check the tail of one list is the head of another

        otherPlannedPathList = list(otherPlannedPath.waypoints)
        # Tuple comparison returns true when all elements are equal
        assert(list(self.waypoints)[-1].coords == otherPlannedPathList[0].coords)

        # Now go through and add the elements from the other planned path to this one.
        for i in range(len(otherPlannedPathList) - 1):
            self.waypoints.append(otherPlannedPathList[i + 1])

        self.numberOfWaypoints = self.numberOfWaypoints + otherPlannedPath.numberOfWaypoints - 1
        self.travelCost = self.travelCost + otherPlannedPath.travelCost
