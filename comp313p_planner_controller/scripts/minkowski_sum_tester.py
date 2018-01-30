#! /usr/bin/env python

from comp313p_planner_controller.occupancy_grid import OccupancyGrid
from comp313p_planner_controller.fifo_planner import FIFOPlanner

# Create the occupancy grid
occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(1, 19):
    occupancyGrid.setCell(11, y, 1)

# Start and goal cells
start = (3, 18)
goal = (20, 0)

# Create the planner on the original map
planner = FIFOPlanner('Depth First Search Original Occupancy Grid', occupancyGrid);
planner.setWindowHeightInPixels(400)
planner.search(start, goal)
path = planner.extractPathToGoal()

# Now try it on our Minkowski sum map
occupancyGrid.expandObstaclesToAccountForCircularRobotOfRadius(0.5)
planner = FIFOPlanner('Depth First Search Robot Radius 0.5', occupancyGrid);
planner.setWindowHeightInPixels(400)
planner.search(start, goal)
path = planner.extractPathToGoal()

# Now try it on our Minkowski sum map
occupancyGrid.expandObstaclesToAccountForCircularRobotOfRadius(0)
planner = FIFOPlanner('Depth First Search Robot Radius 0 (Same As Original Map)', occupancyGrid);
planner.setWindowHeightInPixels(400)
planner.search(start, goal)
path = planner.extractPathToGoal()


# Now try it on our Minkowski sum map
occupancyGrid.expandObstaclesToAccountForCircularRobotOfRadius(2)
planner = FIFOPlanner('Depth First Search Robot Radius 4', occupancyGrid);
planner.setRunInteractively(True)
planner.setWindowHeightInPixels(400)
planner.search(start, goal)
path = planner.extractPathToGoal()

