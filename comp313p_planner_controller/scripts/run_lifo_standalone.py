#! /usr/bin/env python

from comp313p_planner_controller.occupancy_grid import OccupancyGrid
from comp313p_planner_controller.lifo_planner import LIFOPlanner

# Create the occupancy grid
occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(1, 19):
    occupancyGrid.setCell(11, y, 1)

# Start and goal cells
start = (3, 18)
goal = (20, 0)

# Create the planner
planner = LIFOPlanner('Depth First Search', occupancyGrid);
planner.setRunInteractively(True)

# Change the height of the window to fit in the VNC viewer
planner.setWindowHeightInPixels(400)

# Run it
planner.search(start, goal)

# Extract the path
path = planner.extractPathToGoal()
