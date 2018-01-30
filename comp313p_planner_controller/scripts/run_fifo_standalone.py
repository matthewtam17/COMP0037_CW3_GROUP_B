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

# Create the planner
planner = FIFOPlanner('Depth First Search', occupancyGrid);
planner.setRunInteractively(True)

# Shrink the window
planner.setWindowHeightInPixels(400)

# Run it
planner.search(start, goal)

# Extract the path
path = planner.extractPathToGoal()
