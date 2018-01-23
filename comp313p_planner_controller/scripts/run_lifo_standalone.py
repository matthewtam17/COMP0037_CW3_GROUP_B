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
planner = LIFOPlanner(occupancyGrid);
planner.setPauseTime(0)

# Run it
planner.plan(start, goal)

# Pause
planner.gridDrawer.waitForKeyPress()

# Show the path
planner.extractPathToGoal()
planner.gridDrawer.waitForKeyPress()

