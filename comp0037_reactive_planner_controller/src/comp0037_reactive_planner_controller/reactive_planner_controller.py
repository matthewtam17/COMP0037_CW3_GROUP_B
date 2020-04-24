# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot motion.

import rospy
import math
import threading
from cell import CellLabel
from planner_controller_base import PlannerControllerBase
from comp0037_mapper.msg import *
from comp0037_reactive_planner_controller.aisle import Aisle

class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)

        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()
        self.aisleToDriveDown = None
        self.Lw = 2
        self.t_fed = 0.2 # The customizable fed wait time from 2.2 description
        self.p_b = 0.8 # The probability of obstacle appearing in B

    def mapUpdateCallback(self, mapUpdateMessage):

        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()

        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return

        self.checkIfPathCurrentPathIsStillGood()

    def checkIfPathCurrentPathIsStillGood(self):

        # This methods needs to check if the current path, whose
        # waypoints are in self.currentPlannedPath, can still be
        # traversed

        waypoints = self.currentPlannedPath.waypoints
        for point in waypoints:
            x,y = point.coords
            status = self.occupancyGrid.getCell(x,y)
            if status == 1:
                print("Path no longer valid - cell blocked at " + str(x) + "," +str(y))
                self.controller.stopDrivingToCurrentGoal()
                break
        # If the route is not viable any more, call
        # self.controller.stopDrivingToCurrentGoal()

    # Choose the first aisle the robot will initially drive down.
    # This is based on the prior.
    def chooseInitialAisle(self, startCellCoords, goalCellCoords):
        rospy.logwarn("Choosing Aisle Initially")
        path_b = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, Aisle.B)
        path_c = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, Aisle.C)
        self._draw_path_by_color(path_b)
        self._draw_path_by_color(path_c, 'red')

        L_cost_via_b = path_b.travelCost + self.Lw * self.t_fed * self.p_b # mynote: the time is fed here as described from the assignemnt
        L_cost_via_c = path_c.travelCost

        aisle_ret = Aisle.B if L_cost_via_b < L_cost_via_c else Aisle.C

        # mynote: assume path_c cost always > path_b cost. This is an assumption the assignment used for B is always the shortest physical path.
        t_expected_threshold = (path_c.travelCost - path_b.travelCost)/(self.Lw * self.p_b)
        lambda_my = 1.0/t_expected_threshold

        str_buf = ['',]
        str_buf.append("Logging Decission for Aisle B or C.")
        str_buf.append("E(T): {:.2f}; L_w: {:.2f}; B_obstacle_prob: {:.2f}".format(self.t_fed, self.Lw, self.p_b))
        str_buf.append("Cost via B: {:.2f}; Cost via C: {:.2f}. Chosen Aisle: {}".format(L_cost_via_b, L_cost_via_c, aisle_ret))
        str_buf.append("E(T) thres: {:.2f}; E(T): {:.2f}".format(t_expected_threshold, self.t_fed))
        str_buf.append("Ans for 2.3, lambda is: {:.2f}".format(lambda_my))

        rospy.logwarn('\n'.join(str_buf))
        return aisle_ret

    # Choose the subdquent aisle the robot will drive down
    def chooseAisle(self, startCellCoords, goalCellCoords):
        return Aisle.D

    # Return whether the robot should wait for the obstacle to clear or not.
    def shouldWaitUntilTheObstacleClears(self, startCellCoords, goalCellCoords):
        #Calculate the path cost for the new path, and the path cost for the original path
        #From the robot's current position
        #The robot's current position is given as startCellCoords.
        #Old remaining path cost from current robot position:
        rospy.loginfo("Calculating original old path cost: " + str(self.currentPlannedPath.travelCost))
        oldPathWaypoints = list(self.currentPlannedPath.waypoints)
        currentIndex = 0
        closestDistance = float('inf')
        currentDistance = float('inf')
        print("robot's current cell: " + str(startCellCoords))
        for cell in oldPathWaypoints:
            # print(currentDistance) # my note: debug del
            # Project the robot onto the lsit of waypoints - obtaining the closest waypoint to the robot
            currentDistance = math.sqrt((cell.coords[0] - startCellCoords[0])**2 + (cell.coords[1] - startCellCoords[1])**2)
            if  currentDistance < closestDistance:
                currentIndex = oldPathWaypoints.index(cell)
                closestDistance = currentDistance

        # Do a check to see if closestDistance is significant - the error from the robot to the closest waypoint
        # If this is significant then there is an error somewhere. If it is small enough then it probably due to
        # some noise in the robot's movement. Want to check that this error is less than width of a cell.
        print("i: " +str(currentIndex))
        oldPathRemainingCost = 0
        for i in range(currentIndex,len(oldPathWaypoints)-1):
            #This is a less computationally expensive solution than recomputing the cost
            #by re-planning a section of the old path
            oldPathRemainingCost = oldPathRemainingCost + self.planner.computeLStageAdditiveCost(oldPathWaypoints[i],oldPathWaypoints[i+1])
        rospy.loginfo("Calculating old Path remaining cost: " + str(oldPathRemainingCost))

        # The planners are deterministic. So if I plan a search from start to currentcoords,
        # then the plannned path of that should be identical to the old path that the robot has traversed.
        # That has some computational cost. So maybe a way of extracting the traversed path from the
        # reactive planner.

        # Compare old_path (not remained path) and new_path
        old_path = self.currentPlannedPath
        newPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, self.chooseAisle(startCellCoords, goalCellCoords)) # my note: a fix to ensure it travels via aisle
        self._draw_path_by_color(old_path, 'yellow')
        self._draw_path_by_color(newPath, 'red')

        #New Path Cost: The calculation part
        newPathTravelCost = newPath.travelCost
        diffPathTravelCost = newPathTravelCost - oldPathRemainingCost
        rospy.logwarn("\nA new path found.\nOld path remained Cost: {:.2f}\nNew path cost: {:.2f};\nDifference: {:.2f}"\
                .format(oldPathRemainingCost, newPathTravelCost, diffPathTravelCost))

        # <Michael test> test my implementation
        startCellCoords_mike = self._get_current_cell_coord()
        path_b = self.planPathToGoalViaAisle(startCellCoords_mike, goalCellCoords, Aisle.B) # Mike: Please ensure using the fixed value Aisle.B here.
        path_c = self.planPathToGoalViaAisle(startCellCoords_mike, goalCellCoords, Aisle.C)
        pCost_new = path_c.travelCost
        pCost_old = path_b.travelCost
        pCost_diff = pCost_new - pCost_old # assume re-planed path cost is always longer than remained path cost
        rospy.logwarn("\nMichael ver: A new path found.\nOld path remained Cost: {:.2f}\nNew path cost: {:.2f};\nDifference: {:.2f}"\
                .format(pCost_old, pCost_new, pCost_diff))
        # </Michael test>

        t_fed = self.t_fed
        t_expected_threshold = 1.0 * diffPathTravelCost/self.Lw
        lambda_my = 1.0/t_expected_threshold # Mike: use 1 here because code only enters this codeblock when an obastacle is seen, so it has to be 1.

        waitCost = self.Lw * t_fed
        string = "\nWait Cost Info:\nE(T): {:.2f}; L_w: {:.2f}; c(L(T)): {:.2f}\nE(T)_thres: {:2f}"\
                    .format(t_fed, self.Lw, waitCost, t_expected_threshold)\
                    + "Ans for 2.2, lambda is: {:.2f}".format(lambda_my)

        if waitCost < diffPathTravelCost:
            return True
        return False

    # This method will wait until the obstacle has cleared and the robot can move.
    def waitUntilTheObstacleClears(self):
        waiting = True
        blocked = False
        while waiting:
            blocked = False
            waypoints = self.currentPlannedPath.waypoints
            for point in waypoints:
                x,y = point.coords
                status = self.occupancyGrid.getCell(x,y)
                if status == 1:
                    # print("Path still blocked - waiting")
                    blocked = True
                    break
            if blocked is False:
                waiting = False


    # Plan a path to the goal which will go down the designated aisle. The code, as
    # currently implemented simply tries to drive from the start to the goal without
    # considering the aisle.
    def planPathToGoalViaAisle(self, startCellCoords, goalCellCoords, aisle):
        #The principle in this function is that driving down an aisle
        #is like introducing an intermediate waypoint
        print("start Coords: " + str(startCellCoords))
        #This function, based on the aisle given, would plan a path from the
        # start coordinates to a cell determined by an aisle
        # then a separate planned path from the aisle to the goal coordinates
        # and then contatenate the two planned paths (which contain the waypoints)
        # together so that the function that runs the robot along each waypoint,
        # driveToGoal(), is able to drive the robot as usual.

        # Note that, if the robot has waited, it might be tasked to drive down the
        # aisle it's currently on. Your code should handle this case.
        if self.aisleToDriveDown is None:
            self.aisleToDriveDown = aisle

        # Make sure that when you write it out, that you define the points
        # such that the lidar can reach the obstacle (i.e. the waypoint defined for
        #B1 should be such that the robot's sensors can actually detect the obstacle

        #The intermediateGoal is used to control the via point of the planned path
        # i.e. what aisle the robot will drive down

        intermediateGoal = (25,25)
        print("aisle: " + str(aisle))
        if aisle == Aisle.A:
            print("Driving down aisle A")
            intermediateGoal = (25,20)
        elif aisle == Aisle.B:
            intermediateGoal = (40,20)
            print("Driving down aisle B")
        elif aisle == Aisle.C:
            intermediateGoal = (60,20)
            print("Driving down aisle C")
        elif aisle == Aisle.D:
            intermediateGoal = (75,20)
            print("Driving down aisle D")
        elif aisle == Aisle.E:
            intermediateGoal = (90,20)
            print("Driving down aisle E")
        else:
            intermediateGoal = (25,20)
            print("Driving down aisle A")

        # Implement your method here to construct a path which will drive the robot
        # from the start to the goal via the aisle.
        #Planning first path from start cell to intermediate
        firstPath = self.planner.search(startCellCoords,intermediateGoal)
        if firstPath is False:
            rospy.logwarn("Could not find a first path to the aisle at (%d, %d)", \
                            intermediateGoal[0], intermediateGoal[1])
            return None
        firstPath = self.planner.extractPathToGoal()
        print("end of first path: " + str(list(firstPath.waypoints)[-1].coords))

        #Planning second path from intermediate to goal
        secondPath = self.planner.search(intermediateGoal,goalCellCoords)
        if secondPath is False:
            rospy.logwarn("Could not find second path to the goal at (%d, %d)", \
                            goalCellCoords[0], goalCellCoords[1])
            return None

        secondPath = self.planner.extractPathToGoal()
        print("start of second path: " + str(list(secondPath.waypoints)[0].coords))
        # We contatenate the two planned paths using the addToEnd() function of the
        # first path.
        firstPath.addToEnd(secondPath)
        currentPlannedPath = firstPath
        #pathToGoalFound = self.planner.search(startCellCoords, goalCellCoords)

        # Extract the path
        #currentPlannedPath = self.planner.extractPathToGoal()

        #Show both the first and second planned path separately, and also the combined path (3 screenshots)

        return currentPlannedPath

    # This method drives the robot from the start to the final goal. It includes
    # choosing an aisle to drive down and both waiting and replanning behaviour.
    # Note that driving down an aisle is like introducing an intermediate waypoint.

    def driveToGoal(self, goal):

        # Get the goal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Set the start conditions to the current position of the robot
        pose = self.controller.getCurrentPose()
        start = (pose.x, pose.y)
        startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

        # Work out the initial aisle to drive down
        aisleToDriveDown = self.chooseInitialAisle(startCellCoords, goalCellCoords)

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.

        while rospy.is_shutdown() is False:

            # Plan a path from the robot's current position to the goal. This is called even
            # if the robot waited and used its existing path. This is more robust than, say,
            # stripping cells from the existing path.

            print 'Planning a new path: start=' + str(start) + '; goal=' + str(goal)

            # Plan a path using the current occupancy grid
            self.gridUpdateLock.acquire()
            self.currentPlannedPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, aisleToDriveDown)
            self.gridUpdateLock.release()

            # my note: put the drawing outside of the method to ensure purity
            self._draw_path_by_color(self.currentPlannedPath, color = 'yellow')

            # If we couldn't find a path, give up
            if self.currentPlannedPath is None:
                print("couldn't find a path")
                return False

            # Drive along the path towards the goal. This returns True
            # if the goal was successfully reached. The controller
            # should stop the robot and return False if the
            # stopDrivingToCurrentGoal method is called.
            print("driving to goal")
            goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, \
                                                          goal.theta, self.planner.getPlannerDrawer())

            rospy.logerr('goalReached=%d', goalReached)

            # If we reached the goal, return
            if goalReached is True:
                return True

            # An obstacle blocked the robot's movement. Determine whether we need to
            # wait or replan.

            # Figure out where we are
            pose = self.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)
            # See if we should wait
            waitingGame = self.shouldWaitUntilTheObstacleClears(startCellCoords, goalCellCoords)
            rospy.logwarn("Waiting?: " + str(waitingGame))
            # Depending upon the decision, either wait or determine the new aisle
            # we should drive down.
            if waitingGame is True:
                self.waitUntilTheObstacleClears()
            else:
                aisleToDriveDown = self.chooseAisle(startCellCoords, goalCellCoords)

        return False

    # my mod: for re-usability
    def _draw_path_by_color(self, path, color = 'yellow'):
        # self.planner.searchGridDrawer.update() # do not flush
        self.planner.searchGridDrawer.drawPathGraphicsWithCustomColour(path, color)
        self.planner.searchGridDrawer.waitForKeyPress()
        return

    def _get_current_cell_coord(self):
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))
        pose = self.controller.getCurrentPose()
        start = (pose.x, pose.y)
        return self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)
