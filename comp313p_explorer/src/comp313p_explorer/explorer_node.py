import rospy

from explorer_node_base import ExplorerNodeBase

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.frontierCoords = None

    def updateFrontiers(self):

        self.frontierCoords = None

        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                if self.isFrontierCell(x, y) is True:
                    self.frontierCoords = (x, y)
                    return True

        return False

    def chooseNewDestination(self):

        if self.frontierCoords is None:
            return False, None

        return True, self.frontierCoords
