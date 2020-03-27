from dijkstra_planner import DijkstraPlanner
import math

# self class implements the A* search algorithm

from enum import Enum

class CostToComeHeuristic(Enum):
    EUCLIDEAN = 1
    MANHATTAN = 2
    OCTILE = 3
    SQUARED_EUCLIDEAN = 4

class AStarPlanner(DijkstraPlanner):
    
    def __init__(self, title, occupancyGrid):
        DijkstraPlanner.__init__(self, title, occupancyGrid)
        self.heuristic = CostToComeHeuristic.OCTILE
        self.alpha = 1

    def setHeuristc(self, heuristic):
        self.heuristic = heuristic
        
    def setHeuristicWeight(self, alpha):
        self.alpha = alpha

    def computePriorityQueueKey(self, cell):
        
        # Compute the cost-to-go using a suitable heuristic
        dXG = abs(cell.coords[0] - self.goal.coords[0])
        dYG = abs(cell.coords[1] - self.goal.coords[1])
 
        if self.heuristic is CostToComeHeuristic.EUCLIDEAN:
            G = math.sqrt(dXG * dXG + dYG * dYG)
        elif self.heuristic is CostToComeHeuristic.MANHATTAN:
            G = dXG + dYG
        elif self.heuristic is CostToComeHeuristic.OCTILE:
            G = max(dXG, dYG) + (math.sqrt(2) -1) * min(dXG, dYG)
        elif self.heuristic is CostToComeHeuristic.SQUARED_EUCLIDEAN:
            G = dXG * dXG + dYG * dYG
            
        # Compute the cost using weighted A*
        key = cell.pathCost + self.alpha * G

        return key
