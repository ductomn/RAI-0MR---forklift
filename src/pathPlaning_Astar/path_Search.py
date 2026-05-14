import numpy as np

from pathPlaning_Astar.forklift_sim import ForkSim


class Node:
    def __init__(self, cost, state, action, parent):
        self.state = state  # [x, y, theta]
        self.action = action  # [v, fi]
        self.cost = cost  # [costFromStart, costToGoal, fullCost]
        self.parent = parent  # last node state from witch it was created

    def __lt__(self, other):
        return self.cost[2] < other.cost[2]  # compare cost


class AstarHybrid:
    def __init__(self, dt, avalibeActions, goal, stateSpace):
        self.dt = dt  # time period for 1 movement in s
        self.goal = goal  # [x, y, fi]
        self.avalibeActions = avalibeActions  # [[v, fi] [action2]....]
        self.forkSimClass = ForkSim()  # simulation model for new states
        self.stateSpace = stateSpace  # [maxX, maxY]

    def lookAround(self, state):
        avalibeStates = []
        for action in self.avalibeActions:
            avalibeStates.append(self.forkSimClass.move(state, action, self.dt))
        return avalibeStates

    def cost(self, oldState, sCostOld, newState):
        x, y, theta = newState
        gx, gy, gtheta = self.goal

        # calculate goal cost
        d = np.sqrt((x - gx) ** 2 + (y - gy) ** 2)
        t = np.abs((theta - gtheta + np.pi) % (2 * np.pi) - np.pi)
        gCost = d + t * 0.5

        # calculate cost from start
        oldX, oldY, oldTheta = oldState
        ds = np.sqrt((x - oldX) ** 2 + (y - oldY) ** 2)
        ts = np.abs((theta - oldTheta + np.pi) % (2 * np.pi) - np.pi)
        sCost = ds + ts * 0.5 + sCostOld

        # full cost
        fCost = sCost + gCost

        return [sCost, gCost, fCost]

    def checkBoundaries(self, state):
        x, y, _ = state
        bx, by = self.stateSpace

        return not (0 <= x <= bx and 0 <= y <= by)

    def checkGoal(self, tol, node):
        x, y, theta = node.state
        gx, gy, gtheta = self.goal

        # calculate error
        d = np.sqrt((x - gx) ** 2 + (y - gy) ** 2)
        t = np.abs((theta - gtheta + np.pi) % (2 * np.pi) - np.pi)
        error = d + t * 0.5
        return error < tol

    def reconstructPath(self, goalNode):
        path = []
        action = []
        node = goalNode
        while node is not None:
            path.append(node.state)
            action.append(node.action)
            node = node.parent

        # Flip
        path.reverse()
        action.reverse()

        return path, action
