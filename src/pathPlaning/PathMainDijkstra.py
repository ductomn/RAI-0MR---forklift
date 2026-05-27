import numpy as np
import heapq

from pathPlaning.forklift_sim import ForkSim

class Dijkstra:
    def __init__(self, dt, availableActions, goal, stateSpace):
        self.dt = dt
        self.availableActions = availableActions
        self.goal = goal
        self.stateSpace = stateSpace
        self.forkSimClass = ForkSim()

    def cost(self, oldState, newState, oldCost):
        x, y, theta = newState
        xOld, yOld, thetaOld = oldState
        ds = np.sqrt((x-xOld)**2 + (y-yOld)**2)
        return oldCost + ds
    
    def checkBoundaries(self, state):
        x, y, theta = state
        xMax, yMax = self.stateSpace
        return not (0 < x < xMax and 0 < y < yMax)
    
    def checkGoal(self, state,  dTol, thTol):
        x, y, theta = state
        gx, gy, gtheta = self.goal

        dError = np.sqrt((x-gx)**2 + (y-gy)**2)
        thError = np.abs((theta - gtheta + np.pi) % (2 * np.pi) - np.pi)

        return thError < thTol and dError < dTol

    def lookAround(self, state):
        states = []
        for action in self.availableActions:
            states.append(self.forkSimClass.move(state,action, self.dt))
        return states

    def reconstructPath(self, goalNode):
        path = []
        actions = []
        node = goalNode
        while node is not None:
            path.append(node.state)
            actions.append(node.action)
            node = node.parent

        path.reverse()
        actions.reverse()
        return path, actions

class Node:
    def __init__(self, cost, state, action, parent):
        self.state = state  # [x, y, theta]
        self.action = action  # [v, fi]
        self.cost = cost  # [costFromStart, costToGoal, fullCost]
        self.parent = parent  # last node state from witch it was created

    def __lt__(self, other):
        return self.cost < other.cost  # compare cost
    
class MainPathPlaning:
    def __init__(self):
        self.path = []  # [x, y, theta]
        self.actions = []  # [v, fi]
        self.index = 1  # this defines index of actual action that is processed
        self.goalReached = False  # am i in goal ? XD

    def startPlaning(self, dt, start, goal, stateSpace, dTol, thetaTol):
        # Reset
        v = 100  # mm/s
        self.path = []  # [x, y, theta]
        self.actions = []  # [v, fi]
        self.index = 1  # this defines index of actual action that is processed

        availableActions = [
            [v * 1.5, np.pi / 6],
            [v * 1.5, np.pi / 8],  
            [v * 2, 0],
            [v * 1.5, -np.pi / 6],
            [v * 1.5, -np.pi / 8],
            [-v, np.pi / 5],
            [-v, 0],
            [-v, -np.pi / 5],
        ]  # possible movements

        # planner
        planer = Dijkstra(dt, availableActions, goal, stateSpace)

        #start parameters
        startNode = Node(0, start, [0, 0], None,)
        openQueue = []
        closed = set()
        openVisited = {}
        heapq.heappush(openQueue, startNode)
        i = 0

        # path planning loop
        while openQueue:
            selNode = heapq.heappop(openQueue)
            if self.stateKey(selNode.state) in closed:
                continue

            # check goal
            if planer.checkGoal(selNode.state, dTol, thetaTol):
                self.path, self.actions = planer.reconstructPath(selNode)
                return
            
            # check boundaries
            if planer.checkBoundaries(goal):
                return
            closed.add(self.stateKey(selNode.state))

            # get new states from actual state
            newStates = planer.lookAround(selNode.state)

            # for new nodes get state, action, parent
            for state, action in zip(newStates, availableActions):
                if planer.checkBoundaries(state):
                    continue

                # new node
                newCost = planer.cost(selNode.state, state, selNode.cost)
                newNode = Node(newCost, state, action, selNode)
                key = self.stateKey(state)

                if key in closed:
                    continue

                if key not in openVisited or newCost < openVisited[key]:
                    i += 1
                    openVisited[key] = newCost
                    heapq.heappush(openQueue, newNode)

                if i >= 1e6:
                    self.actions = planer.reconstructPath(newNode)[1]
                    return






    def stateKey(self, stateCheck):
        # this function is only for unpacking states
        x, y, theta = stateCheck
        return (round(float(x), -1), round(float(y), -1), round(float(theta), 1))

    def newGoalState(self, goalState):
        # unpack states
        x, y, theta = goalState

        # define how far i want to move in mm
        c = 0
        # calculate change in mm
        dx = c * np.cos(theta)
        dy = c * np.sin(theta)

        # add change to state
        nx = x + dx
        ny = y + dy

        return [nx, ny, theta]

    def inGoal(self, epsilon, realState, goalState):
        # goal state
        gx, gy, gtheta = goalState
        # realState [x, y, theta]
        rx, ry, rtheta = realState

        # check if in goal
        errPos = np.sqrt((rx - gx) ** 2 + (ry - gy) ** 2)
        errTheta = (np.abs((rtheta - gtheta + np.pi) % (2 * np.pi) - np.pi)) * 10

        self.goalReached = epsilon >= np.sqrt(errPos**2 + errTheta**2)

    def error(self, epsilonPos, realState):
        """
        This function checks if the real state is close enough to the planned "actual" state. NOT goal.
        """

        # check if empty
        if not self.path or self.index >= len(self.path):
            return True  # treat as error → trigger re-plan

        # realState [x, y, theta]
        rx, ry, rtheta = realState

        # sim state
        sx, sy, stheta = self.path[self.index]

        # Calculate actual error
        errPos = np.sqrt((rx - sx) ** 2 + (ry - sy) ** 2)
        errTheta = (np.abs((rtheta - stheta + np.pi) % (2 * np.pi) - np.pi))

        self.index += 1
        return epsilonPos <= errPos**2