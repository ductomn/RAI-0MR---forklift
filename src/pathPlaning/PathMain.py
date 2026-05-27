import numpy as np

import heapq
from pathPlaning.path_Search import AstarHybrid


class Node:
    def __init__(self, cost, state, action, parent):
        self.state = state  # [x, y, theta]
        self.action = action  # [v, fi]
        self.cost = cost  # [costFromStart, costToGoal, fullCost]
        self.parent = parent  # last node state from witch it was created

    def __lt__(self, other):
        return self.cost[2] < other.cost[2]  # compare cost


class MainPathPlaning:
    def __init__(self):
        self.path = []  # [x, y, theta]
        self.actions = []  # [v, fi]
        self.index = 1  # this defines index of actual action that is processed
        self.goalReached = False  # am i in goal ? XD

    def startPlaning(self, dt, start, goal, stateSpace, tol, thetaTol):
        # Reset
        v = 100  # mm/s
        self.path = []  # [x, y, theta]
        self.actions = []  # [v, fi]
        self.index = 1  # this defines index of actual action that is processed

        avalibeActions = [
            [v * 1.5, np.pi / 6],  # 30
            [v * 1.5, np.pi / 8],  # 22,5 ==> 45 == 50
            [v * 2, 0],
            [v * 1.5, -np.pi / 6],
            [v * 1.5, -np.pi / 8],
            [-v, np.pi / 5],
            [-v, 0],
            [-v, -np.pi / 5],
        ]  # this defines avalibe movements

        # define planer class
        planer = AstarHybrid(dt, avalibeActions, goal, stateSpace)

        # define starting node
        startNode = Node(
            [0, planer.cost(start, 0, start)[1], planer.cost(start, 0, start)[1]],
            start,
            [0, 0],
            None,
        )

        # Init parameters needed for pathPlaning
        open_set = []
        closed = set()
        open_visited = {}  # state_key
        heapq.heappush(open_set, startNode)
        i = 0

        # main path calculation loop
        while open_set:
            # 1. select node
            selectedNode = heapq.heappop(open_set)

            #  ceckGoal
            if planer.checkGoal(tol, thetaTol, selectedNode):
                self.path, self.actions = planer.reconstructPath(selectedNode)
                return
            elif planer.checkBoundaries(goal):
                return

            closed.add(self.state_key(selectedNode.state))

            # 2. expand node
            newStates = planer.lookAround(selectedNode.state)

            # 3. assign state + action + parent to nodes
            for state, action in zip(newStates, avalibeActions):
                # checkBoundaries
                if planer.checkBoundaries(state):
                    continue

                # calculate new cost for node
                newCost = planer.cost(selectedNode.state, selectedNode.cost[0], state)

                # save node
                newNode = Node(newCost, state, action, selectedNode)

                # check if visited
                if self.state_key(state) in closed:
                    continue

                # check if in visited if ewerithing is ok save
                key = self.state_key(state)
                if key not in open_visited or newCost[2] < open_visited[key]:
                    i += 1
                    open_visited[key] = newCost[2]
                    heapq.heappush(open_set, newNode)

                # Stop if too long search
                if i >= 1e5:
                    self.actions = planer.reconstructPath(newNode)[1]
                    return

    def state_key(self, stateCheck):
        # this function is only for unpacking states
        x, y, theta = stateCheck
        return (round(float(x), -1), round(float(y), -1), round(float(theta), 1))

    def newGoalState(self, goalState):
        # unpack states
        x, y, theta = goalState

        # define how far i want to move in mm
        c = -200
        # calculate change in mm
        dx = c * np.cos(theta)
        dy = c * np.sin(theta)

        # add change to state
        nx = x + dx
        ny = y + dy

        return [nx, ny, theta]

    def inGoal(self, epsilon, epsilonTheta, realState, goalState):
        # goal state
        gx, gy, gtheta = goalState
        # realState [x, y, theta]
        rx, ry, rtheta = realState

        # check if in goal
        errPos = np.sqrt((rx - gx) ** 2 + (ry - gy) ** 2)
        errTheta = (np.abs((rtheta - gtheta + np.pi) % (2 * np.pi) - np.pi)) * 10

        self.goalReached = epsilon >= errPos and epsilonTheta >= errTheta

    def error(self, epsilonPos, epsilonTheta, realState):
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
        return epsilonPos <= errPos and epsilonTheta <= errTheta
