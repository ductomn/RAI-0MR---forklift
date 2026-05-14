import numpy as np

import heapq
from pathPlaning_Astar.path_Search import Node
from pathPlaning_Astar.path_Search import AstarHybrid


class MainPathPlaning:
    def __init__(self):
        self.path = []  # [x, y, theta]
        self.actions = []  # [v, fi]
        self.index = 0  # this defines index of actual action that is processed
        self.badPath = []  # Here is saved estimated path witch didnt came to the end

    def startPlaning(self, dt, start, goal, stateSpace, tol):
        # Reset
        self.path = []  # [x, y, theta]
        self.actions = []  # [v, fi]
        self.index = 0  # this defines index of actual action that is processed
        self.badPath = []  # Here is saved estimated path witch didnt came to the end

        avalibeActions = [
            [3, np.pi / 5],
            [3, np.pi / 8],
            [3, 0],
            [3, -np.pi / 5],
            [3, -np.pi / 8],
            [-3, np.pi / 5],
            [-3, 0],
            [-3, -np.pi / 5],
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
        open = []
        closed = set()
        open_visited = {}  # state_key
        heapq.heappush(open, startNode)
        i = 0

        # main path calculation loop
        while open:
            # 1. select node
            selectedNode = heapq.heappop(open)

            #  ceckGoal
            if planer.checkGoal(tol, selectedNode):
                self.path, self.actions = planer.reconstructPath(selectedNode)
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
                    heapq.heappush(open, newNode)

                # Stop if too long search
                if i >= 1e6:
                    self.badPath, self.actions = planer.reconstructPath(newNode)
                    return

    def state_key(self, stateCheck):
        x, y, theta = stateCheck
        return (round(float(x), 1), round(float(y), 1), round(float(theta), 1))

    def error(self, epsilon, realState):
        # check if empty
        if not self.path or self.index >= len(self.path):
            return True  # treat as error → trigger re-plan

        # realState [x, y, theta]
        rx, ry, rtheta = realState

        # sim state
        sx, sy, stheta = self.path[self.index]

        # Calculate actual error
        errPos = np.sqrt((rx - sx) ** 2 + (ry - sy) ** 2)
        errTheta = np.abs((rtheta - stheta + np.pi) % (2 * np.pi) - np.pi)

        self.index += 1
        return epsilon <= np.sqrt(errPos**2 + errTheta**2)
