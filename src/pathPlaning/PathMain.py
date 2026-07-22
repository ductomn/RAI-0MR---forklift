import numpy as np

import heapq
from pathPlaning.path_Search import AstarHybrid
from pathPlaning.kinodynamicRRT import KinodynamicRRT
from pathPlaning.PathMainDijkstra import Dijkstra


class Node:
    def __init__(self, cost, state, action, parent):
        self.state = state  # [x, y, theta]
        self.action = action  # [v, fi]
        self.cost = cost  # [costFromStart, costToGoal, fullCost]
        self.parent = parent  # last node state from witch it was created

    def __lt__(self, other):
        if isinstance(self.cost, (list, tuple, np.ndarray)):
            return self.cost[2] < other.cost[2]
        return self.cost < other.cost


class MainPathPlaning:
    # The old target was 200 mm behind the pallet marker. A 100 mm target is
    # the midpoint between that virtual target and the detected marker.
    GOAL_STANDOFF_MM = 100.0

    def __init__(self):
        self.path = []  # [x, y, theta]
        self.actions = []  # [v, fi]
        self.index = 1  # this defines index of actual action that is processed
        self.goalReached = False  # am i in goal ? XD
        self.plan_complete = False

        v = 100  # mm/s
        self.avalibeActions = [
            [v * 1.5, np.pi / 6],  # 30
            [v * 1.5, np.pi / 8],  # 22,5 ==> 45 == 50
            [v * 2, 0],
            [v * 1.5, -np.pi / 6],
            [v * 1.5, -np.pi / 8],
            [-v, np.pi / 5],
            [-v, 0],
            [-v, -np.pi / 5],
        ]  # this defines avalibe movements

    def startAstarHybrid(self, dt, start, goal, stateSpace, tol, thetaTol):
        # Reset
        self.path = []  # [x, y, theta]
        self.actions = []  # [v, fi]
        self.index = 1  # this defines index of actual action that is processed
        self.plan_complete = False

        # define planer class
        planer = AstarHybrid(dt, self.avalibeActions, goal, stateSpace)

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
        best_node = startNode
        i = 0

        # main path calculation loop
        while open_set:
            # 1. select node
            selectedNode = heapq.heappop(open_set)

            #  ceckGoal
            if selectedNode.cost[1] < best_node.cost[1]:
                best_node = selectedNode
            if planer.checkGoal(tol, thetaTol, selectedNode):
                self.path, self.actions = planer.reconstructPath(selectedNode)
                self.plan_complete = True
                return
            elif planer.checkBoundaries(goal):
                return

            closed.add(self.state_key(selectedNode.state))

            # 2. expand node
            newStates = planer.lookAround(selectedNode.state)

            # 3. assign state + action + parent to nodes
            for state, action in zip(newStates, self.avalibeActions):
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
                    # Keep the best partial trajectory for visualisation. It is
                    # deliberately marked incomplete so autonomous control will
                    # not execute it as though the goal had been reached.
                    self.path, self.actions = planer.reconstructPath(best_node)
                    return

    def startKinodynamicRRT(self, dt, start, goal, stateSpace, tol):
        # Reset
        self.path = []  # [x, y, theta]
        self.actions = []  # [v, fi]
        self.index = 1  # this defines index of actual action that is processed
        self.plan_complete = False

        planer = KinodynamicRRT(dt, self.avalibeActions, goal, stateSpace)
        self.path, self.actions = planer.plan(start, tol)
        self.plan_complete = bool(self.path)

    def startDijkstra(self, dt, start, goal, stateSpace, dTol, thetaTol):
        """Run Dijkstra while keeping the shared path-planner state."""
        self.path = []
        self.actions = []
        self.index = 1
        self.plan_complete = False

        planer = Dijkstra(dt, self.avalibeActions, goal, stateSpace)
        start_node = Node(0, start, [0, 0], None)
        open_queue = [start_node]
        closed = set()
        open_visited = {}
        best_node = start_node
        best_distance = float("inf")
        expansions = 0

        while open_queue:
            selected_node = heapq.heappop(open_queue)
            selected_key = self.state_key(selected_node.state)
            if selected_key in closed:
                continue

            distance_to_goal = np.linalg.norm(
                np.asarray(selected_node.state[:2]) - np.asarray(goal[:2])
            )
            if distance_to_goal < best_distance:
                best_node = selected_node
                best_distance = distance_to_goal

            if planer.checkGoal(selected_node.state, dTol, thetaTol):
                self.path, self.actions = planer.reconstructPath(selected_node)
                self.plan_complete = True
                return
            if planer.checkBoundaries(goal):
                return

            closed.add(selected_key)
            for state, action in zip(
                planer.lookAround(selected_node.state), self.avalibeActions
            ):
                if planer.checkBoundaries(state):
                    continue

                new_cost = planer.cost(
                    selected_node.state, state, selected_node.cost
                )
                key = self.state_key(state)
                if key in closed:
                    continue
                if key not in open_visited or new_cost < open_visited[key]:
                    open_visited[key] = new_cost
                    heapq.heappush(
                        open_queue,
                        Node(new_cost, state, action, selected_node),
                    )
                    expansions += 1

                if expansions >= 100_000:
                    self.path, self.actions = planer.reconstructPath(best_node)
                    return

    def clear(self):
        self.path = []
        self.actions = []
        self.index = 1
        self.goalReached = False
        self.plan_complete = False

    def state_key(self, stateCheck):
        # this function is only for unpacking states
        x, y, theta = stateCheck
        return (round(float(x), -1), round(float(y), -1), round(float(theta), 1))

    def newGoalState(self, goalState):
        """Return a virtual approach goal with clearance from the pallet."""
        x, y, theta = goalState
        offset = -self.GOAL_STANDOFF_MM
        return [
            x + offset * np.cos(theta),
            y + offset * np.sin(theta),
            theta,
        ]

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
        errTheta = np.abs((rtheta - stheta + np.pi) % (2 * np.pi) - np.pi)

        self.index += 1
        return epsilonPos <= errPos and epsilonTheta <= errTheta
