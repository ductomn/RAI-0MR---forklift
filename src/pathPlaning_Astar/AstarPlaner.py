import numpy as np

import heapq
from path_Search import Node
from path_Search import AstarHybrid


def MainPathPlaning(dt, start, goal, stateSpace, tol):

    avalibeActions = [
        [3, np.pi / 5],
        [3, 0],
        [3, -np.pi / 5],
    ]  # this defines avalibe movements

    # define planer class
    planer = AstarHybrid(dt, avalibeActions, goal, stateSpace)

    # define starting node
    cost = planer.cost(start, 0, start)
    startNode = Node(cost, start, [0, 0], None)

    # Init parameters needed for pathPlaning
    closed = []
    open = []
    heapq.heappush(open, startNode)

    # main path calculation loop
    while open:
        # 1. select node
        selectedNode = heapq.heappop(open)
        closed.append(selectedNode)

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

            #  ceckGoal
            if planer.checkGoal(tol, newNode):
                return planer.reconstructPath(newNode)

            # check if visited
            if any(np.allclose(newNode.state, c.state, atol=0.1) for c in closed):
                continue
            # save
            heapq.heappush(open, newNode)
