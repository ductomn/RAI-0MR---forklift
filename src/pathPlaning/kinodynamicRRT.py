import numpy as np

from pathPlaning.forklift_sim import ForkSim


class Node:
    def __init__(self, state, action=None, parent=None):
        self.state = state  # [x, y, theta]
        self.action = action if action is not None else [0, 0] # [v, fi]
        self.parent = parent 


def distance(state1, state2, angle_scale=10):
    x1, y1, theta1 = state1
    x2, y2, theta2 = state2

    position_error = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    theta_error = abs((theta1 - theta2 + np.pi) % (2 * np.pi) - np.pi)
    error = position_error + theta_error * angle_scale

    return error


class KinodynamicRRT:
    def __init__(self, dt, avalibeActions, goal, stateSpace, maxIter=1200, goalBias=0.25):
        self.dt = dt # time period for 1 movement in s
        self.goal = goal # [x, y, fi]
        self.avalibeActions = avalibeActions  # [[v, fi] [action2]....]
        self.forkSimClass = ForkSim()  # simulation model for new states
        self.stateSpace = stateSpace  # [maxX, maxY]

        self.maxIter = maxIter  
        if goalBias < 0 or goalBias > 1:
            raise ValueError("GoalBias is probability -> must be between 0 and 1.")
        self.goalBias = goalBias    # probability of sampling the goal state directly

    def plan(self, start, tol):
        """
        Iterate until goal reached within tolerance or until maxIter reached.

        Parameters
        ----------
        start : {array of floats}
            Starting point [x, y, theta]
        tol : {float}
            Tolerance of goal position

        Returns
        -------
        out : {tuple[list, list]}
            Tuple containing path and actions [path, actions]
            
        """
        tree = [Node(start)]
        goal_tolerance = tol
        best_goal_node = tree[0]
        best_goal_distance = distance(tree[0].state, self.goal)

        for _ in range(self.maxIter):
            random_state = self.sampleState()
            nearest_node = min(tree, key=lambda node: distance(node.state, random_state))

            new_node = self.expand(nearest_node, random_state)
            if new_node is None:
                continue

            tree.append(new_node)
            goal_distance = distance(new_node.state, self.goal)
            if goal_distance < best_goal_distance:
                best_goal_node = new_node
                best_goal_distance = goal_distance

            if goal_distance < goal_tolerance:
                return self.reconstructPath(new_node)

        if best_goal_node.parent is not None and best_goal_distance < goal_tolerance * 2.5:
            return self.reconstructPath(best_goal_node)

        return [], []

    def sampleState(self):
        """
        Generate a random state within State Space. 
        Goal is generated with probability of self.goalBias.
        """
        if np.random.rand() < self.goalBias:
            return self.goal

        bx, by = self.stateSpace
        return np.array(
            [
                np.random.uniform(0, bx),
                np.random.uniform(0, by),
                np.random.uniform(-np.pi, np.pi),
            ]
        )

    def expand(self, nearestNode, randomState):
        """
        Expand the tree from nearestNode in the direction of randomState.
        The expansion respects the avalibeActions and the defined movement of forkSimClass.
        """
        best_state = None
        best_action = None
        best_distance = float("inf")

        for action in self.avalibeActions:
            candidate_state = self.forkSimClass.move(nearestNode.state, action, self.dt)

            if self.checkBoundaries(candidate_state):
                continue

            candidate_distance = distance(candidate_state, randomState)
            if candidate_distance < best_distance:
                best_state = candidate_state
                best_action = action
                best_distance = candidate_distance

        if best_state is None:
            return None

        return Node(best_state, best_action, nearestNode)

    def checkBoundaries(self, state):
        x, y, _ = state
        bx, by = self.stateSpace

        return not (0 <= x <= bx and 0 <= y <= by)

    def checkGoal(self, tol, node):
        return distance(node.state, self.goal) < tol

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
