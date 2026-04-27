import numpy as np

from forklift_sim import ForkSim


class Nodes:
    def __init__(self):
        self.states = []  # [[x, y, theta], [state2] ....]
        self.actions = []  # [[v, fi], [action2] ...]
        self.costs = []  # [[costD, costTheta], [cost2]]

    def saveNode(self, newCost, newState, newAction):
        self.states.append(newState)
        self.actions.append(newAction)
        self.costs.append(newCost)


class AstarHybrid:
    def __init__(self, dt, avalibeActions):
        self.dt = dt  # time period for 1 movement in s
        self.goal = []  # [x, y, fi]
        self.avalibeActions = avalibeActions  # [[v, fi] [action2]....]
        self.forkSimClass = ForkSim()  # simulation model for new states

    def lookAround(self, state):
        avalibeStates = []
        for action in self.avalibeActions:
            avalibeStates.append(self.forkSimClass.move(state, action, self.dt))
        return avalibeStates

    def cost(self, state):
        x, y, theta = state
        gx, gy, gtheta = self.goal

        # calculate cost
        d = np.sqrt((x - gx) ** 2 + (y - gy) ** 2)
        t = np.abs((theta - gtheta + np.pi) % (2 * np.pi) - np.pi)
        return d + t * 0.5


def MainPathPlaning(dt):
    avalibeActions = [
        [3, np.pi / 3],
        [3, 0],
        [3, -np.pi / 3],
    ]  # this defines avalibe movements

    nodes = Nodes()
    planer = AstarHybrid(dt, avalibeActions)
