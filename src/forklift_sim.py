import numpy as np


## Bycicle model with back wheel rotation
class ForkSim:
    def __init__(self, geometry=0.01, maxMin=[20, np.pi]):
        self.L = geometry  # mm
        self.limFi = maxMin[0]  # rad
        self.limV = maxMin[1]  # m/s

    def move(self, state, action, dt):
        # state = [x, y, theta]
        # v = velocity
        # fi = steering
        # dt = time interval

        v, fi = action
        x, y, theta = state

        # Saturation
        if fi > self.limFi:
            fi = self.limFi
        elif fi < -self.limFi:
            fi = -self.limFi

        if v > self.limV:
            v = self.limV
        elif v < -self.limV:
            v = -self.limV

        # Model sim movement
        dx = v * np.cos(theta)
        dy = v * np.sin(theta)
        dtheta = -(v / self.L) * np.tan(fi)

        xNew = x + dx * dt
        yNew = y + dy * dt
        thetaNew = theta + dtheta * dt

        # Saturation of theta
        thetaNew = (thetaNew + np.pi) % (2 * np.pi) - np.pi

        return np.array([xNew, yNew, thetaNew])


class CorrectPath:
    def __init__(self, state0, action0):
        self.states = [state0]  # [x, y, theta]
        self.actions = [action0]  # [v, fi]
        self.index = 0  # this defines index of actual action that is processed

    def save(self, newState, newAction):
        self.states.append(newState)
        self.actions.append(newAction)

    def error(self, realState):
        # realState [x, y, theta]
        rx, ry, rtheta = realState

        # sim state
        sx, sy, stheta = self.states[self.index]

        # Calculate actual error
        errPos = np.sqrt((rx - sx) ** 2 + (ry - sy) ** 2)
        errTheta = np.abs((rtheta - stheta + np.pi) % (2 * np.pi) - np.pi)

        return [errPos, errTheta]
