import numpy as np


## Bycicle model with back wheel rotation
class ForkSim:
    def __init__(self, geometry=90, maxMin=[np.pi / 4, 500]):
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
        fi = np.clip(fi, -self.limFi, self.limFi)
        v = np.clip(v, -self.limV, self.limV)

        # Model sim movement
        dtheta = -(v / self.L) * np.tan(fi)
        thetaNew = theta + dtheta * dt

        # Saturation of theta
        thetaNew = (thetaNew + np.pi) % (2 * np.pi) - np.pi

        # calculate new position
        if abs(fi) < 1e-5:  # Close enough to zero
            xNew = x + v * np.cos(theta) * dt
            yNew = y + v * np.sin(theta) * dt
        else:
            R = self.L / np.tan(fi)
            xNew = x + R * (np.sin(thetaNew) - np.sin(theta))
            yNew = y - R * (np.cos(thetaNew) - np.cos(theta))

        # dx = v * np.cos(thetaNew)
        # dy = v * np.sin(thetaNew)

        # xNew = x + dx * dt
        # yNew = y + dy * dt

        return np.array([xNew, yNew, thetaNew])
