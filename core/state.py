import numpy as np

class State(object):
    def __init__(self, x = 0, y = 0, theta = 0, steerAngle = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.steerAngle = steerAngle

    def get_random_state(self):
        self.x = np.random.random()
        self.y = np.random.random()
        self.theta = np.random.random()
        self.steerAngle = np.random.random()


