import core.pyThreejsVis as vis
import core.map as mp
import core.state as st
import core.car as cr

import numpy as np
import matplotlib.pyplot as plt


class Simulation(object):
    def __init__(self):

        self.map = mp.Map()
        self.map.map_from_png()

        self.car = cr.Car()

        self.vis = vis.robotPlt(map=self.map)
        self.vis.plotWorld()

        self.states = []
        self.states.append(self.car.get_state())
        self.vis.state_subscriber(self.states)

    def plot_map(self):
        plt.title("Map")
        plt.imshow(self.map.data, cmap='gray')
        plt.show()

    def state_subscriber(self, states):
        self.states = states
        self.vis.state_subscriber(self.states)

    def get_car_states(self):
        return self.states


