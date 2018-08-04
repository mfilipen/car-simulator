import core.point as pt

import numpy as np
import matplotlib.pyplot as plt

class Trajectory(object):
    def __init__(self, start, end):
        self.start = start
        self.end = end

        self.trajectory = []
        self.trajectory.append(start)
        self.trajectory.append(end)

    def plot_trajectory(self):

        x = [p.x for p in self.trajectory]
        y = [p.y for p in self.trajectory]

        plt.figure(figsize=(12, 3))
        plt.suptitle("Trajectory")

        plt.subplot(131)
        plt.plot(range(len(x)),x, 'o-')
        plt.title("x(t)")

        plt.subplot(132)
        plt.plot(range(len(y)), y, 'o-')
        plt.title("y(t)")

        plt.subplot(133)
        plt.plot(x, y,'o-')
        plt.title("(x,y)")

        plt.subplots_adjust(top=0.8)
        plt.show()

    def add_point(self, point):
        last_point = self.trajectory.pop()
        self.trajectory.append(point)
        self.trajectory.append(last_point)

if __name__ == '__main__':
    start = pt.Point(0, 0)
    middle = pt.Point(1, 5)
    end = pt.Point(10, 10)

    tr = Trajectory(start, end)

    tr.add_point(middle)
    tr.plot_trajectory()
