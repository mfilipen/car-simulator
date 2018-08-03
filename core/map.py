import numpy as np
import matplotlib.pyplot as plt
import cv2

class Map(object):

    def __init__(self, resolution = 0.1, width = 100, height = 100):

        # The map resolution [m/cell]
        self.resolution = resolution
        # Map width [cells]
        self.width = width
        # Map height [cells]
        self.height = height

        # The map data, in row-major order, starting with (0,0).  Occupancy
        # probabilities are in the range [0,100].  Unknown is -1.

        self.data = np.zeros((width, height), dtype=int)

    def map_from_png(self, path = "./core/map.png"):
        im = plt.imread(path)
        gray = im[:, :, 3] * 100
        self.data = gray

if __name__ == "__main__":
    map = Map()
    map.map_from_png()

    plt.title("map")
    plt.imshow(map.data, cmap='gray')
    plt.show()

