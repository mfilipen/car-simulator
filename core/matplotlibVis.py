import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from core.tf import *


def vectorInLine(start, end):
    return np.concatenate((np.ravel(start.getA().T), np.ravel(end.getA().T)))


## plot Reference Frame
def plotRF(H=np.eye(4), length=0.1):
    o = H * O
    i = H * I
    j = H * J
    k = H * K
    soa = np.array([vectorInLine(o, T(-o) * i), vectorInLine(o, T(-o) * j), vectorInLine(o, T(-o) * k)])
    X, Y, Z, _, U, V, W, _ = zip(*soa)
    plt.quiver(X, Y, Z, U, V, W,
               color=['r', 'g', 'b', 'r', 'r', 'g', 'g', 'b', 'b'], arrow_length_ratio=0.08, length=length)


class robotPlt():
    def __init__(self, sizeOfMap=5, rWheel=0.05, hWheel=0.025, wheelBase=0.325, track=0.2):
        self.sizeOfMap = sizeOfMap
        self.rWheel = rWheel
        self.hWheel = hWheel

        self.wheelBase = wheelBase
        self.track = track

    def printCoordinateFrame(self):
        global fig, ax
        fig = plt.figure(figsize=(9, 5))
        ax = fig.add_subplot(111, aspect='equal', projection='3d')

        ticks = np.arange(0, self.sizeOfMap, 1)

        ax.set_xlim(-self.sizeOfMap, self.sizeOfMap)
        ax.set_ylim(-self.sizeOfMap, self.sizeOfMap)
        ax.set_zlim(0, 2 * self.sizeOfMap)

        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')

        ax.view_init(azim=180, elev=90)
        ax.zaxis.set_ticklabels([])
        plotRF(length=1)

    def plotRobot(self, x, y, theta, steerAngle):
        self.printCoordinateFrame()

        z = self.rWheel
        Hbaselink = Tx(x) * Ty(y) * Tz(z) * Rz(theta)
        plotRF(Hbaselink)

        HbaseEnd = Hbaselink * Tx(self.wheelBase)
        # plotRF(HbaseEnd)

        HwheelLeftFrontHinge = HbaseEnd * Ty(self.track / 2)
        # plotRF(HwheelLeftFrontHinge)

        HwheelRightFrontHinge = HbaseEnd * Ty(-self.track / 2)
        # plotRF(HwheelRightFrontHinge)

        HwheelLeftFront = HwheelLeftFrontHinge * Rz(steerAngle)
        plotRF(HwheelLeftFront)

        HwheelRightFront = HwheelRightFrontHinge * Rz(steerAngle)
        plotRF(HwheelRightFront)

        HwheelLeftRearHinge = Hbaselink * Ty(self.track / 2)
        # plotRF(HwheelLeftRearHinge)

        HwheelRightRearHinge = Hbaselink * Ty(-self.track / 2)
        # plotRF(HwheelRightRearHinge)

        HwheelLeftRear = HwheelLeftRearHinge
        plotRF(HwheelLeftRear)

        HwheelRightRear = HwheelRightRearHinge
        plotRF(HwheelRightRear)
        plt.show()
