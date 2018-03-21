from pythreejs import *
from IPython.display import display
from ipywidgets import HTML, Text
from traitlets import link, dlink
from core.tf import *
import time


def getOrigin(r=0.025, color='yellow', X=[0, 0, 0]):
    origin = Mesh(geometry=SphereGeometry(radius=r),
                  material=MeshLambertMaterial(color=color),
                  position=X)
    return origin


def tf(rf, H):
    v = H * O
    v = (np.ravel(v.getA())[0:3])
    rf.position = (v[0], v[1], v[2])

    R = H.getA()[0:3, 0:3].T.ravel()
    rf.setRotationFromMatrix(R)

    return rf


def getJ(length=0.2, arrowLengthRatio=0.1, color='green'):
    arrowLength = length * arrowLengthRatio
    lineLength = length - arrowLength

    rLine = length / 150
    rArrow = length / 50

    line = Mesh(geometry=CylinderGeometry(radiusTop=rLine,
                                          radiusBottom=rLine,
                                          height=lineLength),
                material=MeshLambertMaterial(color=color),
                position=[0, lineLength / 2, 0])

    arrow = Mesh(geometry=CylinderGeometry(radiusTop=0,
                                           radiusBottom=rArrow,
                                           height=arrowLength),
                 material=MeshLambertMaterial(color=color),
                 position=[0, (lineLength + arrowLength) / 2, 0])

    origin = getOrigin()
    origin.add(children=line)
    line.add(children=arrow)

    return origin


def getK(length=0.2):
    k = getJ(length=length, color='blue')
    return tf(k, Rx(np.pi / 2))


def getI(length=0.2):
    k = getJ(length=length, color='red')
    return tf(k, Rz(-np.pi / 2))


def getCoordinateFrame(length=0.2):
    origin = getOrigin()
    ortK = getK(length=length)
    ortI = getI(length=length)
    ortJ = getJ(length=length)

    origin.add(ortK)
    origin.add(ortI)
    origin.add(ortJ)

    return origin


def plot3D(rf):
    scene = Scene()

    tf(rf, Rx(-np.pi / 2))

    key_light = [DirectionalLight(color='#ffffff', intensity=0.6, position=[10, 10, 10])]
    ambient_light = [AmbientLight(color='#777777')]

    scene.add(rf)
    scene.add(key_light)
    scene.add(ambient_light)

    c = PerspectiveCamera(position=[2, 2, 2])
    renderer = Renderer(camera=c, background='black', background_opacity=1,
                        scene=scene, controls=[OrbitControls(controlling=c)],
                        width=800, height=800)
    display(renderer)


def getLine(X1, X2, color='black'):
    linegeom = Geometry(vertices=[X1, X2], colors=[color, color])
    line = Line(geometry=linegeom,
                material=LineBasicMaterial(linewidth=20, vertexColors='VertexColors'),
                type='LinePieces',
                )
    return line


def updateLine(line, r, color='green'):
    line.geometry = Geometry(vertices=[[0, 0, 0], [0, r, 0]], colors=[color, color])


def getCylinder(length, r1, r2, color):
    cylinder = Mesh(geometry=CylinderGeometry(radiusTop=r1,
                                              radiusBottom=r2,
                                              height=length),
                    material=MeshLambertMaterial(color=color),
                    position=[0, 0, 0])
    return cylinder


def getGrid(size=10):
    world = getOrigin()

    h = size / 2
    for xi in np.arange(-h, h + 1, 1):
        X1 = [xi, -h, 0]
        X2 = [xi, h, 0]
        world.add(getLine(X1, X2))

    for yi in np.arange(-h, h + 1, 1):
        X1 = [-h, yi, 0]
        X2 = [h, yi, 0]
        world.add(getLine(X1, X2))
    return world


def putCylinder(length, H, r1=0.01, r2=0.01, color='orange'):
    cylinder = getCylinder(length, r1, r2, color)
    tf(cylinder, H)
    return cylinder


def putPlane(H, size=10, color='#e0e0e0'):
    plane = Mesh(geometry=PlaneGeometry(
        width=size,
        height=size),
        material=MeshLambertMaterial(color=color),
        position=[0, 0, 0])
    tf(plane, H)
    return plane


def rf(H):
    origin = getCoordinateFrame()
    tf(origin, H)
    return origin


def getWheelLine(Hwheel, r):
    X1 = Hwheel * O
    X2 = Hwheel * Ty(r) * O

    X1 = (np.ravel(X1.getA())[0:3]).tolist()
    X2 = (np.ravel(X2.getA())[0:3]).tolist()

    return getLine(X1, X2, color='green')


def getICR(H):
    point = getOrigin(r=0.015)
    tf(point, H)
    return point


class robotPlt:
    def __init__(self, sizeOfMap=5, rWheel=0.05, hWheel=0.025, wheelBase=0.325, track=0.2):
        self.sizeOfMap = sizeOfMap
        self.rWheel = rWheel
        self.hWheel = hWheel

        self.wheelBase = wheelBase
        self.track = track
        self.world = getOrigin()

    def getRadius(self, alpha):
        r = self.wheelBase / np.tan(alpha)
        return r

    def getRadiusInner(self, alpha):
        r = self.getRadius(alpha)
        return (r - self.track / 2)

    def getRadiusOuter(self, alpha):
        r = self.getRadius(alpha)
        return (r + self.track / 2)

    def getSteerAngles(self, alpha):
        r = self.getRadius(alpha)
        alphaInner = np.arctan(self.wheelBase / (self.getRadiusInner(alpha)))
        alphaOuter = np.arctan(self.wheelBase / (self.getRadiusOuter(alpha)))

        return [alphaInner, alphaOuter]

    def getWheel(self):
        wheel = Mesh(geometry=CylinderGeometry(
            radiusTop=self.rWheel,
            radiusBottom=self.rWheel,
            height=self.hWheel,
        ),
            material=MeshLambertMaterial(color='black'),
            position=[0, 0, 0])
        return wheel

    def putWheel(self, H):
        wheel = self.getWheel()
        tf(wheel, H)
        return wheel

    def robot3d(self):
        Hbaselink = Tx(0)
        self.baselink = rf(Hbaselink)

        HbaseEnd = Tx(self.wheelBase)
        baseEnd = tf(getOrigin(), HbaseEnd)
        self.baselink.add(baseEnd)

        HwheelLeftFrontHinge = Ty(self.track / 2)
        wheelLeftFrontHinge = tf(getOrigin(), HwheelLeftFrontHinge)
        baseEnd.add(wheelLeftFrontHinge)
        HwheelRightFrontHinge = Ty(-self.track / 2)
        wheelRightFrontHinge = tf(getOrigin(), HwheelRightFrontHinge)
        baseEnd.add(wheelRightFrontHinge)

        alphaInner = 0
        alphaOuter = 0
        steerAngle = 0

        HwheelLeftFront = Rz(alphaInner)
        self.wheelLeftFront = rf(HwheelLeftFront)
        wheelLeftFrontHinge.add(self.wheelLeftFront)
        ##self.baselink.add(tf(getOrigin(),HwheelLeftFront))

        # HwheelCentralFront = HbaseEnd * Rz(steerAngle)
        # self.baselink.add(rf(HwheelCentralFront))
        # self.baselink.add(getWheelLine(HwheelCentralFront, getRadius(steerAngle)/np.cos(steerAngle)))
        ##self.baselink.add(tf(getOrigin(),HwheelRightFront))

        HwheelRightFront = Rz(alphaOuter)
        self.wheelRightFront = rf(HwheelRightFront)
        wheelRightFrontHinge.add(self.wheelRightFront)
        ##self.baselink.add(tf(getOrigin(),HwheelRightFront))

        r = 0
        Hicr = Ty(r)
        self.ICR = getICR(Hicr)
        self.baselink.add(self.ICR)

        self.rLine = getWheelLine(Tx(0), r)
        self.baselink.add(self.rLine)
        self.lfLine = getWheelLine(Tx(0), r)
        self.wheelLeftFront.add(self.lfLine)
        self.rfLine = getWheelLine(Tx(0), r)
        self.wheelRightFront.add(self.rfLine)

        HwheelLeftRearHinge = Ty(self.track / 2)

        # self.baselink.add(HwheelLeftRearHinge)

        HwheelRightRearHinge = Ty(-self.track / 2)
        # self.baselink.add(HwheelLeftRearHinge)

        HwheelLeftRear = HwheelLeftRearHinge
        wheelLeftRear = tf(getOrigin(), HwheelLeftRear)
        ##self.baselink.add(rf(HwheelLeftRear))
        self.baselink.add(wheelLeftRear)

        HwheelRightRear = HwheelRightRearHinge
        wheelRightRear = tf(getOrigin(), HwheelRightRear)
        ##self.baselink.add(rf(HwheelRightRear))
        self.baselink.add(wheelRightRear)

        self.wheelLeftFront.add(self.putWheel(Tx(0)))
        self.wheelRightFront.add(self.putWheel(Tx(0)))
        wheelLeftRear.add(self.putWheel(Tx(0)))
        wheelRightRear.add(self.putWheel(Tx(0)))
        # self.baselink.add(putWheel(HwheelCentralFront))


        self.baselink.add(putCylinder(length=self.track, H=Tx(0)))
        baseEnd.add(putCylinder(length=self.track, H=Tx(0)))
        self.baselink.add(
            putCylinder(r1=0.02, r2=0.01, length=self.wheelBase, H=Tx(self.wheelBase / 2) * Rz(np.pi / 2)))

        # plot3D(self.baselink)
        return self.baselink

    def state(self, x, y, theta, steerAngle):
        z = self.rWheel
        H = Tx(x) * Ty(y) * Tz(z) * Rz(theta)
        tf(self.baselink, H)

        if (steerAngle != 0):
            r = self.getRadius(steerAngle)
            alphaInner, alphaOuter = self.getSteerAngles(steerAngle)
            ri = self.getRadiusInner(steerAngle) / np.cos(alphaInner)
            ro = self.getRadiusOuter(steerAngle) / np.cos(alphaOuter)

            tf(self.wheelLeftFront, Rz(alphaInner))
            tf(self.wheelRightFront, Rz(alphaOuter))

            Hicr = Ty(r)
            tf(self.ICR, Hicr)

            updateLine(self.rLine, r)
            updateLine(self.lfLine, ri)
            updateLine(self.rfLine, ro)

    def showStates(self, x, y, theta, steerAngle, dt=0):
        self.world.add(self.baselink)
        for i in range(0, np.size(x)):
            self.state(x[i], y[i], theta[i], steerAngle[i])
            time.sleep(dt)

    def flushWorld(self):
        self.world.children = [getCoordinateFrame(length=1), getGrid(), putPlane(Tz(-0.01))]

        # world.add(getOrigin(color='red', X=[1,0,0]))
        # world.add(getOrigin(color='green',X=[0,1,0]))
        # world.add(getOrigin(color='blue',X=[0,0,1]))

    def plotWorld(self):
        self.flushWorld()
        self.robot3d()

        scene = Scene()

        tf(self.world, Rx(-np.pi / 2))

        key_light = [DirectionalLight(color='#ffffff', intensity=0.6, position=[10, 10, 10])]
        ambient_light = [AmbientLight(color='#777777')]

        scene.add(self.world)
        scene.add(key_light)
        scene.add(ambient_light)

        c = PerspectiveCamera(position=[2, 2, 2])
        renderer = Renderer(camera=c, background='black', background_opacity=1,
                            scene=scene, controls=[OrbitControls(controlling=c)],
                            width=800, height=800)
        display(renderer)
