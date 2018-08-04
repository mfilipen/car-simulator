import numpy as np
from IPython.display import Latex
import sympy as sym

sym.init_printing()


def print(a):
    display(Latex(matrix2Latex(a)))


def matrix2Latex(a):
    if len(a.shape) > 2:
        raise ValueError('bmatrix can at most display two dimensions')
    lines = str(a).replace('[', '').replace(']', '').splitlines()
    rv = [r'\begin{bmatrix}']
    rv += ['  ' + ' & '.join(l.split()) + r'\\' for l in lines]
    rv += [r'\end{bmatrix}']
    return '\n'.join(rv)


def getMatrix(x11, x12, x13, x14,
              x21, x22, x23, x24,
              x31, x32, x33, x34):
    return np.matrix(((x11, x12, x13, x14),
                      (x21, x22, x23, x24),
                      (x31, x32, x33, x34),
                      (0, 0, 0, 1)))


def getVector(x11, x12, x13):
    return np.matrix([x11, x12, x13, 1]).T


def Rx(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = getMatrix(1, 0, 0, 0,
                  0, c, -s, 0,
                  0, s, c, 0)
    return R


def Ry(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = getMatrix(c, 0, s, 0,
                  0, 1, 0, 0,
                  -s, 0, c, 0)
    return R


def Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = getMatrix(c, -s, 0, 0,
                  s, c, 0, 0,
                  0, 0, 1, 0)
    return R


def Tx(x):
    T = getMatrix(1, 0, 0, x,
                  0, 1, 0, 0,
                  0, 0, 1, 0)
    return T


def Ty(y):
    T = getMatrix(1, 0, 0, 0,
                  0, 1, 0, y,
                  0, 0, 1, 0)
    return T


def Tz(z):
    T = getMatrix(1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, z)
    return T


def T(v):
    v = v.getA()
    T = getMatrix(1, 0, 0, v[0],
                  0, 1, 0, v[1],
                  0, 0, 1, v[2])
    return T

def RxSym(theta):
    c, s = sym.cos(theta), sym.sin(theta)
    R = sym.Matrix([[1, 0, 0, 0],
                    [0, c, -s, 0],
                    [0, s, c, 0],
                    [0, 0, 0, 1]])
    return R


def RySym(theta):
    c, s = sym.cos(theta), sym.sin(theta)

    R = sym.Matrix([[c, 0, s, 0],
                    [0, 1, 0, 0],
                    [-s, 0, c, 0],
                    [0, 0, 0, 1]])

    return R


def RzSym(theta):
    c, s = sym.cos(theta), sym.sin(theta)

    R = sym.Matrix([[c, -s, 0, 0,],
                    [s, c, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    return R


O = getVector(0, 0, 0)
I = getVector(1, 0, 0)
J = getVector(0, 1, 0)
K = getVector(0, 0, 1)
