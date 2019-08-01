#/usr/bin/env python

import numpy as np
from math import sin, cos, acos, sqrt

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return np.array(v)

class Quaternion:

    def from_axisangle(theta, v):
        theta = theta
        v = normalize(v)

        new_quaternion = Quaternion()
        new_quaternion._axisangle_to_q(theta, v)
        return new_quaternion

    def from_value(value): 
        new_quaternion = Quaternion()
        new_quaternion._val = value
        return new_quaternion

    def _axisangle_to_q(self, v, theta):
        v = normalize(v)
        x, y, z = v
        theta /= 2
        w = cos(theta)
        x = x * sin(theta)
        y = y * sin(theta)
        z = z * sin(theta)
        return w, x, y, z

    def __mul__(self, b):

        if isinstance(b, Quaternion):
            return self.q_mult(b)
        elif isinstance(b, (list, tuple, np.ndarray)):
            if len(b) != 3:
                raise Exception("Input vector has invalid length {len(b)}")
            return self.qv_mult(b)
        else:
            raise Exception("Multiplication with unknown type {type(b)}")

    def q_mult(self,q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

        return w, x, y, z

    def qv_mult(self, q1, v1):
        v1 = (v1[0], v1[1],v1[2])
        q1 = (q1[0], q1[1], q1[2], q1[3])
        q2 = (0.0,) + v1

        print 
        print v1
        print q1
        print q2
        print 
        return self.q_mult(self.q_mult(q1,q2),self.q_conjugate(q1))

    def q_conjugate(self,q):
        w, x, y, z = q
        return (w, -x, -y, -z)

