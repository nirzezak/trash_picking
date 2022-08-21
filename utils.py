import math
import numpy as np

def add_element_wise(l1, l2):
    """
    :param l1, l2: lists
    :returns l1 + l2 element-wise
    """
    return [sum(x) for x in zip(l1, l2)]


def calc_angle(a, b, c):
    """
    @param a, b, c: 2d points
    @returns the angle abc in radians
    abc defines two angles, we return the angle with the lower absolute number
    """
    angle = math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0])
    angle = angle % (2 * np.pi)
    return angle if angle < np.pi else - 2 * np.pi + angle
