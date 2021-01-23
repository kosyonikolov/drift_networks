import math


def rotate_2d(x, y, angle):
    sinA = math.sin(angle)
    cosA = math.cos(angle)

    return x * cosA - y * sinA, x * sinA + y * cosA
