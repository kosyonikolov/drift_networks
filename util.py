import math


def rotate_2d(x, y, angle):
    sinA = math.sin(angle)
    cosA = math.cos(angle)

    return x * cosA - y * sinA, x * sinA + y * cosA


class Vec2:
    def __init__(self, x, y):
        self.x = x
        self.y = y
