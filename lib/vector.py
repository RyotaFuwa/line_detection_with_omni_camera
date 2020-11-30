import numpy as np


class Vector2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __getitem__(self, i):
        if i == 0:
            return self.x
        elif i == 1:
            return self.y
        raise Exception('a 2d vector only has two elements')

    def length(self):
        return np.sqrt(np.square(self.x) + np.square(self.y))

    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)

    def __mul__(self, v):
        return Vector2D(self.x * v, self.y * v)

    def __truediv__(self, v):
        return Vector2D(self.x / v, self.y / v)

    def floor(self):
        return Vector2D(int(self.x), int(self.y))

    def unit(self):
        return self.__truediv__(self.length())

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def arg(self, other):
        len_self = self.length()
        len_other = other.length()
        if len_self == 0 or len_other == 0:
            return 0
        rad_value = np.arccos(self.dot(other) / (len_self * len_other))
        rad_sign = 1 if self.y < other.y else -1
        return rad_sign * rad_value
