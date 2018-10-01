from enum import Enum


class Color(Enum):
    RED = 3
    GREEN = 1
    PURPLE = 2


class Shape(Enum):
    DIAMOND = 3
    WAVY = 2
    CIRCLE = 1


class Fill(Enum):
    SOLID = 1
    DASH = 3
    EMPTY = 2