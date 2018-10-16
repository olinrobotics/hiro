from Attribute import *

"""
Authors: Cassandra Overney and Khang Vu
Purpose: Defines a single set card by its four attributes and the coordinate 
Run: This file is imported in Game.py and Turn.py

"""


class Set:
    """
    The Set class represents the cards that opencv detects. This class also takes three cards and determines if they
    are a Set.
    """

    def __init__(self, color, num, shape, fill, coord):
        self.color = color
        self.shape = shape
        self.num = num
        self.fill = fill
        self.coord = coord

    @staticmethod
    def is_set(card1, card2, card3):
        """
        Check if 3 cards are a set

        :param card1: Set object 1
        :param card2: Set object 2
        :param card3: Set object 3
        :return: True if is set and False if not set
        """
        colors = [card1.color, card2.color, card3.color]
        shapes = [card1.shape, card2.shape, card3.shape]
        fills = [card1.fill, card2.fill, card3.fill]
        nums = [card1.num, card2.num, card3.num]
        if not card1.check_attribute(colors):
            return False
        if not card1.check_attribute(shapes):
            return False
        if not card1.check_attribute(fills):
            return False
        if not card1.check_attribute(nums):
            return False
        return True

    @staticmethod
    def check_attribute(array):
        """
        Checks a given attribute of the three Set cards to see if they either are all the same, completely different
        or somewhere in between

        :param array: array of the attributes of the three cards
        :return: True if attribute is all the same or all different, false otherwise
        """
        if array[0] == array[1] and array[1] == array[2]:
            return True
        elif array[0] != array[1] and array[1] != array[2] and array[0] != array[2]:
            return True
        else:
            return False


if __name__ == "__main__":
    ex1 = Set(Color.BLUE, Shape.CIRCLE, 1, Fill.DASH)
    ex2 = Set(Color.GREEN, Shape.WAVY, 2, Fill.EMPTY)
    ex3 = Set(Color.BLUE, Shape.CIRCLE, 3, Fill.SOLID)

    print ex1.shape
    print Set.is_set(ex1, ex2, ex3)
