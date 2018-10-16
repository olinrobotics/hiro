"""
By Khang Vu, 2017
Last modified Dec 13, 2017

The is the cell represented in a sudoku that has row, col, and potential numbers
If potential numbers has only one element, then that is the result number for the cell
Related: cell.py
"""
class Cell:
    def __init__(self, n=2, row=0, col=0, potentials=None, number=None):
        self.n = n
        self.row = row
        self.col = col
        self.box = self.get_box()
        if number is not None:
            self.potentials = [number]
        else:
            if potentials is None:
                potentials = range(1, n ** 2 + 1)
            self.potentials = potentials

    def get_number(self):
        """
        Get the value of the cell
        :return: int: value at cell [row, col] or -1 if there are more than 1 potentials
        """
        if len(self.potentials) == 1:
            return self.potentials[0]
        else:
            return -1

    def set_number(self, number=-1):
        """
        Set num from 1 to n^2. If number is out of range, reset potentials
        :param number: int
        :return: int: None
        """

        if number < 1 or number > self.n ** 2:
            self.potentials = range(1, self.n ** 2 + 1)
        else:
            self.potentials = [number]

    def set_potentials(self, potentials=None):
        """
        Set potentials for this cell or init potentials
        :param potentials: an array
        :return: Void
        """
        if potentials is None:
            potentials = range(1, self.n ** 2 + 1)
        self.potentials = potentials

    def get_box(self):
        """
        Get box position in Sudoku grid based on row and col
        :return: int: box position
        """
        r = self.row / self.n
        c = self.col / self.n
        return r * self.n + c

    def get_pos_in_box(self):
        """
        Get cell position in a box based on row and col
        :return: int: cell position
        """
        r = self.row % self.n
        c = self.col % self.n
        return r * self.n + c

    def get_pos_in_sudoku(self):
        """
        Get cell position in sudoku grid based on row and col
        :return: int: cell position
        """
        return self.row * self.n ** 2 + self.col

    def get_rc_num(self):
        if self.get_number() == -1:
            return -1, -1, -1

        return self.row, self.col, self.get_number()