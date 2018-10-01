#!/usr/bin/env python

import numpy as np
import random
import time

"""
the internal game board for the Connect 4 AI
written by Kevin Zhang

used as an importable package for connect4.py in this folder

holds the board as an array of arrays, update and visualization functions.
checks for when game is over, and also who won. can also detect ties.

"""

class C4Board(object):
    """
    the connect 4 board, held as an array of array of 0 (empty), 1(player 1), and
    2 (player 2)
    """

    def __init__(self, render):
        self.action_space = [0,1,2,3,4,5,6]
        self.n_actions = len(self.action_space)
        self.turn = 0
        self.render = render
        self.init_board()


    def init_board(self):
        """
        starts the board as a 7x6 with all 0s
        """

        self.board = []
        for i in range(7):
            column = [0]*6
            self.board.append(column)
        self.column_stack = [5,5,5,5,5,5,5]


    def reset(self):
        """
        resets the board for another game, refreshes variables
        """

        self.init_board()
        self.turn = 0
        self.visualize()
        return self.board


    def make_move(self, column, player):
        """
        physical implementation of a move, where a cell in the board is changed based
        on an action from a player, will detect ties here
        """

        move = (column, self.column_stack[column])
        if self.column_stack[column] == -1:
            return self.board, move
        self.board[column][self.column_stack[column]] = player
        self.column_stack[column] -= 1

        if np.sum(self.column_stack) == -7:
            return self.board, False
        return self.board, move


    def connect_4(self, move):
        """
        checks for 4 in a row in all directions
        """

        column = move[0]
        row = move[1]

        # checks for row victory
        count = 0
        for i in range(6):
            if  self.board[i][row] != 0 and self.board[i][row] == self.board[i+1][row]:
                count += 1
                if count == 3:
                    # print "ROW"
                    return True
            else:
                count = 0

        # checks for column victory
        count = 0
        for i in range(5):
            if self.board[column][i] != 0 and self.board[column][i] == self.board[column][i+1]:
                count += 1
                if count == 3:
                    # print "COL"
                    return True
            else:
                count = 0

        # checks for major diagonal victory
        count = 0
        column1 = move[0]
        row1 = move[1]
        while column1 > 0 and row1 > 0:
            column1 -= 1
            row1 -= 1
        while column1 < 6 and row1 < 5:
            if self.board[column1][row1] != 0 and self.board[column1][row1] == self.board[column1+1][row1+1]:
                count += 1
                if count == 3:
                    # print "MAJ DIAG"
                    return True
            else:
                count = 0
            column1 += 1
            row1 += 1

        # checks for minor diagonal victory
        count = 0
        column2 = move[0]
        row2 = move[1]
        while column2 > 0 and row2 < 5:
            column2 -= 1
            row2 += 1
        while column2 < 6 and row2 > 0:
            if self.board[column2][row2] != 0 and self.board[column2][row2] == self.board[column2+1][row2-1]:
                count += 1
                if count == 3:
                    # print "MIN DIAG"
                    return True
            else:
                count = 0
            column2 += 1
            row2 -= 1
        return False


    def step(self, action, player):
        """
        the update board method, will check for a winner and if game is done
        """

        s_, move = self.make_move(action, player)

        if  move == False:
            done = "Draw"
        elif self.connect_4(move):
            done = True
        else:
            done = False

        self.visualize()

        return s_, done


    def visualize(self):
        """
        visualizer for the board in terminal, can be toggled
        """

        if self.render:
            print "TURN", self.turn
            self.turn += 1
            for i in range(6):
                line = []
                for j in range(7):
                    piece = self.board[j][i]
                    if piece == 0:
                        line.append(" ")
                    else:
                        line.append("K" if piece == 1 else "Z")

                print '|'+'|'.join(line)+'|'

            print "\n"*10




if __name__=="__main__":
# testing testing 1 2 3
    test = C4Board(render=False)
    print test.step(1, 1)
    print test.step(1, 1)
    print test.step(1, 2)
    print test.step(2, 2)
    print test.step(2, 1)
    print test.step(2, 1)
    print test.step(2, 2)
    print test.step(3, 1)
    print test.step(3, 1)
    print test.step(3, 2)
    print test.step(3, 1)
    print test.step(4, 2)
    print test.step(4, 2)
    print test.step(4, 2)
    print test.step(4, 1)
    print test.step(0, 1)
