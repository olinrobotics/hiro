#!/usr/bin/env python


import numpy as np
import random
import time

class C4Board(object):

    def __init__(self, render):
        self.action_space = [0,1,2,3,4,5,6]
        self.n_actions = len(self.action_space)
        self.turn = 0
        self.render = render
        self.init_board()


    def init_board(self):
        self.board = []
        for i in range(7):
            column = [0]*6
            self.board.append(column)
        self.column_stack = np.array([5,5,5,5,5,5,5])


    def reset(self):
        self.init_board()
        self.turn = 0
        self.visualize()
        return str(self.board)


    def make_move(self, column, player):
        move = (column, self.column_stack[column])
        if self.column_stack[column] == -1:
            return str(self.board), move
        self.board[column][self.column_stack[column]] = player
        self.column_stack[column] -= 1

        if np.sum(self.column_stack) == -7:
            return str(self.board), False
        return str(self.board), move


    def connect_4(self, move):
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
        while column1 < 4 and row1 < 5:
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
        while column2 < 4 and row2 > 0:
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
        s_, move = self.make_move(action, player)

        if move == False:
            reward1 = 0
            reward2 = 0
            done = True
            return s_, reward1, reward2, done
        if  self.connect_4(move) and player == 1:
            reward1 = 1
            reward2 = -1
            done = True
        elif self.connect_4(move) and player == 2:
            reward1 = -1
            reward2 = 1
            done = True
        else:
            reward1 = reward2 = 0
            done = False

        self.visualize()


        return s_, reward1, reward2, done


    # def next_turn(self):
    #     for col in range(7):
    #         for row in range(6):
    #             if self.board[col][row] == 1:
    #                 self.board[col][row] = 2
    #             elif self.board[col][row] == 2:
    #                 self.board[col][row] = 1
    #
    #     return self.board


    def visualize(self):
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
    test = C4Board()
    print test.step(0, 1)
    print test.step(1, 2)
    print test.step(1, 1)
    print test.step(2, 2)
    print test.step(2, 1)
    print test.step(2, 1)
    print test.step(3, 1)
    print test.step(3, 2)
    print test.step(3, 2)
    print test.step(3, 1)
