import numpy as np


class Connect4:

    def __init__(self):
        self.board = np.zeros((6, 7))

    def displayBoard(self):
        print(self.board)

    def input(self, marker):    # marker is 1 for X or 2 for O
        inputting = True
        while(inputting):
            col = input('Player '+str(marker)+" Pick a column 1-7: ")
            try:
                col = int(col) - 1
                inputting = False
            except ValueError:
                print('Please enter a number')
                continue

        for i in range(0, 6):
            if self.board[5-i][col] == 0:
                self.board[5-i][col] = marker
                break
            elif 5-i == 1:
                print('Column full')
            else:
                continue

    def canInput(self, col):
        col -= 1
        for i in range(0, 6):
            if self.board[5-i][col] == 0:
                return True
            elif 5-i == 1:
                return False
            else:
                continue

    def rawInput(self, marker, col):    # marker is 1 for X or 2 for O
        col -= 1
        for i in range(0, 6):
            if self.board[5-i][col] == 0:
                self.board[5-i][col] = marker
                break
            else:
                continue

    def clearBoard(self):
        self.board = np.zeros((7, 6))

# checks not working yet
    def checkVertLines(self, marker):
        # Checkin lines
        for i in range(0, 7):
            for j in range(3, 6):
                if self.board[j][i] == self.board[j-1][i] == \
                        self.board[j-2][i] == self.board[j-3][i] == marker:
                    return True
        return False

    def checkHorizLines(self, marker):
        for j in range(0, 6):
            for i in range(0, 4):
                if self.board[j][i] == self.board[j][i+1] == \
                        self.board[j][i+2] == self.board[j][i+3] == marker:
                    return True
        return False

    def checkDiagR(self, marker):
        for j in range(3, 6):
            for i in range(0, 4):
                if self.board[j][i] == self.board[j-1][i+1] == \
                        self.board[j-2][i+2] == self.board[j-3][i+3] == marker:
                    return True
        return False

    def checkDiagL(self, marker):
        checkedNums = []
        indices = []
        for j in range(3, 6):
            for i in range(3, 7):
                checkedNums.append(self.board[j, i])
                indices.append(str(j) + ',' + str(i))
                if self.board[j][i] == self.board[j-1][i-1] == \
                        self.board[j-2][i-2] == self.board[j-3][i-3] == marker:
                    return True
        return False
