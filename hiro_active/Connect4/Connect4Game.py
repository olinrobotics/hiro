import numpy as np


class GameWrapper:
    """
    Designed for use with GameTree generation.
    All functions require board as input
    """

    def canInput(self, col, board):
        """
        Checks whether the player can place a piece in all columns.
        Returns a list of length 7 (one index per column)
        If the column is valid, then list(i) = 1, if not, list(i)=0
        """
        col -= 1
        canInput = []
        for i in range(0, 6):
            if board[5-i][col] == 0:
                board[5-i][col] != 0
                canInput[i] = 1
            elif 5-i == 1:
                canInput[i] = 0
            else:
                continue
            return canInput

    def rawInput(self, marker, col, board):    # marker is 1 for X or 2 for O
        col -= 1
        for i in range(0, 6):
            if board[5-i][col] == 0:
                board[5-i][col] = marker
                return board
            else:
                continue

    def clearBoard(board):
        board = np.zeros((7, 6))
        return board

# checks not working yet
    def checkVertLines(self, marker, board):
        # Checkin lines
        for i in range(0, 7):
            for j in range(3, 6):
                if board[j][i] == board[j-1][i] == \
                        board[j-2][i] == board[j-3][i] == marker:
                    return True
        return False

    def checkHorizLines(self, marker, board):
        for j in range(0, 6):
            for i in range(0, 4):
                if board[j][i] == board[j][i+1] == \
                        board[j][i+2] == board[j][i+3] == marker:
                    return True
        return False

    def checkDiagR(self, marker, board):
        for j in range(3, 6):
            for i in range(0, 4):
                if board[j][i] == board[j-1][i+1] == \
                        board[j-2][i+2] == board[j-3][i+3] == marker:
                    return True
        return False

    def checkDiagL(self, marker, board):
        checkedNums = []
        indices = []
        for j in range(3, 6):
            for i in range(3, 7):
                checkedNums.append(board[j, i])
                indices.append(str(j) + ',' + str(i))
                if board[j][i] == board[j-1][i-1] == \
                        board[j-2][i-2] == board[j-3][i-3] == marker:
                    return True
        return False
