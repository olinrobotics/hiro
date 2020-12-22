from connect44 import Connect4

c = Connect4()


class GameWrapper:
    """
    Wrapper class designed to allow for Connect 4 class to work with gameTree
    """

    def getValidMoves(self, board):
        """
        Checks whether the player can place a piece in all columns.
        Returns a list of length 7 (one index per column)
        If the column is valid, then list(i) = 1, if not, list(i)=0
        """
        inputList = []
        for i in range(0, 6):
            if c.canInput(i):
                inputList[i] = 1
            else:
                inputList[i] = 0

    def getGameEnded():
        if c.checkDiagL(1) or c.checkDiagR(1) or c.checkHorizLines(1) \
                or c.checkVertLines(1):
            return 1
        elif c.checkDiagL(2) or c.checkDiagR(2) or c.checkHorizLines(2) \
                or c.checkVertLines(2):
            return -1
        else:
            return 0
