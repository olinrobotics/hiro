from connect44 import Connect4

c = Connect4()
WON = False
WON1 = False
WON2 = False

while not WON:
    c.input(1)
    c.displayBoard()
    if c.checkHorizLines(1) or c.checkVertLines(1) or c.checkDiagR(1) \
            or c.checkDiagL(1):
        print('Player 1 Wins!')
        WON = True
    c.input(2)
    c.displayBoard()
    if c.checkHorizLines(2) or c.checkVertLines(2) or c.checkDiagR(2) \
            or c.checkDiagL(2):
        print('Player 2 Wins!')
        WON = True
