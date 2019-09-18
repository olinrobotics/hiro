import chess
import chess.uci
import numpy as np
import copy

temporary_board = [[0]*8 for i in range(8)]

board = chess.Board(fen="rn1qkb2/2p1pp2/1pb2npp/p2pPP1r/1P6/3P2P1/P1P4P/RNBQKBNR w KQq - 0 1")
#print(board.unicode())
fen = board.board_fen()
print(fen)
print(board)
row = 0 
column = 0
for i in range(len(fen)):
    if fen[i] == "/":
        row += 1
        column = 0
    elif fen[i].isdigit() == True:
        for k in range(int(fen[i])):
            temporary_board[row][column] = 0
            column +=1
    elif fen[i].isupper()==True:
        temporary_board[row][column] = 1
        column += 1
    elif fen[i].isupper()==False:
        temporary_board[row][column] = 2
        column += 1

previous_board = [[0]*8 for i in range(8)]

row = 7 
column = 0
for i in range(8):
    for j in range(8):
        previous_board[i][j] = temporary_board[row][column]
        row -= 1
    column += 1
    row = 7

print(previous_board)

current_board = copy.deepcopy(previous_board)

current_board[0][3] = current_board[0][1]
current_board[0][1] = 0

print(previous_board)
print(current_board)

row = 0 
column = 0

loc = "temporary"

for i in range(8):
    for j in range(8):
        if previous_board[i][j] <> 0 and current_board[i][j] == 0:
            loc = str(i) + str(j)

print(loc)

for i in range(8):
    for j in range(8):
        if previous_board[i][j] <> current_board[i][j] and current_board[i][j] <> 0:
            loc = loc + str(i) + str(j)


print(loc)

change_to_cord = {'0':'a', '1':'b', '2':'c', '3':'d', '4':'e', '5':'f', '6':'g', '7':'h'}

loc = change_to_cord[loc[0]] + str(int(loc[1])+1) + change_to_cord[loc[2]] + str(int(loc[3])+1)

move = chess.Move.from_uci(loc)

print(move)

print(board.is_capture(move))

board.push(move)

print(board)

    


                

            


