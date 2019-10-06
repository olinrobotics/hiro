import chess
import chess.uci
import numpy as np
import copy
"""
Program to compare the last positino of chess pieces on data chessboard with the new position data from OpenCV.
Had to rotate the data chessboard 90 degrees to compare.
At end, recognizes move made by human and returns a data in form of UCI engine movement ex)a1a4
"""
def get_move(previous_fen, current_position):
    #create 8*8 array. Code to rotate 90 degress
    temporary_board = [[0]*8 for i in range(8)]

    board = chess.Board(fen=previous_fen)
    fen = board.board_fen()

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
    print("previous")
    print(previous_board)

    #get the current position, possibly from OpenCV
    current_board = current_position
    print("current")
    print(current_board)

    row = 0 
    column = 0

    loc = "temporary"

    #compare position
    for i in range(8):
        for j in range(8):
            if previous_board[i][j] <> 0 and current_board[i][j] == 0:
                loc = str(i) + str(j)

    for i in range(8):
        for j in range(8):
            if previous_board[i][j] <> current_board[i][j] and current_board[i][j] <> 0:
                loc = loc + str(i) + str(j)

    #loc is supposed to be in form of 0124, with start row column to end row column
    if len(loc) > 4:
        raise ValueError('The length is over 4')
        return

    #changes 0124 form to a1c4 form
    change_to_cord = {'0':'a', '1':'b', '2':'c', '3':'d', '4':'e', '5':'f', '6':'g', '7':'h'}

    loc = change_to_cord[loc[0]] + str(int(loc[1])+1) + change_to_cord[loc[2]] + str(int(loc[3])+1)

    move = chess.Move.from_uci(loc)

    return move

        


                

            


