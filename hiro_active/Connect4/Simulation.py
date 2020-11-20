
import numpy as np
from scipy.signal import convolve2d


def winning_move(board):
    hor = np.array([[1, 1, 1, 1]])
    ver = np.transpose(hor)
    diag1 = np.eye(4, dtype=np.uint8)
    diag2 = np.fliplr(diag1)
    detection = [hor, ver, diag1, diag2]

    for i in detection:
        if (convolve2d(board == 1, i, mode="valid") == 4).any():
            return True
    return False


def check_player(current, player):
    new = np.zeros((6,7))
    for i in range(len(current)):
        for j in range(len(current[i])):
            if current[i][j] == player:
                new[i][j] = 1
            elif current[i][j] != player or current[i][j] == None:
                new[i][j] = 0
    return new


def init_board():
    columns = 7
    rows = 6

    return np.full((rows, columns),None)


def make_move(bd, player, col):
    invalid = True
    for i in range(len(bd)-1, -1 , -1):
        if bd[i][col] == None:
            bd[i][col] = player
            invalid = False
            break
        elif i == 0:
            print("invalid move try again")
    return bd, invalid


def game(board):

    while True:
        invalid = True
        while invalid is True:
            col = int(input('Team X choose column (1->7): ')) - 1
            board, invalid = make_move(board, 'X', col)
        play = check_player(board, 'X')
        done = winning_move(play)
        print(play)
        if done:
            break

        invalid = True
        while invalid is True:
            col = int(input('Team O choose column (1->7): ')) - 1
            board, invalid = make_move(board, 'O', col)
        play = check_player(board, 'O')
        done = winning_move(play)
        print(board)
        print(play)
        if done:
            break


if __name__ == "__main__":
    b = init_board()
    print(b)
    game(b)