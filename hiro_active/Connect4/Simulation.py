import numpy as np
from scipy.signal import convolve2d

class WinState():
    def __init__(self, terminated=False, winner=0):
        self.terminated = terminated
        self.winner = winner

class Connect4Board():
    def __init__(self, rows=6, columns=7, np_pieces=None):
        self.rows = rows
        self.columns = columns
        if np_pieces is not None:
            self.np_pieces = np_pieces
        else:
            self.np_pieces = np.full((rows, columns), 0)
        hor = np.array([[1, 1, 1, 1]])
        ver = np.transpose(hor)
        diag1 = np.eye(4, dtype=np.uint8)
        diag2 = np.fliplr(diag1)
        self.detection = [hor, ver, diag1, diag2]
        self.current_player = 1 # player 1 or -1
        self.win_state = WinState()

    def check_winning(self):
        # Boolean mask current player with 1
        copy = np.zeros((6, 7))
        for i in range(self.rows):
            for j in range(self.columns):
                if self.np_pieces[i][j] == self.current_player:
                    copy[i][j] = 1

        # Check if current player is winning
        for i in self.detection:
            if (convolve2d(copy == 1, i, mode="valid") == 4).any():
                self.win_state = WinState(True, self.current_player)
                return

        # Check draw
        draw = True
        for j in range(self.columns):
            if self.np_pieces[0][j] == 0:
                draw = False
                break

        if draw:
            self.win_state = WinState(True, 0)
        else:
            self.win_state = WinState(False, 0)

    def make_move(self, col):
        for i in range(self.rows - 1, -1, -1):
            if self.np_pieces[i][col] == 0:
                self.np_pieces[i][col] = self.current_player
                self.check_winning()
                if not self.win_state.terminated:
                    self.switch_player()
                return self.win_state

        raise ValueError("invalid move try again")

    def get_valid_moves(self):
        return self.np_pieces[0] == 0

    def switch_player(self):
        self.current_player = -self.current_player

    def clone(self, np_pieces=None):
        if np_pieces is None:
            return Connect4Board(self.rows, self.columns, np_pieces=np.copy(self.np_pieces))
        else:
            return Connect4Board(self.rows, self.columns, np_pieces=np.copy(board))

    def game(self):
        while True:
            invalid = True
            while invalid is True:
                col = int(input(f'Team {self.current_player} choose column (1->7): ')) - 1
                invalid = self.make_move(col)

            print(self.np_pieces)
            if self.win_state.terminated:
                print(f'Player {self.current_player} won!')
                break


if __name__ == "__main__":
    b = Connect4Board()
    print(b)
    b.game()
