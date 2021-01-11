import numpy as np
from scipy.signal import convolve2d


class WinState:
    def __init__(self, terminated=False, winner=0):
        self.terminated = terminated
        self.winner = winner


class Connect4Board:
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
        self.current_player = 1  # player 1 or -1
        self.win_state = WinState()

    def check_winning(self):
        # Boolean mask current player with 1
        copy = self.np_pieces == self.current_player

        # Check if current player is winning
        for i in self.detection:
            if (convolve2d(copy == 1, i, mode="valid") == 4).any():
                self.win_state = WinState(terminated=True, winner=self.current_player)
                return self.win_state

        # Check draw
        draw = self.get_valid_moves().sum() == 0
        self.win_state = WinState(terminated=draw, winner=0)
        return self.win_state

    def make_move(self, col):
        if self.get_valid_moves().sum() == 0:
            return self.check_winning()  # Draw

        for i in range(self.rows - 1, -1, -1):
            if self.np_pieces[i][col] == 0:
                self.np_pieces[i][col] = self.current_player
                self.check_winning()
                if not self.win_state.terminated:
                    self.switch_player()
                return self.win_state

        raise ValueError("Invalid move!")

    def get_valid_moves(self):
        return self.np_pieces[0] == 0

    def switch_player(self):
        self.current_player = -self.current_player

    def clone(self, np_pieces=None):
        if np_pieces is None:
            np_pieces = self.np_pieces

        return Connect4Board(self.rows, self.columns, np_pieces=np.copy(np_pieces))

    def game(self):
        while not self.win_state.terminated:
            while True:
                try:
                    col = int(input("choose column (1->7): "))
                    # col = int(input(f'Team {self.current_player} choose column (1->{self.columns}): '))
                    if 1 <= col <= self.columns:
                        break
                except ValueError:
                    pass
                print('Invalid input. Try again!')

            self.make_move(col - 1)
            self.display()

        if self.win_state.winner is 0:
            print('Draw!')
        else:
            print("Player ", self.win_state.winner, 'won!')

    def display(self):
        print(self.np_pieces)

if __name__ == "__main__":
    b = Connect4Board(np_pieces=np.array([[ 0,  0,  0,  0,  0,  0,  0],
                                         [ -1,  1,  -1,  1,  -1,  1, -1],
                                         [ 1, -1,  1, -1,  1, -1,  1],
                                         [ 1, -1,  1, -1,  1, -1,  1],
                                         [-1,  1, -1,  1, -1,  1, -1],
                                         [ 1, -1,  1, -1,  1, -1,  1]]))
    b.display()
    b.game()
