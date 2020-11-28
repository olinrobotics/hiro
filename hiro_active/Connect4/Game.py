from Simulation import Connect4Board


class Game:
    def __init__(self, rows=6, columns=7):
        self._base_board = Connect4Board(rows=rows, columns=columns)

    def getInitBoard(self):
        """
        Returns:
            startBoard: a representation of the board (ideally this is the form
                        that will be the input to your neural network)
        """
        return self._base_board.np_pieces

    def getBoardSize(self):
        """
        Returns:
            (x,y): a tuple of board dimensions
        """
        return self._base_board.rows, self._base_board.columns

    def getActionSize(self):
        """
        Returns:
            actionSize: number of all possible actions
        """
        return self._base_board.columns

    def getNextState(self, board, player, action):
        """
        Input:
            board: current board
            player: current player (1 or -1)
            action: action taken by current player
        Returns:
            nextBoard: board after applying action
            nextPlayer: player who plays in the next turn (should be -player)
        """
        cloned = self._base_board.clone(np_pieces=board)
        cloned.make_move(action)
        return cloned.np_pieces, -player

    def getValidMoves(self, board, player):
        """
        Input:
            board: current board
            player: current player
        Returns:
            validMoves: a binary vector of length self.getActionSize(), 1 for
                        moves that are valid from the current board and player,
                        0 for invalid moves
        """
        return self._base_board.clone(np_pieces=board).get_valid_moves()

    def getGameEnded(self, board, player):
        """
        Input:
            board: current board
            player: current player (1 or -1)
        Returns:
            r: 0 if game has not ended. 1 if player won, -1 if player lost,
               small non-zero value for draw.

        """
        b = self._base_board.clone(np_pieces=board)
        win_state = b.win_state
        if win_state.terminated:
            if win_state.winner is None:
                return 1e-4
            elif win_state.winner == player:
                return 1
            elif win_state.winner == -player:
                return -1
            else:
                raise ValueError('Unexpected win state found: ', win_state)
        else:
            return 0

    def getCanonicalForm(self, board, player):
        """
        Input:
            board: current board
            player: current player (1 or -1)
        Returns:
            canonicalBoard: returns canonical form of board. The canonical form
                            should be independent of player. For e.g. in chess,
                            the canonical form can be chosen to be from the pov
                            of white. When the player is white, we can return
                            board as is. When the player is black, we can invert
                            the colors and return the board.
        """
        return board * player

    def getSymmetries(self, board, pi):
        """
        Input:
            board: current board
            pi: policy vector of size self.getActionSize()
        Returns:
            symmForms: a list of [(board,pi)] where each tuple is a symmetrical
                       form of the board and the corresponding pi vector. This
                       is used when training the neural network from examples.
        """
        # Board is left/right symmetric in Connect 4
        return [(board, pi), (board[:, ::-1], pi[::-1])]

    def stringRepresentation(self, board):
        """
        Input:
            board: current board
        Returns:
            boardString: a quick conversion of board to a string format.
                         Required by MCTS for hashing.
        """
        return str(board)

    def display(self, board):
        print(" -----------------------")
        print(' '.join(map(str, range(len(board[0])))))
        print(board)
        print(" -----------------------")


if __name__ == '__main__':
    game = Game()
    board = game._base_board.np_pieces
    print(game.getInitBoard())
    print(game.getBoardSize())
    print(game.getActionSize())
    print(game.getNextState(board, 1, 1))
    print(game.getValidMoves(board, 1))
    print(game.getGameEnded(board, 1))
    print(game.getCanonicalForm(board, 1))
    print(game.getSymmetries(board, board))
    print(game.stringRepresentation(board))
    print(game.display(board))