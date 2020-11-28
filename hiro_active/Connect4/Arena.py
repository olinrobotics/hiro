from tqdm import tqdm


class Arena:
    def __init__(self, player1, player2, game):
        """
        Input:
            player 1,2: two functions that takes board as input, return action
            game: Game object
            display: a function that takes board as input and prints it (e.g.
                     display in othello/OthelloGame). Is necessary for verbose
                     mode.
        see othello/OthelloPlayers.py for an example. See pit.py for pitting
        human players/other baselines with each other.
        """
        self.player1 = player1
        self.player2 = player2
        self.game = game

    def playGame(self, verbose=False):
        """
        Executes one episode of a game.
        Returns:
            either
                winner: player who won the game (1 if player1, -1 if player2)
            or
                draw result returned from the game that is neither 1, -1, nor 0.
        """
        current_player = [self.player2, None, self.player1]
        board = self.game.getInitBoard()
        iteration = 0
        while self.game.GetGameEnded(board, current_player) == 0:
            iteration += 1
            if verbose:
                print(f"Turn {iteration} \t Player {current_player}")
                self.game.display(board)

            canonical = self.game.getCanonicalForm(board, current_player)

            action = players[current_player + 1](canonical)
            valids = self.game.getValidMoves(canonical, 1)

            # TODO: How to ensure RL produces valid moves?
            if valids[action] == 0:
                print(f'Action {action} is not valid')
                assert valids[action] > 0

            board, current_player = self.game.getNextState(board, current_player, action)

        if verbose:
            print(f'Game over: Turn {iteration} | Result {str(self.game.getGameEnded(board, 1))}')
            self.game.display(board)
        return current_player * self.game.getGameEnded(board, current_player)

    def playGames(self, num, verbose=False):
        """
        Plays num games in which player1 starts num/2 games and player2 starts
        num/2 games.
        Returns:
            oneWon: games won by player1
            twoWon: games won by player2
            draws:  games won by nobody
        """
        num = int(num / 2)
        oneWins = 0
        twoWins = 0
        draws = 0

        for _ in tqdm(range(num), desc="Arena.playGames (1)"):
            gameResult = self.playGame(verbose=verbose)
            if gameResult == 1:
                oneWins += 1
            elif gameResult == -1:
                twoWins += 1
            else:
                draws += 1

        self.player1, self.player2 = self.player2, self.player1

        for _ in tqdm(range(num), desc="Arena.playGames (2)"):
            gameResult = self.playGame(verbose=verbose)
            if gameResult == -1:
                oneWins += 1
            elif gameResult == 1:
                twoWins += 1
            else:
                draws += 1

        return oneWins, twoWins, draws