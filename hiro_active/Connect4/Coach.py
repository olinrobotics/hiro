from collections import deque
from tqdm import tqdm
from random import shuffle
from pickle import Pickler, Unpickler
import os
import sys
from Arena import Arena

class Coach:
    def __init__(self, game, nnet, args):
        self.game = game
        self.nnet = nnet
        self.pnet = self.nnet.__class__(self.game)  # init the competitor network with the same class as nnet
        self.args = args
        self.mcts = MTCS(self.game, self.nnet, self.args)
        self.trainExamplesHistory = []
        self.skipFirstSelfPlay = False

    def train(self):
        """
        Performs numIters iterations with numEps episodes of self-play in each
        iteration. After every iteration, it retrains neural network with
        examples in trainExamples (which has a maximum length of maxlenofQueue).
        It then pits the new neural network against the old one and accepts it
        only if it wins >= updateThreshold fraction of games.
        """
        for i in range(1, self.args.numIters + 1):
            print(f'Starting iteration #{i} ...')

            # Self play to generate initial examples. Will skip self play if already loaded examples
            if not self.skipFirstSelfPlay or i > 1:
                iterationTrainExamples = deque([], maxlen=self.args.maxlenOfQueue)

                for _ in tqdm(range(self.args.numEps), desc="Self Play"):
                    self.mcts = MCTS(self.game, self.nnet, self.args)
                    iterationTrainExamples += self.executeEpisode()

                self.trainExamplesHistory.append(iterationTrainExamples)

            # TODO: why do we pop the first one
            # If we have too many train examples than needed, remove the oldest one
            if len(self.trainExamplesHistory) > self.args.numItersForTrainExamplesHistory:
                print(f'Removing the oldest entry in trainExamples. len(trainExamplesHistory = {len(self.trainExamplesHistory)}')
                self.trainExamplesHistory.pop(0)

            # backup history to a file
            self.saveTrainExamples(i - 1)

            # shuffle examples before training
            trainExamples = self.shuffle(self.trainExamplesHistory)

            # training new network, keeping a copy of the old one
            self.nnet.saveTrainExamples(folder=self.args.checkpoint, filename='temp.pth.tar')
            self.pnet.loadTrainExample(folder=self.args.checkpoint, filename='temp.pth.tar')
            pmcts = MCTS(self.game, self.pnet, self.args)

            self.nnet.train(trainExamples)
            nmcts = MTCS(self.game, self.nnet, self.args)

            print('PITTING AGAINST PREVIOUS VERSION')
            arena = Arena(lambda x: np.argmax(pmcts.getActionProb(x, temp=0)),
                          lambda x: np.argmax(nmcts.getActionProb(x, temp=0)), self.game)
            pwins, nwins, draws = arena.playGames(self.args.arenaCompare)

            print(f'NEW/PREV WINS: {nwins}/{pwins} | DRAWS: {draws}')
            if pwins + nwins == 0 or float(nwins) / (pwins + nwins) < self.args.updateThreshold:
                print('REJECTING NEW MODEL')
                self.nnet.load_checkpoint(folder=self.args.checkpoint, filename='temp.pth.tar')
            else:
                print('ACCEPTING NEW MODEL')
                self.nnet.save_checkpoint(folder=self.args.checkpoint, filename=self.getCheckpointFile(i))
                self.nnet.save_checkpoint(folder=self.args.checkpoint, filename='best.pth.tar')

    def executeEpisode(self):
        """
        This function executes one episode of self-play, starting with player 1.
        As the game is played, each turn is added as a training example to
        trainExamples. The game is played till the game ends. After the game
        ends, the outcome of the game is used to assign values to each example
        in trainExamples.
        It uses a temp=1 if episodeStep < tempThreshold, and thereafter
        uses temp=0.
        Returns:
            trainExamples: a list of examples of the form (canonicalBoard, pi, v)
                           pi is the MCTS informed policy vector, v is +1 if
                           the player eventually won the game, else -1.
        """
        trainExamples = []
        board = self.game.getInitBoard()
        player = 1
        episodeStep = 0
        while True:
            episodeStep += 1
            canonicalBoard = self.game.getCanonicalForm(board, player)
            temp = int(episodeStep < self.args.tempThreshold)

            pi = self.mcts.getActionProb(canonicalBoard, temp=temp)
            sym = self.game.getSymmetries(canonicalBoard, pi)
            for b, p in sym:
                trainExamples.append([b, player, p, None])

            action = np.random.choice(len(pi), p=pi)  # pick an action based on policy pi
            board, player = self.game.getNextState(board, player, action)

            result = self.game.getGameEnded(board, player)
            if result != 0:
                return [(x[0], x[2], result * ((-1) ** (x[1] != player))) for x in trainExamples]

    def shuffle(self, trainExamplesHistory):
        trainExamples = []
        for e in self.trainExamplesHistory:
            trainExamples.extend(e)  # TODO: shouldn't this be append? -- extend: formatting
                                     # if x = [a,b,c], x.append([a,b]) -> [a,b,c,[a,b]], x.extend([a,b]) -> [a,b,c,a,b]
        shuffle(trainExamples)
        return trainExamples

    def getCheckpointFile(self, iteration):
        return 'checkpoint_' + str(iteration) + '.pth.tar'

    def saveTrainExamples(self, iteration):
        folder = self.args.checkpoint
        if not os.path.exists(folder):
            os.makedirs(folder)
        filename = os.path.join(folder, self.getCheckpointFile(iteration) + ".examples")
        with open(filename, "wb+") as f:
            Pickler(f).dump(self.trainExamplesHistory)
        f.closed

    def loadTrainExample(self):
        modelFile = os.path.join(self.args.load_folder_file[0], self.args.load_folder_file[1])
        examplesFile = modelFile + ".examples"
        if not os.path.isfile(examplesFile):
            print(f'File "{examplesFile}" with trainExamples not found!')
            r = input("Continue? [y|n]")
            if r != "y":
                sys.exit()
        else:
            log.info("File with trainExamples found. Loading it...")
            with open(examplesFile, "rb") as f:
                self.trainExamplesHistory = Unpickler(f).load()
            log.info('Loading done!')

            # examples based on the model were already collected (loaded)
            self.skipFirstSelfPlay = True
