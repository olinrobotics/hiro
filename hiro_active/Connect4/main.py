import logging
from Coach import Coach
from NN_Wrapper import NeuralNet as nn
from Game import Game

log = logging.getLogger(__name__)

def main(args, load_model = False):
    log.info('Loading %s...', Game.__name__)
    g = Game(6)

    log.info('Loading %s...', nn.__name__)
    nnet = nn(g)

    if load_model:
        log.info('Loading checkpoint "%s/%s"...', args['load_folder_file'])
        nnet.load_checkpoint(args['load_folder_file'][0], args['load_folder_file'][1])
    else:
        log.warning('Not loading a checkpoint!')

    log.info('Loading the Coach...')
    c = Coach(g, nnet, args)

    if load_model:
        log.info("Loading 'trainExamples' from file...")
        c.loadTrainExamples()

    log.info('Starting the learning process ')
    c.train()


if __name__ == "__main__":
    args = {'load_folder_file' : ['/dev/models/8x100x50','best.pth.tar'],
            'numIters': 1000,
            'numEps': 100,              # Number of complete self-play games to simulate during a new iteration.
            'tempThreshold': 15,        #
            'updateThreshold': 0.6,     # During arena playoff, new neural net will be accepted if threshold or more of games are won.
            'maxlenOfQueue': 200000,    # Number of game examples to train the neural networks.
            'numMCTSSims': 25,          # Number of games moves for MCTS to simulate.
            'arenaCompare': 40,         # Number of games to play during arena play to determine if new net will be accepted.
            'cpuct': 1,}
    # print(args['numIters'])
    main(args)
