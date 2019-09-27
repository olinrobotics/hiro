import chess
import chess.uci

board = chess.Board()

print(board)
print(board.fen())

engine = chess.uci.popen_engine("stockfish")
engine.uci()
f = open("rent_board.txt", 'w')
data = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w - - 0 1"
f.write(data)
f.close

board = chess.Board(fen=data)


while not board.is_game_over():
    f = open("rent_board.txt", 'r')
    board = chess.Board(fen=f.read())
    f.close
    k = board.turn
    print(k)
    engine.position(board)
    move = engine.go(movetime=1000)
    print(move.bestmove)
    print(type(move.bestmove))
    print(board.is_capture(move.bestmove))
    board.push(move.bestmove)
    print(board)
    print(board.fen())
    f = open("rent_board.txt", 'w')
    data = board.fen()
    f.write(data)
    f.close

print(board.result())