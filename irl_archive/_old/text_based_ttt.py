#!/usr/bin/env python
import copy
import numpy as np
import random
import time
import math
import operator
import itertools

class Game:
	def __init__(self):
		#A blank space is represented by 0, an "O" is 1, and "X" is 10, we start with blank board
		self.board =   [1, 10, 10,
						10, 1, 1,
						1, 0, 10]

		self.corners = [0,2,6,8] #indices of the corner locations
		self.sides = [1,3,5,7]
		self.middle = 4
		self.difficulty = 10



	def is_winner(self, board):
		"""
		Computes whether board will win
		"""

		board_sums = [0, 0, 0, 0, 0, 0, 0, 0] #These are the 8 different lines.
		board_sums[0] = board[0] + board[1] + board[2] #Top Horizontal Line
		board_sums[1] = board[3] + board[4] + board[5] #Middle Horizontal Line
		board_sums[2] = board[6] + board[7] + board[8] #Bottom Horizontal Line

		board_sums[3] = board[0] + board[3] + board[6] #Left Vertical Line
		board_sums[4] = board[1] + board[4] + board[7] #Middle Vertical Line
		board_sums[5] = board[2] + board[5] + board[8] #Right Vertical Line

		board_sums[6] = board[0] + board[4] + board[8] #LR Diagonal Line
		board_sums[7] = board[2] + board[4] + board[6] #RL Diagonal Line

		#There are 4 cases - NoWin, EdWin, Player Win and Tie, Tie is accounted for
		#in a piece of code further down.
		#print board_sums
		for b_sum in board_sums:
			if b_sum == 30:
				print "WINNER 1"
				return 1
			elif b_sum == 3:
				print "WINNER 2"
				return 2
		return False

	def is_free(self, board, index):
	 	#Function is to check whether a space is occupied or not.
	 	if board[index] == 0:
	 		return True
	 	else:
	 		return False

	##CURRENTLY UNUSED TODO: Figure out why
	# def is_board_full(self):
	#  	for i in range(9):
	# 		if self.is_free(self.board, i):
	#  			return False
	#  	return True

	def next_move(self):
		#Returns index of next move
		#Checks if Edwin can win on this move

		legal_moves = []
		corner_moves = []
		side_moves = []
		difficulty_rand = random.randint(0,10)

		#if randomly generated number is > difficulty, make a random move
		if self.difficulty < difficulty_rand:
			board_copy = copy.deepcopy(self.board)
			for i in range(9):
				if self.is_free(board_copy, i):
					legal_moves.append(i)
			return random.choice(legal_moves)

		else:
			#Checks if Edwin wins this turn
			for i in range(9): #don't use len(self.board), it's a numpy array
				board_copy = copy.deepcopy(self.board)
				if self.is_free(board_copy, i):
					board_copy[i] = 10
					if self.is_winner(board_copy) == 1:
						return i

			#Checks if player can win the next turn
			for i in range(9):
				board_copy = copy.deepcopy(self.board)
				if self.is_free(board_copy, i):
					board_copy[i] = 1
					if self.is_winner(board_copy) == 2:
						return i

			#Otherwise, prioritizes grabbing corners.
			for i in range(4):
				board_copy = copy.deepcopy(self.board)
				if self.is_free(board_copy, self.corners[i]):
					corner_moves.append(self.corners[i])
			if len(corner_moves) != 0:
				return random.choice(corner_moves)

			#Otherwise, get the middle.
			if self.is_free(board_copy, self.middle):
				board_copy = copy.deepcopy(self.board)
				return self.middle

			#Otherwise, get a side.
			for i in range(4):
				board_copy = copy.deepcopy(self.board)
				if self.is_free(board_copy, self.sides[i]):
					side_moves.append(self.sides[i])
			if len(side_moves) != 0:
				return random.choice(side_moves)

			return "TIE"

	def manual_field_scan(self):
		print "USER TURN, CURRENT BOARD:"

		print self.board[:3]
 		print self.board[3:6]
 		print self.board[6:9]
		#TODO: Update board with current values
		for i in range(9):
			if self.is_free(self.board, i):
				val_in = int(raw_input("Value at "+str(i)+": "))
				self.board[i] = val_in
				if val_in == 1:
					return


	def edwin_move(self, index):
		#edwin moves to desired location and draws
		print "EDWIN MOVING TO: ", index
	 	self.board[index] = 10

 		# print self.board[:3]
 		# print self.board[3:6]
 		# print self.board[6:9]

	def run(self):
		running = True
		# turn = random.randint(0,1)
		turn = 0
		print "running"

		if turn == 0:
			print "Your turn first!"

		while running:
			if turn == 0: #Player turn.
				self.manual_field_scan()
				if self.is_winner(self.board) == 2:
					print "USER WINS"
					running = False
				turn = 1
				continue
			elif turn == 1: #Edwin's turn
				next_move_ind = self.next_move()
				if next_move_ind == "TIE":
					print "TIE"
					running = False
					continue
				self.edwin_move(next_move_ind)
				turn = 0
				if self.is_winner(self.board) == 1:
					print "EDWIN WINS"
					running = False


if __name__ == '__main__':
	gm = Game()
	gm.run()
