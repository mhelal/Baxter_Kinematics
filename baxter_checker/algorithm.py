#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from copy import deepcopy
import pygame

RED = (255,0,0)
WHITE = (255,255,255)
BLACK = (0,0,0)

def minimax(position, depth, max_player, game):
    """Perform the minimax algorithm to find the best move."""
    if depth == 0 or position.winner() != None:  # If the maximum depth is reached or the game is over
        return position.evaluate(), position  # Evaluate the position and return it
    if max_player:  # If it's the max player's turn
        maxEval = float('-inf')  # Set the maximum evaluation to negative infinity
        best_move = None  # Set the best move to None
        for move in get_all_moves(position, WHITE, game):  # For all possible moves
            evaluation = minimax(move, depth-1, False, game)[0]  # Evaluate the move
            maxEval = max(maxEval, evaluation)  # Update the maximum evaluation
            if maxEval == evaluation:  # If the maximum evaluation is equal to the evaluation
                best_move = move  # Set the best move to the current move
        return maxEval, best_move  # Return the maximum evaluation and the best move
    else:  # If it's the min player's turn
        minEval = float('inf')  # Set the minimum evaluation to positive infinity
        best_move = None  # Set the best move to None
        for move in get_all_moves(position, BLACK, game):  # For all possible moves
            evaluation = minimax(move, depth-1, True, game)[0]  # Evaluate the move
            minEval = min(minEval, evaluation)  # Update the minimum evaluation
            if minEval == evaluation:  # If the minimum evaluation is equal to the evaluation
                best_move = move  # Set the best move to the current move
        return minEval, best_move  # Return the minimum evaluation and the best move

def simulate_move(piece,move,board, game,skip):
    """Simulate a move on the board."""
    board.move(piece, move[0], move[1])  # Move the piece on the board
    if skip:  # If a piece is skipped
        board.remove(skip)  # Remove the skipped piece from the board
    return board  # Return the new board

def get_all_moves(board,color, game):
    """Get all possible moves for a given color on the board."""
    moves=[]
    for piece in board.get_all_pieces(color):  # For all pieces of the given color
        valid_moves = board.get_valid_moves(piece)  # Get the valid moves for the piece
        for move, skip in valid_moves.items():  # For all valid moves
            temp_board = deepcopy(board)  # Create a temporary board
            temp_piece = temp_board.get_piece(piece.row, piece.col)  # Get the piece on the temporary board
            new_board = simulate_move(temp_piece, move, temp_board, game, skip)  # Simulate the move on the temporary board
            moves.append(new_board)  # Add the new board to the list of possible moves
    return moves  # Return the list of possible moves
