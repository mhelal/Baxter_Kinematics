#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import pygame
from constants import RED, WHITE, BLUE, SQUARE_SIZE, BLACK
from board import Board
from baxter_do import BaxterDo

class Game:
    def __init__(self, win):
        self._init()  # Call the _init function to initialize the game
        self.win = win  # Set the Pygame window

    def update(self):
        """Draw the board, the valid moves, and update the Pygame display."""
        self.board.draw(self.win)
        self.draw_valid_moves(self.valid_moves)
        pygame.display.update()

    def _init(self):
        """Initialize the game."""
        self.selected = None  # Set the selected piece to None
        self.board = Board()  # Create a new board
        self.baxter = BaxterDo()  # Create a new BaxterDo object
        self.turn = BLACK  # Set the turn to black
        self.valid_moves = {}  # Set the valid moves to an empty dictionary
        self.start_pos = None  # Set the start position to None
        self.end_pos = None  # Set the end position to None

    def winner(self):
        """Get the winner of the game."""
        return self.board.winner()

    def reset(self):
        """Reset the game."""
        self._init()

    def select(self, row, col):
        """Select a piece on the board."""
        if self.selected:  # If a piece is already selected
            result = self._move(row, col)  # Try to move the selected piece to the new position
            if not result:  # If the move is not valid
                self.selected = None  # Deselect the piece
                self.select(row, col)  # Select the new piece instead
        piece = self.board.get_piece(row, col)  # Get the piece at the selected position
        if piece != 0 and piece.color == self.turn:  # If the piece is not empty and it's the player's turn
            self.start_pos = -col + 3, row - 4  # Store the start position in human-readable notation
            self.selected = piece  # Select the piece
            self.valid_moves = self.board.get_valid_moves(piece)  # Get the valid moves for the selected piece
            return True  # Return True if a piece is selected
        else:
            self.valid_moves = {}  # Clear the valid moves if no piece is selected
        return False  # Return False if no piece is selected

    def _move(self, row, col):
        """Move the selected piece to the new position."""
        piece = self.board.get_piece(row, col)  # Get the piece at the new position
        if self.selected and piece == 0 and (row, col) in self.valid_moves:  # If a piece is selected and the move is valid
            self.board.move(self.selected, row, col)  # Move the piece on the board
            self.end_pos = -col + 3, row - 4  # Store the end position in human-readable notation
            x_start, y_start = self.start_pos
            x_end, y_end = self.end_pos
            print('start:', self.baxter.cal_board_pos(x_start, y_start), 'end:', self.baxter.cal_board_pos(x_end, y_end))  # Print the start and end positions in Baxter notation
            self.baxter.baxter_move(self.baxter.cal_board_pos(x_start, y_start), self.baxter.cal_board_pos(x_end,y_end))  # Move the piece on the physical board using Baxter
            skipped = self.valid_moves[(row, col)]  # Get the piece that was skipped
            if skipped:
                self.board.remove(skipped)  # Remove the skipped piece from the board
                print('remove piece:', skipped)  # Print the position of the removed piece
            self.change_turn()  # Change the turn
        else:
            return False

        return True

    def draw_valid_moves(self, moves):
        """Draw the valid moves on the board."""
        for move in moves:
            row, col = move
            pygame.draw.circle(self.win, BLUE, (col * SQUARE_SIZE + SQUARE_SIZE//2, row * SQUARE_SIZE + SQUARE_SIZE//2), 15)

    def change_turn(self):
        """Change the turn."""
        self.valid_moves = {}  # Clear the valid moves
        if self.turn == BLACK:  # If it's black's turn
            self.turn = WHITE  # Set it to white's turn
        else:
            self.turn = BLACK  # Otherwise, set it to black's turn

    def get_board(self):
        """Get the current board."""
        return self.board

    def ai_move(self, board):
        """Make a move for the AI."""
        self.board = board  # Set the board to the new board
        self.change_turn()  # Change the turn to the player's turn
