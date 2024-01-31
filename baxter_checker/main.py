#!/usr/bin/env python2
# -*- coding: utf-8 -*-


import rospy
import baxter_interface
import pygame
from constants import WIDTH, HEIGHT, SQUARE_SIZE, RED, WHITE  # Importing constant values from the checkers module
from game import Game  # Importing the Game class from the checkers module
from board import Board  # Importing the Board class from the checkers module
from baxter_do import BaxterDo, baxter_node  # Importing the BaxterDo class from the checkers module
from algorithm import minimax  # Importing the minimax function from the minimax module

FPS = 60  # Setting the number of frames per second for Pygame

WIN = pygame.display.set_mode((WIDTH, HEIGHT))  # Creating a Pygame display with the specified dimensions
pygame.display.set_caption('Checkers')  # Setting the window caption to "Checkers"

def get_row_col_from_mouse(pos):
    """A helper function that takes in the position of the mouse and returns the corresponding row and column on the board."""
    x, y = pos
    row = y // SQUARE_SIZE
    col = x // SQUARE_SIZE
    return row, col

def main():
    """The main function that runs the game."""
    run = True
    clock = pygame.time.Clock()  # Creating a Pygame clock object
    game = Game(WIN)  # Creating a new Game object with the Pygame display
    baxter_n = baxter_node()
    board = Board()  # Creating a new Board object
    baxter = BaxterDo()  # Creating a new BaxterDo object
    
    while run:
        clock.tick(FPS)  # Limiting the frame rate to 60 FPS
        if game.turn == WHITE:  # If it is the white player's turn
            value, new_board = minimax(game.get_board(), 4, WHITE, game)  # Use the minimax algorithm to get the best move for the AI
             # Moving the piece on the physical board using Baxter
            if new_board.get_remove_pos() != []:  # If there are pieces that need to be removed
                print('piece pos removed:', new_board.get_remove_pos())  # Printing the positions of the removed pieces
                for remove in new_board.get_remove_pos():
                    print('position remove:', remove[0], remove[1])  # Printing the positions of the removed pieces
                    baxter.baxter_remove(baxter.cal_board_pos(remove[0], remove[1]))  # Removing the piece on the physical board using Baxter
                new_board.remove_pos = []  # Resetting the list of removed pieces
	    print('piece position:', new_board.get_old_new_pos())  # Printing the old and new positions of the moved piece
            pos_start, pos_end = new_board.get_old_new_pos()  # Getting the old and new positions of the moved piece
            print('start:', baxter.cal_board_pos(pos_start[0], pos_start[1]), 'end:', baxter.cal_board_pos(pos_end[0], pos_end[1]))  # Printing the start and end positions in Baxter notation
            baxter.baxter_move(baxter.cal_board_pos(pos_start[0], pos_start[1]), baxter.cal_board_pos(pos_end[0], pos_end[1])) 
            game.ai_move(new_board)  # Making the move on the virtual board

        if game.winner() != None:  # If there is a winner
            print(game.winner())  # Print the winner
            run = False  # Stop the game loop
        for event in pygame.event.get():  # For each event in the Pygame event
            if event.type == pygame.QUIT:  # If the user clicks the close button
                run = False  # Stop the game loop
            
            if event.type == pygame.MOUSEBUTTONDOWN:  # If the user clicks the mouse
                pos = pygame.mouse.get_pos()  # Get the position of the mouse
                a, b = pos 
                row, col = get_row_col_from_mouse(pos)  # Get the corresponding row and column on the board
                print('mouse clicked:', -col+3, row-4)  # Print the position in human-readable notation
                game.select(row, col)  # Select the piece on the virtual board
		

        game.update()  # Update the Pygame display
    
    pygame.quit()  # Quit Pygame
main()

    


