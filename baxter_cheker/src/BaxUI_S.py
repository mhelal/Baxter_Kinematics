#python
import cv2
import numpy as np

import os
import sys
import argparse
import cv

def showList(the_array):
	selected = 0
	runit = True
	while runit == True:
		img = cv2.imread('bg.png')
		width = 350
		height = 65
		gap = 35

		startLX=100
		startL2X=560
		startLY=175
		offset=height+gap
		font = cv2.FONT_HERSHEY_SIMPLEX
		fontoffsetX = 42
		fontoffsetY = 20
		purple = (50,13,30)
		blue = (255,168,0)
		

		for i in range(len(the_array)):
			if i == selected:
				color = blue
			else:
				color = purple
			if i < 4:
				cv2.rectangle(img,(startLX ,startLY + (i*offset)),(startLX+width,startLY+height + (i*offset)),color,-1)
				cv2.putText(img,the_array[i][0],(startLX+fontoffsetY,startLY + (i*offset)+fontoffsetX), font, 1,(255,255,255),2)
			elif i < 8:
				j = i - 4
				cv2.rectangle(img,(startL2X ,startLY + (j*offset)),(startL2X+width,startLY+height + (j*offset)),color,-1)
				cv2.putText(img,the_array[i][0],(startL2X+fontoffsetY,startLY + (j*offset)+fontoffsetX), font, 1,(255,255,255),2)
		cv2.imshow('MENU',img)
		#cv2.imwrite('temp.png',img)
		#send_image()
		s = cv.WaitKey()
	
		if s == 65288:
			runit = False

		if s == 10:
			if the_array[selected][1] != "":
				globals()[the_array[selected][1]]()
			else:
				print "nothing defined"

		if s == 65362:
			if selected > 0:
				selected = selected - 1

		if s == 65364:
			if selected < len(the_array)-1:
				selected = selected + 1








def showOption(the_array):
	selected = 0
	runit = True
	while runit == True:
		img = cv2.imread('bg.png')
		width = 350
		height = 65
		gap = 35

		startLX=100
		startL2X=560
		startLY=175
		offset=height+gap
		font = cv2.FONT_HERSHEY_SIMPLEX
		fontoffsetX = 42
		fontoffsetY = 20
		purple = (50,13,30)
		blue = (255,168,0)
		green = (0,255,0)
		

		for i in range(len(the_array)-1):
			if i == selected:
				color = green
			else:
				color = purple

			cv2.putText(img,the_array[0][0],(startLX+fontoffsetY,startLY+fontoffsetY), font, 1,(0,0,0),2)

			cv2.putText(img,"currently set to:",(startLX+fontoffsetY,startLY+fontoffsetY+50), font, 0.6,(0,0,0),2)
			cv2.putText(img,globals()[the_array[0][1]],(startLX+fontoffsetY+20,startLY+fontoffsetY+70), font, 0.6,(0,0,0),2)

			cv2.rectangle(img,(startL2X ,startLY + (i*offset)),(startL2X+width,startLY+height + (i*offset)),color,-1)
			cv2.putText(img,the_array[i+1][0],(startL2X+fontoffsetY,startLY + (i*offset)+fontoffsetX), font, 1,(255,255,255),2)
		cv2.imshow('MENU',img)
		s = cv.WaitKey()
		if s == 65288:
			runit = False

		if s == 10:
			if the_array[selected][1] != "":
				globals()[the_array[0][1]] = the_array[selected+1][1]
			else:
				print "nothing defined"

		if s == 65362:
			if selected > 0:
				selected = selected - 1

		if s == 65364:
			if selected < len(the_array)-1:
				selected = selected + 1
