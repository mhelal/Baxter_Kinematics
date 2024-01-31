#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import baxter_interface
import scipy.optimize
import numpy as np
from baxter_interface import CHECK_VERSION
import time




#print("Robot left_arm : " + str(left_arm))
#print("Robot joint_angles : " + str(joint_angles))

#print("Updating Robot lefy arm angle ... ")
#joint_angles['left_w0'] += 0.1
#joint_angles['left_w1'] += 0.7
#print(joint_angles['left_w2'])
#left_arm.move_to_joint_positions(joint_angles)

#print('Now updated')

#print(theta_baxter)
def baxter_node():
	global joint_angles, left_arm, remove_place
	print("Initializing node... ")
	rospy.init_node("my_Tests")
	print("Getting robot state... ")
	rs = baxter_interface.RobotEnable(CHECK_VERSION)
	init_state = rs.state().enabled
	print("Robot state : " + str(init_state))
	
	left_arm = baxter_interface.Limb('left')
	joint_angles = left_arm.joint_angles()
	if baxter_interface.Gripper('left').calibrated() == False:
		baxter_interface.Gripper('left').calibrate()
	remove_place= BaxterDo().cal_board_pos(5,2)
class BaxterDo:
	def __init__(self):
		

		
		self.x_center=167.68533098
		self.y_center=-549.89416256
		self.z_center=245.37919932-350
		self.position_weight = 0.1
		self.orientation_weight = 200.0
		#backup position
		self.cam_pos=[-0.5495486172599495, -0.978679742670894, -1.1991894809294221, 1.4346555318698333, 0.5645049299418322, 1.535514768673299, -0.501228222441559-0.45]

	def get_join_angles(self,theta):
		global joint_angles
		joint_angles['left_s0'] = theta[0]
    		joint_angles['left_s1'] = theta[1]
    		joint_angles['left_e0'] = theta[2]
    		joint_angles['left_e1'] = theta[3]
    		joint_angles['left_w0'] = theta[4]
    		joint_angles['left_w1'] = theta[5]
    		joint_angles['left_w2'] = theta[6]

	def cal_board_pos(self,t0,t1):
		x = self.x_center - 200*t0*0.173
		y = self.y_center - 191.1764706*t0*0.173
		z = self.z_center
		new_pos = self.cal_board_pos_x_y_z(x,y,z,t1)
		return new_pos
	def cal_board_pos_x_y_z(self,x,y,z,t1):
	    x_pos = x + 200*t1*0.173
	    y_pos = y - 191.1764706*t1*0.173
	    z_pos = z 
	    new_pos=[x_pos,y_pos,z_pos]
	    return new_pos
	def forward_kinematics(self,theta):
	    # Link lengths
	    L0 = 270.35
	    L1 = 69.00
	    L2 = 364.35
	    L3 = 69.00
	    L4 = 374.29
	    L5 = 10.00
	    L6 = 368.30

	    # Homogeneous transformation matrices for each joint
	    T01 = np.array([[np.cos(theta[0]), -np.sin(theta[0]), 0, 0],
	                    [np.sin(theta[0]), np.cos(theta[0]), 0, 0],
	                    [0, 0, 1, 0],
	                    [0, 0, 0, 1]])

	    T12 = np.array([[-np.sin(theta[1]), -np.cos(theta[1]), 0, L1],
	                    [0, 0, 1, 0],
	                    [-np.cos(theta[1]), np.sin(theta[1]), 0, 0],
	                    [0, 0, 0, 1]])

	    T23 = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0, 0],
	                    [0, 0, -1, -L2],
	                    [np.sin(theta[2]), np.cos(theta[2]), 0, 0],
	                    [0, 0, 0, 1]])

	    T34 = np.array([[np.cos(theta[3]), -np.sin(theta[3]), 0, L3],
	                    [0, 0, 1, 0],
	                    [-np.sin(theta[3]), -np.cos(theta[3]), 0, 0],
	                    [0, 0, 0, 1]])

	    T45 = np.array([[np.cos(theta[4]), -np.sin(theta[4]), 0, 0],
	                    [0, 0, -1, -L4],
	                    [np.sin(theta[4]), np.cos(theta[4]), 0, 0],
	                    [0, 0, 0, 1]])

	    T56 = np.array([[np.cos(theta[5]), -np.sin(theta[5]), 0, L5],
	                    [0, 0, 1, 0],
	                    [-np.sin(theta[5]), -np.cos(theta[5]), 0, 0],
	                    [0, 0, 0, 1]])

	    T67 = np.array([[np.cos(theta[6]), -np.sin(theta[6]), 0, 0],
	                    [0, 0, -1, 0],
	                    [np.sin(theta[6]), np.cos(theta[6]), 0, 0],
	                    [0, 0, 0, 1]])

	    # Total homogeneous transformation matrix
	    T07 = np.matmul(T01, np.matmul(T12, np.matmul(T23, np.matmul(T34, np.matmul(T45, np.matmul(T56, T67))))))

	   
	    # Extract position and orientation of end-effector
	    position = T07[0:3, 3]
	    orientation = T07[0:3, 0:3]

	    return position, orientation
	def cost_function(self,theta, desired_position, desired_orientation, position_weight, orientation_weight):
	    position, orientation = self.forward_kinematics(theta)

	    # Cost function for position error
	    position_error = position - desired_position

	    # Cost function for orientation error
	    orientation_error = np.sum((orientation - desired_orientation)**2)

	    # Total cost
	    cost = position_weight * np.sum(position_error**2) + orientation_weight * orientation_error
	    # print(cost)
	    return cost
	def inverse_kinematics(self,desired_position, desired_orientation, position_weight, orientation_weight,theta_initial):
	    # Initial guess for joint angles
	    #theta_initial = theta_baxter
	    
	    # Solve for joint angles using gradient descent
	    result = scipy.optimize.least_squares(self.cost_function, theta_initial, args=(desired_position, desired_orientation, position_weight, orientation_weight), method='dogbox')
	    return result.x
	def baxter_move(self, start_pos, end_pos):
		desired_position, desired_orientation = self.forward_kinematics(self.cam_pos)
		theta = self.inverse_kinematics(start_pos, desired_orientation,self.position_weight, self.orientation_weight, self.cam_pos)
		print('theta start:', theta)
		self.get_join_angles(theta)
		left_arm.move_to_joint_positions(joint_angles)
		#
		start_pos[2]=245.37919932-475
		theta = self.inverse_kinematics(start_pos, desired_orientation,self.position_weight, self.orientation_weight, theta)
		self.get_join_angles(theta)
		left_arm.move_to_joint_positions(joint_angles)
		baxter_interface.Gripper('left').close()
		rospy.sleep(.3)
		#
		start_pos[2]=245.37919932-350
		theta = self.inverse_kinematics(start_pos, desired_orientation,self.position_weight, self.orientation_weight, theta)
		self.get_join_angles(theta)
		left_arm.move_to_joint_positions(joint_angles)
		#
		theta = self.inverse_kinematics(end_pos, desired_orientation,self.position_weight, self.orientation_weight, self.cam_pos)
		print('theta end:', theta)
		self.get_join_angles(theta)
		left_arm.move_to_joint_positions(joint_angles)
		#
		end_pos[2]=245.37919932-475
		theta = self.inverse_kinematics(end_pos, desired_orientation,self.position_weight, self.orientation_weight, theta)
		self.get_join_angles(theta)
		left_arm.move_to_joint_positions(joint_angles)
		baxter_interface.Gripper('left').open()
		rospy.sleep(.3)
		#
		end_pos[2]=245.37919932-350
		theta = self.inverse_kinematics(end_pos, desired_orientation,self.position_weight, self.orientation_weight, theta)
		self.get_join_angles(theta)
		left_arm.move_to_joint_positions(joint_angles)
		#
		
		self.get_join_angles(self.cam_pos)
		left_arm.move_to_joint_positions(joint_angles)
	def baxter_remove(self, remove_pos):
		global remove_place
		desired_position, desired_orientation = self.forward_kinematics(self.cam_pos)
		theta = self.inverse_kinematics(remove_pos, desired_orientation,self.position_weight, self.orientation_weight, self.cam_pos)
		print('theta remove:', theta)
		self.get_join_angles(theta)
		left_arm.move_to_joint_positions(joint_angles)
		#
		remove_pos[2]=245.37919932-475
		theta = self.inverse_kinematics(remove_pos, desired_orientation,self.position_weight, self.orientation_weight, theta)
		self.get_join_angles(theta)
		left_arm.move_to_joint_positions(joint_angles)
		baxter_interface.Gripper('left').close()
		rospy.sleep(.3)
		#
		remove_pos[2]=245.37919932-350
		theta = self.inverse_kinematics(remove_pos, desired_orientation,self.position_weight, self.orientation_weight, theta)
		self.get_join_angles(theta)
		left_arm.move_to_joint_positions(joint_angles)
		##############################################################################
		
		
		theta = self.inverse_kinematics(remove_place, desired_orientation,self.position_weight, self.orientation_weight, self.cam_pos)
		print('theta end:', theta)
		self.get_join_angles(theta)
		left_arm.move_to_joint_positions(joint_angles)
		#
		remove_place[2]=245.37919932-475
		theta = self.inverse_kinematics(remove_place, desired_orientation,self.position_weight, self.orientation_weight, theta)
		self.get_join_angles(theta)
		left_arm.move_to_joint_positions(joint_angles)
		baxter_interface.Gripper('left').open()
		rospy.sleep(.3)
		#
		remove_place[2]=245.37919932-350
		theta = self.inverse_kinematics(remove_place, desired_orientation,self.position_weight, self.orientation_weight, theta)
		self.get_join_angles(theta)
		left_arm.move_to_joint_positions(joint_angles)
		#
		remove_place[0]-=30
		remove_place[1]+=28.5
		self.get_join_angles(self.cam_pos)
		left_arm.move_to_joint_positions(joint_angles)