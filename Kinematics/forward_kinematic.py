import numpy as np

# Link lengths
L0 = 270.35
L1 = 69.00
L2 = 364.35
L3 = 69.00
L4 = 374.29
L5 = 10.00
L6 = 368.30

# Joint angles (all set to zero)
theta1 = np.pi/18
theta2 = np.pi/9
theta3 = np.pi/6
theta4 = (2*np.pi)/9
theta5 = (5*np.pi)/18
theta6 = np.pi/3
theta7 = (np.pi*7)/18

# Homogeneous transformation matrices for each joint
T01 = np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                [np.sin(theta1), np.cos(theta1), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

T12 = np.array([[-np.sin(theta2), -np.cos(theta2), 0, L1],
                [0, 0, 1, 0],
                [-np.cos(theta2), np.sin(theta2), 0, 0],
                [0, 0, 0, 1]])

T23 = np.array([[np.cos(theta3), -np.sin(theta3), 0, 0],
                [0, 0, -1, -L2],
                [np.sin(theta3), np.cos(theta3), 0, 0],
                [0, 0, 0, 1]])

T34 = np.array([[np.cos(theta4), -np.sin(theta4), 0, L3],
                [0, 0, 1, 0],
                [-np.sin(theta4), -np.cos(theta4), 0, 0],
                [0, 0, 0, 1]])

T45 = np.array([[np.cos(theta5), -np.sin(theta5), 0, 0],
                [0, 0, -1, -L4],
                [np.sin(theta5), np.cos(theta5), 0, 0],
                [0, 0, 0, 1]])

T56 = np.array([[np.cos(theta6), -np.sin(theta6), 0, L5],
                [0, 0, 1, 0],
                [-np.sin(theta6), -np.cos(theta6), 0, 0],
                [0, 0, 0, 1]])

T67 = np.array([[np.cos(theta7), -np.sin(theta7), 0, 0],
                [0, 0, -1, 0],
                [np.sin(theta7), np.cos(theta7), 0, 0],
                [0, 0, 0, 1]])

# Total homogeneous transformation matrix
T07 = np.matmul(T01, np.matmul(T12, np.matmul(T23, np.matmul(T34, np.matmul(T45, np.matmul(T56, T67))))))

# Extract position and orientation of end-effector
position = T07[0:3, 3]
orientation = T07[0:3, 0:3]

print("Position of end-effector:")
print(position)
print("Orientation of end-effector:")
print(orientation)
print('matrix:')
print(T07)
