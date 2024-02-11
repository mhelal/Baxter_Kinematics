import numpy as np
import scipy.optimize

def forward_kinematics(theta):
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

def cost_function(theta, desired_position, desired_orientation):
    position, orientation = forward_kinematics(theta)

    # Cost function for position error
    position_error = position - desired_position

    # Cost function for orientation error
    orientation_error = np.sum((orientation - desired_orientation)**2)

    # Total cost
    cost = np.sum(position_error**2) + orientation_error
    print(cost)
    return cost


def inverse_kinematics(desired_position, desired_orientation):
    # Initial guess for joint angles
    theta_initial = np.zeros(7)

    # Bounds for joint angles
    bounds = ((-np.pi*(141/180), np.pi*(51/180)), (-np.pi*(123/180), np.pi*(60/180)), (-np.pi*(173/180), np.pi*(173/180)), (-np.pi*(3/180), np.pi*(150/180)), (-np.pi*(175/180), np.pi*(175/180)), (-np.pi*(90/180), np.pi*(120/180)), (-np.pi*(175/180), np.pi*(175/180)))

    # Solve for joint angles using gradient descent
    result = scipy.optimize.minimize(cost_function, theta_initial, args=(desired_position, desired_orientation), method='dogbox', bounds=bounds, tol=1e-6)
    return result.x

if __name__ == '__main__':
    # theta_initial_s = [np.pi/18,np.pi/9,np.pi/6,(2*np.pi)/9,(5*np.pi)/18,np.pi/3,(np.pi*7)/18]
    theta_initial_s = [0,(np.pi*(-31))/180,0,(43*np.pi)/180,0,(np.pi*72)/180,0]
    desired_position, desired_orientation = forward_kinematics(theta_initial_s)

    # Solve for joint angles
    theta = inverse_kinematics(desired_position, desired_orientation)

    # Print results
    print("Joint angles:")
    print(theta)
    print('test:')
    print(theta_initial_s)
    print("Cost value:", cost_function(theta, desired_position, desired_orientation))
    print('old orientation:')
    print(desired_orientation)
    print('old position:')
    print(desired_position)
    print('new position')
    print(forward_kinematics(theta))


