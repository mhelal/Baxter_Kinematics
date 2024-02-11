# Kinematics

# Forward Kinematics

Generally, the Forward Kinematics (FPK) is given by the joint values (theta values) to calculate the pose includes position and orientation of end-effector (Baxter's gripper is end-effector)

I USE BAXTER'S LEFT ARM IN THIS PROGRAM
<img src="https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/kinematic_images/left_hand.png" width="900">

**Given:** $$(\theta_0,\theta_1,\theta_2,\theta_3,\theta_4,\theta_5,\theta_6)$$

**Calculate**: $$\left[ _{7}^{0}T \right] and \left[ _{G}^{W}T \right]$$

**Where** : {G} is the left-arm end-effector(gripper) frame and {W} is the World fixed reference frame on the floor.


**First**
I import 'numpy' library for this program

```python
import numpy as np
```
_`pip install numpy`_ for window or _`pip3 install numpy`_ for macos

**Then (1)**
Set the lengtths following by this table

<img src="https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/kinematic_images/length_hand.png" width="600">

```python
# Link lengths
L0 = 270.35
L1 = 69.00
L2 = 364.35
L3 = 69.00
L4 = 374.29
L5 = 10.00
L6 = 368.30
```
**Then (2)** I set theta0-6 (10,20,30,40,50,60,70 degree) for example (you can set any theta values within the joint limits that you want to calculate)

<img src="https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/kinematic_images/joint_limits.png" width="900">

```python
# Joint angles 
theta0 = np.pi/18
theta1 = np.pi/9
theta2 = np.pi/6
theta3 = (2*np.pi)/9
theta4 = (5*np.pi)/18
theta5 = np.pi/3
theta6 = (np.pi*7)/18
```

**Then (3)** Obtain the seven neighboring homogeneous transformation matrics as a function of the joint angles for the 7-dof left arm. 

<img src="https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/kinematic_images/Screenshot%202024-02-11%20153336.png" width="900">

Better, you can copy and paste the code below :))):

```python
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
```

**Then (4)** Total all the homogeneous T matrix 

<img src="https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/kinematic_images/total_T.png" width="900">

```python
# Total homogeneous transformation matrix
T07 = np.matmul(T01, np.matmul(T12, np.matmul(T23, np.matmul(T34, np.matmul(T45, np.matmul(T56, T67))))))
```
**Finally (5)** Extract position and orientation of end-effector

<img src="https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/kinematic_images/overall_FPK.png" width="900">

Where the first 3 cols and rows (r11 to r33) are orientation of end-effector and (x7,y7,z7) are position of end-effector 


```python
# Extract position and orientation of end-effector
position = T07[0:3, 3]
orientation = T07[0:3, 0:3]

print("Position of end-effector:")
print(position)
print("Orientation of end-effector:")
print(orientation)
print('matrix:')
print(T07)
```

**Whole code for forward kinematics**

![whole code FPK](https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/forward_kinematic.py)

# Inverse Kinematic

Generally, the Inverse Kinematic (IPK) is opposite with the Forward Kinematic (FPK), that is given the pose includes position and orientation of end-effector to calculate the joints values (theta $\theta$).

**Given:** the constant DH Parameters and the end-effector pose

<img src="https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/kinematic_images/matrix_end_effector.png" width="400">

**Calculate:** the joint angles $\theta$ below $$(\theta_0,\theta_1,\theta_2,\theta_3,\theta_4,\theta_5,\theta_6)$$

**First** 

I import 'numpy' and 'scipy.optimize' library for this program

```python
import numpy as np
import scipy.optimize
```

**Then (6)** forward_kinematics

I still use FPK as a function and return position and orientation in this program 

```python
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
```

**Then (7)** cost_function and inverse_kinematics

I create two functions, cost_function and inverse_kinematics. These functions work together to solve the problem of inverse kinematics. The first is the cost_function, which calculates the error cost between the end effector's current (position, orientation) and desired (position, orientation). The cost is the weighted sum of the position and direction errors. The current position and direction are calculated using the forward_kinematics function with the given matching angles. 

The cost_function is given 5 values, there are joint angles (theta $\theta$), desired position, desired orientation, position_weight (I choose 0.1) and orientation_weight (I choose 200). This function returns cost values.

```python
def cost_function(theta, desired_position, desired_orientation, position_weight, orientation_weight):
    position, orientation = forward_kinematics(theta)

    # Cost function for position error
    position_error = position - desired_position

    # Cost function for orientation error
    orientation_error = np.sum((orientation - desired_orientation)**2)

    # Total cost
    cost = position_weight * np.sum(position_error**2) + orientation_weight * orientation_error
    print(cost)
    return cost
```

The inverse_kinematics function uses an optimization algorithm to find matching angles that minimize the cost funtion. It takes as input the desired position and orientation, weights for position and orientation. Optimized matching angles are returned as result.x.

```python
def inverse_kinematics(desired_position, desired_orientation):
    # Initial guess for joint angles
    theta_initial = np.zeros(7)

    # Solve for joint angles using gradient descent
    result = scipy.optimize.least_squares(cost_function, theta_initial, args=(desired_position, desired_orientation, 0.1, 200), method='dogbox')
    return result.x
```

**Finally** main function

The given is the position and orientation that desire to go there, so I have to use FPK to calculate in this example program. Howver, I dont use that when apply for Baxter
```python
if __name__ == '__main__':
    # example joint angles (0,-31,0,43,0,72,0)
    theta_initial_s = [0,(np.pi*(-31))/180,0,(43*np.pi)/180,0,(np.pi*72)/180,0]
    # use forward_kinematics function to get desired_position and desired_orientation 
    desired_position, desired_orientation = forward_kinematics(theta_initial_s) # Given position and orientation

    # Solve for joint angles
    theta = inverse_kinematics(desired_position, desired_orientation)

    # Print results
    print("Joint angles:")
    print(theta)
    print('test:')
    print(theta_initial_s)
    print("Cost value:", cost_function(theta, desired_position, desired_orientation, 0.1, 200))
    print('old orientation:')
    print(desired_orientation)
    print('old position:')
    print(desired_position)
    print('new position')
    print(forward_kinematics(theta))
```

**Whole code for inverse kinematics** 

![whole code IPK](https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/inverse_kinematic.py)



