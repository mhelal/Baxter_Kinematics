
# INTRODUCTION
Hi everyone, my name is Quang Nam Phi. I graduated Computer Science (AI) at UH 2023. Hopefully this project assists you in achiving hight marks AI robotic and applications module.

ABOUT THIS PROJECT: the project integrate a computer vision system, a robotic manipulation system, and a game algorithm to enable the Baxter robot plays draughts with human. Based on The YOLOv8 object detection model. the computer vision system was trained to accurately detect the chess board and game piecess in real-time. The robot manipulation system employed forward and inverse kinematics to control Baxter's arm movements, allowing precise and efficient interactions with the board and pieces. The game algorithm utilized the minimax algorithm to enable strategic decision-making during gameplay.

# QUICK RUNNING 
IMPORTANT REQUIRE:
- Running in E300 LAB at UH
- Baxter1 (it should be the first baxter robot when you entry the lab)
 <img src="https://github.com/Kun10x/Baxter/blob/main/images/495f5bde30849adac395.jpg" width="500">
Start PC and the Baxter robot by pressing power button on the body behind the left arm.
When the green head stripe light is stable, open terminal window on PC and type:

_`cd rosdemo`_ or _`cd ros_ws`_

Then connect to Baxter 1:

_`baxter1.sh`_

Before running programs, you must enable the robot and untuck its arms. 

_`rosrun baxter_tools enable_robot.py -e`_

_`rosrun baxter_tools tuck_arms.py -u`_  (if one of arms doesnt work, "ctrl + c" and run the commad again)

Running my program by following commands:

_`cd ..`_ 

_`cd quangnam`_

_`source ./devel/setup.sh`_

_`rosrun baxter_checker main.py`_











# Baxter plays checker
Baxter robot plays checker game with human
# Kinematic Inverse and forward
WHY USING Kinematic algorthm?

Kinematic is the foundational block for designing and controlling robotics, allowing engineers and researchers to predict the motion of a robot's limbs and body without needing to consider the forces and moments causing the motion. This is crucial for planning and executing precise movements, especilly in applications requiting high levels of accuracy and efficiency. 

In robotics, kinematics is divided into two main categories: FORWARD KINEMATICS and INVERSE KINEMATICS

FORWARD: it is about determining the position and orientation of the robot's end effector given its joint parameters.

INVERSE KINEMATICS: it involves calculating the joint paramters that are needed to place the end effector at desired position and orientation.

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
**Next (2)** I set theta0-6 (10,20,30,40,50,60,70 degree) for example (you can set any theta values within the joint limits that you want to calculate)

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

**After that (3)** Obtain the seven neighboring homogeneous transformation matrics as a function of the joint angles for the 7-dof left arm. 

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

**Whole code**

![whole code](https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/forward_kinematic.py)



<img src="https://github.com/Kun10x/Baxter/blob/main/images/Baxters-joints-Rethink-Robotics-also-provides-Joint-Velocity-Wobbler-Example-where.png" width="500">



