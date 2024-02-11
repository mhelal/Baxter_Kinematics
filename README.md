
# 1. INTRODUCTION
Hi everyone, my name is Quang Nam Phi. I graduated Computer Science (AI) at UH 2023. Hopefully this project assists you in achiving AI robotic and applications module.

ABOUT THIS PROJECT: the project integrate a computer vision system, a robotic manipulation system, and a game algorithm to enable the Baxter robot plays draughts with human. Based on The YOLOv8 object detection model. the computer vision system was trained to accurately detect the chess board and game piecess in real-time. The robot manipulation system employed forward and inverse kinematics to control Baxter's arm movements, allowing precise and efficient interactions with the board and pieces. The game algorithm utilized the minimax algorithm to enable strategic decision-making during gameplay.

# IMPORTANT

These are three seperate parts provided:

**1. Kinematics (Robotic Manipulation)**

**2. Checkers (draughts) with AI algorithm (MinMax) - Implementation & Visualization**

**3. Computer vision**

**I integrated (1) Kinematics and (2) a Checkers game that utilizes the MinMax algorithm in my project last year. Although I developed a (3) computer vision component, I did not incorporate it because Yolov8 necessitates Python version 3.11**

# 2. QUICK RUNNING 
IMPORTANT REQUIRE:

**- Running in E300 LAB at UH**

**- Baxter1 (it should be the first baxter robot when you entry the lab)**

**- Please check the ground to ensure that the table is positioned correctly according to the markings I've made**

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


# 3. Kinematics Forward and Inverse
WHY USING Kinematic algorthm?

Kinematic is the foundational block for designing and controlling robotics, allowing engineers and researchers to predict the motion of a robot's limbs and body without needing to consider the forces and moments causing the motion. This is crucial for planning and executing precise movements, especilly in applications requiting high levels of accuracy and efficiency. 

In robotics, kinematics is divided into two main categories: FORWARD KINEMATICS and INVERSE KINEMATICS

**Forward kinematic![FPK](https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/READ_ME.md#forward-kinematics)**

FORWARD: it is about determining the position and orientation of the robot's end effector given its joint parameters.

**Inverse kinematic![IPK](https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/READ_ME.md#inverse-kinematic)**

INVERSE KINEMATICS: it involves calculating the joint paramters that are needed to place the end effector at desired position and orientation.

# 4. Checkers (draughts) with AI algorithm (MinMax)

The minimax algorithm is the decision-making strategy for Baxter during gameplay. This algorithm allowed Baxter to search the game tree and select the optimal move based on the current board state

**Game tree generation**

<img src="https://github.com/Kun10x/Baxter_Kinematics/blob/main/Kinematics/kinematic_images/game_tree.png" width="600">

A game tree was generated for each move, representing all possible game states resulting from the player's moves. The game tree was constructed recursively, with alternating levels representing Baxter's and the human player's moves. The depth of the game tree was limited to a predefined value to balance the computational complexity and the quality of the decision-making.

**Minimax Algorithm and game play**

This is interesting videos to know how to create draughts game play and minimax algorithm.

**Part 1**

<a href="https://www.youtube.com/watch?v=mYbrH1Cl3nw&list=LL&index=50&t=2338s">
  <img src="https://github.com/Kun10x/Baxter_Kinematics/blob/main/images/tim_minmax.png" width="600">
</a>

**Part2**

<a href="https://www.youtube.com/watch?v=mYbrH1Cl3nw&t=11s">
  <img src="https://github.com/Kun10x/Baxter_Kinematics/blob/main/images/tim_part2.png" width="600">
</a>

**Whole checker_gameplay**

![whole code checker_gameplay](https://github.com/Kun10x/Baxter_Kinematics/tree/main/Checkers-AI-gameplay)

# 5. Computer vision (Update soon!!!!!!)



