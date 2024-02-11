
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

<img src="https://github.com/Kun10x/Baxter_Kinematics/blob/main/images/Screenshot%202023-05-01%20003802.png" width="600">

<img src="https://github.com/Kun10x/Baxter_Kinematics/blob/main/images/Screenshot%202023-05-07%20232910.png" width="600">

# 6. Create Word Space (WS)

**Initialise and Configure your ROS workspace:**
Assuming your working ROS directory is called my_ros_ws (you can actually call it anything you
want), type the following shell commands to set up and configure your workspace so it is integrated
into the ROS system:

_`cd my_ros_ws`_  (i.e. to the ROS working directory you have just created)

_`mkdir src`_ (create a src subdirectory my_ros_ws/src. Note: If you plan to replace or modify an existing ROS package, use: wstool init src instead )

_`cd src`_ (change the working directory to the src sub directory) 

_`catkin_init_workspace`_ (Tell the ROS system to look in my_ros_ws/src for executable files)

_`cd ..`_ (change up to the workspace directory, my_ros_ws)

_`catkin_make`_ (Create various configuration files, also builds any C, C++ progs etc)

If these commands are successful, you will find that two new directories have appeared in the
my_ros_ws directory, `my_ros_ws/build` and `my_ros_ws/devel` . The final step to setup
your working directory is to type the command:

`source ./devel/setup.sh` (if the CWD is `my_ros_ws/`)

If you want to use your workspace for several programs, it is recommended to create a seperate 'package' directory for each program. Do this by changing to the my_ros_ws/src directory:

`cd src`

`catkin_create_pkg my_test_package std_msgs rospy baxter_interface`

(Note: You may need to add other ROS dependencies and possibly the ROS distribution version for
your package if it depends upon other ROS packages). Then:

`cd ..` (Change back up to: my_ros_ws directory)

`catkin_make` (to make sure that your new package is integrated into ROS)

From now on, after mounting your remote storage directory (as shown above) to initialise your
workspace into the ROS system, you simply need to:

`cd my_ros_ws` (or if using a USB drive: `cd /media/NNN-NNNN/my_ros_ws`)

`source ./devel/setup.sh`

You are now ready to create your first Baxter program!

**First Baxter Python Program**

To test your workspace, create the following Python program (rosHello.py) using your favourite text editor (emacs, Geany, Gedit etc.), and save it in the `my_ros_ws/src/test_package/src` directory of your new ROS workspace:

```python
#!/usr/bin/env python2
# -*- coding: utf-8 -*-
'''
rosHello.py
A test program for ROS to make sure that your workspace has been
intialised properly
M.L.Walters, Jan 1015, V1
Note, must be run from the ros_ws directory, using:
rosrun <packageName> <programName>
E.g.
rosrun my_test_package rosHello.py
'''
# ROS can only use Python2, but to use Python 3 syntax, include:
from __future__ import print_function, division
input = raw_input
# Following lines will only work on E300 PCs with ROS and Baxter
# SDK installed
import rospy
import baxter_interface
def main():
 print("hello world")
 a = input("Press ENTER to continue")
 return 0
if __name__ == '__main__':
 main()
```

When you save your program, you should make it directly executable by typing (from your
`my_ros_ws` directory):

`chmod +x src/my_test_package/src/<programName>.py`

`catkin_make`  (to rebuild all executables and paths, config files etc. Usually not necessary for Python programs, but esssential for C, C++ etc.)

To run your program, then type:

`rosrun my_test_package <programName>`

Where `<programName>` is the name of your Python program file, including the .py extension.

Notes: It is important that you type this program in yourself. The indentation is important both to aid
code readability and also Python uses indents to delimit program blocks (see Lecture notes). The
lines beginning with # or ''' are comments. Always put the program title, author, version and date as
comments at the beginning of any program you write. This will help you to keep track of different
versions and modifications of you program. You can also include any other notes as necessary to aid
readers of your program (including yourself after coming back to it after several months!). Also
include the line “#!/usr/bin/python2” as the first line of your program so that it can be run
automatically under Linux. When you run the program, you may well find that there is a syntax or
other error. Try to find where the error is yourself by reading through the program step by step, and
understanding what the program is doing. 



# 7. References

