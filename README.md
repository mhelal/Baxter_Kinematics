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
