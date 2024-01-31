#! /usr/bin/python
"""
Baxter draughts program
Baxtor actions module
M L Walters, July 2014

Problems:
    Misses jumps in: get_move_visual()?
    Minor issue: Can miss out intermediate move if close to navpos 
    

"""

# Standard libs 
import math
pi = math.pi
import pickle
import numpy

from time import sleep
from os import system
import os
# Standard ROS libs
import rospy
import roslib
import std_msgs.msg
import tf
import geometry_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

ppath = os.path.dirname(os.path.realpath(__file__))
# Baxter libs
import baxter_interface
#from baxter_core_msgs.msg import navigaterState
from moveit_commander import conversions
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    )
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
    )
import BaxUI

# global variables
listener = None # Global access to ROS TF node
home_joint_angles = None # Left arm home/camera position
board_dict = None
#occupancy_dict ={}
zOffset = 0
zApproach = 0.1
takeq = 0

current_image = None

# Start of function defs

def move_piece (startpos, endpos, limb="left"):
    """
    Will move a game piece from location on the game board. The positions can be
    given by either:
        A string of the form "<C>n", where <C> is a letter from A to H, n is a number 
        from 1 to 8
    The linb string may be either "left" or "right" and will use the right or left arm 
    respectively?? 
    """
    if (startpos in board_dict) and (endpos in board_dict):
        print "Moving piece from ", startpos, " to ", endpos, "."
        say ("Moving piece from "+ startpos+ " to "+ endpos+".")
        move_arm_xyz("left", board_dict[startpos]+[zApproach])
        move_arm_xyz("left", board_dict[startpos]+[zOffset])
        # Gripper not work!!!!!!
        baxter_interface.Gripper('left').close()
        rospy.sleep(.3)
        move_arm_xyz("left", board_dict[startpos]+[zApproach])
        # Extra move here to avoid swiping board
        move_arm_xyz("left", board_dict[endpos]+ [zApproach])
        move_arm_xyz("left", board_dict[endpos]+[zOffset])
        baxter_interface.Gripper('left').open()
        rospy.sleep(.3)
        move_arm_xyz("left", board_dict[endpos]+[zApproach])
    else:
        print """"positions should be a string with a board position
        in the form Cn, or some other learned place name."""
    return

def moveTo(placeName=None, approach = 0, limb = "left"):
    if placeName in board_dict:
        move_arm_xyz(limb, board_dict[placeName] + [approach])


def take_piece(takepos, colour="black", limb="left"):
    """
    Will take a piece at takepos, and put in the take area relevant to the player colour,
    "Black" or "White". Parameter limb may be either "left" or "right"
    """
    global takeq
    print "Taking piece for ", colour, " player."
    move_arm_xyz("left", board_dict[takepos]+[zApproach])
    move_arm_xyz("left", board_dict[takepos]+[zOffset])
    # Close Gripper
    baxter_interface.Gripper('left').close()
    rospy.sleep(.3)   
    move_arm_xyz("left", board_dict[takepos]+[zApproach])
   # Move to next free slot in take area
    move_arm_xyz("left", board_dict["take"+str(takeq)]+[zApproach], [pi, 0, pi * 3.0/4.0])
    move_arm_xyz("left", board_dict["take"+str(takeq)]+[zOffset+0.002], [pi, 0, pi * 3.0/4.0])
    baxter_interface.Gripper('left').open()
    rospy.sleep(.2)  
    move_arm_xyz("left", board_dict["take"+str(takeq)]+[zApproach], [pi, 0, pi * 3.0/4.0])
    takeq= takeq + 1    
    return 

def move_home(limb = "left"):
    """
    Moves baxters "left" or "right" arms to Home position, so that camera in "left" or "right"
    arm can view the board from above. The free arm moves to a position where the human player
    can use the arm mounted controls
    """
    global board_dict, home_joint_angles
    print "Moving to Home position"
    
    if home_joint_angles == None:
        # Avoid using IK as returns awkward postures
        print "Moving to campos"
        move_arm_xyz("left", board_dict["campos"])
        home_joint_angles = baxter_interface.Limb('left').joint_angles()
    else:
        print "Moving to home_joint_angles position"
        print home_joint_angles
        baxter_interface.Limb('left').move_to_joint_positions(home_joint_angles)
    sleep(1.0) # allow position to settle for camera
    return


def move_navpos():
    print "Moving right arm for easy access to Navigator"
    say("Moving right arm for easy access to Navigator")
    """#### Misss this move out if close to navpos
    baxter_interface.Limb('right').move_to_joint_positions({   'right_s0':0.785,
                                                                'right_s1':-0.517,
                                                                'right_w0':-0.575,
                                                                'right_w1':1.832,
                                                                'right_w2':0.531,
                                                                'right_e0':0.311,
                                                                'right_e1':1.287
                                                                })
    """
    baxter_interface.Limb('right').move_to_joint_positions({    'right_s0':0.036, 
                                                                'right_s1':0.309,
                                                                'right_w0':-1.535,
                                                                'right_w1':2.094,
                                                                'right_w2':1.035,
                                                                'right_e0':0.680,
                                                                'right_e1':1.007
                                                                })
                                                                
def calibrate_board():
    """
    Sets up the robot so that it knows the board position. Settings are saved to a file
    so should only need calibrating if the workspace changes. 
    Manually move the arm and centalise the gripper over the bottom left square (a0),
    then press RETURN. Then manually move the arm and centalise the gripper over the
    top right square (h7), then press RETURN. 
    """
    global board_dict, zOffset, zApproach, listener
    print "Calibrating:"
    #check if gripper is calibrated, if not, reboot it and calibrate
    #print baxter_interface.Gripper('left').calibrated(), " cali"
    if baxter_interface.Gripper('left').calibrated() == False:
        #print "cal"
        #baxter_interface.Gripper('left').reboot()
        baxter_interface.Gripper('left').calibrate()

    # get A0 position
    print "Manually centalise left gripper over A0 square.",
    msg=raw_input("Press RETURN")
    try:
        (trans,rot) = listener.lookupTransform('/base', '/left_gripper', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass#raise #continue
    #print "trans A0 = ", trans, "rot A0 = ",rot
    a0 = [trans[0], trans[1]]
    zA0= trans[2]
    print "Position A0 = ", a0
    
    # Get H7 position    
    print "Manually centalise left gripper over H7 square.",
    msg=raw_input("Press RETURN")
    try:
        (trans,rot) = listener.lookupTransform('/base', '/left_gripper', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass#raise #continue
    #print "Position H7 = (", trans ,"), (", rot, ")"
    h7 = [trans[0], trans[1]]
    zH7 = trans[2]
    print "Position H7 = ", h7
    
    # Work out board positions
    if abs(zA0 - zH7) > 0.05: # May need to adjust this (0.05m = 5mm) for consistant operation
        print "Board not level"
        return
    else:
        print "Board level witnin limits"
        zOffset= zA0 - 0.008 + (zA0 - zH7)/2
    print "Pick height  = ", zOffset, zA0, zH7
    board_dict["campos"]=[0,0,0]
    board_dict["campos"][0] = a0[0] - 0.080 # 0.070 # x position
    board_dict["campos"][1] = a0[1] + 0.440 # 0.440 # y position
    board_dict["campos"][2]= zOffset + 0.462
    print "Camera position = ", board_dict["campos"]
    board_dict["zoffset"]= zOffset
    
    # Calculate board xy positions
    # A0 position
    x = a0[0]
    y = a0[1]
    # H7 position
    hx = h7[0]
    hy = h7[1]
    print "A0 = (",x, y, ")"
    dx = (x - hx)/7
    dy = (y - hy)/7
    print "dx, dy = ", dx, dy
    for c in "ABCDEFGH":
        for n in "01234567":
            board_dict[c+n] = [x,y]
            x = x - dx
            print c+n, board_dict[c+n]
        x = board_dict["A0"][0]
        y = y - dy
    
    # Calculate take storage positions
    x = board_dict["H7"][0] 
    y = board_dict["H7"][1] + 0.1
    # print "Base take position = ", x, y
    for takeno in range(0,5):
        board_dict["take"+str(takeno)] = [x,y]
        x = x + 0.06
        #print takeno, x,y
    y = y + 0.06
    x = board_dict["H7"][0]
    for takeno in range(5, 9):
        board_dict["take"+str(takeno)] = [x,y]
        x = x + 0.06
    y = y + 0.06
    x = board_dict["H7"][0]
    for takeno in range(9, 13):
        board_dict["take"+str(takeno)] = [x,y]
        x = x + 0.06
    #print board_dict
    # Save new positions to board.cfg
    print "Saving cfg file"
    pickle.dump(board_dict, open (ppath+"/board.cfg", "wb"))
    print "Saved game configuration to ./src/baxter_game/board.cfg"
    return

def move_arm_xyz(limb ="left", xyz =[0.67, 0.25, 0.27], rot = [pi, 0, pi]):
    #try:
    pose = xyz+rot
    angles = ik_solver_request( limb, pose)
    #print angles
    if angles!= None:
        print "Moving to position" #, pose
        baxter_interface.Limb(limb).move_to_joint_positions(angles)
    #except:
    #    print "Out of range"
    
def ik_solver_request(input_limb, input_pose):
    print "IK solver request:"
    #input error checking
    if len(input_pose) == 6:
        quaternion_pose = conversions.list_to_pose_stamped(input_pose, "base")
    elif len(input_pose) == 7:
        quaternion_pose = input_pose
    else:
        print """Invalid Pose List:
    Input Pose to function: ik_solver_request must be a list of:
    6 elements for an RPY pose, or
    7 elements for a Quaternion pose"""
        return

    if input_limb == "right" or input_limb == "left":
        limb = input_limb
    else:
        print """Invalid Limb:
        Input Limb to function: ik_solver_request must be a string:
        'right' or 'left'"""
        return
    #request/response handling
    #print quaternion_pose
    node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    ik_service = rospy.ServiceProxy(node, SolvePositionIK)
    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(quaternion_pose)
    try:
        rospy.wait_for_service(node, 5.0)
        ik_response = ik_service(ik_request)
    except (rospy.ServiceException, rospy.ROSException), error_message:
        raise
        rospy.logerr("Service request failed: %r" % (error_message,))
    if (ik_response.isValid[0]):
        print("PASS: Valid joint configuration found")
        #convert response to JP control dict
        limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
#        print limb_joints
        return limb_joints
    else:
        print("FAILED: No valid joint configuration for this pose found")

def init():
    global listener, board_dict, zOffset, zApproach, previous_image
    roslib.load_manifest('baxter_game') #we load the manifest of the package we are in
    rospy.init_node ('baxter_game')
    listener = tf.TransformListener()
    BaxUI.reset_display() # Blanks BaxUI display
    # Set up board
    # Inititialise board to game translation table
    # Translates "Cn" type cords to xyz coords (nn)
    
    # print board [7][7] # debug

    # If "board.cfg" file exists, load it
    # board_dict contains robot cords for pick positions etc.
    try:
        print "Looking for board.cfg file."
        board_dict = pickle.load(open (ppath+"/board.cfg", "rb"))
        print "Successfully loaded board.cfg"
    except:
        print "File board.cfg not found, please calibrate_board" 
        board_dict={}
        #raise # debug
        calibrate_board()

    if "zoffset" in board_dict: # z level of board i.e. pick height, z cord
        zOffset=board_dict["zoffset"]
    else:
        zOffset = 0.0
    zApproach= zOffset + 0.1 # pick approach height - hardcoded
    if "campos" not in board_dict:
        board_dict["campos"] = [0.645, 0.377, 0.332]
    else:
        print "campos loaded", board_dict["campos"]
    
    #check if gripper is calibrated, if not, reboot it and calibrate
    #print baxter_interface.Gripper('left').calibrated(), " cali"
    if baxter_interface.Gripper('left').calibrated() == False:
        print "cal"
        #baxter_interface.Gripper('right').reboot()
        baxter_interface.Gripper('left').calibrate()
    baxter_interface.Gripper('left').set_holding_force(10) # % holding force (0% -100%)
    baxter_interface.Gripper('left').set_moving_force(10) 
    baxter_interface.Gripper('left').open()
    
    # Initialise the left hand camera to view board
    rh_camera_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,update_image)
    move_home("left")
    move_navpos()
    previous_image = get_image()
    

    

def update_image(data):
    """
    Callback for updating/displaying image from cams
    """
    global current_image
    #callback to update image stream from camera
    #print "Getting image from left arm cam"
    im_grey = CvBridge().imgmsg_to_cv2(data, 'mono8') # "bgr8") # for colour
    im_grey = im_grey[100:380, 360:640]
    thresh,im_bw = cv2.threshold(im_grey,140,255,cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    # cut out ROI - draughts board
    #print thresh,
    cv2.namedWindow("BW Image") # Create a window for display.
    cv2.imshow("BW Image", im_bw)
    cv2.waitKey(3) # Throws warning if 3ms wait not present
    current_image = im_bw


def get_image():
    global current_image, fileNo
    #process image
    # Auto threshold
    im_bw=[]
    while im_bw == []: # keep trying as update_image runs in background
        try:
            im_bw = current_image.copy()
            print "**",im_bw
        except:
            im_bw = []
            pass
    return im_bw



def get_move_visual():
    global previous_image
    print "Vision processing"
    previous_image = get_image()
    # User moves then presses navigator button
    mainmenu = [["Finished move!","stop"]]
    BaxUI.showList(mainmenu)
    BaxUI.stop()
    board_image = get_image()
    print "got board_image" #debug
    im_xor = cv2.bitwise_xor(board_image, previous_image)
    im_end = cv2.bitwise_and(im_xor, previous_image)
    im_start = cv2.bitwise_and(im_xor, board_image)
    #cv2.namedWindow( "Start Image", cv2.CV_WINDOW_AUTOSIZE ) # Debug.
    #cv2.imshow( "Start Image", im_start)
    #cv2.namedWindow( "End Image", cv2.CV_WINDOW_AUTOSIZE ) # Debug
    #cv2.imshow( "End Image", im_end)
    #cv2.waitKey(3)
    
    start_pos = None
    end_pos = None
    xindex = 0
    yindex = 7 
    for x in [10, 43, 75, 110, 142, 175, 210, 245]:
        for y in [255, 220, 150,185, 115, 80, 45, 10]:
            roi_start = im_start[y:y+25, x:x+25]
            #cv2.rectangle(im_start,(x, y),(x + 25,y + 25),(255,),-1) # check alignment
            roi_end = im_end[y:y+25, x:x+25]
            #cv2.rectangle(im_end,(x, y),(x + 25,y + 25),(255,),-1) # check alignment
            #print [x, y] # debug
            #print [xindex,yindex] # debug
#### problem here does not pick up yindex pos end move for jumps! Threshold?
            if int(roi_start.mean())>100:
                # Because ysequence is reducing will end up with first
                # move from position
                start_pos = [xindex,yindex]
            if int(roi_end.mean())>110:
                end_pos = [xindex, yindex]
            yindex -= 1
        yindex = 7
        xindex += 1
    move = ["HGFEDCBA"[start_pos[0]]+str(start_pos[1]), 
            "HGFEDCBA"[end_pos[0]]+str(end_pos[1])]   
    print "Move = ", move
    return move
    #int(raw_input ("Select move number: ")) # Quick test w/o vision



moveItem = 0 # menu item selected - integer

def set_move(selected = 0):
    global moveItem
    # Callback from BaxUI
    print "Button ", selected, " selected"
    BaxUI.runit=False
    moveItem = selected +1


def get_move(robot_legal, bax_move=1):
    """
    1
    """
    #global moveItem
    #print "Robot Legal Moves"
    # set BaxUI callback
    BaxUI.set_move = set_move
    # Show options on navigator
    print "Waiting for navigator response"
    say("Please make your move") 
    if bax_move == 1:
        # Baxter moves for player
        index = 0
        mainmenu= [0] * len(robot_legal)
        for m in robot_legal:
            #print str(index + 1)+": "+ str(m)
            mainmenu[index]= [str(robot_legal[index]), "set_move"]
            index += 1
        usr_input = BaxUI.showList(mainmenu) 
        #print moveItem 
        return usr_input
    else:
        print "Watching board for player to move!"
        #usr_input = integer index to robot_legal selected 
        usr_move = False
        usr_input = 0
        while usr_move == False:
            usr_move = get_move_visual()
            for m in robot_legal:
                if m[0] == usr_move[0] and m[1] == usr_move[1]:
                    print "Legal Move! ", m
                    return usr_input 
                else:
                    usr_input += 1
            print "Illegal Move!"
            usr_move = False
            usr_input = 0    


fileNo=0    
def save_image(filename="BoardImage"):
    global fileNo
    image = get_image()
    filename= filename + str(fileNo)+".png"
    fileNo += 1
    cv2.imwrite(filename, image)
    print "Saved to: ",filename, fileNo

# Start of module code

def say( msg="Ok"):
    system("espeak '"+ msg+"'")

    
# Only run if main module for testing
if __name__ == "__main__":
    init() # Initialise ROS node (has to be done in main module!)
    # Test

    #move_home("left")
    #move_piece ("A0", "B1")
    #moveTo("take0") 
    
    #calibrate_board() # Only required if robot/table/board is moved
    # Get right arm angles/position for navigator access
    #print baxter_interface.Limb('right').joint_angles()
    move_navpos()
    move_home("left")

        
    get_image()

    # Interactively test
    while 1: 
        msg = raw_input("Function to test or 'q' to Quit: ")
        if msg =="q": break
        try:
            print eval(msg)
        except:
            pass
    
