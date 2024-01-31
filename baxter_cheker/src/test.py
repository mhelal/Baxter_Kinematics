#! /usr/bin/python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np
import threading
print("Initializing node... ")
rospy.init_node("my_Tests")
print("Getting robot state... ")
rs = baxter_interface.RobotEnable(CHECK_VERSION)
init_state = rs.state().enabled
print("Robot state : " + str(init_state))

left_arm = baxter_interface.Limb('left')
joint_angles = left_arm.joint_angles()


print("Robot left_arm : " + str(left_arm))
print("Robot joint_angles : " + str(joint_angles))

print("Updating Robot lefy arm angle ... ")
#joint_angles['left_w2'] -= 0.1
#joint_angles['left_w1'] -= 0.2
print(joint_angles['left_w2'])
left_arm.move_to_joint_positions(joint_angles)

print('Now updated')


#movement




# Define the chessboard pattern size (8x8)
pattern_size = (7, 7)

# Define the termination criteria for the corner refinement process
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Define the known size of one of the squares on the chessboard
square_size = 336.0  # in millimeters

# Define the focal length of the camera
focal_length = 3.04e-3  # in meters
fov = 90.0
red = (0,0,255)
green = (0,255,0)
white = (255,255,255)

# Pre-calculate the chessboard corner refinement sub-pixel search window
sub_pixel_search_window = (11, 11)


def move_right():
    # Get the current joint angles
    joint_angles = left_arm.joint_angles()

    # Increment the joint angle of the s0 joint
    joint_angles['left_s0'] += 0.1

    # Set the new joint angles
    left_arm.move_to_joint_positions(joint_angles)

def move_left():
    # Get the current joint angles
    joint_angles = left_arm.joint_angles()

    # Decrement the joint angle of the s0 joint
    joint_angles['left_s0'] -= 0.1

    # Set the new joint angles
    left_arm.move_to_joint_positions(joint_angles)

def move_down():
    # Get the current joint angles
    joint_angles = left_arm.joint_angles()

    # Decrement the joint angle of the s1 joint
    joint_angles['left_s1'] -= 0.1

    # Set the new joint angles
    left_arm.move_to_joint_positions(joint_angles)

def move_up():
    # Get the current joint angles
    joint_angles = left_arm.joint_angles()

    # Increment the joint angle of the s1 joint
    joint_angles['left_s1'] += 0.1

    # Set the new joint angles
    left_arm.move_to_joint_positions(joint_angles)

def search_chessboard():
    # Get the current joint angles
    joint_angles = left_arm.joint_angles()

    # Increment the joint angle of the s0 joint
    joint_angles['left_s0'] += 0.1

    # Set the new joint angles
    left_arm.move_to_joint_positions(joint_angles)


class VideoDisplay:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.image_callback)

    def image_callback(self, msg):
        # Convert the ROS image message to an OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners in the image
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if found:
            # Refine the chessboard corners
            cv2.cornerSubPix(gray, corners, sub_pixel_search_window, (-1, -1), criteria)

            # Draw the chessboard corners on the image
            cv2.drawChessboardCorners(image, pattern_size, corners, found)

            # Update the message to display
            message = "Chessboard Found"
            color = green

            # Calculate the center of the chessboard
            center = np.mean(corners, axis=0)

            # Get the shape of the image
            height, width, _ = image.shape

            # Access the first element of the center array
            center = center[0]

            # Draw a green dot to represent the center of the chessboard
            cv2.circle(image, (int(center[0]), int(center[1])), 10, (0, 255, 0), -1)
            #print(center[0],center[1])
            number_of_pixels_per_square = np.linalg.norm(center[0] - center[1])
	    #print(number_of_pixels_per_square)
            print(np.tan(np.radians(180)))
            distance = (square_size * (focal_length / number_of_pixels_per_square) / (2 * np.tan(np.radians(fov / 2))))*100

            
                # Calculate the size of one pixel in the image
                #number_of_pixels_per_square = np.linalg.norm(corners[0] - corners[1])
                #pixel_size = square_size / number_of_pixels_per_square
                # Calculate the distance of the chessboard from the robot
                #distance = (square_size * (focal_length / number_of_pixels_per_square) / (2 * np.tan(np.radians(fov / 2))))*100
        else:
            # Display the message on the image
            message = "Chessboard Lost"
            color = red
            distance = 0

        cv2.putText(image, "Distance: {:.4f} m".format(distance), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, white, 2, cv2.LINE_AA)
        cv2.putText(image, message, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,1, color, 2, cv2.LINE_AA)

        # Display the result
        cv2.imshow('Tracking and Aligning the Center of a Chessboard', image)

        # Break the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User interrupted the program.")

def capture_video_thread():
    video_display = VideoDisplay()
    rospy.spin()
def align_center_thread():
    while True:
        # Convert the ROS image message to an OpenCV image
        bridge = cv_bridge.CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Find the chessboard corners in the frame
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if found:
            # Calculate the center of the chessboard
            center = np.mean(corners,axis=0)
            # Get the shape of the frame
            height, width, _ = image.shape

            # Access the first element of the center array
            center = center[0]

            # Draw a green dot to represent the center of the chessboard
            cv2.circle(image, (int(center[0]), int(center[1])), 10, (0, 255, 0), -1)

            # Detect the position of the chessboard
            if center[0] > width // 2 + 20:
                print("Move the camera to the right to align the center of the board.")
            elif center[0] < width // 2 - 20:
                print("Move the camera to the left to align the center of the board.")
            elif center[1] > height // 2 + 20:
                print("Move the camera down to align the center of the board.")
            elif center[1] < height // 2 - 20:
                print("Move the camera up to align the center of the board.")
            else:
                print("The center of the board is aligned.")
                # Get the shape of the frame
                height, width, _ = image.shape
                # Calculate the size of one pixel in the image
                number_of_pixels_per_square = np.linalg.norm(corners[0] - corners[1])
                pixel_size = square_size / number_of_pixels_per_square
                # Calculate the distance of the chessboard from the camera
                distance = (square_size * (focal_length / number_of_pixels_per_square) / (2 * np.tan(np.radians(fov / 2))))*100
                print("Distance: {:.4f} m".format(distance))

#video_thread = threading.Thread(target=capture_video_thread)
#align_thread = threading.Thread(target=align_center_thread)
#video_thread.start()
#align_thread.start()
#video_thread.join()
#align_thread.join()
capture_video_thread()

cv2.destroyAllWindows()