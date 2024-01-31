#! /usr/bin/python
import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
import baxter_interface

# Create an instance of the Limb class
left_arm = baxter_interface.Limb('left')
# Define the chessboard pattern size (8x8)
pattern_size = (7, 7)

# Define the termination criteria for the corner refinement process
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Define the known size of one of the squares on the chessboard
square_size = 32.0  # in millimeters

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
            print(center[0], center[1])

            # Detect the position of the chessboard
            #if center[0] > width // 2 + 20:
               #print("Move the robot to the right to align the center of the board.")
                #move_right()
            #elif center[0] < width // 2 - 20:
                #print("Move the robot to the left to align the center of the board.")
                #move_left()
           # elif center[1] > height // 2 + 20:
               # print("Move the robot up to align the center of the board.")
               # move_up()
            #elif center[1] < height // 2 - 20:
               # print("Move the robot down to align the center of the board.")
               # move_down()
            #else:
                #print("The center of the board is aligned.")
                # Calculate the size of one pixel in the image
               # number_of_pixels_per_square = np.linalg.norm(corners[0] - corners[1])
               # pixel_size = square_size / number_of_pixels_per_square
                # Calculate the distance of the chessboard from the robot
               # distance = (square_size * (focal_length / number_of_pixels_per_square) / (2 * np.tan(np.radians(fov / 2))))*100
        else:
            # Display the message on the image
            message = "Chessboard Lost"
            color = red
            distance = 0

        cv2.putText(image, "Distance: {:.4f} m".format(distance), (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    1, white, 2, cv2.LINE_AA)
        cv2.putText(image, message, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1, color, 2, cv2.LINE_AA)

        # Display the result
        cv2.imshow('Tracking and Aligning the Center of a Chessboard', image)

        # Break the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User interrupted the program.")

if __name__ == '__main__':
    rospy.init_node('video_display')
    video_display = VideoDisplay()
    rospy.spin()

cv2.destroyAllWindows()

