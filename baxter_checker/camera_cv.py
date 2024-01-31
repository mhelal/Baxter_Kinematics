#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
import baxter_interface
from baxter_interface import CHECK_VERSION
import torch
from ultralytics import YOLO


print("Initializing node... ")
rospy.init_node("camera")
print("Getting robot state... ")
rs = baxter_interface.RobotEnable(CHECK_VERSION)
init_state = rs.state().enabled
print("Robot state : " + str(init_state))
left_arm = baxter_interface.Limb('left')
joint_angles = left_arm.joint_angles()
print("Robot left_arm : " + str(left_arm))
print("Robot joint_angles : " + str(joint_angles))


def draw_chessboard(img, chessboard, square_size):
    for i in range(8):
        for j in range(8):
            if (i + j) % 2 == 0:
                color = (0, 0, 0)
            else:
                color = (255, 255, 255)
            x_start, y_start = i * square_size, j * square_size
            cv2.rectangle(img, (x_start, y_start), (x_start + square_size, y_start + square_size), color, -1)
            if chessboard[j, i] == 2:
                cv2.circle(img, (x_start + square_size // 2, y_start + square_size // 2),
                           square_size // 2 - 5, (255, 255, 255), -1)
                cv2.circle(img, (x_start + square_size // 2, y_start + square_size // 2),
                           square_size // 2 - 6, (0, 0, 0), -1)
            elif chessboard[j, i] == 1:
                cv2.circle(img, (x_start + square_size // 2, y_start + square_size // 2),
                           square_size // 2 - 5, (0, 0, 0), -1)
                cv2.circle(img, (x_start + square_size // 2, y_start + square_size // 2),
                           square_size // 2 - 6, (255, 255, 255), -1)
            

def get_piece_position(x1, y1, x2, y2,x1_cb,y1_cb,square_size):
    row, col = int((y2 - y1) /2), int((x2 - x1) /2)
    row, col = (y1 + row-y1_cb)//square_size,(x1 + col-x1_cb)//square_size
    print(row,col)
    return row, col

# Load the trained model
model_path = 'C:/Users/Trinh/Downloads/baxter_checker_data/best.pt'
model = YOLO(model_path)
threshold = 0.25
class_name_dict = {0: 'chess_board', 1: 'white', 2: 'black'}

# Initialize the webcam

class VideoDisplay:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.image_callback)
    def image_callback(self, msg):
        # Convert the ROS image message to an OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #image = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        results = model(frame)[0]
        chessboard_bbox = None
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result

            if score > threshold:
                # Scale the bounding box coordinates back to the original frame size
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 4)
                cv2.putText(frame, class_name_dict[int(class_id)].upper(), (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
                # x1, y1, x2, y2, score, class_id = result
                if class_name_dict[int(class_id)] == 'chess_board':
                    chessboard_bbox = (int(x1), int(y1), int(x2), int(y2))
                if class_name_dict[int(class_id)] == 'white':
                    if chessboard_bbox is not None:
                        print('white:', x1,y1,x2,y2)
                        x1_cb, y1_cb, x2_cb, y2_cb = chessboard_bbox
                        square_size = (x2_cb - x1_cb) // 8
                        row, col = get_piece_position(int(x1), int(y1), int(x2), int(y2), x1_cb, y1_cb, square_size)
                        if 0 <= row < 8 and 0 <= col < 8:
                            chessboard_matrix[row, col] = 1
                if class_name_dict[int(class_id)] == 'black':
                    if chessboard_bbox is not None:
                        print('black:', x1,y1,x2,y2)
                        x1_cb, y1_cb, x2_cb, y2_cb = chessboard_bbox
                        square_size = (x2_cb - x1_cb) // 8
                        row, col = get_piece_position(int(x1), int(y1), int(x2), int(y2), x1_cb, y1_cb, square_size)
                        if 0 <= row < 8 and 0 <= col < 8:
                            chessboard_matrix[row, col] = 2
    
        if chessboard_bbox is not None:
            visual_chessboard = np.zeros((8 * 50, 8 * 50, 3), dtype=np.uint8)
            print(chessboard_matrix)
            draw_chessboard(visual_chessboard, chessboard_matrix, 50)
            cv2.imshow('Visual Chessboard', visual_chessboard)
            chessboard_matrix = np.zeros((8, 8), dtype=int)
        # Display the result
        cv2.imshow('Tracking Chessboard', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User interrupted the program.")

if __name__ == '__main__':
    #rospy.init_node('video_display')
    video_display = VideoDisplay()
    rospy.spin()

cv2.destroyAllWindows()


