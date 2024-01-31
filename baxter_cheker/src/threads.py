import cv2
import numpy as np
import threading

# Open the webcam
cap = cv2.VideoCapture(0)

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

def capture_video_thread():
    while True:
        # Capture a frame from the webcam
        ret, frame = cap.read()

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners in the frame
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if found:
            # Refine the chessboard corners
            cv2.cornerSubPix(gray, corners, sub_pixel_search_window, (-1, -1), criteria)

            # Draw the chessboard corners on the frame
            cv2.drawChessboardCorners(frame, pattern_size, corners, found)

            # Update the message to display
            message = "Chessboard Found"
            color = green
        else:
            # Display the message on the frame
            message = "Chessboard Lost"
            color = red
            distance = 0

        cv2.putText(frame, message, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1, color, 2, cv2.LINE_AA)

        # Display the result
        cv2.imshow('Tracking and Aligning the Center of a Chessboard', frame)

        # Break the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def align_center_thread():
    while True:
        # Capture a frame from the webcam
        ret, frame = cap.read()

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners in the frame
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if found:
            # Calculate the center of the chessboard
            center = np.mean(corners,axis=0)
            # Get the shape of the frame
            height, width, _ = frame.shape

            # Access the first element of the center array
            center = center[0]

            # Draw a green dot to represent the center of the chessboard
            cv2.circle(frame, (int(center[0]), int(center[1])), 10, (0, 255, 0), -1)

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
                height, width, _ = frame.shape
                # Calculate the size of one pixel in the image
                number_of_pixels_per_square = np.linalg.norm(corners[0] - corners[1])
                pixel_size = square_size / number_of_pixels_per_square
                # Calculate the distance of the chessboard from the camera
                distance = (square_size * (focal_length / number_of_pixels_per_square) / (2 * np.tan(np.radians(fov / 2))))*100
                print("Distance: {:.4f} m".format(distance))
video_thread = threading.Thread(target=capture_video_thread)
align_thread = threading.Thread(target=align_center_thread)
video_thread.start()
align_thread.start()
video_thread.join()
align_thread.join()
cap.release()
cv2.destroyAllWindows()