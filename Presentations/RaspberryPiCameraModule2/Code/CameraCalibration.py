/**
 * @file LiveCameraTest.py
 * @brief Simple script to test live camera feed capture (e.g., on Jetson/Pi).
 * @author Srikanth
 * @date Nov 6, 2025
 */

import cv2

# Initialize video capture. 0 is usually the default built-in webcam.
# On Jetson/Pi, this might need to be adjusted (e.g., adding specific API backends).
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Main loop to continuously read and display frames
print("Starting live camera feed. Press 'ESC' to exit...")
while True:
	# Read a frame from the camera
	ret, frame = cap.read()

	# If frame is not read correctly (e.g., camera disconnected), break the loop
	if not ret:
		print("Error: Failed to read frame from camera.")
		break
        
	# Display the resulting frame in a window named "Live"
	cv2.imshow("Live", frame)

	# Wait for a key press for 1ms. 
    # If the key is 'ESC' (ASCII 27), break the loop
	if cv2.waitKey(1) & 0xFF == 27:
		break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
print("Camera released and windows closed.")