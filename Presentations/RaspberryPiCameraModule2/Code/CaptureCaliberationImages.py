#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file       : CaptureCaliberationImages.py
@author     : Srikanth N
@date       : Nov 6, 2025 
@version    : 1.0
@brief      : Captures calibration images from the Raspberry Pi Camera V2 
              on an NVIDIA Jetson Nano using OpenCV and GStreamer.
@details    : This script utilizes the 'nvarguscamerasrc' GStreamer plugin 
              for high-performance CSI camera access on the Jetson platform. 
              It captures a set number of still images, saving them as 'calib_XX.jpg', 
              which is suitable for camera calibration (e.g., lens distortion correction).
"""
import cv2
import time # Added for potential use in a calibration loop

# --- Configuration Constants ---
# --- Doxygen Grouping Example ---
## @defgroup Configuration
# @{
## @brief The GStreamer pipeline string for the CSI camera on Jetson Nano.
#  @details This pipeline enables hardware acceleration and is crucial for optimal CSI camera performance.
#           It captures at 3264x2464 and scales down to 1280x720 for processing.
GSTREAMER_PIPELINE = (
    "nvarguscamerasrc sensor_id=0 ! "  # Use camera sensor 0 (CSI port 0)
    "video/x-raw(memory:NVMM), width=3264, height=2464, framerate=21/1, format=NV12 ! "
    "nvvidconv flip-method=0 ! "  # 0=none, 2=180 degrees, etc.
    "video/x-raw, width=1280, height=720, format=BGRx ! "  # Output resolution for processing/display
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink"
)

## @brief The total number of calibration images to capture.
NUM_CAPTURES = 20
## @}

def capture_calibration_images():
    """!
    @brief Initializes the CSI camera and captures a sequence of calibration images.
    
    @details Attempts to open the camera using the defined GStreamer pipeline.
             If successful, it enters a loop to capture `NUM_CAPTURES` images, 
             saving each with a sequential filename (`calib_00.jpg` to `calib_19.jpg`).
             It is designed to mimic the batch capture capability of `raspistill`.
    
    @return int : Returns 0 on success, -1 on failure to open the camera.
    """
    
    cap = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Error: Could not open camera using the GStreamer pipeline.")
        return -1
    
    print("Camera opened successfully. Preparing to capture calibration images...")
    
    for i in range(NUM_CAPTURES):
        # A small delay to allow a user to reposition the checkerboard for calibration
        # time.sleep(1) # Uncomment if you want an automatic delay
        
        ret, frame = cap.read()
        
        if ret:
            filename = f"calib_{i:02d}.jpg"
            # cv2.imwrite() is used to save the image to disk
            cv2.imwrite(filename, frame) 
            print(f"Captured {filename}")
        else:
            print("Error: Could not read frame from camera. Stopping capture loop.")
            break

    # Clean up resources
    cap.release()
    cv2.destroyAllWindows()
    print("Capture complete and camera released.")
    return 0

if __name__ == "__main__":
    capture_calibration_images()