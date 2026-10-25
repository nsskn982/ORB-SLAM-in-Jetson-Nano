/**
 * @file motion_detection.py
 * @brief Simple script to detect motion using a GPIO pin on a NVIDIA Jetson board.
 * @author NVIDIA
 * @date 2025-11-06
 * @details This script initializes BCM pin 17 as an input and checks its current state.
 *          It prints "Motion detected" if the pin is high, otherwise "No motion".
 */

import Jetson.GPIO as GPIO
import time

# Disable warnings that might appear when re-running the script
GPIO.setwarnings(False) 

# Set the pin numbering mode to BCM (Broadcom SOC channel number)
GPIO.setmode(GPIO.BCM)

# Configure pin 17 as an input pin
# The internal pull-up/down resistors can be set here if needed, e.g., pull_up_down=GPIO.PUD_UP
GPIO.setup(17, GPIO.IN)

# Check the current input status of pin 17
if GPIO.input(17): 
    # The pin is currently high (3.3V)
    print("Motion detected") 
else:
    # The pin is currently low (0V)
    print("No motion") 
    
# It is good practice to clean up the GPIO channels when the script finishes.
# Uncomment the line below when implementing this in a production environment or a loop.
# GPIO.cleanup()
