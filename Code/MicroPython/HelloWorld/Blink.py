## @file Blink.py
#
# @brief Python script 'Hello World'

############################
# Creator of the File: Elmar Wings
# Date created: 28.8.2024
# Path: Code/HelloWorld/Blink.py
# Version: 2.0
# Reviewed by: 
# Review Date: 
############################
#
#@mainpage Hello World
#@section intro_sec Introduction
#If you a new hardware based of Arduino, you have to configure the Arduino IDE. After connecting the hardware with the PC, you have to configure the IDE. If you have to test the communication between PC and the hardware, you have to test it with a simple program. If the program works, you are sure that the process 'Creating a sketch' -> 'Downloading a sketch' -> 'Running a sketch'. If the program works, then the LED is switching on and off every 1sec.
#@section Blink_example
#The simple file
#


# Hello World for microcontroller boards
import pyb

## built-in red LED 
redLED   = pyb.LED(1)  

## built-in green LED
greenLED = pyb.LED(2) 
## built-in blue LED  
blueLED  = pyb.LED(3) 

## endless loop with LED.on and LED.off
while True:
    # Turns on the red LED
    redLED.on()
    # Makes the script wait for 1 second (1000 miliseconds)
    pyb.delay(1000)
    # Turns off the red LED
    redLED.off()
    pyb.delay(1000)
    greenLED.on()
    pyb.delay(1000)
    greenLED.off()
    pyb.delay(1000)
    blueLED.on()
    pyb.delay(1000)
    blueLED.off()
    pyb.delay(1000)