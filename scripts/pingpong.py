#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
####                        balancing ping pong ball                        ####
################################################################################
# Adapted the code from dynamixel sdk python example
# If you have any question, please reach out to wkdo@stanford.edu
# Won Kyung Do, Ph.D. Student in Mechanical Engineering at Stanford University, advised by Prof. Monroe Kennedy III

# moment of inertia: 3.2e-7 kgm^2
# mass: 0.002kg 


import os
import cv2
import threading
import numpy as np

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

class RobotController:
    def __init__(self):

        # Control table address
        self.torque_enable_addr = 64               # Control table address is different in Dynamixel model
        self.goal_position_addr = 116
        self.present_position_addr = 132
        # Protocol version
        self.protocol_version = 2.0               # See which protocol version is used in the Dynamixel

        # Default setting
        self.dxl_id = 1                 # Dynamixel ID : 1
        self.baudrate = 1000000             # Dynamixel default baudrate : 57600
        self.devicename = 'COM4'    # Check which port is being used on your controller
                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.torque_enable = 1                 # Value for enabling the torque
        self.torque_disable = 0                 # Value for disabling the torque
        self.dxl_min_pos_val = 770           # Dynamixel will rotate between this value
        self.dxl_max_pos_val = 2350            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        self.dxl_moving_status_threshold = 20                # Dynamixel moving status threshold
        self.dxl_middle_pos_val = 1560

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.devicename)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.protocol_version)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id, self.torque_enable_addr, self.torque_enable)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def set_goal_position(self, position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.dxl_id, self.goal_position_addr, position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))




class RobotThread(threading.Thread):
    """
    A class representing a thread that controls a robot for playing ping pong.

    Attributes:
    - robot_controller: an instance of the RobotController class that controls the robot
    - videoidx: an integer representing the index of the video capture device (default: 0)
    - inc: an integer representing the increment of the motor position (default: 70)
    - cap: a VideoCapture object that captures video from the video capture device
    - flag: a boolean indicating whether the robot should move based on the position of the ball
    """

    def __init__(self, robot_controller,videoidx = 0, inc=20):
        threading.Thread.__init__(self)
        self.rc = robot_controller
        self.inc = inc
        self.cap = cv2.VideoCapture(videoidx)

        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        # self.cap.set(cv2.CAP_PROP_FPS, 60)
        # self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
        # # set the focus to infinity
        # self.cap.set(cv2.CAP_PROP_FOCUS, 0)
        # # make sure auto exposure and auto white balance are turned off
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)

        self.flag = False

        # save the position value 
        self.position_array = np.zeros(4)
        
    def ringbuff(self, new_value):
        """ keep save the new value to the array and pop the oldest value """
        self.position_array = np.append(self.position_array, new_value)
        self.position_array = np.delete(self.position_array, 0)
        return self.position_array

        
    def run(self):
        """
        The main function that runs the thread. It captures video from the video capture device,
        detects the position of the ball, and moves the robot based on the position of the ball.
        """

        # Define range of orange color in HSV, with much tighter bounds for better detection
        lower_orange = np.array([10, 80, 230])
        upper_orange = np.array([40, 255, 255])
        # upper_orange = np.array([40, 128, 255])

        while True:
            # key = getch()

            # Capture frame-by-frame
            ret, frame = self.cap.read()

            # Convert frame to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Threshold the HSV image to get only orange colors
            mask = cv2.inRange(hsv, lower_orange, upper_orange)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Draw contours on original frame
            cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
            # from countours, find the largest contour and get its center
            if len(contours) > 0:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    # print("Center of largest contour: ({}, {})".format(center_x, center_y))
                

            # Display the resulting frame
            cv2.imshow('frame', frame)

            if self.flag:
                current_position, dxl_comm_result, dxl_error = self.rc.packetHandler.read4ByteTxRx(self.rc.portHandler, self.rc.dxl_id, self.rc.goal_position_addr)

                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.rc.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.rc.packetHandler.getRxPacketError(dxl_error))
                else:

                    self.position_array = self.ringbuff(center_x)
                    # from 4 value of position array, get average velocity
                    velocity = (self.position_array[3] - self.position_array[0])/4

                    ##### experiment code
                    # new_position = 1550 + int((center_x - 335)*0.3) - int((current_position - 1550)*damping_factor) - int(velocity*0.1)

                    pos = ((center_x - 335)*0.00025)
                    # vel = (velocity*0.01)
                    # vel = (velocity*0.00015)
                    vel = (velocity*0.0006)
                    val = pos  + vel # we changed the sign of the velocity term
                    val = np.asarray(val)
                    np.clip(val, -1, 1, out=val)
                    theta = np.arccos(val)
                    theta = 90-theta*180/np.pi
                    dyna_theta = 1550 + (theta/360)*4000
                    new_position = int(dyna_theta)

                    if new_position < self.rc.dxl_min_pos_val:
                        new_position = self.rc.dxl_min_pos_val
                    if new_position > self.rc.dxl_max_pos_val:
                        new_position = self.rc.dxl_max_pos_val
                    self.rc.set_goal_position(new_position)

            key = cv2.waitKey(2)

            # when 'w' is pressed, toggle the movement of the motor dependent to the ball position
            if key == ord('w'):
                print("w pressed")
                self.flag = not self.flag

            # when 'e' is pressed, move the motor to the left
            if key == ord('e'):
                print("e pressed")
                current_position, dxl_comm_result, dxl_error = self.rc.packetHandler.read4ByteTxRx(self.rc.portHandler, self.rc.dxl_id, self.rc.goal_position_addr)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.rc.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.rc.packetHandler.getRxPacketError(dxl_error))
                else:
                    new_position = current_position - self.inc
                    if new_position < self.rc.dxl_min_pos_val:
                        new_position = self.rc.dxl_min_pos_val
                    self.rc.set_goal_position(new_position)

            # when 'r' is pressed, move the motor to the right
            if key == ord('r'):
                print("r pressed")
                current_position, dxl_comm_result, dxl_error = self.rc.packetHandler.read4ByteTxRx(self.rc.portHandler, self.rc.dxl_id, self.rc.goal_position_addr)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.rc.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.rc.packetHandler.getRxPacketError(dxl_error))
                else:
                    new_position = current_position + self.inc
                    if new_position > self.rc.dxl_max_pos_val:
                        new_position = self.rc.dxl_max_pos_val
                    self.rc.set_goal_position(new_position)

            if key == ord('q'):
                break

        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.rc.packetHandler.write1ByteTxRx(self.rc.portHandler, self.rc.dxl_id, self.rc.torque_enable_addr, self.rc.torque_disable)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.rc.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.rc.packetHandler.getRxPacketError(dxl_error))

        # Close port
        self.rc.portHandler.closePort()

        # Release the capture and destroy all windows
        self.cap.release()
        cv2.destroyAllWindows()



if __name__ == '__main__':
    video_index = 0
    robot_controller = RobotController()
    robot_thread = RobotThread(robot_controller)


    robot_thread.start()

    robot_thread.join()



