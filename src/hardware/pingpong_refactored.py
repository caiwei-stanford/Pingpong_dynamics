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
import queue

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

from dynamixel_sdk import *  # Uses Dynamixel SDK library


class RobotController:
    def __init__(self):

        # Control table address
        self.torque_enable_addr = 64  # Control table address is different in Dynamixel model
        self.goal_position_addr = 116
        self.present_position_addr = 132
        # Protocol version
        self.protocol_version = 2.0  # See which protocol version is used in the Dynamixel

        # Default setting
        self.dxl_id = 1  # Dynamixel ID : 1
        self.baudrate = 57600  # Dynamixel default baudrate : 57600
        self.devicename = '/dev/cu.usbserial-FT7W9245'  # Check which port is being used on your controller
        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.torque_enable = 1  # Value for enabling the torque
        self.torque_disable = 0  # Value for disabling the torque
        self.dxl_min_pos_val = 770  # Dynamixel will rotate between this value
        self.dxl_max_pos_val = 2350  # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        self.dxl_moving_status_threshold = 20  # Dynamixel moving status threshold
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
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id,
                                                                       self.torque_enable_addr, self.torque_enable)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def set_goal_position(self, position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.dxl_id,
                                                                       self.goal_position_addr, position)
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
    - balance: a boolean indicating whether the robot should move based on the position of the ball
    """

    def __init__(self, robot_controller, videoidx=0, inc=20):
        threading.Thread.__init__(self)
        self.rc = robot_controller
        self.inc = inc
        self.cap = cv2.VideoCapture(videoidx)

        self.set_camera()

        self.balance = False
        self.running = True
        self.Kp = 0.00025
        self.Kd = 0.0006
        # save the position value
        self.position_array = np.zeros(4)

        # initialize queue
        self.frame_queue = queue.Queue(maxsize=1)

    def set_camera(self):
        """ changes default camera properties """
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # set the focus to infinity
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)
        # make sure auto exposure and auto white balance are turned off
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)

    def ringbuff(self, new_value):
        """ keep save the new value to the array and pop the oldest value """
        self.position_array = np.append(self.position_array, new_value)
        self.position_array = np.delete(self.position_array, 0)
        return self.position_array

    def get_velocity(self):
        return (self.position_array[3] - self.position_array[0]) / 4

    @staticmethod
    def detect_ball(self, frame):
        lower_orange = np.array([10, 80, 230])
        upper_orange = np.array([40, 255, 255])
        center_x, center_y = None, None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            moments = cv2.moments(largest)
            if moments["m00"] != 0:
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])

        return center_x, center_y, contours

    def get_goal_position(self, dist):
        pos_term = dist * self.Kp
        vel_term = self.get_velocity() * self.Kd
        val = pos_term + vel_term
        val = np.clip(val, -1, 1)

        theta = 90 - np.degrees(np.arccos(val))
        dyn_theta = 1550 + (theta / 360) * 4000

        return int(dyn_theta)

    def move_motor_left(self):
        current_position, dxl_comm_result, dxl_error = self.rc.packetHandler.read4ByteTxRx(
            self.rc.portHandler, self.rc.dxl_id, self.rc.goal_position_addr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.rc.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.rc.packetHandler.getRxPacketError(dxl_error))
        else:
            new_position = current_position - self.inc
            if new_position < self.rc.dxl_min_pos_val:
                new_position = self.rc.dxl_min_pos_val
            self.rc.set_goal_position(new_position)

    def move_motor_right(self):
        current_position, dxl_comm_result, dxl_error = self.rc.packetHandler.read4ByteTxRx(
            self.rc.portHandler, self.rc.dxl_id, self.rc.goal_position_addr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.rc.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.rc.packetHandler.getRxPacketError(dxl_error))
        else:
            new_position = current_position + self.inc
            if new_position > self.rc.dxl_max_pos_val:
                new_position = self.rc.dxl_max_pos_val
            self.rc.set_goal_position(new_position)

    def shutdown(self):
        dxl_comm_result, dxl_error = self.rc.packetHandler.write1ByteTxRx(
            self.rc.portHandler, self.rc.dxl_id, self.rc.torque_enable_addr, self.rc.torque_disable
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(self.rc.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print(self.rc.packetHandler.getRxPacketError(dxl_error))

        self.cap.release()
        self.rc.portHandler.closePort()

    def run(self):
        """
        Thread function to capture video, detect ball, and control motor.
        """

        while self.running:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                print("️Frame capture failed")
                continue

            # draw ball outline and center
            center_x, center_y, contours = self.detect_ball(self, frame)
            cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
            cv2.circle(frame, (center_x, center_y), 8, (255, 0, 0), -1)

            # Queue the frame for display
            if not self.frame_queue.full():
                self.frame_queue.put(frame)

            # Ball-following logic
            if self.balance and center_x is not None:
                try:
                    current_pos, result, error = self.rc.packetHandler.read4ByteTxRx(
                        self.rc.portHandler, self.rc.dxl_id, self.rc.goal_position_addr
                    )

                    if result != COMM_SUCCESS:
                        print("Dynamixel Comm Error:", self.rc.packetHandler.getTxRxResult(result))
                    elif error != 0:
                        print("Dynamixel Packet Error:", self.rc.packetHandler.getRxPacketError(error))
                    else:
                        self.position_array = self.ringbuff(center_x)

                        # Position calculation
                        dist = center_x - (frame.shape[1] // 2)
                        if abs(dist) < 10 and abs(self.get_velocity()) < 2:
                            continue

                        new_pos = self.get_goal_position(dist)

                        # Clamp and send
                        new_pos = max(self.rc.dxl_min_pos_val, min(new_pos, self.rc.dxl_max_pos_val))
                        self.rc.set_goal_position(new_pos)

                        print(f" center_x: {center_x}, velocity: {self.get_velocity():.2f}, goal: {new_pos}")

                except Exception as e:
                    print(f"Motor control error: {e}")

        # shutdown
        self.shutdown()


if __name__ == '__main__':
    video_index = 0
    robot_controller = RobotController()
    robot_thread = RobotThread(robot_controller)

    robot_thread.start()
    try:
        while True:

            if not robot_thread.frame_queue.empty():
                frame = robot_thread.frame_queue.get()
                cv2.imshow('frame', frame)

            key = cv2.waitKey(1) & 0xFF

            # when 'w' is pressed, toggle the movement of the motor dependent to the ball position
            if key == ord('w'):
                print("w pressed")
                robot_thread.balance = not robot_thread.balance

            # when 'e' is pressed, move the motor to the left
            if key == ord('e'):
                print("e pressed")
                robot_thread.move_motor_left()

            # when 'r' is pressed, move the motor to the right
            if key == ord('r'):
                print("r pressed")
                robot_thread.move_motor_right()

            if key == ord('q'):
                robot_thread.running = False
                break

    finally:
        robot_thread.cap.release()
        robot_thread.rc.portHandler.closePort()
        cv2.destroyAllWindows()
        robot_thread.join()
