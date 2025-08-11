import cv2
import threading
import queue

import numpy as np
from dynamixel_sdk import *
from robot_base import RobotBase
from data_exch import DataExchange


class RobotDevice(RobotBase, threading.Thread):
    def __init__(self, device_name='/dev/tty.usbserial-FT7W9245', dxl_id=1, baudrate=57600, video_idx=0):
        super().__init__()
        self.command_queue = queue.Queue()
        self.frame_queue = queue.Queue(maxsize=1)
        self.running = True
        self.inc = 20

        # Position buffer for velocity estimation
        self.position_array = np.zeros(4)

        # Dynamixel setup
        self.dxl_id = dxl_id
        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(2.0)
        self.baudrate = baudrate

        self.torque_enable_addr = 64
        self.goal_position_addr = 116
        self.present_position_addr = 132
        self.torque_enable = 1
        self.torque_disable = 0
        self.dxl_min_pos_val = 770
        self.dxl_max_pos_val = 2350

        # Camera setup
        self.cap = cv2.VideoCapture(video_idx)
        self._init_camera()

        # Check motor
        self._init_motor()

        # Setup data exchange
        self.data_ex = DataExchange(ring_buff_size=10000)

    def _init_motor(self):
        if not self.port_handler.openPort():
            raise RuntimeError("Failed to open port")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError("Failed to set baudrate")
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, self.torque_enable_addr, self.torque_enable)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            raise RuntimeError("Failed to enable torque")


    def _init_camera(self):
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    def set_lever_angle(self, pos):
        pos = max(self.dxl_min_pos_val, min(pos, self.dxl_max_pos_val))
        self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, self.goal_position_addr, int(pos))

    def get_lever_angle(self):
        pos, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.dxl_id, self.present_position_addr)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            # handle errors if necessary, for now just return last known
            return self.position_array[-1]
        return pos

    def ringbuff(self, new_value):
        self.position_array = np.append(self.position_array, new_value)[1:]
        return self.position_array

    def get_velocity(self):
        return (self.position_array[3] - self.position_array[0]) / 4

    def move_motor_left(self):
        current_position = self.get_lever_angle()
        new_position = current_position - self.inc
        if new_position < self.dxl_min_pos_val:
            new_position = self.dxl_min_pos_val
        self.set_lever_angle(new_position)

    def move_motor_right(self):
        current_position = self.get_lever_angle()
        new_position = current_position + self.inc
        if new_position > self.dxl_max_pos_val:
            new_position = self.dxl_max_pos_val
        self.set_lever_angle(new_position)

    def read_frame(self):
        ret, frame = self.cap.read()
        return frame if ret else None

    def exec_cmd(self):
        if not self.command_queue.empty():
            command, value = self.command_queue.get()
            if command == "set_lever_angle":
                self.set_lever_angle(value)
            elif command == "left":
                self.move_motor_left()
            elif command == "right":
                self.move_motor_right()
            else:
                print(f"unrecognized command {command}")

    def send_data(self):
        """send data to main thread (ball position, ball velocity, lever angle)
        """
        # ToDo: get ball position center
        data_entry = np.array([0, 0, self.get_velocity(), self.get_lever_angle()], dtype=float)
        self.data_ex.set_data(data_entry)

    # ToDo: implement async for updating, command, data
    def run(self):
        while self.running:
            # Capture frame
            frame = self.read_frame()
            if frame is not None and not self.frame_queue.full():
                self.frame_queue.put(frame)

            # Update position buffer for velocity tracking
            angle = self.get_lever_angle()
            self.ringbuff(angle)

            # Handle commands
            self.exec_cmd()

            # Send data every cycle
            self.send_data()

    def stop(self):
        self.running = False
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, self.torque_enable_addr, self.torque_disable)
        self.cap.release()
        self.port_handler.closePort()
