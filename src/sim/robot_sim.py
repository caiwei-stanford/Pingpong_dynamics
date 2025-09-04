"""RobotSimulator class
"""
import numpy as np
import time
import threading
import queue
import asyncio
from data_exch import DataExchange
from robot_base import RobotBase


class RobotSimulator(RobotBase, threading.Thread):
    """RobotSimulator class
    """

    def __init__(self, time_step=0.01, report_period=0.2):
        super().__init__()

        self._loop = asyncio.new_event_loop()
        self._bg_task = None

        self.command_queue = queue.Queue()

        self._lever_angle = 0  # in radians
        self._ball_position = np.array([0.0, 0.0])  # in meters
        self._ball_r = 0.0  # internal DOF: ball position along the lever
        self._ball_r_dot = 0.0  # internal DOF: ball velocity along the lever
        self._ball_r_max = 0.2
        self._lever_angle_max = 1.3
        self._lever_angle_min = -1.3
        self._friction_accel = 0.001
        self._start_time = 0
        self._current_time = 0
        self._previous_time = 0
        self._time_step = time_step
        self._report_period = report_period
        self._running = False
        self.data_ex = DataExchange(ring_buff_size=10000)

    def exec_cmd(self):
        if not self.command_queue.empty():
            command, value, cmd_time = self.command_queue.get()
            if command == "set_lever_angle":
                self.set_lever_angle(value)
            elif command == "right":
                self.set_lever_angle(self._lever_angle + 0.1)
            elif command == "left":
                self.set_lever_angle(self._lever_angle - 0.1)
            else:
                print(f"unrecognized command {command}")

    async def exec_cmd_loop(self):
        while self._running:
            self.exec_cmd()
            await asyncio.sleep(self._time_step)

    def set_lever_angle(self, angle):
        angle = min([angle, self._lever_angle_max])
        angle = max([angle, self._lever_angle_min])
        self._lever_angle = angle

    def get_velocity(self):
        return self._ball_r_dot

    def get_r_position(self):
        return self._ball_r

    def send_data(self):
        """send data to main thread (time, ball position, lever angle)
        """
        data_entry = np.array([self._current_time - self._start_time,
                               self._ball_position[0], self._ball_position[1],
                               self._lever_angle], dtype=float)
        self.data_ex.set_data(data_entry)

    async def send_data_loop(self):
        while self._running:
            self.send_data()
            await asyncio.sleep(self._time_step)

    def update_ball_position(self):
        """update ball position to current time using time integration

        To do: add effect of centrifugal force when lever_angle changes
        """
        dt = self._current_time - self._previous_time
        accel = -9.8 * np.sin(self._lever_angle) - self._friction_accel * np.sign(self._ball_r_dot)
        self._ball_r_dot += accel * dt
        self._ball_r += self._ball_r_dot * dt
        if self._ball_r > self._ball_r_max:
            self._ball_r = self._ball_r_max
            self._ball_r_dot = -np.abs(self._ball_r_dot)
        if self._ball_r < -self._ball_r_max:
            self._ball_r = -self._ball_r_max
            self._ball_r_dot = np.abs(self._ball_r_dot)
        self._previous_time = self._current_time

        self._ball_position[0] = self._ball_r * np.cos(self._lever_angle)
        self._ball_position[1] = self._ball_r * np.sin(self._lever_angle)
        # time.sleep(0.0012)

    async def update_ball_position_loop(self):
        while self._running:
            self.update_ball_position()
            await asyncio.sleep(self._time_step)

    async def main_loop(self):
        self._current_time = self._start_time = time.time()
        self._previous_time = self._start_time

        tasks = [
            asyncio.create_task(self.update_ball_position_loop()),
            asyncio.create_task(self.exec_cmd_loop()),
            asyncio.create_task(self.send_data_loop())
        ]

        try:
            while self._running:
                self._current_time = time.time()
                await asyncio.sleep(self._time_step)  # main clock tick
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    def run(self):
        print("robot thread started")
        self._running = True
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self.main_loop())
        finally:
            self._loop.close()

    def stop(self):
        self._running = False
