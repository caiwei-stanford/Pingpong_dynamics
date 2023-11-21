"""RobotSimulator class
"""

import numpy as np
import time, threading
from data_exch import DataExchange

class RobotSimulator(threading.Thread):
    """RobotSimulator class
    """
    def __init__(self, time_step=0.01, report_period=0.2, ring_buff_size=10000):
        threading.Thread.__init__(self)
        self._lever_angle = 0                       # in radians
        self._ball_position = np.array([0.0, 0.0])  # in meters
        self._ball_r = 0.0        # internal DOF: ball position along the lever
        self._ball_r_dot = 0.0    # internal DOF: ball velocity along the lever
        self._ball_r_max = 0.2
        self._lever_angle_max = 1.3
        self._lever_angle_min = -1.3
        self._start_time = 0
        self._current_time = 0
        self._time_step = time_step
        self._report_period = report_period
        self._running = False
        self.data_ex = DataExchange(ring_buff_size)
    
    def set_lever_angle(self, angle):
        self._lever_angle = angle
        self._lever_angle = min([self._lever_angle, self._lever_angle_max])
        self._lever_angle = max([self._lever_angle, self._lever_angle_min])

    def send_report(self):
        """send data to main thread (time, ball position, lever angle)
        """
        data_entry = np.array([self._current_time-self._start_time,
                               self._ball_position[0], self._ball_position[1],
                               self._lever_angle], dtype=np.float)
        self.data_ex.set_data(data_entry)

    def update_ball_position(self):
        """update ball position to current time using time integration

        To do: replace this with a physics simulation
        """
        self._ball_r += np.random.normal(0, 0.01, 1)
        self._ball_r = min([self._ball_r, self._ball_r_max])
        self._ball_r = max([self._ball_r, -self._ball_r_max])

        self._ball_position[0] = self._ball_r * np.cos(self._lever_angle)
        self._ball_position[1] = self._ball_r * np.sin(self._lever_angle)
        time.sleep(0.0012)

    def run(self):
        """Simulate the physics of the robot, run on a separate thread

        Periodically send ball position to main thread
        Update lever_angle from main thread
        """
        print("robot thread started")
        self._running = True
        cycle_num = 0
        self._current_time = self._start_time = time.time()
        self.send_report()
        while self._running:
            self._current_time = time.time()
            cycle_num += 1

            self.update_ball_position()

            if (cycle_num % int(self._report_period / self._time_step)) == 0:
                self.send_report()

            remaining_time = self._time_step * cycle_num - (time.time() - self._start_time)
            if remaining_time > 0:
                time.sleep(remaining_time)
            else:
                print("Warning: cycle time too short")

    def stop(self):
        self._running = False
