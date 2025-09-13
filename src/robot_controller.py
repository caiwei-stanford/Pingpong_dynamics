import csv
import math
import random

import cv2
import time
import numpy as np
import matplotlib.pyplot as plt


class RobotController:
    def __init__(self, robot_device):

        # playback variables
        self.playback_enabled = False
        self.playback_records = None
        self.playback_start_time = None
        self.playback_index = None

        self.device = robot_device
        self.running = False

        #balancing variables
        self.Kp = 0.00025
        self.Kd = 0.0006
        self.balance_enabled = False
        self.position_array = np.zeros(4)

    def update_position(self, new_value):
        self.position_array = np.append(self.position_array, new_value)
        self.position_array = np.delete(self.position_array, 0)

    def get_velocity(self, dt=0.01):
        return (self.position_array[3] - self.position_array[0]) / 4

    def control_step(self, error):
        pos_term = error * self.Kp
        vel_term = self.get_velocity() * self.Kd
        val = pos_term + vel_term
        val = np.clip(val, -1, 1)
        theta = 90 - np.degrees(np.arccos(val))
        new_lever_angle = 1550 + (theta / 360) * 4000
        self.send_cmd("set_lever_angle", int(new_lever_angle), time.time())

    def send_cmd(self, command, value, cmd_time):
        self.device.command_queue.put((command, value, cmd_time))

    def reset(self):
        self.device.command_queue.put(("set_lever_angle", 1550, time.time()))

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

    def load_playback(self, filename):
        """Load CSV playback file into memory and reset state."""
        with open(filename, newline='') as f:
            reader = csv.reader(f)
            data = list(reader)

        records = []
        for row in data:
            try:
                t, cx, cy, angle = row
                records.append([float(t), float(cx), float(cy), float(angle)])
            except ValueError:
                continue  # skip header/malformed rows

        if not records:
            print("No valid playback data found.")
            return

        self.playback_records = records
        self.playback_start_time = time.time()
        self.playback_index = 0
        print(f"Playback loaded: {len(records)} samples")

    def playback_step(self):
        """Advance playback by one step if it's time."""
        if self.playback_index >= len(self.playback_records):
            return  # nothing left to play

        target_time, _, _, lever_angle_rad = self.playback_records[self.playback_index]
        now = time.time() - self.playback_start_time

        if now >= target_time:
            self.send_cmd("set_lever_angle", lever_angle_rad, time.time())
            self.playback_index += 1  # move to next

    @staticmethod
    def plot_data(self, data, fig=None, ax=None, block=False, pause_seconds=0.01):
        if fig is None:
            try:
                fig = plt.figure(figsize=(12, 6))
                ax = [fig.add_subplot(1, 2, 1), fig.add_subplot(1, 2, 2)]
            except NameError:
                print('plt not defined')
                return
        # plot data history as function of time
        ax[0].clear()
        ax[0].plot(data[:, 0], data[:, 1], 'r-')
        ax[0].plot(data[:, 0], data[:, 2], 'm-')
        ax[0].plot(data[:, 0], data[:, 3], 'b-')
        ax[0].set_xlabel('time (s)')

        # show animation of the ball on lever
        ax[1].clear()
        draw_lever_radius = 0.22
        background_circle = plt.Circle((0, 0), draw_lever_radius, color='k', fill=False)
        ax[1].add_artist(background_circle)
        ax[1].set_aspect('equal')
        ax[1].set_xlim([-0.25, 0.25])
        ax[1].set_ylim([-0.25, 0.25])
        ax[1].set_xlabel('x (m)')
        ax[1].set_ylabel('y (m)')
        draw_angle = data[-1, 3]
        draw_ball_position = np.array([data[-1, 1], data[-1, 2]])
        draw_ball_radius = 0.02
        draw_ball = plt.Circle(draw_ball_position, draw_ball_radius, color='r', fill=False)
        draw_lever = plt.Line2D([-draw_lever_radius * np.cos(draw_angle), draw_lever_radius * np.cos(draw_angle)],
                                [-draw_lever_radius * np.sin(draw_angle), draw_lever_radius * np.sin(draw_angle)],
                                color='b')
        ax[1].add_artist(draw_ball)
        ax[1].add_artist(draw_lever)

        plt.draw()
        plt.show(block=block)
        plt.pause(pause_seconds)

    def run_sim(self):
        try:
            fig = plt.figure(figsize=(12, 6))
            ax = [fig.add_subplot(1, 2, 1), fig.add_subplot(1, 2, 2)]
        except NameError:
            print('plt not defined')

        try:
            self.device.start()

            self.load_playback("session_data.csv")

            cv2.namedWindow("hidden", cv2.WINDOW_NORMAL)
            cv2.moveWindow("hidden", -1000, -1000)  # Move it off-screen
            while True:
                # obtain data from robot
                ring_buff, image = self.device.data_ex.get_data()
                print("robot running...    [ctrl-c to stop] time = %f ball_position = (%f,%f) lever_angle = %f"
                      % (ring_buff[-1, 0], ring_buff[-1, 1], ring_buff[-1, 2], ring_buff[-1, 3]))

                # Plot data
                self.plot_data(self, data=ring_buff[-100:, :], fig=fig, ax=ax)

                if self.balance_enabled:
                    pass

                if self.playback_enabled:
                    self.playback_step()

                # ToDo: put control algorithm here
                key = cv2.waitKey(1) & 0xFF
                self.key_pressed(key)
        except KeyboardInterrupt:
            print("Stop robot")
            self.device.stop()
            cv2.destroyAllWindows()
            self.device.join()

    def run_device(self):
        self.device.start()
        self.running = True

        filename = "session_data2.csv"

        # time controls
        sample_interval = 0.02
        start_time = time.time()
        next_sample_time = start_time
        duration = 300
        records = []

        # lever controls
        last_move_time = start_time
        hold_time = random.uniform(0.0, 0.05)
        current_angle = 1550

        print(f"Starting session: {duration}s, logging to {filename}")

        try:
            self.reset()

            while self.running and (time.time() - start_time < duration):
                # frame control
                frame, frame_time = self.device.frame_queue.get(block=True)
                center_x, center_y, contours = self.detect_ball(self, frame)
                self.update_position(center_x)

                # annotate frame
                if contours:
                    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
                if center_x and center_y:
                    cv2.circle(frame, (center_x, center_y), 8, (255, 0, 0), -1)

                # random lever control
                now = time.time()
                if now - last_move_time > hold_time:
                    step = random.uniform(40, 70)
                    direction = random.choice([-1, 1])
                    current_angle = current_angle + (step * direction)
                    current_angle = np.clip(current_angle, 770, 2350)

                    self.send_cmd("set_lever_angle", current_angle, time.time())

                    last_move_time = now
                    hold_time = random.uniform(0.02, 0.03)

                # logging control
                if now >= next_sample_time:
                    lever_angle_rad = ((self.device.get_lever_angle() - 1550) / 4000) * 2 * np.pi
                    print(f"time: {now - start_time}, center: {center_x, center_y}, angle: {lever_angle_rad}")
                    records.append([now - start_time, center_x, center_y, lever_angle_rad])
                    next_sample_time += sample_interval

                cv2.imshow("Ball Balancer", frame)

                key = cv2.waitKey(1) & 0xFF
                self.key_pressed(key)

            # save file data
            np.savetxt(filename, np.array(records),
                       delimiter=", ", header="time, ball_x, ball_y, lever_angle", comments="")
            print(f"Session finished, saved {len(records)} samples to {filename}")

            # print("robot running...    time = %f ball_position = (%f,%f) velocity = %f"
            #       % (time.time() - start_time, center_x, center_y, self.device.get_velocity()))
            #
            # if self.balance_enabled and center_x is not None:
            #     error = center_x - (frame.shape[1] // 2)
            #
            #     # Control with dead zone of 10 pixels
            #     if abs(error) < 10 and abs(self.device.get_velocity()) < 1:
            #         self.balance_enabled = False
            #         print("balance success")
            #         continue
            #
            #     self.control_step(error)
        finally:
            self.device.stop()
            cv2.destroyAllWindows()

    def key_pressed(self, key):
        if key == ord('q'):
            self.balance_enabled = False
            self.running = False
        elif key == ord('e'):
            self.send_cmd("right", None, time.time())
        elif key == ord('r'):
            self.send_cmd("left", None, time.time())
        elif key == ord('w'):
            self.balance_enabled = not self.balance_enabled
        elif key == ord("p"):
            self.playback_enabled = not self.playback_enabled
