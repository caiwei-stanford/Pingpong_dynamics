import math
import cv2
import time
import numpy as np
import matplotlib.pyplot as plt


class RobotController:
    def __init__(self, robot_device):
        self.device = robot_device
        self.Kp = 0.00025
        self.Kd = 0.0006
        self.balance_enabled = False
        self.running = False

    def control_step(self, error):
        pos_term = error * self.Kp
        vel_term = self.device.get_velocity() * self.Kd
        val = pos_term + vel_term
        val = np.clip(val, -1, 1)
        theta = 90 - np.degrees(np.arccos(val))
        new_lever_angle = 1550 + (theta / 360) * 4000
        self.send_cmd("set_lever_angle", int(new_lever_angle), time.time())

    def send_cmd(self, command, value, cmd_time):
        self.device.command_queue.put((command, value, cmd_time))

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
                    dist = self.device.get_r_position()
                    if abs(dist) == 0 and abs(self.device.get_velocity()) < 1:
                        self.balance_enabled = False
                        print("balance success")
                        continue
                    self.control_step(dist)

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
        start_time = time.time()
        try:
            while self.running:
                frame, frame_time = self.device.frame_queue.get(block=True)
                center_x, center_y, contours = self.detect_ball(self, frame)
                self.device.update_position(center_x)

                if contours:
                    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
                if center_x and center_y:
                    cv2.circle(frame, (center_x, center_y), 8, (255, 0, 0), -1)

                print("robot running...    time = %f ball_position = (%f,%f) velocity = %f"
                      % (time.time() - start_time, center_x, center_y, self.device.get_velocity()))

                if self.balance_enabled and center_x is not None:
                    error = math.sqrt(center_x ** 2 + center_y ** 2) - (frame.shape[1] // 2)

                    # Control with dead zone of 10 pixels
                    if abs(error) < 10 and abs(self.device.get_velocity()) < 1:
                        self.balance_enabled = False
                        print("balance success")
                        continue

                    self.control_step(error)

                cv2.imshow("Ball Balancer", frame)

                key = cv2.waitKey(1) & 0xFF
                self.key_pressed(key)

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
