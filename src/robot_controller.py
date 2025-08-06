#import cv2
import numpy as np

from hardware.robot_device import RobotDevice


class RobotController:
    def __init__(self, robot_device):
        self.device = robot_device
        self.Kp = 0.00025
        self.Kd = 0.0006
        self.balance_enabled = False

    def control_step(self, error):
        pos_term = error * self.Kp
        vel_term = self.device.get_velocity() * self.Kd
        val = pos_term + vel_term
        val = np.clip(val, -1, 1)
        theta = 90 - np.degrees(np.arccos(val))
        new_lever_angle = 1550 + (theta / 360) * 4000
        self.send_cmd("set_lever_angle", new_lever_angle)

    def send_cmd(self, command, value):
        self.device.command_queue.put((command, value))


    @staticmethod
    def detect_ball(self, frame):
        import cv2
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

    def run(self):
        import cv2
        self.device.start()

        try:
            while True:
                if not self.device.frame_queue.empty():
                    frame = self.device.frame_queue.get()
                    center_x, center_y, contours = self.detect_ball(self, frame)

                    if contours:
                        cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
                    if center_x and center_y:
                        cv2.circle(frame, (center_x, center_y), 8, (255, 0, 0), -1)

                    frame_center = frame.shape[1] // 2
                    if self.balance_enabled and center_x is not None:
                        error = center_x - frame_center

                        # Control with dead zone of 10 pixels
                        if abs(error) > 10:
                            self.control_step(error)

                    cv2.imshow("Ball Balancer", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('e'):
                    self.send_cmd("right", None)
                elif key == ord('r'):
                    self.send_cmd("left", None)
                elif key == ord('w'):
                    self.balance_enabled = not self.balance_enabled

        finally:
            self.device.stop()
            self.device.join()
            cv2.destroyAllWindows()