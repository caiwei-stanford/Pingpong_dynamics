import cv2
import numpy as np

from robot_device import RobotDevice


class RobotController:
    def __init__(self, robot_device):
        self.device = robot_device
        self.Kp = 0.00025
        self.Kd = 0.0006
        self.balance_enabled = False

    def control_step(self, target_position):
        current = self.device.get_lever_angle()
        velocity = self.device.get_velocity()
        error = target_position - current
        correction = self.Kp * error - self.Kd * velocity
        new_lever_angle = current + correction
        self.device.command_queue.put(("set_lever_angle", new_lever_angle))


def detect_ball(frame):
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


def main():
    robot_device = RobotDevice()
    robot_device.start()
    controller = RobotController(robot_device)

    try:
        while True:
            if not robot_device.frame_queue.empty():
                frame = robot_device.frame_queue.get()
                center_x, center_y, contours = detect_ball(frame)

                if contours:
                    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
                if center_x and center_y:
                    cv2.circle(frame, (center_x, center_y), 8, (255, 0, 0), -1)

                frame_center = frame.shape[1] // 2
                if center_x is not None:
                    error = center_x - frame_center

                    # Control with dead zone of 10 pixels
                    if abs(error) > 10:
                        controller.control_step(center_x)

                cv2.imshow("Ball Balancer", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('e'):
                robot_device.command_queue.put(("right", None))
            elif key == ord('r'):
                robot_device.command_queue.put(("left", None))
            elif key == ord('w'):
                controller.balance_enabled = not controller.balance_enabled

    finally:
        robot_device.running = False
        robot_device.join()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
