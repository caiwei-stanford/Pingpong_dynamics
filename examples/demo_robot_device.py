import sys
import os

src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from robot_controller import RobotController
from hardware.robot_device import RobotDevice

robot_device = RobotDevice()
controller = RobotController(robot_device)

controller.run()