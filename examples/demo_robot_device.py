import os, sys
my_paths = ['../src', '../src/hardware']
[sys.path.append(os.path.abspath(path)) for path in my_paths if not path in sys.path]

from robot_controller import RobotController
from robot_device import RobotDevice

robot_device = RobotDevice()
controller = RobotController(robot_device)

robot_device.start()

