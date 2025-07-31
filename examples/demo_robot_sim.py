import os, sys
my_paths = ['../src', '../src/sim']
[sys.path.append(os.path.abspath(path)) for path in my_paths if not path in sys.path]

from robot_controller import RobotController
from robot_sim import RobotSimulator

robot_device = RobotSimulator(time_step=0.05, report_period=0.2)
controller = RobotController(robot_device)

# ToDo: rather do this through controller
#robot_device.start()
#controller.start_robot()

# ToDo: send command through controller
#controller.send_cmd(("set_lever_position", 0.1))

