import sys
import os

src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from robot_controller import RobotController
from sim.robot_sim import RobotSimulator

robot_device = RobotSimulator(time_step=0.05, report_period=0.2)
controller = RobotController(robot_device)

controller.run_sim()

