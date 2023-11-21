#!/usr/bin/env python

import numpy as np
import time, threading
from robot_sim import RobotSimulator

class RobotOperator:
    """RobotOperator class

    Use sensor data and algorithm to control the robot
    """
    def __init__(self, robot):
        self.robot = robot
        pass

    def run(self):
        try:
            self.robot.start()
            while True:
                # obtain data from robot
                ring_buff, image = self.robot.data_ex.get_data()
                print("robot running...    [ctrl-c to stop] time = %f ball_position = (%f,%f) lever_angle = %f"
                       % (ring_buff[-1,0], ring_buff[-1,1], ring_buff[-1,2], ring_buff[-1,3]))
                
                # To do: put control algorithm here

                time.sleep(0.5)
        
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            print("Stop robot")
            self.robot.stop()


def main():
    robot = RobotSimulator(time_step=0.01, report_period=0.2)
    robot_operator = RobotOperator(robot)

    robot_operator.run()

if __name__ == '__main__':
    main()
