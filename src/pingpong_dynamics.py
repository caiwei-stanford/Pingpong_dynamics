#!/usr/bin/env python

import numpy as np
import time, threading
from robot_sim import RobotSimulator
import matplotlib.pyplot as plt

class RobotOperator:
    """RobotOperator class

    Use sensor data and algorithm to control the robot
    """
    def __init__(self, robot):
        self.robot = robot
        pass

    def plot_data(self, data, fig=None, ax=None, block=False, pause_seconds=0.01):
        if fig==None:
            try: fig, ax = plt.subplots()
            except NameError: print('plt not defined'); return
        ax.clear()
        ax.plot(data[:,0], data[:,1], 'r-')
        ax.plot(data[:,0], data[:,2], 'm-')
        ax.plot(data[:,0], data[:,3], 'b-')
        ax.set_xlabel('time (s)')
        plt.draw()
        plt.show(block=block)
        plt.pause(pause_seconds)

    def run(self):
        try: fig, ax = plt.subplots()
        except NameError: print('plt not defined');

        try:
            self.robot.start()
            while True:
                # obtain data from robot
                ring_buff, image = self.robot.data_ex.get_data()
                print("robot running...    [ctrl-c to stop] time = %f ball_position = (%f,%f) lever_angle = %f"
                       % (ring_buff[-1,0], ring_buff[-1,1], ring_buff[-1,2], ring_buff[-1,3]))
                
                # Plot data
                self.plot_data(data=ring_buff[-100:,:], fig=fig, ax=ax)

                # To do: put control algorithm here

                time.sleep(0.5)
        
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            print("Stop robot")
            self.robot.stop()

            # for debugging purpose
            ring_buff, image = self.robot.data_ex.get_data()
            print("ring_buff = ", ring_buff[-20:, :])


def main():
    robot = RobotSimulator(time_step=0.05, report_period=0.2)
    robot_operator = RobotOperator(robot)

    robot_operator.run()

if __name__ == '__main__':
    main()
