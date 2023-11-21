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
            try:
                fig = plt.figure(figsize=(12, 6))
                ax = [fig.add_subplot(1,2,1), fig.add_subplot(1,2,2)]
            except NameError: print('plt not defined'); return
        ax[0].clear()
        ax[0].plot(data[:,0], data[:,1], 'r-')
        ax[0].plot(data[:,0], data[:,2], 'm-')
        ax[0].plot(data[:,0], data[:,3], 'b-')
        ax[0].set_xlabel('time (s)')

        # To do: show animation
        ax[1].clear()
        draw_lever_radius = 0.25
        background_circle = plt.Circle((0, 0), draw_lever_radius, color='k', fill=False)
        ax[1].add_artist(background_circle)
        ax[1].set_aspect('equal')
        ax[1].set_xlim([-0.3, 0.3])
        ax[1].set_ylim([-0.3, 0.3])
        draw_angle = data[-1,3]
        draw_ball_position = np.array([data[-1,1], data[-1,2]])
        draw_ball_radius = 0.02
        draw_ball = plt.Circle(draw_ball_position, draw_ball_radius, color='r', fill=False)
        draw_lever = plt.Line2D([-draw_lever_radius*np.cos(draw_angle), draw_lever_radius*np.cos(draw_angle)],
                                [-draw_lever_radius*np.sin(draw_angle), draw_lever_radius*np.sin(draw_angle)], color='b')
        ax[1].add_artist(draw_ball)
        ax[1].add_artist(draw_lever)

        plt.draw()
        plt.show(block=block)
        plt.pause(pause_seconds)

    def run(self):
        try:
             fig = plt.figure(figsize=(12, 6))
             ax = [fig.add_subplot(1,2,1), fig.add_subplot(1,2,2)]
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

                # randomly change lever angle
                last_lever_angle = ring_buff[-1,3]
                self.robot.set_lever_angle(last_lever_angle + np.random.normal(0, 0.01, 1))

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
