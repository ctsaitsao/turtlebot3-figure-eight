#!/usr/bin/env python
""" Helper functions that perform calculations related to the trajectory and
    control of the turtlebot and 2R arm.
"""

from math import sin, cos, acos, atan2, sqrt, pi


class FigureEight:
    """ Responsible for all calculations related to feedforward figure eight
        trajectory generation. Provides the trajectory coordinates and
        derivatives at any given time for a set of parameters.
    """
    def __init__(self, width, height, period):
        self.__width = width
        self.__height = height
        self.__period = period
        self.x = 0
        self.y = 0
        self.xdot = 0
        self.ydot = 0
        self.xddot = 0
        self.yddot = 0

    def update_feedforward_traj(self, t):
        """ Updates feedforward coordinates for a current time.

            Args:
                t (float): current time
        """
        self.x = self.__width/2 * sin(2*pi*t/self.__period)
        self.y = self.__height/2 * sin(4*pi*t/self.__period)
        self.xdot = self.__width*pi/self.__period * cos(2*pi*t/self.__period)
        self.ydot = 2*self.__height*pi/self.__period \
            * cos(4*pi*t/self.__period)
        self.xddot = -2*self.__width*pi**2/self.__period**2 \
            * sin(2*pi*t/self.__period)
        self.yddot = -8*self.__height*pi**2/self.__period**2 \
            * sin(4*pi*t/self.__period)


def v(xdot, ydot):
    """ Forward velocity of turtlebot. """
    return sqrt(xdot**2 + ydot**2)


def w(xdot, ydot, xddot, yddot):
    """ Planar angular velocity of turtlebot. """
    return (yddot * xdot - ydot * xddot) / (xdot**2 + ydot**2)


def traj_2R(t, L1, L2, T):
    """ Calculates coords for a 2R arm trajectory that causes the end-effector
        to move back and forth at a constant y position.

    Args:
        t (float): current time
        L1 (float): length of first link
        L2 (float): length of second link
        T (float): period (duration) of trajectory

    Returns:
        x (float): x coordinate of end-effector
        y (float): y coordinate of end-effector
    """
    h = 2/3 * (L1 + L2)
    x = 0.9 * cos(2*pi*t/T * sqrt((L1 + L2)**2 - h**2))
    y = h

    return x, y


def IK_2R(x, y, L1, L2):
    """ Calculates inverse kinematics for a 2R robot.

        Uses lefty solution from Modern Robotics p. 188.

        Args:
            x (float): x position of end-effector
            y (float): y position of end-effector
            L1 (float): length of first link
            L2 (float): length of second link

        Returns:
            theta1 (float): angle of lower joint
            theta2 (float): angle of upper joint
    """
    alpha = acos((x**2 + y**2 + L1**2 - L2**2)
                 / (2 * L1 * sqrt(x**2 + y**2)))
    beta = acos((L1**2 + L2**2 - x**2 - y**2)
                / (2 * L1 * L1))
    gamma = atan2(y, x)

    theta1 = gamma - alpha
    theta2 = pi - beta

    return theta1, theta2
