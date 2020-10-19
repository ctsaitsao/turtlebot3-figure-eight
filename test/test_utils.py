""" Unit testing homework2 utils functions. """

import unittest
from homework2.utils import FigureEight, v, w, traj_2R, IK_2R


class TestUtils(unittest.TestCase):
    def __init__(self, *args):
        super(TestUtils, self).__init__(*args)

        self.__width = 3
        self.__height = 2
        self.__period = 5
        self.__fig_eight = FigureEight(self.__width, self.__height,
                                       self.__period)
        self.__L1 = 3
        self.__L2 = 4
        self.__r = 0.5

    def test_FigureEight(self):
        t = 0
        self.__fig_eight.update_feedforward_traj(t)

        self.assertAlmostEqual(self.__fig_eight.x, 0, places=4)
        self.assertAlmostEqual(self.__fig_eight.y, 0, places=4)
        self.assertAlmostEqual(self.__fig_eight.xdot, 1.8850, places=4)
        self.assertAlmostEqual(self.__fig_eight.ydot, 2.5133, places=4)
        self.assertAlmostEqual(self.__fig_eight.xddot, 0, places=4)
        self.assertAlmostEqual(self.__fig_eight.yddot, 0, places=4)

        t = 3
        self.__fig_eight.update_feedforward_traj(t)

        self.assertAlmostEqual(self.__fig_eight.x, -0.8817, places=4)
        self.assertAlmostEqual(self.__fig_eight.y, 0.9511, places=4)
        self.assertAlmostEqual(self.__fig_eight.xdot, -1.5250, places=4)
        self.assertAlmostEqual(self.__fig_eight.ydot, 0.7766, places=4)
        self.assertAlmostEqual(self.__fig_eight.xddot, 1.3923, places=4)
        self.assertAlmostEqual(self.__fig_eight.yddot, -6.0074, places=4)

    def test_v(self):
        t = 0
        self.__fig_eight.update_feedforward_traj(t)

        self.assertAlmostEqual(v(self.__fig_eight.xdot, self.__fig_eight.ydot),
                               3.1416, places=4)

        t = 3
        self.__fig_eight.update_feedforward_traj(t)

        self.assertAlmostEqual(v(self.__fig_eight.xdot, self.__fig_eight.ydot),
                               1.7113, places=4)

    def test_w(self):
        t = 0
        self.__fig_eight.update_feedforward_traj(t)

        self.assertAlmostEqual(w(self.__fig_eight.xdot,
                                 self.__fig_eight.ydot,
                                 self.__fig_eight.xddot,
                                 self.__fig_eight.yddot),
                               0, places=4)

        t = 3
        self.__fig_eight.update_feedforward_traj(t)

        self.assertAlmostEqual(w(self.__fig_eight.xdot,
                                 self.__fig_eight.ydot,
                                 self.__fig_eight.xddot,
                                 self.__fig_eight.yddot),
                               2.7588, places=4)

    def test_traj_2R(self):
        t = 0
        x, y = traj_2R(t, self.__L1, self.__L2, self.__period)

        self.assertAlmostEqual(x, 0.9, places=4)
        self.assertAlmostEqual(y, 4.6667, places=4)

        t = 3
        x, y = traj_2R(t, self.__L1, self.__L2, self.__period)

        self.assertAlmostEqual(x, 0.6140, places=4)
        self.assertAlmostEqual(y, 4.6667, places=4)

    def test_IK_2R(self):
        x = 0.9
        y = 4.6667
        theta1, theta2 = IK_2R(x, y, self.__L1, self.__L2)

        self.assertAlmostEqual(theta1, 0.3878, places=4)
        self.assertAlmostEqual(theta2, 1.7052, places=4)

        x = 0.6140
        y = 4.6667
        theta1, theta2 = IK_2R(x, y, self.__L1, self.__L2)

        self.assertAlmostEqual(theta1, 0.4356, places=4)
        self.assertAlmostEqual(theta2, 1.7295, places=4)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(homework2, 'test_utils', TestUtils)
