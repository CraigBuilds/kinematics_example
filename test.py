import math
from main import *
import unittest


class TestRobotFunctions(unittest.TestCase):
    def setUp(self):
        self.link1 = Link(id="l1", length=1.0)
        self.link2 = Link(id="l2", length=1.0)
        self.joint1 = Joint(id="q1", type=JointType.REVOLUTE, angle=0.0)
        self.joint2 = Joint(id="q2", type=JointType.REVOLUTE, angle=0.0)
        self.robot = Robot(
            components=[self.joint1, self.link1, self.joint2, self.link2]
        )

    def test_forward_kinematics(self):
        positions = forward_kinematics(self.robot)
        self.assertEqual(
            positions, [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        )  # Both joints have an angle of 0, so we just go right on the x-axis.

        self.joint1.angle = 90 * math.pi / 180
        self.joint2.angle = 90 * math.pi / 180
        positions = [
            (round(x, 2), round(y, 2)) for x, y in forward_kinematics(self.robot)
        ]
        self.assertEqual(
            positions, [(0.0, 0.0), (0.0, 1.0), (0.0, 2.0)]
        )  # Both joints have an angle of 90, so we just go up on the y-axis.

    def test_inverse_kinematics_unreachable(self):
        # Target outside the reach of the robot (links are length 1, so max reach is (2,2))
        solutions = inverse_kinematics([self.link1, self.link2], (3.5, 0))
        self.assertEqual(solutions, {})

    def test_inverse_kinematics(self):
        target_pos = (1.0, 0)
        solutions = inverse_kinematics([self.link1, self.link2], target_pos)
        self.assertTrue("elbow_up" in solutions)
        self.assertTrue("elbow_down" in solutions)
        # apply forward kinematics to check if the solutions are correct
        elbow_up = solutions["elbow_up"]
        set_joint_angles(self.robot, elbow_up)
        positions = [
            (round(x, 2), round(y, 2)) for x, y in forward_kinematics(self.robot)
        ]
        self.assertEqual(positions, [(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)])

        elbow_down = solutions["elbow_down"]
        set_joint_angles(self.robot, elbow_down)
        positions = [
            (round(x, 2), round(y, 2)) for x, y in forward_kinematics(self.robot)
        ]
        self.assertEqual(positions, [(0.0, 0.0), (0.5, -1.0), (1.0, 0.0)])


if __name__ == "__main__":
    unittest.main()
