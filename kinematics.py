from math import sin, cos, atan2, acos, pi
from typing import Dict, List, Tuple, Union
from enum import Enum
from dataclasses import dataclass


class JointType(Enum):
    REVOLUTE = 1


@dataclass
class Joint:
    id: str
    type: JointType
    angle: float


@dataclass
class Link:
    id: str
    length: float


@dataclass
class Robot:
    base = (0.0, 0.0)
    components: List[Union[Joint, Link]]


def forward_kinematics(robot: Robot) -> List[Tuple[float, float]]:
    """
    Calculate the positions of the joints of the robot
    """
    x, y = robot.base
    joint_positions = [(x, y)]
    direction = 0.0
    for component in robot.components:
        if isinstance(component, Joint):
            # set the direction of the next link
            direction += component.angle
        elif isinstance(component, Link):
            # move in the direction of the link
            x += component.length * cos(direction)
            y += component.length * sin(direction)
        joint_positions.append((x, y))
    return joint_positions


def inverse_kinematics_2dof(
    links: List[Link], target_end_effector_pos: Tuple[float, float]
) -> Dict[str, Dict[str, Joint]]:
    """
    Find the joint angles that put the Robot end effector at the target position
    For now, this function assumes a two-link robot (2DOF).
    The target_end_effector_pos angle is unconstrained here.
    """
    L1 = links[0].length
    L2 = links[1].length
    x, y = target_end_effector_pos
    d2 = x ** 2 + y ** 2
    d = d2 ** 0.5

    base_angle = atan2(y, x)

    # C:= cos of the first angle in a triangle defined by L1 and L2
    # the formula comes from the Law of Cosines
    C = (L1 ** 2 + d2 - L2 ** 2) / (2 * L1 * d)

    # Check if the target is reachable.
    # If C > 1, it means the target is outside the workspace of the robot
    # If C < -1, it means the target is inside the workspace, but not reachable
    if not -1 <= C <= 1:
        # The target is not reachable
        return {}

    # B:= cos of the second angle
    B = (L1 ** 2 + L2 ** 2 - d2) / (2 * L1 * L2)
    c = acos(C)
    b = acos(B)

    return {
        "elbow_up": {
            "q1": Joint(id="q1", type=JointType.REVOLUTE, angle=base_angle + c),
            "q2": Joint(id="q2", type=JointType.REVOLUTE, angle=b - pi),
        },
        "elbow_down": {
            "q1": Joint(id="q1", type=JointType.REVOLUTE, angle=base_angle - c),
            "q2": Joint(id="q2", type=JointType.REVOLUTE, angle=pi - b),
        },
    }

def set_joint_angles(robot: Robot, new_joint_angles: Dict[str, Joint]):
    """
    Set the joint angles of the robot
    """
    for joint in [
        component for component in robot.components if isinstance(component, Joint)
    ]:
        if joint.id in new_joint_angles:
            joint.angle = new_joint_angles[joint.id].angle
