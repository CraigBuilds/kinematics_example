from math import sin, cos, atan2, sqrt, acos, pi
from typing import Dict, List, Optional, Tuple, Union
from enum import Enum
from matplotlib.figure import Figure # type: ignore
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg # type: ignore
from matplotlib.backend_bases import MouseEvent # type: ignore
from dataclasses import dataclass
from copy import deepcopy
import tkinter as tk

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
    angle = 0.0
    for component in robot.components:
        if isinstance(component, Joint):
            angle += component.angle
        elif isinstance(component, Link):
            x += component.length * cos(angle)
            y += component.length * sin(angle)
        joint_positions.append((x, y))
    return joint_positions

def inverse_kinematics(links: List[Link], target_end_effector_pos: Tuple[float, float]) -> Dict[str, Dict[str, Joint]]:
    """
    Find the joint angles that put the Robot end effector at the target position
    For now, this function assumes a two-link robot (2DOF).
    """
    L1 = links[0].length
    L2 = links[1].length
    x, y = target_end_effector_pos
    d2 = x**2 + y**2
    d = d2**0.5

    base_angle = atan2(y, x)

    # C:= cos of the first angle in a triangle defined by L1 and L2
    # the formula comes from the Law of Cosines
    C = (L1**2 + d2 - L2**2) / (2*L1*d)

    # Check if the target is reachable.
    # If C > 1, it means the target is outside the workspace of the robot
    # If C < -1, it means the target is inside the workspace, but not reachable
    if not -1 <= C <= 1:
        # The target is not reachable
        return {}

    # B:= cos of the second angle
    B = (L1**2 + L2**2 - d2) / (2*L1*L2)
    c = acos(C)
    b = acos(B)

    # consider returning only one if c is close to 0
    return {
        "elbow_up": {
            "q1": Joint(id="q1", type=JointType.REVOLUTE, angle=base_angle+c), 
            "q2": Joint(id="q2", type=JointType.REVOLUTE, angle=b-pi)
        },
        "elbow_down": {
            "q1": Joint(id="q1", type=JointType.REVOLUTE, angle=base_angle-c), 
            "q2": Joint(id="q2", type=JointType.REVOLUTE, angle=pi-b)
        }
    }


def set_joint_angles(robot: Robot, new_joint_angles: Dict[str, Joint]):
    """
    Set the joint angles of the robot
    """
    for joint in [component for component in robot.components if isinstance(component, Joint)]:
        if joint.id in new_joint_angles:
            joint.angle = new_joint_angles[joint.id].angle


class GUI(tk.Frame):
    """
    Plot the robot arm in a 2D plot with labeled sliders to control the joint angles and lengths
    """
    def __init__(self, master: tk.Tk):
        super().__init__(master)
        self.robot = Robot(
            components=[
                Joint(id="q1", type=JointType.REVOLUTE, angle=0.0),
                Link(id="a1", length=1.0),
                Joint(id="q2", type=JointType.REVOLUTE, angle=0.0),
                Link(id="a2", length=1.0),
            ]
        )
        self.inverse_kinematic_solutions: Dict[str, Robot] = {}
        self.canvas = FigureCanvasTkAgg(Figure(figsize=(5, 4), dpi=100), master=master)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self.target_pos: Optional[Tuple[float, float]] = None
        self.canvas.mpl_connect('button_press_event', self.on_click)
        self.sliders: Dict[str, tk.Scale] = {}
        self.create_labeled_sliders()
        self.update()

    def update(self):
        self.canvas.figure.clear()
        ax = self.canvas.figure.add_subplot(111)
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        ax.set_aspect('equal')
        joint_positions = forward_kinematics(self.robot)
        #plot the robot
        for i in range(len(joint_positions) - 1):
            ax.plot([joint_positions[i][0], joint_positions[i+1][0]], [joint_positions[i][1], joint_positions[i+1][1]], 'o-')
        #print the end effector position
        end_effector_pos = joint_positions[-1]
        ax.text(end_effector_pos[0], end_effector_pos[1], f"({round(end_effector_pos[0], 2)}, {round(end_effector_pos[1], 2)})")
        #if clicked on the plot, plot the target position and the possible solutions
        if self.target_pos is not None:
            #plot end effector target
            ax.plot(self.target_pos[0], self.target_pos[1], 'rx')
            #calculate the possible solutions
            links = [component for component in self.robot.components if isinstance(component, Link)]
            solutions = inverse_kinematics(links, self.target_pos)
            #plot the possible solutions
            for name, solution in solutions.items():
                self.inverse_kinematic_solutions[name] = deepcopy(self.robot)
                set_joint_angles(self.inverse_kinematic_solutions[name], solution)
                joint_positions = forward_kinematics(self.inverse_kinematic_solutions[name])
                for i in range(len(joint_positions) - 1):
                    ax.plot([joint_positions[i][0], joint_positions[i+1][0]], [joint_positions[i][1], joint_positions[i+1][1]], 'x--')

        self.canvas.draw()

    def create_labeled_sliders(self):
        for component in self.robot.components:
            if isinstance(component, Joint):
                slider = tk.Scale(self.master, from_=-3.14, to=3.14, resolution=0.001, takefocus=1, highlightthickness=1.0, length=300, orient=tk.HORIZONTAL, label=component.id, command=self.on_slider_change)
                slider.pack()
                self.sliders[component.id] = slider

    def on_slider_change(self, event):
        for component in self.robot.components:
            slider = self.sliders.get(component.id)
            if slider is None:
                continue
            if isinstance(component, Joint):
                component.angle = slider.get()
            elif isinstance(component, Link):
                component.length = slider.get()
        self.update()

    def on_click(self, event: MouseEvent):
        if event.button == 3:
            self.target_pos = None
        elif (event.xdata is not None) and (event.ydata is not None):
            self.target_pos = (event.xdata, event.ydata)
        self.update()


if __name__ == "__main__":
    root = tk.Tk()
    app = GUI(master=root)
    app.mainloop()