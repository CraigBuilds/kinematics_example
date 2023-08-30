from dataclasses import dataclass
import tkinter as tk
from tracemalloc import start
from typing import Callable, List, Tuple, Union
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from math import atan2
from matplotlib.backend_bases import MouseEvent

class JointCanvas(FigureCanvasTkAgg):
    """
    Plot the robot arm in a 2D plot with labeled sliders to control the joint angles and lengths
    """

    def __init__(
        self,
        on_click: Callable[[Union['SingleClick', 'ClickAndDragStart', 'ClickAndDragEnd']], None],
        master: tk.Frame,
    ):
        """
        Create a canvas with one axis subplot
        """
        self.on_click = on_click
        super().__init__(Figure(figsize=(5, 5), dpi=100))
        # pack inside the master frame
        self_as_tk = self.get_tk_widget()
        self_as_tk.pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        # add event listeners
        self.mpl_connect("button_press_event", self.__on_mouse_press)
        self.mpl_connect("button_release_event", self.__on_mouse_release)
        # draw the canvas
        self.ax = self.figure.add_subplot(111)
        self.draw()

    def clear(self):
        """
        Clear the canvas subplot
        """
        self.ax.clear()
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.ax.set_aspect("equal")

    def plot_joints(self, joint_positions: List[Tuple[float, float]], style: str = "o-"):
        """
        Plot the joint positions on the canvas, joining them with lines. The points are drawn in order of the list.
        """
        for i in range(len(joint_positions) - 1):
            self.ax.plot(
                [joint_positions[i][0], joint_positions[i + 1][0]],
                [joint_positions[i][1], joint_positions[i + 1][1]],
                style,
            )
        self.draw()

    def print_end_effector_pose(self, joint_positions: List[Tuple[float, float]]):
        """
        Write the end effector position and angle on the canvas
        """
        end_effector_pos = joint_positions[-1]
        prior_joint = joint_positions[-2]
        end_effector_angle = atan2(
            end_effector_pos[1] - prior_joint[1], end_effector_pos[0] - prior_joint[0]
        )
        self.ax.text(
            end_effector_pos[0],
            end_effector_pos[1],
            f"({round(end_effector_pos[0], 2)},{round(end_effector_pos[1], 2)}, {round(end_effector_angle, 2)})",
        )
        self.draw()

    def draw_target_crosshair(self, x: float, y: float):
        """
        Draw a cross at x,y
        """
        self.ax.plot(x, y, "rx")
        self.draw()

    def __on_mouse_release(self, event: MouseEvent):
        #TODO
        ...

    def __on_mouse_press(self, event: MouseEvent):
        """
        If right click, clear the target position, otherwise emit a click event
        """
        # right click to clear the target position
        if event.button == 3:
           self.clear()
           return
        is_within_plot = (event.xdata is not None) and (event.ydata is not None)
        if is_within_plot:
            self.on_click(SingleClick(x=event.xdata, y=event.ydata))
        #TODO emit click and drag event so we can set the target pose, not just the target position

@dataclass
class SingleClick:
    x: float
    y: float
@dataclass
class ClickAndDragStart:
    x: float
    y: float
@dataclass
class ClickAndDragEnd:
    start_x: float
    start_y: float
    end_x: float
    end_y: float