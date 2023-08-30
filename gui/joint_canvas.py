from dataclasses import dataclass
import tkinter as tk
from typing import Callable, List, Optional, Tuple, Union
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from math import atan2, cos, sin
from matplotlib.backend_bases import MouseEvent

class JointCanvas(FigureCanvasTkAgg):
    """
    Plot the robot arm in a 2D plot with labeled sliders to control the joint angles and lengths
    """

    def __init__(
        self,
        on_click: Callable[[Union['SingleClick', 'ClickAndDrag']], None],
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
        self.mpl_connect("motion_notify_event", self.__on_mouse_drag)
        # draw the canvas
        self.ax = self.figure.add_subplot(111)
        self.draw()
        # cache joint positions so they can be redrawn from outside the plot function
        self.__joint_positions: List[Tuple[float, float]] = []

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
        self.__joint_positions = joint_positions
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

    def __draw_target_crosshair(self, x: float, y: float):
        """
        Draw a cross at x,y
        """
        self.ax.plot(x, y, "rx")
        self.draw()

    def __draw_target_arrow(self, x: float, y: float, angle: float):
        """
        Draw an arrow at x,y pointing in the direction of angle
        """
        self.ax.arrow(
            x,
            y,
            0.5 * cos(angle),
            0.5 * sin(angle),
            head_width=0.1,
            head_length=0.1,
            fc="k",
            ec="k",
        )
        self.draw()

    __start_pos: Optional[Tuple[float, float]] = None
    def __on_mouse_press(self, event: MouseEvent):
        self.__start_pos = (event.xdata, event.ydata)

    def __on_mouse_release(self, event: MouseEvent):
        """
        If right click, clear the target position, otherwise emit a click event
        """
        # right click to clear the target position
        if event.button == 3:
           self.clear()
           self.__start_pos = None
           return
        #if not in plot area, return
        if (event.xdata is None) or (event.ydata is None):
            self.__start_pos = None
            return

        end_x, end_y = event.xdata, event.ydata
        start_x, start_y = self.__start_pos
        # if distance between start and end is small, emit a single click event
        if (start_x - end_x) ** 2 + (start_y - end_y) ** 2 < 0.1:
            #emit event so kinematics can be calculated
            self.on_click(SingleClick(x=event.xdata, y=event.ydata))
            # draw the target crosshair
            self.__draw_target_crosshair(event.xdata, event.ydata)
            self.__start_pos = None
            return
        # otherwise, emit a click and drag event
        else:
            #emit event so kinematics can be calculated
            angle = atan2(end_y - start_y, end_x - start_x)
            self.on_click(ClickAndDrag(x=start_x, y=start_y, angle=angle))
            # draw the target crosshair
            self.__draw_target_crosshair(start_x, start_y)
            self.__draw_target_arrow(start_x, start_y, angle)
        
        self.__start_pos = None

    def __on_mouse_drag(self, event: MouseEvent):
        if self.__start_pos is None:
            return
        if (event.xdata is None) or (event.ydata is None):
            return
        current_pos = (event.xdata, event.ydata)
        #draw a line from start to current position
        self.clear()
        self.plot_joints(self.__joint_positions)
        self.ax.plot(
            [self.__start_pos[0], current_pos[0]],
            [self.__start_pos[1], current_pos[1]],
            "r-",
        )
        self.draw()
        


@dataclass
class SingleClick:
    x: float
    y: float
@dataclass
class ClickAndDrag:
    x: float
    y: float
    angle: float