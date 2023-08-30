from kinematics import *
import tkinter as tk
from typing import Union
from copy import deepcopy
from gui.joint_canvas import JointCanvas, SingleClick, ClickAndDrag
from gui.sliders import Sliders


class GUI(tk.Frame):
    """
    Plot the robot arm in a 2D plot with labeled sliders to control the joint angles and lengths
    """

    def __init__(self, robot: Robot, master: tk.Tk):
        super().__init__(master)
        self.robot = robot
        self.joint_canvas = JointCanvas(
            on_click=self.on_click,
            master=self,
        )
        self.sliders = Sliders(
            robot=self.robot,
            update=self.update,
            master=self.master,
        )
        self.update()

    def update(self):
        """
        This function is called whenever the sliders are moved or the canvas is clicked
        """
        self.joint_canvas.clear()
        # calculate the joint positions
        joint_positions = forward_kinematics(self.robot)
        # plot the robot
        self.joint_canvas.plot_joints(joint_positions)
        # print the end effector position
        self.joint_canvas.print_end_effector_pose(joint_positions)

    def on_click(self, click: Union[SingleClick, ClickAndDrag]):
        """
        This function is called whenever the canvas is clicked
        """
        self.joint_canvas.clear()
        self.update()
        # calculate the inverse kinematics
        solutions = inverse_kinematics_2dof(
            links = [link for link in self.robot.components if isinstance(link, Link)],
            target_end_effector_pos=(click.x, click.y),
        )
        for solution in solutions.values():
            robot_copy = deepcopy(self.robot)
            set_joint_angles(robot_copy, solution)
            joint_positions = forward_kinematics(robot_copy)
            self.joint_canvas.plot_joints(joint_positions, style="o--")