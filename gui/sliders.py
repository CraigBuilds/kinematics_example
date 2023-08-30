import tkinter as tk
from typing import Dict, Callable
from kinematics import Robot, Joint, Link

class Sliders:
    def __init__(
        self,
        robot: Robot,
        update: Callable[[], None],
        master: tk.Frame
    ):
        self.update = update
        self.master = master
        self.robot = robot
        self.sliders: Dict[str, tk.Scale] = {}
        """
        create sliders for each joint, and set the callback to on_slider_change
        """
        for component in self.robot.components:
            if isinstance(component, Joint):
                slider = tk.Scale(
                    self.master,
                    from_=-3.14,
                    to=3.14,
                    resolution=0.001,
                    takefocus=1,
                    highlightthickness=1.0,
                    length=300,
                    orient=tk.HORIZONTAL,
                    label=component.id,
                    command=self.on_slider_change,
                )
                slider.pack()
                self.sliders[component.id] = slider

    """
    Update the robot components with the slider values and call the update function
    """
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
