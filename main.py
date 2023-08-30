from gui.gui import GUI
import tkinter as tk
from kinematics import *

#TODO general solution for inverse kinematics
#TODO inverse kinematics solution for pose as well as position

def main():
    root = tk.Tk()
    robot = Robot(
        components=[
            Joint(id="q1", type=JointType.REVOLUTE, angle=0.0),
            Link(id="a1", length=1.0),
            Joint(id="q2", type=JointType.REVOLUTE, angle=0.0),
            Link(id="a2", length=1.0),
        ]
    )
    GUI(robot, root)
    root.mainloop()


if __name__ == "__main__":
    main()
