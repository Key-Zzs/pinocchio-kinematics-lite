import pinocchio
import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz
from placo_utils.tf import tf

"""
7axis robot reaching a given target (introduction example).
"""

# Loading the robot
import os
urdf_path = os.path.expanduser("~/pyAgxArm/asserts/agx_arm_urdf-main/nero/urdf/nero_description.urdf")
robot = placo.RobotWrapper(urdf_path, placo.Flags.ignore_collisions)

# Creating the solver
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)

effector_task = solver.add_frame_task("link7", np.eye(4))
effector_task.configure("effector", "soft", 1.0, 1.0)

viz = robot_viz(robot)

t = 0
dt = 0.01
solver.dt = dt


@schedule(interval=dt)
def loop():
    global t
    t += dt

    # Updating the target
    effector_task.T_world_frame = tf.translation_matrix([1.5, np.sin(t), 1.0])

    # Solving the IK
    solver.solve(True)
    robot.update_kinematics()

    # Displaying the robot, effector and target
    viz.display(robot.state.q)
    robot_frame_viz(robot, "link7")
    frame_viz("target", effector_task.T_world_frame)


run_loop()
