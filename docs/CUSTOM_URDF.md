# Custom URDF Usage

`PinocchioKinematics` is the robot-agnostic entry point.

```python
from pinocchio_kinematics_lite import PinocchioKinematics

kin = PinocchioKinematics(
    urdf_path="path/to/robot.urdf",
    end_effector_frame="tool0",
    active_joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
)
```

For bundled robots, prefer the profile registry:

```python
from pinocchio_kinematics_lite import create_robot_kinematics

kin = create_robot_kinematics("franka_panda")
```

## Finding Frame Names

After loading a URDF:

```python
print(kin.list_frames())
print(kin.list_joints())
```

Choose the frame whose pose should be controlled as `end_effector_frame`. Common names are `tool0`, `ee_link`, `flange`, or the last link name.

## Active Joints

If `active_joint_names` is provided, the order defines the order of every active joint vector `q`.

If `active_joint_names=None`, the library uses scalar joints from the Pinocchio model, excluding any `locked_joint_names`. This default is convenient for simple fixed-base serial arms, but explicit names are safer for production code.

## Pose Inputs

IK accepts:

- a 4x4 homogeneous matrix
- `pinocchio.SE3`
- a dict with `position` plus `quaternion` or `rpy`
- a tuple `(position, quaternion_or_rpy)`

Transform helpers:

- `pose_matrix_from_xyz_quat`
- `pose_matrix_from_xyz_rpy`
- `xyz_quat_from_pose_matrix`
- `xyz_rpy_from_pose_matrix`

## Limited Support

The high-level API focuses on fixed-base serial manipulators with one scalar coordinate per active joint. Current limitations:

- floating-base robots are not fully supported by active-joint vector helpers
- mimic joints and mechanically coupled grippers are not expanded into coupling constraints
- closed-chain mechanisms are not supported
- collision checking and motion planning are out of scope
- full dynamics workflows are out of scope

For complex robots, build a reduced or simplified URDF for the arm chain you want to solve.

## Adding A Built-In Profile

Profile names are registered in `src/pinocchio_kinematics_lite/profiles/registry.py`.
Add a `RobotProfile` entry with:

- `name`
- `resource_parts`
- `end_effector_frame`
- `active_joint_names`
- optional `root_frame`
- optional `joint_limits`
- optional aliases and environment-variable override

For multi-branch robots, register one profile per active chain instead of exposing every scalar joint at once.
