# Piper API Documentation

> This document describes the `pyAgxArm` API for Piper robotic arms, covering instance creation, status reading, motion control, and advanced parameter configuration.

## Table of Contents

- [Switch to 中文](#piper-机械臂-api-使用文档)
- [Import Module](#import-module)
- [Create Instance and Connect](#create-instance-and-connect)
  - [Create Configuration — create_agx_arm_config()](#create-configuration--create_agx_arm_config)
  - [Create Arm Driver Instance — AgxArmFactory.create_arm()](#create-arm-driver-instance--agxarmfactorycreate_arm)
  - [Connect — connect()](#connect--connect)
  - [Initialize End Effector — init_effector()](#initialize-end-effector--init_effector)
- [General Status](#general-status)
  - [Get Joint Count — joint_nums](#get-joint-count--joint_nums)
  - [Check Communication — is_ok()](#check-communication--is_ok)
  - [Get Data Receive Frequency — get_fps()](#get-data-receive-frequency--get_fps)
- [Data Reading](#data-reading)
  - [MessageAbstract Return Value Overview](#messageabstract-return-value-overview)
  - [Get Arm Status — get_arm_status()](#get-arm-status--get_arm_status)
  - [Get Joint Angles — get_joint_angles()](#get-joint-angles--get_joint_angles)
  - [Get Flange Pose — get_flange_pose()](#get-flange-pose--get_flange_pose)
  - [Get Motor States — get_motor_states()](#get-motor-states--get_motor_states)
  - [Get Driver States — get_driver_states()](#get-driver-states--get_driver_states)
  - [Get Joint Enable Status — get_joint_enable_status()](#get-joint-enable-status--get_joint_enable_status)
  - [Get All Joint Enable Status List — get_joints_enable_status_list()](#get-all-joint-enable-status-list--get_joints_enable_status_list)
  - [Get Firmware Info — get_firmware()](#get-firmware-info--get_firmware)
- [Parameter Settings](#parameter-settings)
  - [Set Speed Percent — set_speed_percent()](#set-speed-percent--set_speed_percent)
  - [Set Installation Position — set_installation_pos()](#set-installation-position--set_installation_pos)
  - [Set Motion Mode — set_motion_mode()](#set-motion-mode--set_motion_mode)
  - [Set Payload — set_payload()](#set-payload--set_payload)
- [TCP Related](#tcp-related)
  - [Set TCP Offset — set_tcp_offset()](#set-tcp-offset--set_tcp_offset)
  - [Get TCP Pose — get_tcp_pose()](#get-tcp-pose--get_tcp_pose)
  - [Flange Pose to TCP Pose — get_flange2tcp_pose()](#flange-pose-to-tcp-pose--get_flange2tcp_pose)
  - [TCP Pose to Flange Pose — get_tcp2flange_pose()](#tcp-pose-to-flange-pose--get_tcp2flange_pose)
- [Leader-Follower Arm](#leader-follower-arm)
  - [Set Leader Mode — set_leader_mode()](#set-leader-mode--set_leader_mode)
  - [Set Follower Mode — set_follower_mode()](#set-follower-mode--set_follower_mode)
  - [Move Leader to Home — move_leader_to_home()](#move-leader-to-home--move_leader_to_home)
  - [Restore Leader Drag Mode — restore_leader_drag_mode()](#restore-leader-drag-mode--restore_leader_drag_mode)
  - [Get Leader Joint Angles — get_leader_joint_angles()](#get-leader-joint-angles--get_leader_joint_angles)
- [Motion Control](#motion-control)
  - [Enable — enable()](#enable--enable)
  - [Disable — disable()](#disable--disable)
  - [Reset — reset()](#reset--reset)
  - [Electronic Emergency Stop — electronic_emergency_stop()](#electronic-emergency-stop--electronic_emergency_stop)
  - [Joint Motion — move_j()](#joint-motion--move_j)
  - [Joint Motion (Follower Mode) — move_js()](#joint-motion-follower-mode--move_js)
  - [Point-to-Point Motion — move_p()](#point-to-point-motion--move_p)
  - [Linear Motion — move_l()](#linear-motion--move_l)
  - [Arc Motion — move_c()](#arc-motion--move_c)
  - [Single Joint MIT Control — move_mit()](#single-joint-mit-control--move_mit)
- [Advanced Parameter Reading and Configuration](#advanced-parameter-reading-and-configuration)
  - [Get Joint Angle/Velocity Limits — get_joint_angle_vel_limits()](#get-joint-anglevelocity-limits--get_joint_angle_vel_limits)
  - [Get Joint Acceleration Limits — get_joint_acc_limits()](#get-joint-acceleration-limits--get_joint_acc_limits)
  - [Get Flange Velocity/Acceleration Limits — get_flange_vel_acc_limits()](#get-flange-velocityacceleration-limits--get_flange_vel_acc_limits)
  - [Get Crash Protection Rating — get_crash_protection_rating()](#get-crash-protection-rating--get_crash_protection_rating)
  - [Calibrate Joint — calibrate_joint()](#calibrate-joint--calibrate_joint)
  - [Set Joint Angle/Velocity Limits — set_joint_angle_vel_limits()](#set-joint-anglevelocity-limits--set_joint_angle_vel_limits)
  - [Set Joint Acceleration Limits — set_joint_acc_limits()](#set-joint-acceleration-limits--set_joint_acc_limits)
  - [Set Flange Velocity/Acceleration Limits — set_flange_vel_acc_limits()](#set-flange-velocityacceleration-limits--set_flange_vel_acc_limits)
  - [Set Crash Protection Rating — set_crash_protection_rating()](#set-crash-protection-rating--set_crash_protection_rating)
  - [Reset Flange Limits to Default — set_flange_vel_acc_limits_to_default()](#reset-flange-limits-to-default--set_flange_vel_acc_limits_to_default)
  - [Reset Joint Limits to Default — set_joint_angle_vel_acc_limits_to_default()](#reset-joint-limits-to-default--set_joint_angle_vel_acc_limits_to_default)
  - [Set Link Velocity/Acceleration Periodic Feedback — set_links_vel_acc_period_feedback()](#set-link-velocityacceleration-periodic-feedback--set_links_vel_acc_period_feedback)

---

## Import Module

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory
```

---

## Create Instance and Connect

### Create Configuration — `create_agx_arm_config()`

**Description:** Generate the configuration dictionary required by the robotic arm, used to create a Driver instance.

**Function Definition:**

```python
create_agx_arm_config(
    robot: Literal["nero", "piper", "piper_h", "piper_l", "piper_x"],
    comm: Literal["can"] = "can",
    firmeware_version: str = "default",
    **kwargs,
) -> dict
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `robot` | `str` | Robotic arm model. Options: `"nero"` / `"piper"` / `"piper_h"` / `"piper_l"` / `"piper_x"` |
| `comm` | `str` | Communication type. Options: `"can"` (default). Note: `comm` is not the CAN channel name; the CAN channel is specified by `channel` |
| `firmeware_version` | `str` | Main controller firmware version, default `"default"` |

**Optional Keyword Arguments (`**kwargs`):**

| Name | Type | Description |
| --- | --- | --- |
| `joint_limits` | `dict` | Custom joint limits (unit: rad). Automatically assigned by default; manually entered limits are not currently applied to actual control. See example below |
| `channel` | `str` | CAN channel name, default `"can0"` |
| `interface` | `str` | CAN interface type, default `"socketcan"`. On Linux, the official CAN module must be `"socketcan"`; on macOS, use a serial CAN module and set to `"slcan"` |
| `bitrate` | `int` | CAN baud rate, default `1000000` (1 Mbps) |
| `enable_check_can` | `bool` | Whether to check the CAN module when creating a Comm instance, default `True` |
| `auto_connect` | `bool` | Whether to automatically create a CAN Bus instance, default `True` |
| `timeout` | `float` | CAN Bus read/write timeout (seconds), default `1.0` |

**Return Value:** `dict`

Example return structure:

```json
{
    "robot": "piper",
    "comm": {
        "type": "can",
        "can": {
            "channel": "can0",
            "interface": "socketcan",
            "bitrate": 1000000,
            "enable_check_can": true,
            "auto_connect": true,
            "timeout": 1.0
        }
    },
    "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "joint_limits": {
        "joint1": [-2.617994, 2.617994],
        "joint2": [0.0, 3.141593],
        "joint3": [-2.967060, 0.0],
        "joint4": [-1.745330, 1.745330],
        "joint5": [-1.221730, 1.221730],
        "joint6": [-2.094396, 2.094396]
    }
}
```

> **Tip:** Joint motion limits are in **radians (rad)**; gripper motion limits are in **meters (m)**.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
print(cfg)
```

---

### Create Arm Driver Instance — `AgxArmFactory.create_arm()`

**Description:** Create the corresponding robotic arm Driver instance via factory method based on the configuration dictionary.

**Function Definition:**

```python
create_arm(cls, config: dict, **kwargs) -> T
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `config` | `dict` | Configuration dictionary generated by `create_agx_arm_config()` |

**Return Value:** `Driver` — Different arm models, communication methods, and firmware versions correspond to different instances.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
```

---

### Connect — `connect()`

**Description:** Establish the connection and start the data reading thread.

**Function Definition:**

```python
connect(self, start_read_thread: bool = True) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `start_read_thread` | `bool` | Whether to start the data reading thread, default `True` |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
```

---

### Initialize End Effector — `init_effector()`

**Description:** Initialize the end effector Driver and return the corresponding effector instance (e.g., gripper / dexterous hand, etc.).

> **Note:** A single `robot` instance can only initialize an end effector **once**. To switch to a different effector type, create a new robotic arm instance.

**Function Definition:**

```python
init_effector(self, effector: str) -> EffectorDriver
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `effector` | `str` | Effector type (recommended to use `robot.OPTIONS.EFFECTOR.xxx` constants) |

**Return Value:** `EffectorDriver`

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
```

---

## General Status

### Get Joint Count — `joint_nums`

**Description:** Get the number of joints of the robotic arm (e.g., Piper has 6).

**Attribute Definition:**

```python
joint_nums: int
```

**Return Value:** `int`

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

print("robotic arm joint_nums =", robot.joint_nums)

for joint_index in range(1, robot.joint_nums + 1):
    start_t = time.monotonic()
    while True:
        if robot.enable(joint_index):
            print(f"enable joint {joint_index} success")
            break
        if time.monotonic() - start_t > 5.0:
            print(f"enable joint {joint_index} timeout (5s)")
            break
        time.sleep(0.01)
```

---

### Check Communication — `is_ok()`

**Description:** Check whether the robotic arm data reception is normal. This value is computed by the SDK's internal data monitoring logic based on whether data has not been received for a period of time.

**Function Definition:**

```python
is_ok(self) -> bool
```

**Return Value:** `bool`

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

time.sleep(0.5)
print("robotic arm is_ok =", robot.is_ok())
```

---

### Get Data Receive Frequency — `get_fps()`

**Description:** Get the data monitoring receive frequency (Hz) of the robotic arm, which is a statistical value from the SDK for data received by the parser.

**Function Definition:**

```python
get_fps(self) -> float
```

**Return Value:** `float` (unit: Hz)

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

time.sleep(0.5)
print("robotic arm fps =", robot.get_fps(), "Hz")
```

---

## Data Reading

### MessageAbstract Return Value Overview

Most read interfaces in this SDK return `MessageAbstract[T] | None`, with the following common fields:

| Field | Type | Description |
| --- | --- | --- |
| `ret.msg` | `T` | Message data body (e.g., `list[float]` or a feedback message struct) |
| `ret.hz` | `float` | Receive frequency of this message type (SDK statistics), unit: Hz |
| `ret.timestamp` | `float` | Message timestamp (SDK recorded), unit: s |

---

### Get Arm Status — `get_arm_status()`

**Description:** Read the overall status feedback of the robotic arm (control mode, motion mode, emergency stop / error status, trajectory point number, etc.).

**Function Definition:**

```python
get_arm_status(self) -> MessageAbstract[ArmMsgFeedbackStatus] | None
```

**Return Value:** `MessageAbstract[ArmMsgFeedbackStatus] | None`

**Message Fields (`.msg`):**

| Field | Type | Description |
| --- | --- | --- |
| `ctrl_mode` | `int` | Control mode (standby / CAN / teach / Ethernet / WiFi / offline trajectory, etc.) |
| `arm_status` | `int` | Robotic arm status (normal / emergency stop / singularity / out of range / collision, etc.) |
| `mode_feedback` | `int` | Mode feedback (MOVE P/J/L/C/MIT, etc.) |
| `teach_status` | `int` | Teach status (start recording / stop recording / execute / pause / resume / terminate, etc.) |
| `motion_status` | `int` | Motion status: `0` reached; `1` not reached |
| `trajectory_num` | `int` | Trajectory point number (feedback in offline trajectory mode) |
| `err_status` | `object` | Error status bits (joint angle out of range / joint communication error, etc.) |

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    arm_status = robot.get_arm_status()
    if arm_status is not None:
        print(arm_status.msg)
        print(arm_status.hz, arm_status.timestamp)
    time.sleep(0.02)
```

---

### Get Joint Angles — `get_joint_angles()`

**Description:** Get the current angle of each joint.

**Function Definition:**

```python
get_joint_angles(self) -> MessageAbstract[list[float]] | None
```

**Return Value:** `MessageAbstract[list[float]] | None`

`.msg` is a `list[float]` of length 6: `[j1, j2, j3, j4, j5, j6]`, unit: **rad**.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    ja = robot.get_joint_angles()
    if ja is not None:
        print(ja.msg)
        print(ja.hz, ja.timestamp)
    time.sleep(0.005)
```

---

### Get Flange Pose — `get_flange_pose()`

**Description:** Get the end flange pose.

> **Terminology:** `flange` refers to the mounting flange / connection face of the last link (end link) of the robotic arm. It is the mechanical mounting interface for tools / end effectors.

**Function Definition:**

```python
get_flange_pose(self) -> MessageAbstract[list[float]] | None
```

**Return Value:** `MessageAbstract[list[float]] | None`

`.msg` is a `list[float]` of length 6: `[x, y, z, roll, pitch, yaw]`

- `x, y, z`: Position coordinates (unit: m)
- `roll, pitch, yaw`: Euler angles (unit: rad, corresponding to rotation around X/Y/Z axes respectively)

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    fp = robot.get_flange_pose()
    if fp is not None:
        print(fp.msg)
        print(fp.hz, fp.timestamp)
    time.sleep(0.005)
```

---

### Get Motor States — `get_motor_states()`

**Description:** Read the high-speed motor feedback (position / velocity / current / torque) for a specified joint.

**Function Definition:**

```python
get_motor_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6]) -> MessageAbstract[ArmMsgFeedbackHighSpd] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index, range: `1~6` |

**Return Value:** `MessageAbstract[ArmMsgFeedbackHighSpd] | None`

**Message Fields (`.msg`):**

| Field | Type | Description |
| --- | --- | --- |
| `position` | `float` | Motor position (rad) |
| `velocity` | `float` | Motor velocity (rad/s) |
| `current` | `float` | Motor current (A) |
| `torque` | `float` | Motor torque (N·m) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

ms = robot.get_motor_states(1)
if ms is not None:
    print(ms.msg.position, ms.msg.velocity, ms.msg.current, ms.msg.torque)
    print(ms.hz, ms.timestamp)
```

---

### Get Driver States — `get_driver_states()`

**Description:** Read the low-speed driver feedback (voltage / temperature / bus current / driver status bits, etc.) for a specified joint.

**Function Definition:**

```python
get_driver_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6]) -> MessageAbstract[ArmMsgFeedbackLowSpd] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index, range: `1~6` |

**Return Value:** `MessageAbstract[ArmMsgFeedbackLowSpd] | None`

**Message Fields (`.msg`):**

| Field | Type | Description |
| --- | --- | --- |
| `vol` | `float` | Driver voltage |
| `foc_temp` | `float` | Driver temperature (°C) |
| `motor_temp` | `float` | Motor temperature (°C) |
| `bus_current` | `float` | Bus current (A) |
| `foc_status` | `object` | Driver status bits (under-voltage / over-temperature / over-current / collision / disabled / stall, etc.) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

ds = robot.get_driver_states(1)
if ds is not None:
    print(ds.msg.vol, ds.msg.foc_temp, ds.msg.motor_temp, ds.msg.bus_current)
    print(ds.msg.foc_status.driver_enable_status)
    print(ds.hz, ds.timestamp)
```

---

### Get Joint Enable Status — `get_joint_enable_status()`

**Description:** Get the enable status of a specified joint motor.

**Function Definition:**

```python
get_joint_enable_status(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255]) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` queries a single joint; `255` queries all joints (internally aggregated using `all([...])`) |

**Return Value:** `bool` — `True` means enabled, `False` means not enabled or no feedback available.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

if robot.get_joint_enable_status(1):
    print("关节 1 电机已使能")
```

---

### Get All Joint Enable Status List — `get_joints_enable_status_list()`

**Description:** Read the enable status list of all joint motors (in order of joints 1~6).

**Function Definition:**

```python
get_joints_enable_status_list(self) -> list[bool]
```

**Return Value:** `list[bool]`

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

print(robot.get_joints_enable_status_list())
```

---

### Get Firmware Info — `get_firmware()`

**Description:** Read the robotic arm firmware information (software version / hardware version / production date, etc.). This interface sends a query frame and waits for the corresponding feedback.

**Function Definition:**

```python
get_firmware(self, timeout: float = 1.0, min_interval: float = 1.0) -> dict | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `timeout` | `float` | Timeout for waiting for feedback (seconds), default `1.0`; `0.0` means non-blocking |
| `min_interval` | `float` | Minimum request interval (seconds), default `1.0` |

**Return Value:** `dict | None`

Common fields:

| Key | Type | Description |
| --- | --- | --- |
| `software_version` | `str` | Software version (e.g., `S-V1.8-2`) |
| `hardware_version` | `str` | Hardware version (e.g., `H-V1.2-1`) |
| `production_date` | `str` | Production date (e.g., `250925`) |
| `node_type` | `str` | Node type |
| `node_number` | `int` | Node number |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

fw = robot.get_firmware()
if fw is not None:
    print(fw)
```

---

## Parameter Settings

### Set Speed Percent — `set_speed_percent()`

**Description:** Set the running speed percentage of the robotic arm in position-velocity mode, applicable to `move_j` / `move_p` / `move_l` / `move_c`.

**Function Definition:**

```python
set_speed_percent(self, percent: int = 100) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `percent` | `int` | Running speed percentage, range `[0, 100]`, default `100` |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_speed_percent(100)
```

---

### Set Installation Position — `set_installation_pos()`

**Description:** Set the installation position of the robotic arm. Supports horizontal, left-facing, and right-facing orientations.

**Function Definition:**

```python
set_installation_pos(self, pos: Literal["horizontal", "left", "right"] = "horizontal") -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `pos` | `str` | Installation orientation, valid values: `'horizontal'` / `'left'` / `'right'`, default: `'horizontal'` (recommended to use `robot.OPTIONS.INSTALLATION_POS.xxx` constants) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_installation_pos(robot.OPTIONS.INSTALLATION_POS.HORIZONTAL)
```

---

### Set Motion Mode — `set_motion_mode()`

**Description:** Set the motion mode.

| Mode | Type | Description |
| --- | --- | --- |
| `move_p` / `move_j` / `move_l` / `move_c` | **Position-velocity mode** | The lower layer smooths received messages to ensure continuous and stable motion |
| `move_mit` / `move_js` | **MIT motor pass-through mode** | The lower layer only forwards messages with **no smoothing**, suitable for direct motor control scenarios |

> **Tip:** When calling any `move_*` motion command, the system **automatically switches to the corresponding motion mode**, so there is usually **no need to manually call `set_motion_mode()`**.

**Function Definition:**

```python
set_motion_mode(self, motion_mode: Literal["p", "j", "l", "c", "mit", "js"] = "p") -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `motion_mode` | `str` | Motion mode, valid values: `'p'` / `'j'` / `'l'` / `'c'` / `'mit'` / `'js'`, default: `'p'` (recommended to use `robot.OPTIONS.MOTION_MODE.xxx` constants) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)
```

---

### Set Payload — `set_payload()`

**Description:** Set the robotic arm payload.

**Function Definition:**

```python
set_payload(self, load: Literal['empty', 'half', 'full'] = 'empty', timeout: float = 1.0) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `load` | `str` | Payload level, valid values: `'empty'` (no load) / `'half'` (half load) / `'full'` (full load), default: `'empty'` (recommended to use `robot.OPTIONS.PAYLOAD.xxx` constants) |
| `timeout` | `float` | Timeout for waiting for feedback (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates that the command acknowledgement was received, but does not guarantee the setting was successful.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_payload(robot.OPTIONS.PAYLOAD.FULL)
```

---

## TCP Related

### Set TCP Offset — `set_tcp_offset()`

**Description:** Set the TCP (Tool Center Point) offset pose relative to the flange (in the **flange coordinate frame**). Default is no offset: `[0, 0, 0, 0, 0, 0]`.

> **Tip:** This offset value is only saved in the SDK/Driver instance and is not sent to the controller.

**Function Definition:**

```python
set_tcp_offset(self, pose: list[float]) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `pose` | `list[float]` | TCP pose offset in the flange coordinate frame `[x, y, z, roll, pitch, yaw]`: `x, y, z` are position (m); `roll, pitch, yaw` are Euler angles (rad). Range: `roll/yaw` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]` |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])
```

---

### Get TCP Pose — `get_tcp_pose()`

**Description:** Get the TCP pose. This interface first reads the flange pose, then applies a rigid body transformation using the offset saved by `set_tcp_offset()` to compute the TCP pose. If no offset is set, the TCP pose is the same as the flange pose.

**Function Definition:**

```python
get_tcp_pose(self) -> MessageAbstract[list[float]] | None
```

**Return Value:** `MessageAbstract[list[float]] | None`

`.msg` is a `list[float]` of length 6: `[x, y, z, roll, pitch, yaw]` (m / rad).

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

while True:
    tcp = robot.get_tcp_pose()
    if tcp is not None:
        print(tcp.msg)
        print(tcp.hz, tcp.timestamp)
    time.sleep(0.02)
```

---

### Flange Pose to TCP Pose — `get_flange2tcp_pose()`

**Description:** Given a flange pose (in the base / world coordinate frame), compute the corresponding TCP pose using the offset saved by `set_tcp_offset()`.

**Function Definition:**

```python
get_flange2tcp_pose(self, flange_pose: list[float]) -> list[float]
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `flange_pose` | `list[float]` | Flange pose `[x, y, z, roll, pitch, yaw]` (m / rad). Range: `roll/yaw` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]` |

**Return Value:** `list[float]` — TCP pose `[x, y, z, roll, pitch, yaw]` (m / rad).

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

# 直接指定法兰位姿
tcp_pose = robot.get_flange2tcp_pose([0.30, 0.0, 0.30, 0.0, 1.5707, 0.0])
print("tcp_pose =", tcp_pose)

# 从当前位姿获取，结果与 get_tcp_pose() 得到的 pose 相同
flange_pose = robot.get_flange_pose()
if flange_pose is not None:
    tcp_pose = robot.get_flange2tcp_pose(flange_pose)
    print("tcp_pose =", tcp_pose)
```

---

### TCP Pose to Flange Pose — `get_tcp2flange_pose()`

**Description:** Given a target TCP pose (in the base / world coordinate frame), compute the corresponding target flange pose using the offset saved by `set_tcp_offset()`. Pass the returned flange pose to `move_p()` to **move the TCP to the target TCP pose**.

**Function Definition:**

```python
get_tcp2flange_pose(self, tcp_pose: list[float]) -> list[float]
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `tcp_pose` | `list[float]` | Target TCP pose `[x, y, z, roll, pitch, yaw]` (m / rad). Range: `roll/yaw` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]` |

**Return Value:** `list[float]` — Target flange pose `[x, y, z, roll, pitch, yaw]` (m / rad), can be directly used with `move_p()`.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

target_tcp_pose = [0.30, 0.0, 0.30, 0.0, 1.5707, 0.0]
target_flange_pose = robot.get_tcp2flange_pose(target_tcp_pose)
print("target_flange_pose =", target_flange_pose)

# robot.move_p(target_flange_pose)  # 注意：会触发运动
```

---

## Leader-Follower Arm

### Set Leader Mode — `set_leader_mode()`

**Description:** Set the robotic arm to **leader zero-force drag mode** (the "leader" in a leader-follower coordination scenario). After entering this mode, the leader arm is typically in a draggable / zero-force drag state.

> **Tip:** This mode is used for leader-follower arm linkage / teaching scenarios. If using a single arm only, this interface can be ignored.

**Function Definition:**

```python
set_leader_mode(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_leader_mode()
```

---

### Set Follower Mode — `set_follower_mode()`

**Description:** Set the robotic arm to **follower controlled mode** (the "follower" in a leader-follower coordination scenario). The follower arm follows the leader arm's control / commands. Can be used together with `set_leader_mode()`.

**Function Definition:**

```python
set_follower_mode(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_follower_mode()
```

---

### Move Leader to Home — `move_leader_to_home()`

**Description:** Move the leader arm back to the Home pose. After completion, it is recommended to call `restore_leader_drag_mode()` to restore the leader arm to the "zero-force drag" state.

**Function Definition:**

```python
move_leader_to_home(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_leader_mode()
robot.move_leader_to_home()
robot.restore_leader_drag_mode()
```

---

### Restore Leader Drag Mode — `restore_leader_drag_mode()`

**Description:** Restore the leader arm to the "zero-force drag" state, typically used after `move_leader_to_home()`.

**Function Definition:**

```python
restore_leader_drag_mode(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_leader_mode()
robot.move_leader_to_home()
robot.restore_leader_drag_mode()
```

---

### Get Leader Joint Angles — `get_leader_joint_angles()`

**Description:** Get the leader arm joint angle message, used for controlling the follower arm.

**Function Definition:**

```python
get_leader_joint_angles(self) -> MessageAbstract[list[float]] | None
```

**Return Value:** `MessageAbstract[list[float]] | None`

`.msg` is a `list[float]` of length 6: `[j1, j2, j3, j4, j5, j6]`, unit: **rad**.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_leader_mode()

while True:
    mja = robot.get_leader_joint_angles()
    if mja is not None:
        print(mja.msg)
        print(mja.hz, mja.timestamp)
    time.sleep(0.005)
```

---

## Motion Control

### Enable — `enable()`

**Description:** Power on and enable the robotic arm.

**Function Definition:**

```python
enable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` enables a single joint; `255` enables all joints, default: `255` |

**Return Value:** `bool` — `True` means enable was successful.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)
```

---

### Disable — `disable()`

**Description:** Power off the robotic arm.

> **Warning:** When executing this command, if the robotic arm joints are in a raised position, they will **fall immediately**. Make sure the robotic arm is in a safe state before using this command.

**Function Definition:**

```python
disable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` disables a single joint; `255` disables all joints, default: `255` |

**Return Value:** `bool` — `True` means disable was successful.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.disable():
    time.sleep(0.01)
```

---

### Reset — `reset()`

**Description:** Reset the robotic arm mode and immediately power off the arm.

> **Warning:** When executing this command, if the robotic arm joints are in a raised position, they will **fall immediately**. Make sure the robotic arm is in a safe state before using this command.

**Function Definition:**

```python
reset(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.reset()
```

---

### Electronic Emergency Stop — `electronic_emergency_stop()`

**Description:** Set the robotic arm to emergency stop state. If the joints are in a raised position when executing, the arm will **slowly descend with constant damping** (will not fall immediately).

**Function Definition:**

```python
electronic_emergency_stop(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.electronic_emergency_stop()
```

---

### Joint Motion — `move_j()`

**Description:** Joint position-velocity control mode, set target angles for each joint.

**Function Definition:**

```python
move_j(self, joints: list[float]) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joints` | `list[float]` | Target angle array of length 6 `[j1, j2, j3, j4, j5, j6]` (unit: rad, precision: 1.74532925199e-5). Joint limits depend on robot variant configuration |

> **Note:** Consecutive execution of this command will overwrite the previous target value.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_j([0, 0.4, -0.4, 0, -0.4, 0])

# 等待运动结束（带 5s 超时）
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("已到达目标位置")
        break
    if time.monotonic() - start_t > 5.0:
        print("等待运动结束超时（5s）")
        break
    time.sleep(0.1)
```

---

### Joint Motion (Follower Mode) — `move_js()`

**Description:** Switch the robotic arm to **JS (follower) mode** (MIT pass-through mode) and send joint target angles. Compared to `move_j`, `move_js` is more oriented toward "fast response" control: **no smoothing, no trajectory planning**; the controller / driver will respond to the target angle as fast as possible.

**Function Definition:**

```python
move_js(self, joints: list[float]) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joints` | `list[float]` | Target angle array of length 6 `[j1, j2, j3, j4, j5, j6]` (unit: rad, precision: 1.74532925199e-5). Joint limits depend on robot variant configuration |

> **Warning: Extremely high risk**
>
> 1. This mode may cause **impact, oscillation, instability**, and other risks. Use only after fully evaluating safety and control stability, and ensure emergency stop is available at all times.
> 2. **No smoothing, no trajectory planning** — the controller / driver attempts to reach the target as fast as possible, which may cause impact and oscillation.
> 3. Consecutive execution of this command will overwrite the previous target value.
> 4. Due to the faster response, joint control force is lower compared to position-velocity mode, and stiffness will also be lower.
> 5. On older firmware versions (below `S-V1.8-5`), if the robotic arm is currently in follower mode and you want to switch to position-velocity control mode, you need to first execute `robot.reset()` (the arm will reset and power off), then execute `move_j` for normal control.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.move_js([0, 0.4, -0.4, 0, -0.4, 0])
```

---

### Point-to-Point Motion — `move_p()`

**Description:** Send a target flange pose. The robotic arm performs joint angle inverse kinematics based on the current joint positions and target pose, then moves accordingly.

**Function Definition:**

```python
move_p(self, pose: list[float]) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `pose` | `list[float]` | Target pose `[x, y, z, roll, pitch, yaw]`: `x, y, z` are position (m, precision: 1e-6); `roll, pitch, yaw` are Euler angles (rad, precision: 1.74532925199e-5). Orientation range: `roll` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]`, `yaw` ∈ `[-π, π]` |

> **Note:** Consecutive execution of this command will overwrite the previous target value.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_p([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

# 等待运动结束（带 5s 超时）
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("已到达目标位置")
        break
    if time.monotonic() - start_t > 5.0:
        print("等待运动结束超时（5s）")
        break
    time.sleep(0.1)
```

---

### Linear Motion — `move_l()`

**Description:** Send a target flange pose. The robotic arm performs linear trajectory planning based on the current pose and target pose.

**Function Definition:**

```python
move_l(self, pose: list[float]) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `pose` | `list[float]` | Target pose `[x, y, z, roll, pitch, yaw]`: `x, y, z` are position (m, precision: 1e-6); `roll, pitch, yaw` are Euler angles (rad, precision: 1.74532925199e-5). Orientation range: `roll` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]`, `yaw` ∈ `[-π, π]` |

> **Note:** Although consecutive execution of this command can overwrite the previous target, since the lower layer needs to re-plan a linear trajectory each time a new point is received, **this command cannot be used to continuously send target points**.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_l([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

# 等待运动结束（带 5s 超时）
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("已到达目标位置")
        break
    if time.monotonic() - start_t > 5.0:
        print("等待运动结束超时（5s）")
        break
    time.sleep(0.1)
```

---

### Arc Motion — `move_c()`

**Description:** Perform arc trajectory planning and execution using three target flange poses: "start / midpoint / end".

**Function Definition:**

```python
move_c(self, start_pose: list[float], mid_pose: list[float], end_pose: list[float]) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `start_pose` | `list[float]` | Start pose `[x, y, z, roll, pitch, yaw]` (m / rad). Orientation range: `roll` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]`, `yaw` ∈ `[-π, π]` |
| `mid_pose` | `list[float]` | Midpoint pose `[x, y, z, roll, pitch, yaw]` (m / rad). Orientation range: `roll` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]`, `yaw` ∈ `[-π, π]` |
| `end_pose` | `list[float]` | End pose `[x, y, z, roll, pitch, yaw]` (m / rad). Orientation range: `roll` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]`, `yaw` ∈ `[-π, π]` |

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
sp = [0.2, 0.0, 0.3, 0.0, 1.5708, 0.0]
mp = [0.2, 0.05, 0.35, 0.0, 1.5708, 0.0]
ep = [0.2, 0.0, 0.4, 0.0, 1.5708, 0.0]
robot.move_c(sp, mp, ep)

# 等待运动结束（带 5s 超时）
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("已到达目标位置")
        break
    if time.monotonic() - start_t > 5.0:
        print("等待运动结束超时（5s）")
        break
    time.sleep(0.1)
```

---

### Single Joint MIT Control — `move_mit()`

**Description:** Use the joint driver's low-level MIT control interface to control a single joint motor. This enables current-simulated torque control.

The controller conceptually computes a reference torque:

$$T_{\text{ref}} = k_p \cdot (p_{\text{des}} - p) + k_d \cdot (v_{\text{des}} - v) + T_{\text{ff}}$$

where \(p/v\) are the measured joint position / velocity.

**Typical usage recommendations:**

| Control Method | Parameter Settings | Description |
| --- | --- | --- |
| **Velocity control** | `kp = 0`, `kd ≠ 0` | Primarily controlled via `v_des` |
| **Torque control** | `kp = 0`, `kd = 0` | Primarily controlled via `t_ff` |
| **Position control** | `kp ≠ 0`, `kd ≠ 0` | Not recommended to set `kd` to 0; increasing damping appropriately can reduce oscillation risk |

> **Warning:** MIT is a relatively low-level control interface. Improper parameters may cause **impact / oscillation / instability**. It is recommended to start with small gains for tuning and use under safe conditions.

**Function Definition:**

```python
move_mit(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    p_des: float = 0.0,
    v_des: float = 0.0,
    kp: float = 10.0,
    kd: float = 0.8,
    t_ff: float = 0.0,
) -> None
```

**Parameters:**

| Name | Type | Range | Unit | Default | Precision |
| --- | --- | --- | --- | --- | --- |
| `joint_index` | `int` | `1~6` | — | — | — |
| `p_des` | `float` | `[-12.5, 12.5]` | rad | `0.0` | 3.815e-4 |
| `v_des` | `float` | `[-45.0, 45.0]` | rad/s | `0.0` | 2.198e-2 |
| `kp` | `float` | `[0.0, 500.0]` | — | `10.0` | 1.221e-1 |
| `kd` | `float` | `[-5.0, 5.0]` | — | `0.8` | 2.442e-3 |
| `t_ff` | `float` | `[-8.0, 8.0]` | N·m | `0.0` | 6.275e-2 |

> **Note:** Consecutive execution of this command will overwrite the previous target value.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

for i in range(1, robot.joint_nums + 1):
    robot.move_mit(
        joint_index=i,
        p_des=0.0,
        v_des=0.0,
        kp=10.0,
        kd=0.8,
        t_ff=0.0,
    )
```

---

## Advanced Parameter Reading and Configuration

### Get Joint Angle/Velocity Limits — `get_joint_angle_vel_limits()`

**Description:** Query the angle limits and velocity limits of a specified joint (feedback from the controller).

**Function Definition:**

```python
get_joint_angle_vel_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index, range: `1~6` |
| `timeout` | `float` | Timeout for waiting for feedback (seconds), default `1.0`; `0.0` means non-blocking |
| `min_interval` | `float` | Minimum request interval (seconds), default `1.0` |

**Return Value:** `MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd] | None`

**Message Fields (`.msg`):**

| Field | Type | Description |
| --- | --- | --- |
| `min_angle_limit` | `float` | Minimum angle limit (rad) |
| `max_angle_limit` | `float` | Maximum angle limit (rad) |
| `min_joint_spd` | `float` | Minimum joint velocity limit (rad/s) |
| `max_joint_spd` | `float` | Maximum joint velocity limit (rad/s) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_joint_angle_vel_limits(1)
if limit is not None:
    print(limit.msg.min_angle_limit, limit.msg.max_angle_limit)
    print(limit.msg.min_joint_spd, limit.msg.max_joint_spd)
```

---

### Get Joint Acceleration Limits — `get_joint_acc_limits()`

**Description:** Query the maximum acceleration limit of a specified joint.

**Function Definition:**

```python
get_joint_acc_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index, range: `1~6` |
| `timeout` | `float` | Timeout for waiting for feedback (seconds), default `1.0`; `0.0` means non-blocking |
| `min_interval` | `float` | Minimum request interval (seconds), default `1.0` |

**Return Value:** `MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit] | None`

**Message Fields (`.msg`):**

| Field | Type | Description |
| --- | --- | --- |
| `max_joint_acc` | `float` | Maximum joint acceleration limit (rad/s²) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_joint_acc_limits(1)
if limit is not None:
    print(limit.msg.max_joint_acc)
    print(limit.hz, limit.timestamp)
```

---

### Get Flange Velocity/Acceleration Limits — `get_flange_vel_acc_limits()`

**Description:** Query the end-effector maximum linear velocity / angular velocity and linear acceleration / angular acceleration limits.

**Function Definition:**

```python
get_flange_vel_acc_limits(self, timeout: float = 1.0, min_interval: float = 1.0) -> MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `timeout` | `float` | Timeout for waiting for feedback (seconds), default `1.0`; `0.0` means non-blocking |
| `min_interval` | `float` | Minimum request interval (seconds), default `1.0` |

**Return Value:** `MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None`

**Message Fields (`.msg`):**

| Field | Type | Description |
| --- | --- | --- |
| `end_max_linear_vel` | `float` | Maximum end-effector linear velocity (m/s) |
| `end_max_angular_vel` | `float` | Maximum end-effector angular velocity (rad/s) |
| `end_max_linear_acc` | `float` | Maximum end-effector linear acceleration (m/s²) |
| `end_max_angular_acc` | `float` | Maximum end-effector angular acceleration (rad/s²) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_flange_vel_acc_limits()
if limit is not None:
    print(
        limit.msg.end_max_linear_vel,
        limit.msg.end_max_angular_vel,
        limit.msg.end_max_linear_acc,
        limit.msg.end_max_angular_acc,
    )
    print(limit.hz, limit.timestamp)
```

---

### Get Crash Protection Rating — `get_crash_protection_rating()`

**Description:** Query the crash protection rating of each joint (list returned by the controller).

**Function Definition:**

```python
get_crash_protection_rating(
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[list[int]] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `timeout` | `float` | Timeout for waiting for feedback (seconds), default `1.0`; `0.0` means non-blocking |
| `min_interval` | `float` | Minimum request interval (seconds), default `1.0` |

**Return Value:** `MessageAbstract[list[int]] | None`

`.msg` is a crash protection rating list (in joint order), where each item is an `int` (range: `0~8`). **The higher the rating, the more sensitive it is, and the more easily the crash protection mechanism is triggered** (more conservative).

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

rating = robot.get_crash_protection_rating()
if rating is not None:
    print(rating.msg)
    print(rating.hz, rating.timestamp)
```

---

### Calibrate Joint — `calibrate_joint()`

**Description:** Perform the zeroing / calibration process for a specified joint (waits for controller ACK / response and returns the result).

**Function Definition:**

```python
calibrate_joint(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | `1~6` calibrates a single joint; `255` calibrates all joints |
| `timeout` | `float` | Response wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates a success response was received; `False` indicates timeout or failure.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

joint_index = 1
robot.disable(joint_index)
time.sleep(0.2)
input("请手动将关节移动到零位位置后按回车继续...")

if robot.calibrate_joint(joint_index):
    print("calibrate_joint success")
```

---

### Set Joint Angle/Velocity Limits — `set_joint_angle_vel_limits()`

**Description:** Set joint angle / velocity limits, with optional read-back verification to check if the settings took effect.

**Function Definition:**

```python
set_joint_angle_vel_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    min_angle_limit: Optional[float] = None,
    max_angle_limit: Optional[float] = None,
    max_joint_spd: Optional[float] = None,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` configures a single joint; `255` configures all joints |
| `min_angle_limit` | `Optional[float]` | Minimum angle limit (rad); `None` means no configuration |
| `max_angle_limit` | `Optional[float]` | Maximum angle limit (rad); `None` means no configuration |
| `max_joint_spd` | `Optional[float]` | Maximum joint velocity limit (rad/s); `None` means no configuration |
| `timeout` | `float` | ACK / verification wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates ACK received and read-back verification passed; `False` indicates timeout / failure / verification failed.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 同时设置角度和速度限制
success = robot.set_joint_angle_vel_limits(
    joint_index=1,
    min_angle_limit=-2.618,
    max_angle_limit=2.618,
    max_joint_spd=3.0,
)
print("set_joint_angle_vel_limits success =", success)

# 仅设置最大速度限制（不改角度限制）
success = robot.set_joint_angle_vel_limits(joint_index=1, max_joint_spd=3.0)
print("set_joint_angle_vel_limits(max_joint_spd) success =", success)
```

---

### Set Joint Acceleration Limits — `set_joint_acc_limits()`

**Description:** Set the maximum acceleration limit for a specified joint.

**Function Definition:**

```python
set_joint_acc_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    max_joint_acc: Optional[float] = None,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` configures a single joint; `255` configures all joints |
| `max_joint_acc` | `Optional[float]` | Maximum acceleration (rad/s²); `None` means no configuration |
| `timeout` | `float` | ACK / verification wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates ACK received and read-back verification passed.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_joint_acc_limits(joint_index=1, max_joint_acc=5.0)
print("set_joint_acc_limits success =", success)
```

---

### Set Flange Velocity/Acceleration Limits — `set_flange_vel_acc_limits()`

**Description:** Set the end-effector velocity / acceleration limits.

**Function Definition:**

```python
set_flange_vel_acc_limits(
    self,
    max_linear_vel: Optional[float] = None,
    max_angular_vel: Optional[float] = None,
    max_linear_acc: Optional[float] = None,
    max_angular_acc: Optional[float] = None,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `max_linear_vel` | `Optional[float]` | Maximum linear velocity (m/s); `None` means no configuration |
| `max_angular_vel` | `Optional[float]` | Maximum angular velocity (rad/s); `None` means no configuration |
| `max_linear_acc` | `Optional[float]` | Maximum linear acceleration (m/s²); `None` means no configuration |
| `max_angular_acc` | `Optional[float]` | Maximum angular acceleration (rad/s²); `None` means no configuration |
| `timeout` | `float` | ACK / verification wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates ACK received and read-back verification passed.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_flange_vel_acc_limits(
    max_linear_vel=0.5,
    max_angular_vel=0.13,
    max_linear_acc=0.8,
    max_angular_acc=0.2,
)
print("set_flange_vel_acc_limits success =", success)
```

---

### Set Crash Protection Rating — `set_crash_protection_rating()`

**Description:** Set the crash protection rating (can specify a single joint or all joints).

**Function Definition:**

```python
set_crash_protection_rating(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    rating: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8] = 0,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` configures a single joint; `255` configures all joints, default: `255` |
| `rating` | `int` | Crash protection rating, range: `[0, 8]` (`0` = no detection), default: `0`. **The higher the rating, the more sensitive it is, and the more easily crash protection is triggered** (more conservative) |
| `timeout` | `float` | ACK / verification wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates ACK received and read-back verification passed.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_crash_protection_rating(joint_index=1, rating=1)
print("set_crash_protection_rating success =", success)
```

---

### Reset Flange Limits to Default — `set_flange_vel_acc_limits_to_default()`

**Description:** Reset the end-effector velocity / acceleration limits to default values.

**Function Definition:**

```python
set_flange_vel_acc_limits_to_default(self, timeout: float = 1.0) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `timeout` | `float` | ACK / response wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates that ACK / response was received within the timeout.

> **Tip:** This interface does not provide read-back verification. To confirm, you can call `get_flange_vel_acc_limits()` to query.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_flange_vel_acc_limits_to_default()
print("set_flange_vel_acc_limits_to_default success =", success)
```

---

### Reset Joint Limits to Default — `set_joint_angle_vel_acc_limits_to_default()`

**Description:** Reset the joint angle / velocity / acceleration limits to default values.

**Function Definition:**

```python
set_joint_angle_vel_acc_limits_to_default(self, timeout: float = 1.0) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `timeout` | `float` | ACK / response wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates that ACK / response was received within the timeout.

> **Tip:** This interface does not provide read-back verification. To confirm, you can call `get_joint_angle_vel_limits()` / `get_joint_acc_limits()` to query.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_joint_angle_vel_acc_limits_to_default()
print("set_joint_angle_vel_acc_limits_to_default success =", success)
```

---

### Set Link Velocity/Acceleration Periodic Feedback — `set_links_vel_acc_period_feedback()`

**Description:** Set the Cartesian velocity / acceleration periodic feedback switch for each joint link (corresponding to CAN periodic frames `0x481~0x486`).

> **Warning:** This feature has been **deprecated** in the lower-level main controller, but the bus may still periodically report the corresponding frames, and the reported data is **all zeros** with no practical meaning. **It is recommended to keep this disabled by default** (`enable=False`) to avoid wasting bandwidth.
>
> There is no direct read-back verification method for this interface. It is recommended to use `candump` to observe whether periodic frames appear for verification:
>
> ```bash
> candump can0 | grep "48[1-6]"
> ```

**Function Definition:**

```python
set_links_vel_acc_period_feedback(self, enable: bool = False, timeout: float = 1.0) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `enable` | `bool` | Whether to enable periodic feedback: `True` to enable; `False` to disable (**recommended to keep disabled by default**) |
| `timeout` | `float` | ACK / response wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates that ACK / response was received within the timeout.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_links_vel_acc_period_feedback(enable=True)
print("enable periodic feedback success =", success)

success = robot.set_links_vel_acc_period_feedback(enable=False)
print("disable periodic feedback success =", success)
```

---

# Piper 机械臂 API 使用文档

> 本文档描述 `pyAgxArm` SDK 为 Piper 系列机械臂提供的 Python API。涵盖实例创建、状态读取、运动控制、参数配置等全部接口。

## 目录

- [切换到 English](#piper-api-documentation)
- [导入模块](#导入模块)
- [创建实例并连接](#创建实例并连接)
  - [创建配置参数 — create_agx_arm_config()](#创建配置参数--create_agx_arm_config)
  - [创建机械臂 Driver 实例 — AgxArmFactory.create_arm()](#创建机械臂-driver-实例--agxarmfaborycreate_arm)
  - [创建连接 — connect()](#创建连接--connect)
  - [初始化末端执行器 — init_effector()](#初始化末端执行器--init_effector)
- [通用状态](#通用状态)
  - [获取关节数量 — joint_nums](#获取关节数量--joint_nums)
  - [通信是否正常 — is_ok()](#通信是否正常--is_ok)
  - [获取数据接收频率 — get_fps()](#获取数据接收频率--get_fps)
- [数据读取](#数据读取)
  - [MessageAbstract 返回值通用说明](#messageabstract-返回值通用说明)
  - [读取机械臂状态 — get_arm_status()](#读取机械臂状态--get_arm_status)
  - [读取关节角度 — get_joint_angles()](#读取关节角度--get_joint_angles)
  - [读取法兰位姿 — get_flange_pose()](#读取法兰位姿--get_flange_pose)
  - [读取电机状态 — get_motor_states()](#读取电机状态--get_motor_states)
  - [读取驱动器状态 — get_driver_states()](#读取驱动器状态--get_driver_states)
  - [读取关节使能状态 — get_joint_enable_status()](#读取关节使能状态--get_joint_enable_status)
  - [读取全部关节使能状态 — get_joints_enable_status_list()](#读取全部关节使能状态--get_joints_enable_status_list)
  - [读取固件信息 — get_firmware()](#读取固件信息--get_firmware)
- [参数设定](#参数设定)
  - [设定运行速度 — set_speed_percent()](#设定运行速度--set_speed_percent)
  - [设定安装位置 — set_installation_pos()](#设定安装位置--set_installation_pos)
  - [设定运动模式 — set_motion_mode()](#设定运动模式--set_motion_mode)
  - [设定负载 — set_payload()](#设定负载--set_payload)
- [TCP 相关](#tcp-相关)
  - [设置 TCP 偏移 — set_tcp_offset()](#设置-tcp-偏移--set_tcp_offset)
  - [获取 TCP 位姿 — get_tcp_pose()](#获取-tcp-位姿--get_tcp_pose)
  - [法兰位姿转 TCP 位姿 — get_flange2tcp_pose()](#法兰位姿转-tcp-位姿--get_flange2tcp_pose)
  - [TCP 位姿转法兰位姿 — get_tcp2flange_pose()](#tcp-位姿转法兰位姿--get_tcp2flange_pose)
- [Leader-Follower 臂](#leader-follower-臂)
  - [设定主导臂（Leader）模式 — set_leader_mode()](#设定主导臂leader模式--set_leader_mode)
  - [设定跟随臂（Follower）模式 — set_follower_mode()](#设定跟随臂follower模式--set_follower_mode)
  - [主导臂（Leader）回 Home — move_leader_to_home()](#主导臂leader回-home--move_leader_to_home)
  - [恢复主导臂（Leader）零力拖动 — restore_leader_drag_mode()](#恢复主导臂leader零力拖动--restore_leader_drag_mode)
  - [读取主导臂（Leader）关节角度 — get_leader_joint_angles()](#读取主导臂leader关节角度--get_leader_joint_angles)
- [运动控制](#运动控制)
  - [使能 — enable()](#使能--enable)
  - [失能 — disable()](#失能--disable)
  - [重置 — reset()](#重置--reset)
  - [电子急停 — electronic_emergency_stop()](#电子急停--electronic_emergency_stop)
  - [关节运动 — move_j()](#关节运动--move_j)
  - [关节运动 (Follower 模式) — move_js()](#关节运动-follower-模式--move_js)
  - [点到点运动 — move_p()](#点到点运动--move_p)
  - [直线运动 — move_l()](#直线运动--move_l)
  - [圆弧运动 — move_c()](#圆弧运动--move_c)
  - [单关节 MIT 控制 — move_mit()](#单关节-mit-控制--move_mit)
- [高级参数读取与配置](#高级参数读取与配置)
  - [读取关节角度/速度限制 — get_joint_angle_vel_limits()](#读取关节角度速度限制--get_joint_angle_vel_limits)
  - [读取关节加速度限制 — get_joint_acc_limits()](#读取关节加速度限制--get_joint_acc_limits)
  - [读取法兰速度/加速度限制 — get_flange_vel_acc_limits()](#读取法兰速度加速度限制--get_flange_vel_acc_limits)
  - [读取碰撞防护等级 — get_crash_protection_rating()](#读取碰撞防护等级--get_crash_protection_rating)
  - [关节置零/标定 — calibrate_joint()](#关节置零标定--calibrate_joint)
  - [配置关节角度/速度限制 — set_joint_angle_vel_limits()](#配置关节角度速度限制--set_joint_angle_vel_limits)
  - [配置关节加速度限制 — set_joint_acc_limits()](#配置关节加速度限制--set_joint_acc_limits)
  - [配置法兰速度/加速度限制 — set_flange_vel_acc_limits()](#配置法兰速度加速度限制--set_flange_vel_acc_limits)
  - [配置碰撞防护等级 — set_crash_protection_rating()](#配置碰撞防护等级--set_crash_protection_rating)
  - [恢复法兰限制默认值 — set_flange_vel_acc_limits_to_default()](#恢复法兰限制默认值--set_flange_vel_acc_limits_to_default)
  - [恢复关节限制默认值 — set_joint_angle_vel_acc_limits_to_default()](#恢复关节限制默认值--set_joint_angle_vel_acc_limits_to_default)
  - [设置 Link 速度/加速度周期反馈 — set_links_vel_acc_period_feedback()](#设置-link-速度加速度周期反馈--set_links_vel_acc_period_feedback)

---

## 导入模块

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory
```

---

## 创建实例并连接

### 创建配置参数 — `create_agx_arm_config()`

**功能说明：** 生成机械臂所需的配置字典，用于后续创建 Driver 实例。

**函数定义：**

```python
create_agx_arm_config(
    robot: Literal["nero", "piper", "piper_h", "piper_l", "piper_x"],
    comm: Literal["can"] = "can",
    firmeware_version: str = "default",
    **kwargs,
) -> dict
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `robot` | `str` | 机械臂型号，可选值：`"nero"` / `"piper"` / `"piper_h"` / `"piper_l"` / `"piper_x"` |
| `comm` | `str` | 通讯类型，可选值：`"can"`（默认）。注意：`comm` 不是 CAN 通道名，CAN 通道由 `channel` 指定 |
| `firmeware_version` | `str` | 主控固件版本，默认 `"default"` |

**可选关键字参数（`**kwargs`）：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_limits` | `dict` | 自定义关节限位（单位：rad）。默认自动赋值，暂不会将手动输入的限位生效到实际控制中。示例见下文 |
| `channel` | `str` | CAN 通道名，默认 `"can0"` |
| `interface` | `str` | CAN 接口类型，默认 `"socketcan"`。Linux 下官方 CAN 模块必须为 `"socketcan"`；macOS 需换用串口 CAN 模块并设为 `"slcan"` |
| `bitrate` | `int` | CAN 波特率，默认 `1000000`（1 Mbps） |
| `enable_check_can` | `bool` | 是否在创建 Comm 实例时检查 CAN 模块，默认 `True` |
| `auto_connect` | `bool` | 是否自动创建 CAN Bus 实例，默认 `True` |
| `timeout` | `float` | CAN Bus 读写超时时间（秒），默认 `1.0` |

**返回值：** `dict`

返回结构示例：

```json
{
    "robot": "piper",
    "comm": {
        "type": "can",
        "can": {
            "channel": "can0",
            "interface": "socketcan",
            "bitrate": 1000000,
            "enable_check_can": true,
            "auto_connect": true,
            "timeout": 1.0
        }
    },
    "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "joint_limits": {
        "joint1": [-2.617994, 2.617994],
        "joint2": [0.0, 3.141593],
        "joint3": [-2.967060, 0.0],
        "joint4": [-1.745330, 1.745330],
        "joint5": [-1.221730, 1.221730],
        "joint6": [-2.094396, 2.094396]
    }
}
```

> **提示：** 关节运动限位单位为 **弧度（rad）**，夹爪运动限位单位为 **米（m）**。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
print(cfg)
```

---

### 创建机械臂 Driver 实例 — `AgxArmFactory.create_arm()`

**功能说明：** 根据配置字典，通过工厂方法创建对应的机械臂 Driver 实例。

**函数定义：**

```python
create_arm(cls, config: dict, **kwargs) -> T
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `config` | `dict` | 由 `create_agx_arm_config()` 生成的配置字典 |

**返回值：** `Driver` — 不同臂型号、通讯方式、固件版本对应不同的实例。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
```

---

### 创建连接 — `connect()`

**功能说明：** 创建连接并启动数据读取线程。

**函数定义：**

```python
connect(self, start_read_thread: bool = True) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `start_read_thread` | `bool` | 是否启动读取数据线程，默认 `True` |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
```

---

### 初始化末端执行器 — `init_effector()`

**功能说明：** 初始化末端执行器 Driver，并返回对应的执行器实例（例如夹爪 / 灵巧手等）。

> **注意：** 同一个 `robot` 实例 **只能初始化一次** 执行器。如需切换到其它执行器类型，请创建新的机械臂实例。

**函数定义：**

```python
init_effector(self, effector: str) -> EffectorDriver
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `effector` | `str` | 执行器类型（建议使用 `robot.OPTIONS.EFFECTOR.xxx` 常量） |

**返回值：** `EffectorDriver`

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
```

---

## 通用状态

### 获取关节数量 — `joint_nums`

**功能说明：** 获取机械臂关节数量（例如 Piper 为 6）。

**属性定义：**

```python
joint_nums: int
```

**返回值：** `int`

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

print("robotic arm joint_nums =", robot.joint_nums)

for joint_index in range(1, robot.joint_nums + 1):
    start_t = time.monotonic()
    while True:
        if robot.enable(joint_index):
            print(f"enable joint {joint_index} success")
            break
        if time.monotonic() - start_t > 5.0:
            print(f"enable joint {joint_index} timeout (5s)")
            break
        time.sleep(0.01)
```

---

### 通信是否正常 — `is_ok()`

**功能说明：** 判断机械臂数据接收是否正常。该值由 SDK 内部的数据监控逻辑根据"最近一段时间是否持续收不到数据"计算得出。

**函数定义：**

```python
is_ok(self) -> bool
```

**返回值：** `bool`

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

time.sleep(0.5)
print("robotic arm is_ok =", robot.is_ok())
```

---

### 获取数据接收频率 — `get_fps()`

**功能说明：** 获取机械臂数据监控的接收频率（Hz），是 SDK 对解析器收到数据的统计值。

**函数定义：**

```python
get_fps(self) -> float
```

**返回值：** `float`（单位：Hz）

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

time.sleep(0.5)
print("robotic arm fps =", robot.get_fps(), "Hz")
```

---

## 数据读取

### MessageAbstract 返回值通用说明

本 SDK 多数读取接口返回 `MessageAbstract[T] | None`，其通用字段如下：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `ret.msg` | `T` | 消息数据本体（例如 `list[float]` 或某个反馈消息结构体） |
| `ret.hz` | `float` | 该消息类型的接收频率（SDK 统计），单位：Hz |
| `ret.timestamp` | `float` | 消息时间戳（SDK 记录），单位：s |

---

### 读取机械臂状态 — `get_arm_status()`

**功能说明：** 读取机械臂整体状态反馈（控制模式、运动模式、急停/异常状态、轨迹点编号等）。

**函数定义：**

```python
get_arm_status(self) -> MessageAbstract[ArmMsgFeedbackStatus] | None
```

**返回值：** `MessageAbstract[ArmMsgFeedbackStatus] | None`

**消息字段（`.msg`）：**

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `ctrl_mode` | `int` | 控制模式（待机 / CAN / 示教 / 以太网 / WiFi / 离线轨迹等） |
| `arm_status` | `int` | 机械臂状态（正常 / 急停 / 奇异 / 超限 / 碰撞等） |
| `mode_feedback` | `int` | 模式反馈（MOVE P/J/L/C/MIT 等） |
| `teach_status` | `int` | 示教状态（开始记录 / 结束记录 / 执行 / 暂停 / 继续 / 终止等） |
| `motion_status` | `int` | 运动状态：`0` 已到达；`1` 未到达 |
| `trajectory_num` | `int` | 轨迹点编号（离线轨迹模式下反馈） |
| `err_status` | `object` | 错误状态位（关节角度超限 / 关节通信异常等） |

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    arm_status = robot.get_arm_status()
    if arm_status is not None:
        print(arm_status.msg)
        print(arm_status.hz, arm_status.timestamp)
    time.sleep(0.02)
```

---

### 读取关节角度 — `get_joint_angles()`

**功能说明：** 获取当前各关节角度。

**函数定义：**

```python
get_joint_angles(self) -> MessageAbstract[list[float]] | None
```

**返回值：** `MessageAbstract[list[float]] | None`

`.msg` 为长度 6 的 `list[float]`：`[j1, j2, j3, j4, j5, j6]`，单位：**rad**。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    ja = robot.get_joint_angles()
    if ja is not None:
        print(ja.msg)
        print(ja.hz, ja.timestamp)
    time.sleep(0.005)
```

---

### 读取法兰位姿 — `get_flange_pose()`

**功能说明：** 获取末端法兰位姿。

> **术语说明：** `flange` 指机械臂最后一个连杆（末端连杆）的安装法兰/连接面，是工具/末端执行器的机械安装接口。

**函数定义：**

```python
get_flange_pose(self) -> MessageAbstract[list[float]] | None
```

**返回值：** `MessageAbstract[list[float]] | None`

`.msg` 为长度 6 的 `list[float]`：`[x, y, z, roll, pitch, yaw]`

- `x, y, z`：位置坐标（单位：m）
- `roll, pitch, yaw`：姿态欧拉角（单位：rad，分别对应绕 X/Y/Z 轴旋转）

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    fp = robot.get_flange_pose()
    if fp is not None:
        print(fp.msg)
        print(fp.hz, fp.timestamp)
    time.sleep(0.005)
```

---

### 读取电机状态 — `get_motor_states()`

**功能说明：** 读取指定关节的电机高速反馈（位置 / 速度 / 电流 / 扭矩）。

**函数定义：**

```python
get_motor_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6]) -> MessageAbstract[ArmMsgFeedbackHighSpd] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号，范围：`1~6` |

**返回值：** `MessageAbstract[ArmMsgFeedbackHighSpd] | None`

**消息字段（`.msg`）：**

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `position` | `float` | 电机位置（rad） |
| `velocity` | `float` | 电机速度（rad/s） |
| `current` | `float` | 电机电流（A） |
| `torque` | `float` | 电机扭矩（N·m） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

ms = robot.get_motor_states(1)
if ms is not None:
    print(ms.msg.position, ms.msg.velocity, ms.msg.current, ms.msg.torque)
    print(ms.hz, ms.timestamp)
```

---

### 读取驱动器状态 — `get_driver_states()`

**功能说明：** 读取指定关节的驱动器低速反馈（电压 / 温度 / 母线电流 / 驱动状态位等）。

**函数定义：**

```python
get_driver_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6]) -> MessageAbstract[ArmMsgFeedbackLowSpd] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号，范围：`1~6` |

**返回值：** `MessageAbstract[ArmMsgFeedbackLowSpd] | None`

**消息字段（`.msg`）：**

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `vol` | `float` | 驱动电压 |
| `foc_temp` | `float` | 驱动温度（°C） |
| `motor_temp` | `float` | 电机温度（°C） |
| `bus_current` | `float` | 母线电流（A） |
| `foc_status` | `object` | 驱动状态位（电压过低 / 过温 / 过流 / 碰撞 / 失能 / 堵转等） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

ds = robot.get_driver_states(1)
if ds is not None:
    print(ds.msg.vol, ds.msg.foc_temp, ds.msg.motor_temp, ds.msg.bus_current)
    print(ds.msg.foc_status.driver_enable_status)
    print(ds.hz, ds.timestamp)
```

---

### 读取关节使能状态 — `get_joint_enable_status()`

**功能说明：** 获取指定关节电机的使能状态。

**函数定义：**

```python
get_joint_enable_status(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255]) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 查询单关节；`255` 查询全部关节（内部使用 `all([...])` 汇总） |

**返回值：** `bool` — `True` 为已使能，`False` 为未使能或当前无反馈。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

if robot.get_joint_enable_status(1):
    print("关节 1 电机已使能")
```

---

### 读取全部关节使能状态 — `get_joints_enable_status_list()`

**功能说明：** 读取全部关节电机的使能状态列表（按关节 1~6 顺序）。

**函数定义：**

```python
get_joints_enable_status_list(self) -> list[bool]
```

**返回值：** `list[bool]`

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

print(robot.get_joints_enable_status_list())
```

---

### 读取固件信息 — `get_firmware()`

**功能说明：** 读取机械臂固件信息（软件版本 / 硬件版本 / 生产日期等）。该接口会下发查询帧并等待对应反馈。

**函数定义：**

```python
get_firmware(self, timeout: float = 1.0, min_interval: float = 1.0) -> dict | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `timeout` | `float` | 等待反馈超时时间（秒），默认 `1.0`；`0.0` 表示非阻塞 |
| `min_interval` | `float` | 最小请求间隔（秒），默认 `1.0` |

**返回值：** `dict | None`

常见字段：

| Key | 类型 | 说明 |
| --- | --- | --- |
| `software_version` | `str` | 软件版本（例如 `S-V1.8-2`） |
| `hardware_version` | `str` | 硬件版本（例如 `H-V1.2-1`） |
| `production_date` | `str` | 生产日期（例如 `250925`） |
| `node_type` | `str` | 节点类型 |
| `node_number` | `int` | 节点编号 |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

fw = robot.get_firmware()
if fw is not None:
    print(fw)
```

---

## 参数设定

### 设定运行速度 — `set_speed_percent()`

**功能说明：** 设定机械臂在位置速度模式下的运行速度百分比，适用于 `move_j` / `move_p` / `move_l` / `move_c`。

**函数定义：**

```python
set_speed_percent(self, percent: int = 100) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `percent` | `int` | 运行速度百分比，范围 `[0, 100]`，默认 `100` |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_speed_percent(100)
```

---

### 设定安装位置 — `set_installation_pos()`

**功能说明：** 设定机械臂安装位置，支持水平、朝左和朝右三个方向。

**函数定义：**

```python
set_installation_pos(self, pos: Literal["horizontal", "left", "right"] = "horizontal") -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `pos` | `str` | 安装方向，可选值：`'horizontal'` / `'left'` / `'right'`，默认：`'horizontal'`（建议使用 `robot.OPTIONS.INSTALLATION_POS.xxx` 常量） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_installation_pos(robot.OPTIONS.INSTALLATION_POS.HORIZONTAL)
```

---

### 设定运动模式 — `set_motion_mode()`

**功能说明：** 设置运动模式。

| 模式 | 类型 | 说明 |
| --- | --- | --- |
| `move_p` / `move_j` / `move_l` / `move_c` | **位置速度模式** | 底层会对接收到的消息进行平滑处理，保证运动连续稳定 |
| `move_mit` / `move_js` | **MIT 电机透传模式** | 底层仅负责消息转发，**不进行任何平滑处理**，适用于直接控制电机的场景 |

> **提示：** 调用任一 `move_*` 运动指令时，系统 **会自动切换至对应的运动模式**，因此通常 **无需手动调用 `set_motion_mode()`**。

**函数定义：**

```python
set_motion_mode(self, motion_mode: Literal["p", "j", "l", "c", "mit", "js"] = "p") -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `motion_mode` | `str` | 运动模式，可选值：`'p'` / `'j'` / `'l'` / `'c'` / `'mit'` / `'js'`，默认：`'p'`（建议使用 `robot.OPTIONS.MOTION_MODE.xxx` 常量） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)
```

---

### 设定负载 — `set_payload()`

**功能说明：** 设定机械臂负载（Payload）。

**函数定义：**

```python
set_payload(self, load: Literal['empty', 'half', 'full'] = 'empty', timeout: float = 1.0) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `load` | `str` | 负载等级，可选值：`'empty'`（空载）/ `'half'`（半载）/ `'full'`（满载），默认：`'empty'`（建议使用 `robot.OPTIONS.PAYLOAD.xxx` 常量） |
| `timeout` | `float` | 等待反馈的超时时间（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示收到指令应答，但并不表示设定一定成功。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_payload(robot.OPTIONS.PAYLOAD.FULL)
```

---

## TCP 相关

### 设置 TCP 偏移 — `set_tcp_offset()`

**功能说明：** 设置 TCP（工具中心点）相对于法兰（`flange`）的偏移位姿（在 **法兰坐标系** 下）。默认无偏移：`[0, 0, 0, 0, 0, 0]`。

> **提示：** 该偏移值仅保存在 SDK/Driver 实例内，不会下发到控制器。

**函数定义：**

```python
set_tcp_offset(self, pose: list[float]) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `pose` | `list[float]` | TCP 在法兰坐标系下的位姿偏移 `[x, y, z, roll, pitch, yaw]`：`x, y, z` 为位置（m）；`roll, pitch, yaw` 为欧拉角（rad）。范围：`roll/yaw` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]` |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])
```

---

### 获取 TCP 位姿 — `get_tcp_pose()`

**功能说明：** 获取 TCP 位姿。该接口会先读取法兰位姿，然后根据 `set_tcp_offset()` 保存的偏移值做刚体变换得到 TCP 位姿。若未设置偏移，则 TCP 位姿与法兰位姿相同。

**函数定义：**

```python
get_tcp_pose(self) -> MessageAbstract[list[float]] | None
```

**返回值：** `MessageAbstract[list[float]] | None`

`.msg` 为长度 6 的 `list[float]`：`[x, y, z, roll, pitch, yaw]`（m / rad）。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

while True:
    tcp = robot.get_tcp_pose()
    if tcp is not None:
        print(tcp.msg)
        print(tcp.hz, tcp.timestamp)
    time.sleep(0.02)
```

---

### 法兰位姿转 TCP 位姿 — `get_flange2tcp_pose()`

**功能说明：** 输入法兰位姿（基座/世界坐标系下），根据 `set_tcp_offset()` 保存的偏移值算出对应的 TCP 位姿。

**函数定义：**

```python
get_flange2tcp_pose(self, flange_pose: list[float]) -> list[float]
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `flange_pose` | `list[float]` | 法兰位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。范围：`roll/yaw` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]` |

**返回值：** `list[float]` — TCP 位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

# 直接指定法兰位姿
tcp_pose = robot.get_flange2tcp_pose([0.30, 0.0, 0.30, 0.0, 1.5707, 0.0])
print("tcp_pose =", tcp_pose)

# 从当前位姿获取，结果与 get_tcp_pose() 得到的 pose 相同
flange_pose = robot.get_flange_pose()
if flange_pose is not None:
    tcp_pose = robot.get_flange2tcp_pose(flange_pose)
    print("tcp_pose =", tcp_pose)
```

---

### TCP 位姿转法兰位姿 — `get_tcp2flange_pose()`

**功能说明：** 输入目标 TCP 位姿（基座/世界坐标系下），根据 `set_tcp_offset()` 保存的偏移值算出对应的目标法兰位姿。将返回的法兰位姿传给 `move_p()`，即可实现 **TCP 运动到目标 TCP 位姿**。

**函数定义：**

```python
get_tcp2flange_pose(self, tcp_pose: list[float]) -> list[float]
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `tcp_pose` | `list[float]` | 目标 TCP 位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。范围：`roll/yaw` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]` |

**返回值：** `list[float]` — 目标法兰位姿 `[x, y, z, roll, pitch, yaw]`（m / rad），可直接用于 `move_p()`。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

target_tcp_pose = [0.30, 0.0, 0.30, 0.0, 1.5707, 0.0]
target_flange_pose = robot.get_tcp2flange_pose(target_tcp_pose)
print("target_flange_pose =", target_flange_pose)

# robot.move_p(target_flange_pose)  # 注意：会触发运动
```

---

## Leader-Follower 臂

### 设定主导臂（Leader）模式 — `set_leader_mode()`

**功能说明：** 将机械臂设置为 **主导臂（Leader）零力拖动模式**（Leader-Follower 协同场景下的"主导臂（Leader Arm）"）。进入该模式后，主导臂（Leader Arm）通常处于可拖动/零力拖动状态。

> **提示：** 该模式用于Leader-Follower 臂联动/示教等场景。若仅使用单臂，可忽略该接口。

**函数定义：**

```python
set_leader_mode(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_leader_mode()
```

---

### 设定跟随臂（Follower）模式 — `set_follower_mode()`

**功能说明：** 将机械臂设置为 **跟随臂（Follower）受控模式**（Leader-Follower 协同场景下的"跟随臂（Follower Arm）"），跟随臂（Follower Arm）跟随主导臂（Leader Arm）控制/指令运行。可与 `set_leader_mode()` 配套使用。

**函数定义：**

```python
set_follower_mode(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_follower_mode()
```

---

### 主导臂（Leader）回 Home — `move_leader_to_home()`

**功能说明：** 让主导臂（Leader Arm）回到 Home 位姿。完成后建议调用 `restore_leader_drag_mode()` 恢复主导臂（Leader Arm）"零力拖动"状态。

**函数定义：**

```python
move_leader_to_home(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_leader_mode()
robot.move_leader_to_home()
robot.restore_leader_drag_mode()
```

---

### 恢复主导臂（Leader）零力拖动 — `restore_leader_drag_mode()`

**功能说明：** 将主导臂（Leader Arm）恢复为"零力拖动"状态，通常用于 `move_leader_to_home()` 之后。

**函数定义：**

```python
restore_leader_drag_mode(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_leader_mode()
robot.move_leader_to_home()
robot.restore_leader_drag_mode()
```

---

### 读取主导臂（Leader）关节角度 — `get_leader_joint_angles()`

**功能说明：** 获取主导臂（Leader Arm）关节角度消息，用于控制跟随臂（Follower Arm）。

**函数定义：**

```python
get_leader_joint_angles(self) -> MessageAbstract[list[float]] | None
```

**返回值：** `MessageAbstract[list[float]] | None`

`.msg` 为长度 6 的 `list[float]`：`[j1, j2, j3, j4, j5, j6]`，单位：**rad**。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_leader_mode()

while True:
    mja = robot.get_leader_joint_angles()
    if mja is not None:
        print(mja.msg)
        print(mja.hz, mja.timestamp)
    time.sleep(0.005)
```

---

## 运动控制

### 使能 — `enable()`

**功能说明：** 将机械臂使能上电。

**函数定义：**

```python
enable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 使能单关节；`255` 使能全部关节，默认：`255` |

**返回值：** `bool` — `True` 为使能成功。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)
```

---

### 失能 — `disable()`

**功能说明：** 将机械臂失电。

> **⚠️ 安全警告：** 执行该指令时，如果机械臂关节处于抬起状态，会 **立刻掉落**。请确保机械臂处于安全状态后再使用。

**函数定义：**

```python
disable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 失能单关节；`255` 失能全部关节，默认：`255` |

**返回值：** `bool` — `True` 为失能成功。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.disable():
    time.sleep(0.01)
```

---

### 重置 — `reset()`

**功能说明：** 将机械臂模式重置并令机械臂立刻失电。

> **⚠️ 安全警告：** 执行该指令时，如果机械臂关节处于抬起状态，会 **立刻掉落**。请确保机械臂处于安全状态后再使用。

**函数定义：**

```python
reset(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.reset()
```

---

### 电子急停 — `electronic_emergency_stop()`

**功能说明：** 将机械臂设置为急停状态。如果执行时机械臂关节处于抬起状态，机械臂会 **缓慢以恒定阻尼落下**（不会立刻掉落）。

**函数定义：**

```python
electronic_emergency_stop(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.electronic_emergency_stop()
```

---

### 关节运动 — `move_j()`

**功能说明：** 关节位置速度控制模式，设定各关节目标角度。

**函数定义：**

```python
move_j(self, joints: list[float]) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joints` | `list[float]` | 长度 6 的目标角度数组 `[j1, j2, j3, j4, j5, j6]`（单位：rad，精度：1.74532925199e-5）。关节限位取决于机械臂型号配置 |

> **注意：** 连续执行该指令会覆盖上一次的目标值。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_j([0, 0.4, -0.4, 0, -0.4, 0])

# 等待运动结束（带 5s 超时）
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("已到达目标位置")
        break
    if time.monotonic() - start_t > 5.0:
        print("等待运动结束超时（5s）")
        break
    time.sleep(0.1)
```

---

### 关节运动 (Follower 模式) — `move_js()`

**功能说明：** 将机械臂切换到 **JS（Follower）模式**（MIT 透传模式），并下发关节目标角度。与 `move_j` 相比，`move_js` 更偏向"快速响应"控制：**不做平滑处理、无轨迹规划**，控制器/驱动器会尽可能快地响应目标角度。

**函数定义：**

```python
move_js(self, joints: list[float]) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joints` | `list[float]` | 长度 6 的目标角度数组 `[j1, j2, j3, j4, j5, j6]`（单位：rad，精度：1.74532925199e-5）。关节限位取决于机械臂型号配置 |

> **⚠️ 风险等级：极高**
>
> 1. 该模式可能导致 **冲击、振荡、失稳** 等风险，请仅在充分评估安全与控制稳定性的前提下使用，并确保随时可急停。
> 2. **无平滑过程、无轨迹规划**，控制器/驱动器尝试以最快响应到达目标，可能产生冲击和振荡。
> 3. 连续执行该指令会覆盖上一次的目标值。
> 4. 由于响应变快，关节的控制力度相较于位置速度模式小，刚度也会变小。
> 5. 在旧版本固件（低于 `S-V1.8-5`）下，如果机械臂当前为Follower 模式，想切换到位置速度控制模式需要先执行 `robot.reset()`（机械臂会重置掉电），然后再执行 `move_j` 才能正常控制。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.move_js([0, 0.4, -0.4, 0, -0.4, 0])
```

---

### 点到点运动 — `move_p()`

**功能说明：** 发送目标法兰位姿，机械臂根据当前关节位置和目标位姿进行关节角度解算并运动。

**函数定义：**

```python
move_p(self, pose: list[float]) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `pose` | `list[float]` | 目标位姿 `[x, y, z, roll, pitch, yaw]`：`x, y, z` 为位置（m，精度：1e-6）；`roll, pitch, yaw` 为欧拉角（rad，精度：1.74532925199e-5）。姿态范围：`roll` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]`，`yaw` ∈ `[-π, π]` |

> **注意：** 连续执行该指令会覆盖上一次的目标值。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_p([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

# 等待运动结束（带 5s 超时）
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("已到达目标位置")
        break
    if time.monotonic() - start_t > 5.0:
        print("等待运动结束超时（5s）")
        break
    time.sleep(0.1)
```

---

### 直线运动 — `move_l()`

**功能说明：** 发送目标法兰位姿，机械臂根据当前位姿和目标位姿进行直线轨迹规划。

**函数定义：**

```python
move_l(self, pose: list[float]) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `pose` | `list[float]` | 目标位姿 `[x, y, z, roll, pitch, yaw]`：`x, y, z` 为位置（m，精度：1e-6）；`roll, pitch, yaw` 为欧拉角（rad，精度：1.74532925199e-5）。姿态范围：`roll` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]`，`yaw` ∈ `[-π, π]` |

> **注意：** 连续执行该指令虽然可以覆盖上一次的目标，但由于底层每接收到新点位都需要重新进行直线规划，因此 **不能使用该指令连续发送目标点**。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_l([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

# 等待运动结束（带 5s 超时）
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("已到达目标位置")
        break
    if time.monotonic() - start_t > 5.0:
        print("等待运动结束超时（5s）")
        break
    time.sleep(0.1)
```

---

### 圆弧运动 — `move_c()`

**功能说明：** 通过"起点 / 中间点 / 终点"三个目标法兰位姿进行圆弧轨迹规划并执行。

**函数定义：**

```python
move_c(self, start_pose: list[float], mid_pose: list[float], end_pose: list[float]) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `start_pose` | `list[float]` | 起点位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。姿态范围：`roll` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]`，`yaw` ∈ `[-π, π]` |
| `mid_pose` | `list[float]` | 中间点位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。姿态范围：`roll` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]`，`yaw` ∈ `[-π, π]` |
| `end_pose` | `list[float]` | 终点位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。姿态范围：`roll` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]`，`yaw` ∈ `[-π, π]` |

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
sp = [0.2, 0.0, 0.3, 0.0, 1.5708, 0.0]
mp = [0.2, 0.05, 0.35, 0.0, 1.5708, 0.0]
ep = [0.2, 0.0, 0.4, 0.0, 1.5708, 0.0]
robot.move_c(sp, mp, ep)

# 等待运动结束（带 5s 超时）
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("已到达目标位置")
        break
    if time.monotonic() - start_t > 5.0:
        print("等待运动结束超时（5s）")
        break
    time.sleep(0.1)
```

---

### 单关节 MIT 控制 — `move_mit()`

**功能说明：** 使用关节驱动底层的 MIT 控制接口，控制单个关节电机，可实现电流模拟的力矩控制。

控制器概念上会计算参考力矩：

$$T_{\text{ref}} = k_p \cdot (p_{\text{des}} - p) + k_d \cdot (v_{\text{des}} - v) + T_{\text{ff}}$$

其中 \(p/v\) 为关节实测位置/速度。

**典型用法建议：**

| 控制方式 | 参数设置 | 说明 |
| --- | --- | --- |
| **速度控制** | `kp = 0`, `kd ≠ 0` | 主要通过 `v_des` 控制 |
| **力矩控制** | `kp = 0`, `kd = 0` | 主要通过 `t_ff` 控制 |
| **位置控制** | `kp ≠ 0`, `kd ≠ 0` | 不建议将 `kd` 设为 0，适当增大阻尼可降低振荡风险 |

> **⚠️ 风险提示：** MIT 属于较底层控制接口，参数不当可能引发 **冲击 / 振荡 / 不稳定**。建议从小增益开始调试，并在安全工况下使用。

**函数定义：**

```python
move_mit(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    p_des: float = 0.0,
    v_des: float = 0.0,
    kp: float = 10.0,
    kd: float = 0.8,
    t_ff: float = 0.0,
) -> None
```

**参数说明：**

| 名称 | 类型 | 范围 | 单位 | 默认值 | 精度 |
| --- | --- | --- | --- | --- | --- |
| `joint_index` | `int` | `1~6` | — | — | — |
| `p_des` | `float` | `[-12.5, 12.5]` | rad | `0.0` | 3.815e-4 |
| `v_des` | `float` | `[-45.0, 45.0]` | rad/s | `0.0` | 2.198e-2 |
| `kp` | `float` | `[0.0, 500.0]` | — | `10.0` | 1.221e-1 |
| `kd` | `float` | `[-5.0, 5.0]` | — | `0.8` | 2.442e-3 |
| `t_ff` | `float` | `[-8.0, 8.0]` | N·m | `0.0` | 6.275e-2 |

> **注意：** 连续执行该指令会覆盖上一次的目标值。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

for i in range(1, robot.joint_nums + 1):
    robot.move_mit(
        joint_index=i,
        p_des=0.0,
        v_des=0.0,
        kp=10.0,
        kd=0.8,
        t_ff=0.0,
    )
```

---

## 高级参数读取与配置

### 读取关节角度/速度限制 — `get_joint_angle_vel_limits()`

**功能说明：** 查询指定关节的角度限制与速度限制（由控制器反馈）。

**函数定义：**

```python
get_joint_angle_vel_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号，范围：`1~6` |
| `timeout` | `float` | 等待反馈超时（秒），默认 `1.0`；`0.0` 表示非阻塞 |
| `min_interval` | `float` | 最小请求间隔（秒），默认 `1.0` |

**返回值：** `MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd] | None`

**消息字段（`.msg`）：**

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `min_angle_limit` | `float` | 最小角度限制（rad） |
| `max_angle_limit` | `float` | 最大角度限制（rad） |
| `min_joint_spd` | `float` | 最小关节速度限制（rad/s） |
| `max_joint_spd` | `float` | 最大关节速度限制（rad/s） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_joint_angle_vel_limits(1)
if limit is not None:
    print(limit.msg.min_angle_limit, limit.msg.max_angle_limit)
    print(limit.msg.min_joint_spd, limit.msg.max_joint_spd)
```

---

### 读取关节加速度限制 — `get_joint_acc_limits()`

**功能说明：** 查询指定关节的最大加速度限制。

**函数定义：**

```python
get_joint_acc_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号，范围：`1~6` |
| `timeout` | `float` | 等待反馈超时（秒），默认 `1.0`；`0.0` 表示非阻塞 |
| `min_interval` | `float` | 最小请求间隔（秒），默认 `1.0` |

**返回值：** `MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit] | None`

**消息字段（`.msg`）：**

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `max_joint_acc` | `float` | 最大关节加速度限制（rad/s²） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_joint_acc_limits(1)
if limit is not None:
    print(limit.msg.max_joint_acc)
    print(limit.hz, limit.timestamp)
```

---

### 读取法兰速度/加速度限制 — `get_flange_vel_acc_limits()`

**功能说明：** 查询末端最大线速度/角速度与线加速度/角加速度限制。

**函数定义：**

```python
get_flange_vel_acc_limits(self, timeout: float = 1.0, min_interval: float = 1.0) -> MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `timeout` | `float` | 等待反馈超时（秒），默认 `1.0`；`0.0` 表示非阻塞 |
| `min_interval` | `float` | 最小请求间隔（秒），默认 `1.0` |

**返回值：** `MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None`

**消息字段（`.msg`）：**

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `end_max_linear_vel` | `float` | 末端最大线速度（m/s） |
| `end_max_angular_vel` | `float` | 末端最大角速度（rad/s） |
| `end_max_linear_acc` | `float` | 末端最大线加速度（m/s²） |
| `end_max_angular_acc` | `float` | 末端最大角加速度（rad/s²） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_flange_vel_acc_limits()
if limit is not None:
    print(
        limit.msg.end_max_linear_vel,
        limit.msg.end_max_angular_vel,
        limit.msg.end_max_linear_acc,
        limit.msg.end_max_angular_acc,
    )
    print(limit.hz, limit.timestamp)
```

---

### 读取碰撞防护等级 — `get_crash_protection_rating()`

**功能说明：** 查询各关节碰撞防护等级（控制器返回列表）。

**函数定义：**

```python
get_crash_protection_rating(
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[list[int]] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `timeout` | `float` | 等待反馈超时（秒），默认 `1.0`；`0.0` 表示非阻塞 |
| `min_interval` | `float` | 最小请求间隔（秒），默认 `1.0` |

**返回值：** `MessageAbstract[list[int]] | None`

`.msg` 为碰撞防护等级列表（按关节顺序），每项为 `int`（范围：`0~8`）。**等级越高越敏感，越容易触发碰撞保护机制**（更保守）。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

rating = robot.get_crash_protection_rating()
if rating is not None:
    print(rating.msg)
    print(rating.hz, rating.timestamp)
```

---

### 关节置零/标定 — `calibrate_joint()`

**功能说明：** 对指定关节执行置零/标定流程（等待控制器 ACK/响应并返回结果）。

**函数定义：**

```python
calibrate_joint(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | `1~6` 标定单关节；`255` 标定全部 |
| `timeout` | `float` | 等待响应超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示收到成功响应；`False` 表示超时或失败。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

joint_index = 1
robot.disable(joint_index)
time.sleep(0.2)
input("请手动将关节移动到零位位置后按回车继续...")

if robot.calibrate_joint(joint_index):
    print("calibrate_joint success")
```

---

### 配置关节角度/速度限制 — `set_joint_angle_vel_limits()`

**功能说明：** 设置关节角度/速度限制，并可通过读回校验是否生效。

**函数定义：**

```python
set_joint_angle_vel_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    min_angle_limit: Optional[float] = None,
    max_angle_limit: Optional[float] = None,
    max_joint_spd: Optional[float] = None,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 配置单关节；`255` 配置全部 |
| `min_angle_limit` | `Optional[float]` | 最小角度限制（rad）；`None` 表示不配置 |
| `max_angle_limit` | `Optional[float]` | 最大角度限制（rad）；`None` 表示不配置 |
| `max_joint_spd` | `Optional[float]` | 最大关节速度限制（rad/s）；`None` 表示不配置 |
| `timeout` | `float` | 等待 ACK/校验超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示已收到 ACK 且读回校验通过；`False` 表示超时/失败/校验未通过。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 同时设置角度和速度限制
success = robot.set_joint_angle_vel_limits(
    joint_index=1,
    min_angle_limit=-2.618,
    max_angle_limit=2.618,
    max_joint_spd=3.0,
)
print("set_joint_angle_vel_limits success =", success)

# 仅设置最大速度限制（不改角度限制）
success = robot.set_joint_angle_vel_limits(joint_index=1, max_joint_spd=3.0)
print("set_joint_angle_vel_limits(max_joint_spd) success =", success)
```

---

### 配置关节加速度限制 — `set_joint_acc_limits()`

**功能说明：** 设置指定关节的最大加速度限制。

**函数定义：**

```python
set_joint_acc_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    max_joint_acc: Optional[float] = None,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 配置单关节；`255` 配置全部 |
| `max_joint_acc` | `Optional[float]` | 最大加速度（rad/s²）；`None` 表示不配置 |
| `timeout` | `float` | 等待 ACK/校验超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示已收到 ACK 且读回校验通过。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_joint_acc_limits(joint_index=1, max_joint_acc=5.0)
print("set_joint_acc_limits success =", success)
```

---

### 配置法兰速度/加速度限制 — `set_flange_vel_acc_limits()`

**功能说明：** 设置末端速度/加速度限制。

**函数定义：**

```python
set_flange_vel_acc_limits(
    self,
    max_linear_vel: Optional[float] = None,
    max_angular_vel: Optional[float] = None,
    max_linear_acc: Optional[float] = None,
    max_angular_acc: Optional[float] = None,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `max_linear_vel` | `Optional[float]` | 最大线速度（m/s）；`None` 表示不配置 |
| `max_angular_vel` | `Optional[float]` | 最大角速度（rad/s）；`None` 表示不配置 |
| `max_linear_acc` | `Optional[float]` | 最大线加速度（m/s²）；`None` 表示不配置 |
| `max_angular_acc` | `Optional[float]` | 最大角加速度（rad/s²）；`None` 表示不配置 |
| `timeout` | `float` | 等待 ACK/校验超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示已收到 ACK 且读回校验通过。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_flange_vel_acc_limits(
    max_linear_vel=0.5,
    max_angular_vel=0.13,
    max_linear_acc=0.8,
    max_angular_acc=0.2,
)
print("set_flange_vel_acc_limits success =", success)
```

---

### 配置碰撞防护等级 — `set_crash_protection_rating()`

**功能说明：** 设置碰撞防护等级（可指定单关节或全部关节）。

**函数定义：**

```python
set_crash_protection_rating(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    rating: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8] = 0,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 配置单关节；`255` 配置全部，默认：`255` |
| `rating` | `int` | 碰撞防护等级，范围：`[0, 8]`（`0` = 不检测），默认：`0`。**等级越高越敏感，越容易触发碰撞保护**（更保守） |
| `timeout` | `float` | 等待 ACK/校验超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示已收到 ACK 且读回校验通过。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_crash_protection_rating(joint_index=1, rating=1)
print("set_crash_protection_rating success =", success)
```

---

### 恢复法兰限制默认值 — `set_flange_vel_acc_limits_to_default()`

**功能说明：** 将末端速度/加速度限制恢复为默认值。

**函数定义：**

```python
set_flange_vel_acc_limits_to_default(self, timeout: float = 1.0) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `timeout` | `float` | 等待 ACK/响应超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示在超时内收到 ACK/响应。

> **提示：** 该接口不提供读回校验。如需确认，可调用 `get_flange_vel_acc_limits()` 查询。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_flange_vel_acc_limits_to_default()
print("set_flange_vel_acc_limits_to_default success =", success)
```

---

### 恢复关节限制默认值 — `set_joint_angle_vel_acc_limits_to_default()`

**功能说明：** 将关节角度/速度/加速度限制恢复为默认值。

**函数定义：**

```python
set_joint_angle_vel_acc_limits_to_default(self, timeout: float = 1.0) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `timeout` | `float` | 等待 ACK/响应超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示在超时内收到 ACK/响应。

> **提示：** 该接口不提供读回校验。如需确认，可调用 `get_joint_angle_vel_limits()` / `get_joint_acc_limits()` 查询。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_joint_angle_vel_acc_limits_to_default()
print("set_joint_angle_vel_acc_limits_to_default success =", success)
```

---

### 设置 Link 速度/加速度周期反馈 — `set_links_vel_acc_period_feedback()`

**功能说明：** 设置各关节 link 的笛卡尔速度/加速度周期反馈开关（对应 CAN 周期帧 `0x481~0x486`）。

> **⚠️ 注意：** 该功能在底层主控中 **已废弃**，但总线仍可能周期上报对应帧，且上报数据 **全为 0**，无实际意义。**建议默认关闭**（`enable=False`），避免占用带宽。
>
> 该接口无直接读回校验方式，建议使用 `candump` 观察周期帧是否出现来验证：
>
> ```bash
> candump can0 | grep "48[1-6]"
> ```

**函数定义：**

```python
set_links_vel_acc_period_feedback(self, enable: bool = False, timeout: float = 1.0) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `enable` | `bool` | 是否开启周期反馈：`True` 开启；`False` 关闭（**建议默认关闭**） |
| `timeout` | `float` | 等待 ACK/响应超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示在超时内收到 ACK/响应。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_links_vel_acc_period_feedback(enable=True)
print("enable periodic feedback success =", success)

success = robot.set_links_vel_acc_period_feedback(enable=False)
print("disable periodic feedback success =", success)
```
