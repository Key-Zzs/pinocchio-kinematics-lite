# %%
from nero.teleop.interface import NeroDualArmServer
import pdb
# %%
import logging
log = logging.getLogger(__name__)

# %%
import numpy as np
import time

# %%
server = NeroDualArmServer(gripper_enabled=True)

# %%
server.left_robot_go_home()

left_joints = server.left_robot_get_joint_positions()
print("Left joints:", left_joints)

# # # %%
# # 单步 servo_p 测试

# # 构造一个小的 delta
# cur_pose = [-0.226,0,0.4,-1.57,0,-3.14]
# delta_pose = np.array([0.0, -0.00125, 0.0125, 0.0, 1.57, 0.0])
# # target_pose = np.array([-0.403, 0.03, 0.265, 1.57, -0.35, -0.07])

# # 调用 servo_p（delta 模式）
# ret = server.servo_p(
#     robot_arm="left_robot",
#     cur_pose=cur_pose,
#     pose=delta_pose.tolist(),
#     delta=True
#     # pose=target_pose.tolist(),
#     # delta=False
# )

# pdb.set_trace()

# # %%
# # 连续 servo_p 测试
# steps = 1000
# dt = 0.02  # 20Hz
# # cur_pose = [-0.226,0,0.4,-1.57,0,-3.14]
# for i in range(steps):
#     # delta_pose = np.array([0.01, -0.00125, 0.0125, 0.0, 0.0, 0.0])
#     if(i < 500):
#         delta_pose = np.array([-0.0005, 0.0, 0.0, 0.0, 0.0, 0.0])
#     else:
#         delta_pose = np.array([0.0005, 0.0, 0.0, 0.0, 0.0, 0.0])
#     time1 = time.time()
#     server.servo_p_OL("left_robot", delta_pose.tolist(), delta=True)
#     time2 = time.time()
#     print(f"Step {i+1}/{steps}, servo_p_OL time: {(time2 - time1) * 1000:.2f} ms")
#     time.sleep(dt)

# %%
# YZ 平面正方形轨迹测试
# 正方形边长 0.2m，分为 20 段，每段 0.01m
steps_per_side = 20  # 每边分20步
side_length = 0.2   # 边长 0.2m
step_size = side_length / steps_per_side  # 每步移动距离
dt = 0.02  # 20Hz 控制周期

print(f"正方形边长: {side_length}m, 每边步数: {steps_per_side}, 每步距离: {step_size}m")

# 正方形轨迹: 
# 边1: y += 0.2 (向前)
# 边2: z += 0.2 (向上)  
# 边3: y -= 0.2 (向后)
# 边4: z -= 0.2 (向下)

# 边1: y 方向 +0.2m
print(f"边1: y 方向 +0.2m")
for i in range(steps_per_side):
    delta_pose = np.array([0.0, step_size, 0.0, 0.0, 0.0, 0.0])
    server.servo_p_OL("left_robot", delta_pose.tolist(), delta=True)
    time.sleep(dt)

# 边2: z 方向 +0.2m
print(f"边2: z 方向 +0.2m")
for i in range(steps_per_side):
    delta_pose = np.array([0.0, 0.0, step_size, 0.0, 0.0, 0.0])
    server.servo_p_OL("left_robot", delta_pose.tolist(), delta=True)
    time.sleep(dt)

# 边3: y 方向 -0.2m
print(f"边3: y 方向 -0.2m")
for i in range(steps_per_side):
    delta_pose = np.array([0.0, -step_size, 0.0, 0.0, 0.0, 0.0])
    server.servo_p_OL("left_robot", delta_pose.tolist(), delta=True)
    time.sleep(dt)

# 边4: z 方向 -0.2m
print(f"边4: z 方向 -0.2m")
for i in range(steps_per_side):
    delta_pose = np.array([0.0, 0.0, -step_size, 0.0, 0.0, 0.0])
    server.servo_p_OL("left_robot", delta_pose.tolist(), delta=True)
    time.sleep(dt)

print("正方形轨迹完成!")