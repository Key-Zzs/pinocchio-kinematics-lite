#!/usr/bin/env python3
"""
测试placo IK solver与NERO机器人 - 可视化版本
"""
import os
import numpy as np
import pinocchio as pin
import placo
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz
from placo_utils.tf import tf

# URDF路径
URDF_PATH = os.path.expanduser("~/pyAgxArm/asserts/agx_arm_urdf-main/nero/urdf/nero_description.urdf")

print("=" * 60)
print("Placo IK Solver 测试 - 可视化")
print("=" * 60)

# 加载机器人模型
print(f"\n加载URDF: {URDF_PATH}")
robot = placo.RobotWrapper(URDF_PATH, placo.Flags.ignore_collisions)

# 打印机器人信息
print(f"模型名称: {robot.model.name}")
print(f"关节数量: {robot.model.nq}")
print(f"帧数量: {robot.model.nframes}")

# 打印所有帧
print("\n所有帧:")
for i in range(robot.model.nframes):
    frame = robot.model.frames[i]
    print(f"  [{i}] {frame.name}")

# 获取零位时的末端位姿
q_zero = np.zeros(robot.model.nq)
robot.state.q = q_zero
robot.update_kinematics()
T_zero = robot.get_T_world_frame("link7")
pos_zero = T_zero[:3, 3]
print(f"\n零位时末端位置: x={pos_zero[0]:.4f}, y={pos_zero[1]:.4f}, z={pos_zero[2]:.4f}")

# 创建IK求解器
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)  # 固定基座

# 添加末端执行器任务
effector_task = solver.add_frame_task("link7", np.eye(4))
effector_task.configure("effector", "soft", 1.0, 1.0)

# 创建可视化
print("\n启动可视化...")
viz = robot_viz(robot)

# 参数
t = 0
dt = 0.02
solver.dt = dt

# 目标轨迹参数（在机器人工作空间内）
# NERO机器人工作空间约0.7m，设置合理的目标
center_x = 0.3  # 前方30cm
center_y = 0.0  # 中心
center_z = 0.50  # 高度50cm
amplitude = 0.05  # 振幅15cm


@schedule(interval=dt)
def loop():
    global t
    t += dt

    # 更新目标位置（圆形轨迹）
    target_x = center_x # + amplitude * np.cos(t)
    target_y = center_y # + amplitude * np.sin(t)
    target_z = center_z + 0.05 * np.sin(2 * t)  # 轻微上下运动
    
    # 设置目标位姿
    effector_task.T_world_frame = tf.translation_matrix([target_x, target_y, target_z])

    # 求解IK
    solver.solve(True)
    robot.update_kinematics()

    # 显示机器人、末端执行器和目标
    viz.display(robot.state.q)
    robot_frame_viz(robot, "link7")
    frame_viz("target", effector_task.T_world_frame)
    
    # 每2秒打印一次状态
    if int(t * 10) % 20 == 0:
        T_current = robot.get_T_world_frame("link7")
        pos_current = T_current[:3, 3]
        error = np.linalg.norm(pos_current - np.array([target_x, target_y, target_z])) * 1000
        print(f"t={t:.1f}s | 目标: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) | "
              f"实际: ({pos_current[0]:.3f}, {pos_current[1]:.3f}, {pos_current[2]:.3f}) | "
              f"误差: {error:.1f}mm")


print("\n开始IK跟踪测试...")
print("目标轨迹: 圆形运动 (半径15cm)")
print("按Ctrl+C停止\n")

run_loop()
