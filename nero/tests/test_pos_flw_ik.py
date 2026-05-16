import time
import math
import os
import sys
import numpy as np
from pyAgxArm import create_agx_arm_config, AgxArmFactory
from pyAgxArm.utiles.tf import rpy_to_rot



# 添加 ik_solver 路径
# sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'robotic_arm_kinematics-main'))

from nero.kinematics.nero_kinematics.nero_ik.ik_solver import (
    fk,
    NeroParams,
    ContinuityParams,
    ContinuityRuntimeState,
    solve_pose_continuous_with_state,
)


# ==========================================
# 1. 模拟规划器定义
# ==========================================
def mock_planner(initial_pose, freq=50.0):
    """生成 Z 轴方向直线往复轨迹"""
    dt = 1.0 / freq
    t = 0.0
    
    start_z = initial_pose[1]
    
    amplitude = 0.05    # 振幅 10cm（上下各 10cm，总行程 20cm）
    cycle_time = 4.0    # 周期 4秒（上 2秒，下 2秒）
    omega = 2 * math.pi / cycle_time 
    
    print(f"轨迹规划: 起始Z={start_z:.3f}m, 振幅={amplitude*100:.1f}cm")
    
    while True:
        target_pose = list(initial_pose)
        # Z 轴直线往复：使用正弦波实现平滑加减速
        target_pose[1] = start_z + amplitude * math.sin(omega * t)
        
        yield target_pose
        t += dt


def wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def pose_error(target_pose, current_pose):
    """Compute 6D pose error [dx, dy, dz, droll, dpitch, dyaw]."""
    e = np.zeros(6, dtype=float)
    for i in range(3):
        e[i] = target_pose[i] - current_pose[i]
    for i in range(3, 6):
        e[i] = wrap_to_pi(target_pose[i] - current_pose[i])
    return e


# ==========================================
# 2. 基于 ik_solver.py 的 IK 求解器
# ==========================================
class AnalyticIkSolver:
    """
    基于 ik_solver.py 的解析 IK 求解器
    使用 ik_arm_angle_with_report 进行单帧求解
    """
    def __init__(self, joint_limits, dt, n_psi=181):
        self.joint_limits = joint_limits
        self.dt = dt
        self.n_psi = n_psi
        
        # 使用默认的 NERO DH 参数
        self.nero_params = NeroParams.default()
        
        # 连续性参数
        self.continuity = ContinuityParams()
        
        # 运行时状态
        self.state = None
    
    def _pose_to_matrix(self, pose):
        """将 6D pose [x, y, z, roll, pitch, yaw] 转换为 4x4 齐次变换矩阵"""
        T = np.eye(4, dtype=float)
        T[:3, :3] = np.array(rpy_to_rot(pose[3], pose[4], pose[5]), dtype=float)
        T[:3, 3] = np.array(pose[:3], dtype=float)
        return T
    
    def _clamp_joints(self, q):
        """关节限位裁剪"""
        q_out = np.array(q, dtype=float)
        for i, (lo, hi) in enumerate(self.joint_limits):
            q_out[i] = min(max(q_out[i], lo), hi)
        return q_out
    
    def init_state(self, current_q):
        """初始化求解器状态（仅调用一次）"""
        current_q = self._clamp_joints(np.array(current_q, dtype=float))
        self.state = ContinuityRuntimeState(q_prev=current_q)
    
    def solve(self, target_pose):
        """
        求解目标位姿对应的关节角
        :param target_pose: 6D pose [x, y, z, roll, pitch, yaw]
        :return: 7维关节角
        """
        T_target = self._pose_to_matrix(target_pose)
        
        # 使用 solve_pose_continuous_with_state 求解
        q_cmd, report, self.state = solve_pose_continuous_with_state(
            T_target, state=self.state, p=self.nero_params, n_psi=self.n_psi, continuity=self.continuity
        )
        
        if q_cmd is None:
            # IK 求解失败，返回上一帧的关节角
            print(f"⚠️ IK 求解失败: {report.get('method')}")
            print(f"   目标位姿: x={target_pose[0]:.3f}, y={target_pose[1]:.3f}, z={target_pose[2]:.3f}")
            print(f"   候选解数量: {report.get('candidate_count', 0)}")
            return self.state.q_prev.copy()
        
        # 关节限位裁剪
        q_cmd = self._clamp_joints(q_cmd)
        
        return q_cmd
# ==========================================
# 2. 机械臂初始化与连接
# ==========================================
def main():
    cfg = create_agx_arm_config(
        robot="nero",
        comm="can",
        channel="can_left",
        interface="socketcan"
    )
    robot = AgxArmFactory.create_arm(cfg)
    
    print("正在连接机械臂...")
    robot.connect()
    time.sleep(0.5)

    print("\n--- 开始执行深度重置 ---")
    print("正在发送 reset() 清除底层的急停锁死标志...")
    # 注意：reset 会令机械臂立刻失电，请确保下方安全
    robot.reset() 
    time.sleep(1.0)  # 给主控足够的时间重启状态机
    
    print("正在切换回正常控制模式...")
    robot.set_normal_mode()
    time.sleep(0.5)
    print("--- 状态机重置完毕 ---\n")

    print("正在使能机械臂...")
    start_t = time.monotonic()
    is_enabled = False
    while time.monotonic() - start_t < 5.0:
        if robot.enable():
            is_enabled = True
            break
        time.sleep(0.1)

    if not is_enabled:
        print("❌ 彻底使能失败！")
        return
        
    print("✅ 机械臂已使能上电！")
    
    # 先用 move_p 移动到一个安全位置
    print("\n正在移动到安全起始位置...")
    robot.set_speed_percent(30)
    # 安全位置：末端朝下，高度适中
    safe_pose = [-0.4, -0.0, 0.4, 1.5708, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
    robot.move_p(safe_pose)
    
    time.sleep(3.0)  # 等待运动完成
    print("已到达安全起始位置")

    print("\n正在获取当前法兰位姿及关节角作为基准...")
    current_pose = None
    current_joints = None
    while current_pose is None or current_joints is None:
        fp = robot.get_flange_pose()
        ja = robot.get_joint_angles()
        if fp is not None: current_pose = fp.msg
        if ja is not None: current_joints = ja.msg
        time.sleep(0.1)

    # 设定跟踪频率为 20Hz
    track_freq = 20.0
    sleep_interval = 1.0 / track_freq

    joint_limits = []
    for i in range(1, 8):
        lo, hi = cfg["joint_limits"][f"joint{i}"]
        joint_limits.append((lo, hi))
    
    # 使用解析 IK 求解器（基于 ik_solver.py）
    ik_solver = AnalyticIkSolver(
        joint_limits=joint_limits,
        dt=sleep_interval,
        n_psi=181,
    )
    
    # ⚠️ 关键步骤 1：仅在最开始，将机器人的真实状态喂给 IK 求解器初始化
    ik_solver.init_state(current_joints)
    
    # ========== 关键：使用 FK 计算理论位姿作为轨迹起点 ==========
    # 这样可以确保轨迹起点与 IK 求解器的期望一致
    print("\n========== 使用 FK 结果作为轨迹起点 ==========")
    q_current = np.array(current_joints, dtype=float)
    T_fk = fk(q_current, ik_solver.nero_params)
    
    # 从 FK 矩阵提取位姿
    fk_pos = T_fk[:3, 3]
    from pyAgxArm.utiles.tf import rot_to_rpy
    fk_rpy = rot_to_rpy(T_fk[:3, :3].tolist())
    
    # 机器人返回的位姿（仅作对比参考）
    robot_pos = np.array(current_pose[:3], dtype=float)
    robot_rpy = np.array(current_pose[3:], dtype=float)
    
    print(f"FK 理论位姿: pos=[{fk_pos[0]:.4f}, {fk_pos[1]:.4f}, {fk_pos[2]:.4f}]")
    print(f"FK 理论位姿: rpy=[{fk_rpy[0]:.4f}, {fk_rpy[1]:.4f}, {fk_rpy[2]:.4f}]")
    print(f"机器人返回:   pos=[{robot_pos[0]:.4f}, {robot_pos[1]:.4f}, {robot_pos[2]:.4f}]")
    print(f"机器人返回:   rpy=[{robot_rpy[0]:.4f}, {robot_rpy[1]:.4f}, {robot_rpy[2]:.4f}]")
    
    pos_diff = np.linalg.norm(fk_pos - robot_pos) * 1000.0
    rpy_diff = np.linalg.norm(np.array(fk_rpy) - robot_rpy) * 180.0 / math.pi
    print(f"位置差异: {pos_diff:.2f} mm")
    print(f"姿态差异: {rpy_diff:.2f} deg")
    print("==========================================\n")
    
    # ⚠️ 关键：使用 FK 结果生成轨迹，而不是机器人返回的位姿
    fk_pose = np.array([fk_pos[0], fk_pos[1], fk_pos[2], fk_rpy[0], fk_rpy[1], fk_rpy[2]], dtype=float)
    planner = mock_planner(fk_pose, freq=track_freq)
    
    # 获取规划器的第一帧目标点，算出一个目标关节角
    first_target_pose = next(planner)
    first_q_cmd = ik_solver.solve(np.array(first_target_pose, dtype=float))

    # ⚠️ 关键步骤 2：用带有平滑曲线的 move_j 安全移动到轨迹起点
    print("正在使用平滑模式 (move_j) 安全对齐伺服控制的第一帧姿态...")
    robot.set_speed_percent(30)
    robot.move_j(first_q_cmd.tolist())
    time.sleep(2.0) # 必须等待，让机械臂完完全全停稳在起点！

    print(f"\n🚀 开始执行 move_js 末端位姿跟踪，刷新频率: {track_freq}Hz...")
    print("提示: 使用 ik_solver.py 解析 IK + 连续性优化 + 1D QP 后处理")
    last_print_t = time.time()
    
    try:
        while True:
            loop_start = time.time()

            # 此处获取传感器数据仅作监控使用，绝对不要干预求解器！
            ja = robot.get_joint_angles()
            fp = robot.get_flange_pose()
            if ja is None or fp is None:
                time.sleep(0.002)
                continue
            print(f"\n当前关节角度: {np.array(ja.msg, dtype=float)}")

            current_x = np.array(fp.msg, dtype=float)
            target_pose = next(planner)
            target_x = np.array(target_pose, dtype=float)

            # ⚠️ 关键步骤 3：只传目标位姿，让 ik_solver 根据内部的理论状态递推下一帧
            q_cmd = ik_solver.solve(target_x)
            print(f"计算出的关节角度: {q_cmd}")
            
            # 下发极速透传指令
            robot.move_js(q_cmd.tolist())

            err = pose_error(target_x, current_x)
            now = time.time()
            if now - last_print_t >= 0.1:
                pos_err_mm = np.linalg.norm(err[:3]) * 1000.0
                rot_err_deg = np.linalg.norm(err[3:]) * 180.0 / math.pi
                print(f"跟踪误差: pos={pos_err_mm:.1f} mm, rot={rot_err_deg:.2f} deg")
                last_print_t = now

            elapsed = time.time() - loop_start
            if elapsed < sleep_interval:
                time.sleep(sleep_interval - elapsed)

    except KeyboardInterrupt:
        print("\n收到键盘中断信号 (Ctrl+C)，停止跟踪...")

    finally:
        print("正在触发电子急停安全停机...")
        robot.electronic_emergency_stop()
        time.sleep(2)
        print("机械臂已进入电子急停状态，脚本退出。")

if __name__ == "__main__":
    main()