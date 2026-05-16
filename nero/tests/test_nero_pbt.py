import time
import math
import os
import numpy as np
import matplotlib.pyplot as plt  # 新增：用于绘制误差曲线
from pyAgxArm import create_agx_arm_config, AgxArmFactory
from pyAgxArm.utiles.tf import rpy_to_rot

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    p = None

# ==========================================
# 0. 误差计算辅助函数 (已解除注释)
# ==========================================
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
# 1. 模拟规划器定义
# ==========================================
def mock_planner(initial_pose, freq=50.0):
    dt = 1.0 / freq
    t = 0.0
    
    start_x = initial_pose[0]
    start_z = initial_pose[2]
    
    radius = 0.1       # 半径 5cm
    cycle_time = 8.0    # 周期 4秒
    omega = 2 * math.pi / cycle_time 
    
    while True:
        target_pose = list(initial_pose)
        # X-Z 平面圆轨迹方程
        target_pose[0] = start_x + radius * math.sin(omega * t)
        target_pose[2] = start_z + radius * (1 - math.cos(omega * t))
        
        yield target_pose
        t += dt
        
# ==========================================
# 2. PyBullet IK 求解器 (实机专用，带零空间防发散)
# ==========================================
class PyBulletIkSolver:
    def __init__(self, urdf_path, joint_limits, dt, ee_link_name="link7"):
        if p is None:
            raise RuntimeError("pybullet 未安装。")
            
        self.physicsClient = p.connect(p.DIRECT) 
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True)
        self.num_joints = p.getNumJoints(self.robot_id)
        self.active_joints = []
        self.ee_link_idx = -1
        
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode('utf-8')
            link_name = info[12].decode('utf-8')
            if info[2] != p.JOINT_FIXED:
                self.active_joints.append(i)
            if link_name == ee_link_name:
                self.ee_link_idx = i
                
        self.ll = [float(limit[0]) for limit in joint_limits]
        self.ul = [float(limit[1]) for limit in joint_limits]
        self.jr = [ul - ll for ul, ll in zip(self.ul, self.ll)]
        
        # 冗余自由度静态停靠姿态，防止零空间偏移发散
        self.rp = [0.0] * len(self.active_joints)

    def init_state(self, current_q):
        for i, joint_idx in enumerate(self.active_joints):
            p.resetJointState(self.robot_id, joint_idx, current_q[i])

    def solve(self, target_pose, current_q=None):
        if current_q is not None:
            self.init_state(current_q)
            
        target_pos = target_pose[:3]
        target_orn = p.getQuaternionFromEuler(target_pose[3:])
        
        ik_solution = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.ee_link_idx,
            targetPosition=target_pos,
            targetOrientation=target_orn,
            lowerLimits=self.ll,
            upperLimits=self.ul,
            jointRanges=self.jr,
            restPoses=self.rp,
            maxNumIterations=100,
            residualThreshold=1e-4
        )
        
        return np.array(ik_solution[:len(self.active_joints)])
        
    def __del__(self):
        try:
            p.disconnect(self.physicsClient)
        except:
            pass

# ==========================================
# 3. 机械臂初始化与连接
# ==========================================
def main():
    cfg = create_agx_arm_config(robot="nero", comm="can", channel="can_left")
    robot = AgxArmFactory.create_arm(cfg)
    
    print("正在连接机械臂...")
    robot.connect()
    time.sleep(0.5)

    print("\n--- 开始执行深度重置 ---")
    robot.disable() 
    time.sleep(0.5)
    robot.reset() 
    time.sleep(1.0)
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

    print("\n正在获取当前法兰位姿及关节角作为基准...")
    current_pose = None
    current_joints = None
    while current_pose is None or current_joints is None:
        fp = robot.get_flange_pose()
        ja = robot.get_joint_angles()
        if fp is not None: current_pose = fp.msg
        if ja is not None: current_joints = ja.msg
        time.sleep(0.1)

    track_freq = 50.0
    sleep_interval = 1.0 / track_freq
    planner = mock_planner(current_pose, freq=track_freq)

    joint_limits = []
    for i in range(1, 8):
        lo, hi = cfg["joint_limits"][f"joint{i}"]
        joint_limits.append((lo, hi))
        
    urdf_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "asserts", "agx_arm_urdf-main", "nero", "urdf", "nero_description.urdf")
    )
    
    ik_solver = PyBulletIkSolver(
        urdf_path=urdf_path,
        joint_limits=joint_limits,
        dt=sleep_interval,
        ee_link_name="link7",
    )
    
    ik_solver.init_state(current_joints)
    first_target_pose = next(planner)
    first_q_cmd = ik_solver.solve(np.array(first_target_pose, dtype=float), current_q=current_joints)

    print("正在使用平滑模式 (move_j) 安全对齐伺服控制的第一帧姿态...")
    robot.set_speed_percent(30)
    robot.move_j(first_q_cmd.tolist())
    time.sleep(2.0)

    # ==========================================
    # 4. 数据记录初始化
    # ==========================================
    time_log = []
    pos_err_log = [] # 记录 [dx, dy, dz, 综合位置误差] (单位 mm)
    rot_err_log = [] # 记录 [droll, dpitch, dyaw, 综合姿态误差] (单位 deg)

    print(f"\n🚀 开始执行 move_js 末端位姿跟踪，刷新频率: {track_freq}Hz...")
    print("提示: 纯前馈模式运行中，请保持关注机械臂状态。")
    
    last_print_t = time.time()
    start_tracking_t = time.time()
    
    try:
        while True:
            loop_start = time.time()

            ja = robot.get_joint_angles()
            fp = robot.get_flange_pose()
            if ja is None or fp is None:
                time.sleep(0.002)
                continue

            current_x = np.array(fp.msg, dtype=float)
            target_pose = next(planner)
            target_x = np.array(target_pose, dtype=float)

            q_cmd = ik_solver.solve(target_x, current_q=ja.msg)
            robot.move_js(q_cmd.tolist())

            # ==========================================
            # 记录本帧误差
            # ==========================================
            err = pose_error(target_x, current_x)
            
            # 位置误差转换到毫米
            dx, dy, dz = err[:3] * 1000.0
            pos_norm = np.linalg.norm(err[:3]) * 1000.0
            
            # 姿态误差转换到角度
            droll, dpitch, dyaw = err[3:] * (180.0 / math.pi)
            rot_norm = np.linalg.norm(err[3:]) * (180.0 / math.pi)

            # 追加记录
            curr_t = time.time() - start_tracking_t
            time_log.append(curr_t)
            pos_err_log.append([dx, dy, dz, pos_norm])
            rot_err_log.append([droll, dpitch, dyaw, rot_norm])

            now = time.time()
            if now - last_print_t >= 1.0:
                print(f"跟踪误差: pos={pos_norm:.1f} mm, rot={rot_norm:.2f} deg")
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
        print("机械臂已进入电子急停状态。")

    # ==========================================
    # 5. 绘制并保存误差曲线
    # ==========================================
    if len(time_log) > 0:
        print("\n正在生成误差曲线图并保存为 tracking_error.png ...")
        pos_err_arr = np.array(pos_err_log)
        rot_err_arr = np.array(rot_err_log)

        # 创建上下两个子图
        plt.subplot(2, 1, 1)
        plt.plot(time_log, pos_err_arr[:, 0], label='dx')
        plt.plot(time_log, pos_err_arr[:, 1], label='dy')
        plt.plot(time_log, pos_err_arr[:, 2], label='dz')
        plt.plot(time_log, pos_err_arr[:, 3], label='||Pos Error||', linestyle='--')
        plt.title('Position Tracking Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Error (mm)')
        plt.legend(loc='best')
        plt.grid(True)

        plt.subplot(2, 1, 2)
        plt.plot(time_log, rot_err_arr[:, 0], label='dRoll')
        plt.plot(time_log, rot_err_arr[:, 1], label='dPitch')
        plt.plot(time_log, rot_err_arr[:, 2], label='dYaw')
        plt.plot(time_log, rot_err_arr[:, 3], label='||Rot Error||', linestyle='--')
        plt.title('Rotation Tracking Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Error (deg)')
        plt.legend(loc='best')
        plt.grid(True)

        plt.tight_layout()
        plt.savefig('tracking_error.png')
        print("误差曲线已成功保存！")

if __name__ == "__main__":
    main()