import time
import numpy as np
from pyAgxArm import create_agx_arm_config, AgxArmFactory
import os
import sys

# 导入PyBullet
try:
    import pybullet as p
    import pybullet_data
except ImportError:
    p = None

# ==============================================
# 【复用】PyBullet逆运动学求解器类（核心）
# ==============================================
class PyBulletIkSolver:
    def __init__(self, urdf_path, joint_limits, dt, ee_link_name="link7"):
        if p is None:
            raise RuntimeError("请先安装pybullet：pip install pybullet")
            
        # 后台模式（无GUI），纯计算
        self.physicsClient = p.connect(p.DIRECT) 
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # 加载机器人模型，固定底座
        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True)
        self.num_joints = p.getNumJoints(self.robot_id)
        self.active_joints = []  # 活动关节索引
        self.ee_link_idx = -1    # 末端执行器索引
        
        # 遍历关节，筛选活动关节+末端
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            link_name = info[12].decode('utf-8')
            if info[2] != p.JOINT_FIXED:
                self.active_joints.append(i)
            if link_name == ee_link_name:
                self.ee_link_idx = i

        # 关节限位配置
        self.ll = [float(limit[0]) for limit in joint_limits]
        self.ul = [float(limit[1]) for limit in joint_limits]
        self.jr = [ul - ll for ul, ll in zip(self.ul, self.ll)]
        # 冗余关节默认姿态
        self.rp = [0.0] * len(self.active_joints)

    # 同步真实机器人的关节角度到PyBullet模型
    def init_state(self, current_q):
        for i, joint_idx in enumerate(self.active_joints):
            p.resetJointState(self.robot_id, joint_idx, current_q[i])

    # 核心：输入末端目标位姿 → 输出关节角度
    def solve(self, target_pose, current_q=None):
        if current_q is not None:
            self.init_state(current_q)
            # 【修复关键】将倾向停留位姿(restPoses)设定为当前的实际关节角
            rest_poses = list(current_q)
        else:
            rest_poses = self.rp
            
        target_pos = target_pose[:3]
        target_orn = p.getQuaternionFromEuler(target_pose[3:])
        
        # PyBullet官方IK求解
        ik_solution = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.ee_link_idx,
            targetPosition=target_pos,
            targetOrientation=target_orn,
            lowerLimits=self.ll,
            upperLimits=self.ul,
            jointRanges=self.jr,
            restPoses=self.rp,  # <--- 使用动态的实际起始角度
            maxNumIterations=100,
            residualThreshold=1e-4
        )
        return np.array(ik_solution[:len(self.active_joints)])
        
    # 释放资源
    def __del__(self):
        try:
            p.disconnect(self.physicsClient)
        except:
            pass

# ==============================================
# 主程序：连接机械臂 + 实时读取 + IK解算
# ==============================================
if __name__ == "__main__":
    # 1. 初始化Nero机械臂（你的原有代码）
    cfg = create_agx_arm_config(robot="nero", comm="can", channel="can_left")
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()
    print("✅ 机械臂连接成功")
    time.sleep(1)

    # 2. 初始化PyBullet IK求解器（关键配置）
    urdf_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "asserts", "agx_arm_urdf-main", "nero", "urdf", "nero_description.urdf")
    )
    DT = 0.01  # 控制步长
    # 从机械臂配置中读取关节限位
    joint_limits = [cfg["joint_limits"][f"joint{i}"] for i in range(1, 8)]
    
    # 创建IK求解器
    ik_solver = PyBulletIkSolver(
        urdf_path=urdf_path,
        joint_limits=joint_limits,
        dt=DT,
        ee_link_name="link7"
    )
    print("✅ PyBullet IK求解器初始化成功")

    # 3. 主循环：读取真实状态 + IK解算
    while True:
        # 读取机械臂真实数据（你的原有代码）
        fp = robot.get_flange_pose()   # 末端位姿 [x,y,z,roll,pitch,yaw]
        ja = robot.get_joint_angles()  # 真实关节角度

        print("fp:", fp)
        print("ja:", ja)
        print("1")

        if ja is not None and fp is not None:
            # --------------------------
            # 打印真实状态
            # --------------------------
            print("\n========================================")
            print(f"📡 真实关节角: {np.round(ja.msg, 4)}")
            print(f"📡 真实末端位姿: {np.round(fp.msg, 4)}")

            # --------------------------
            # 【核心】PyBullet IK解算
            # 用「当前末端位姿」作为目标，解算关节角
            # --------------------------
            current_pose = np.array(fp.msg)          # 当前末端位姿
            current_joints = np.array(ja.msg)        # 当前真实关节角
            ik_joints = ik_solver.solve(current_pose, current_joints)  # IK求解

            # 打印IK解算结果
            print(f"🔧 IK解算关节角: {np.round(ik_joints, 4)}")
            print("========================================")

        time.sleep(0.5)