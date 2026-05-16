import time
import math
from pyAgxArm import create_agx_arm_config, AgxArmFactory


# ==========================================
# 1. 模拟夹爪规划器定义
# ==========================================
def mock_gripper_planner(freq=20.0):
    """
    生成夹爪平滑开合的轨迹 (正弦波)
    频率决定了步长，返回目标宽度 (单位: m)
    """
    dt = 1.0 / freq
    t = 0.0
    
    # AgxGripper 的宽度范围为 [0.0, 0.1] 米
    # 我们设定在一个安全范围内往复：0.00m (闭合) 到 0.08m (张开)
    min_w = 0.0
    max_w = 0.08
    amplitude = (max_w - min_w) / 2.0
    center_w = min_w + amplitude
    
    cycle_time = 4.0    # 完整开合一次的周期 (4秒)
    omega = 2 * math.pi / cycle_time 
    
    print(f"夹爪轨迹规划: 中心={center_w*100:.1f}cm, 振幅={amplitude*100:.1f}cm")
    
    while True:
        # 使用正弦波生成平滑的宽度目标指令
        # 从中心点开始，先张开后闭合
        target_width = center_w + amplitude * math.sin(omega * t)
        
        # 裁剪确保安全，不超硬限位
        target_width = max(0.0, min(target_width, 0.1))
        
        yield target_width
        t += dt


# ==========================================
# 2. 机械臂与夹爪初始化及主控制循环
# ==========================================
def main():
    # 1. 初始化底层配置与机械臂实例
    cfg = create_agx_arm_config(
        robot="nero", 
        comm="can",
        channel="can_left",
        interface="socketcan"
    )
    robot = AgxArmFactory.create_arm(cfg)
    
    # 2. ⚠️ 关键步骤：在 connect 前初始化末端执行器
    print("正在初始化 AgxGripper 夹爪实例...")
    gripper = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
    
    # 3. 建立连接并启动读取线程
    print("正在连接总线并启动数据监控...")
    robot.connect()
    time.sleep(0.5)

    # ==========================================
    # --- 机械臂状态机重置与使能 (参考 test_pos_flw_ik) ---
    # ==========================================
    print("\n--- 开始执行深度重置 ---")
    print("正在发送 reset() 清除底层的急停锁死标志...")
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
        print("❌ 彻底使能失败！程序退出。")
        return
        
    print("✅ 机械臂已使能上电！")
    
    # 先用 move_p 移动到一个安全位置
    print("\n正在移动到机械臂安全起始位置...")
    robot.set_speed_percent(30)
    # 安全位置：末端朝下，高度适中
    safe_pose = [-0.4, -0.0, 0.4, 1.5708, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
    robot.move_p(safe_pose)
    
    time.sleep(3.0)  # 等待运动完成
    print("✅ 已到达机械臂安全起始位置")
    # ==========================================

    # 4. 检查夹爪通信状态
    print("\n正在检查夹爪状态...")
    if not gripper.is_ok():
        print("⚠️ 警告: 夹爪通信似乎未就绪，请检查硬件连接或 CAN 总线状态！")
    else:
        print(f"✅ 夹爪连接正常，当前数据刷新率: {gripper.get_fps():.1f} Hz")

    # 5. 获取夹爪初始状态
    initial_status = None
    for _ in range(10):
        initial_status = gripper.get_gripper_status()
        if initial_status is not None:
            break
        time.sleep(0.1)
        
    if initial_status is not None:
        cw = initial_status.msg.width
        cf = initial_status.msg.force
        print(f"夹爪当前状态: 宽度 {cw*100:.2f} cm, 夹持力 {cf:.2f} N")
    else:
        print("⚠️ 暂未收到夹爪反馈，将继续尝试发送指令...")

    # 6. 设置控制参数与规划器
    track_freq = 20.0  # 控制频率 20Hz
    sleep_interval = 1.0 / track_freq
    planner = mock_gripper_planner(freq=track_freq)
    
    # 先将夹爪移动到一个已知的安全起点（张开 8cm），提供 1N 的夹持力
    print("\n正在张开夹爪至初始状态 (8cm)...")
    gripper.move_gripper(width=0.08, force=1.0)
    time.sleep(2.0)

    print(f"\n🚀 开始执行夹爪连续开合跟踪，刷新频率: {track_freq}Hz...")
    last_print_t = time.time()
    
    try:
        while True:
            loop_start = time.time()

            # --- 核心控制逻辑 ---
            # 1. 从规划器获取这一帧的期望宽度
            target_w = next(planner)
            
            # 2. 下发控制指令 (保持 1.0N 的力向目标宽度运动)
            gripper.move_gripper(width=target_w, force=1.0)

            # --- 监控逻辑 (不干预控制) ---
            now = time.time()
            if now - last_print_t >= 0.2:  # 每 0.2 秒打印一次日志
                status = gripper.get_gripper_status()
                if status is not None:
                    curr_w = status.msg.width
                    curr_f = status.msg.force
                    print(f"目标宽度: {target_w*100:5.2f} cm | 当前反馈: 宽度 {curr_w*100:5.2f} cm, 力 {curr_f:4.2f} N")
                last_print_t = now

            # --- 维持固定控制频率 ---
            elapsed = time.time() - loop_start
            if elapsed < sleep_interval:
                time.sleep(sleep_interval - elapsed)

    except KeyboardInterrupt:
        print("\n收到键盘中断信号 (Ctrl+C)，停止跟踪...")

    finally:
        # 7. 安全退出处理
        print("\n正在进行安全收尾操作...")
        
        # 退出前为了安全，将夹爪完全松开
        print("正在松开夹爪释放物品...")
        gripper.move_gripper(width=0.08, force=0.5)
        time.sleep(1.0)
        
        # 禁用夹爪驱动
        print("正在禁用夹爪驱动机能...")
        if gripper.disable_gripper():
            print("✅ 夹爪已安全禁用。")
        else:
            print("⚠️ 夹爪禁用指令下发，但无法确认是否成功。")
            
        print("正在触发机械臂电子急停安全停机...")
        robot.electronic_emergency_stop()
        time.sleep(2.0)
        
        print("✅ 机械臂已进入电子急停状态，脚本安全退出。")

if __name__ == "__main__":
    main()
