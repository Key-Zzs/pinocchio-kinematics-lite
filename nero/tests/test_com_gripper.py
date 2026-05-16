import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

def diagnose_gripper():
    print("--- 开始夹爪通信诊断 ---")
    
    # 1. 检查基础配置
    cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0", interface="socketcan")
    robot = AgxArmFactory.create_arm(cfg)
    
    # 2. 严格遵循时序：先初始化执行器，再 connect
    print("正在初始化末端执行器...")
    gripper = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
    
    print("正在连接总线 (启动读线程)...")
    robot.connect()
    time.sleep(0.5)
    
    # 3. ⚠️ 关键步骤：激活机械臂主控 (可能需要主控使能才会转发夹爪报文)
    print("\n--- 激活机械臂主控 ---")
    print("正在发送 reset() 清除急停锁死标志...")
    robot.reset()
    time.sleep(1.0)
    
    print("正在切换回正常控制模式...")
    robot.set_normal_mode()
    time.sleep(0.5)
    
    print("正在使能机械臂...")
    start_t = time.monotonic()
    while time.monotonic() - start_t < 5.0:
        if robot.enable():
            print("✅ 机械臂使能成功")
            break
        time.sleep(0.1)
    else:
        print("❌ 机械臂使能超时 (5秒)")
    
    print("--- 主控激活完毕 ---\n")
    
    # 给一点时间让总线数据飘过来
    time.sleep(1.0)
    
    # 4. 第一步排查：检查 FPS
    fps = gripper.get_fps()
    print(f"\n[诊断 1] 当前夹爪数据接收频率 (FPS): {fps} Hz")
    if fps == 0.0:
        print("❌ 分析: 接收频率为 0。")
        print("   可能原因: ")
        print("   - 夹爪物理线缆未插紧或未上电。")
        print("   - CAN 通道配置错误。")
        print("   - 夹爪驱动器未使能或故障。")
    
    # 5. 第二步排查：检查返回的 Status 对象
    status = gripper.get_gripper_status()
    print(f"\n[诊断 2] 尝试强读一次夹爪状态: {'成功获取' if status else '返回 None'}")
    
    if status is None:
        print("❌ 分析: get_gripper_status() 返回了 None。")
        print("   可能原因: 物理连接断开或读线程未正常工作。")
    else:
        # 6. 第三步排查：如果能读到数据，检查底层硬件错误位
        print("✅ 收到数据！检查驱动器硬件级报错位 (foc_status):")
        foc = status.msg.foc_status
        
        errors_found = False
        if foc.voltage_too_low:
            print("   ❌ 故障: 夹爪电压过低！")
            errors_found = True
        if foc.motor_overheating:
            print("   ❌ 故障: 夹爪电机过温！")
            errors_found = True
        if foc.driver_overcurrent:
            print("   ❌ 故障: 夹爪驱动过流！")
            errors_found = True
        if foc.driver_overheating:
            print("   ❌ 故障: 夹爪驱动过温！")
            errors_found = True
        if foc.driver_error_status:
            print("   ❌ 故障: 夹爪驱动处于通用错误状态！")
            errors_found = True
            
        if not errors_found:
            print("   ✅ 硬件健康：未检测到驱动层面的报错。")
            print(f"   当前状态: 宽度 {status.msg.width:.4f}m, 力 {status.msg.force:.2f}N")
            
        print(f"   当前夹爪使能状态: {'已使能' if foc.driver_enable_status else '未使能 (可能需要重新控制/重置)'}")

    print("\n--- 诊断结束 ---")

if __name__ == "__main__":
    diagnose_gripper()