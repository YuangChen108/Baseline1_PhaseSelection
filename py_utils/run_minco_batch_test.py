import subprocess
import time
import os
import signal
import sys

# ================= 配置区域 =================
TOTAL_RUNS = 1          # 总共跑多少次
INIT_WAIT_TIME = 5       # 启动ROS节点后等待初始化的时间 (秒)
SIM_DURATION = 25        # 每次实验运行的最长时间 (确保足够降落)
LOG_FILE = "batch_test_log.txt" # 简单的运行日志
# ===========================================

def cleanup_ros():
    """强制清理所有 ROS 相关进程，防止僵尸节点影响下一次实验"""
    print("🧹 清理残留进程...")
    # 强制杀掉 roslaunch, rosmaster, rviz 以及你的规划节点
    os.system("killall -9 roslaunch rosmaster rviz fake_planning_nodelet planning_node > /dev/null 2>&1")
    time.sleep(2) # 等待释放端口

def run_experiment(iteration):
    print(f"\n{'='*40}")
    print(f"🚀 开始第 {iteration + 1} / {TOTAL_RUNS} 次实验")
    print(f"{'='*40}")

    # 1. 启动环境 (小船目标)
    # 对应: roslaunch fake_planning fake_boat_target.launch
    print("-> 1. 启动虚拟环境 (Fake Boat)...")
    cmd_env = ["roslaunch", "fake_planning", "fake_boat_target.launch"]
    # 使用 subprocess.Popen 后台运行，不阻塞脚本
    proc_env = subprocess.Popen(cmd_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2) 

    # 2. 启动规划器 (Tracker)
    # 对应: roslaunch planning simulation_tl.launch
    print("-> 2. 启动规划器 (Planner)...")
    cmd_planner = ["roslaunch", "planning", "simulation_tl.launch"]
    proc_planner = subprocess.Popen(cmd_planner, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # 等待系统初始化完成 (TF树建立、地图加载等)
    print(f"-> 等待系统初始化 ({INIT_WAIT_TIME}s)...")
    time.sleep(INIT_WAIT_TIME)

    # 3. 触发降落指令
    # 对应: ./sh_utils/auto_land.sh
    print("-> 3. 触发降落 (Trigger Land)...")
    trigger_script = "./sh_utils/auto_land.sh"
    
    # 检查脚本是否有执行权限
    if not os.access(trigger_script, os.X_OK):
        print("⚠️ 警告: 脚本没有执行权限，正在尝试添加权限...")
        os.system(f"chmod +x {trigger_script}")

    try:
        # 运行 shell 脚本 (这是瞬间触发的)
        subprocess.run(trigger_script, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"❌ 触发脚本执行失败: {e}")
        return

    # 4. 等待降落完成并记录数据
    # 你的 C++ 代码会在检测到接触时自动写入 CSV
    print(f"-> ⏳ 正在降落，等待 {SIM_DURATION} 秒...")
    time.sleep(SIM_DURATION)

    # 5. 结束本次实验
    print("-> 🛑 结束本次实验，关闭进程...")
    proc_planner.terminate()
    proc_env.terminate()
    
    # 等待子进程退出
    try:
        proc_planner.wait(timeout=2)
        proc_env.wait(timeout=2)
    except subprocess.TimeoutExpired:
        print("-> 强制杀死卡住的进程")
        proc_planner.kill()
        proc_env.kill()

    # 执行彻底清理
    cleanup_ros()

def main():
    # 捕获 Ctrl+C 以便优雅退出
    def signal_handler(sig, frame):
        print("\n\n⚠️ 检测到中断，正在清理并退出...")
        cleanup_ros()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)

    # 先清理一次环境
    cleanup_ros()

    # 记录开始时间
    start_time = time.time()

    for i in range(TOTAL_RUNS):
        try:
            run_experiment(i)
        except Exception as e:
            print(f"❌ 实验 {i+1} 发生未知错误: {e}")
            cleanup_ros()
            continue

    total_time = (time.time() - start_time) / 60
    print(f"\n✅ 所有 {TOTAL_RUNS} 次实验完成！耗时: {total_time:.2f} 分钟")
    print("请检查 landing_stats_v1.csv 查看结果。")

if __name__ == "__main__":
    main()