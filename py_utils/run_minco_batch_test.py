import subprocess
import time
import os
import signal
import sys

# ================= 配置区域 =================
TOTAL_RUNS = 25         # 总共跑多少次
INIT_WAIT_TIME = 8       # NEW: 增加初始化时间，确保 TF 树完全建立 (建议 8-10s)
SIM_DURATION = 12        # NEW: 增加仿真时长，确保降落动作完全结算
LOG_FILE = "batch_test_log.txt" 
# ===========================================

def cleanup_ros():
    """彻底摧毁 ROS 世界，重建秩序"""
    print("🧹 正在强力清理残留进程...")
    
    # NEW: 使用 pkill 匹配关键词，比 killall 更稳
    # 一次性杀掉所有相关的
    cmd = "pkill -9 -f 'ros|gazebo|planning|nodelet|rviz|target'"
    os.system(cmd)
    
    # 彻底清理可能锁死端口的进程
    os.system("killall -9 rosmaster roscore > /dev/null 2>&1")
    
    # NEW: 删掉这一行！它是导致你卡死的元凶
    # os.system("rosnode cleanup") 
    
    # 给系统 3 秒钟喘气（通常 3 秒足够释放端口了）
    time.sleep(3) 
    print("✨ 宇宙已重置。")

def run_experiment(iteration):
    print(f"\n{'='*40}")
    print(f"🚀 开始第 {iteration + 1} / {TOTAL_RUNS} 次实验")
    print(f"{'='*40}")

    # 1. 显式启动一个全新的 roscore (可选，也可以让 roslaunch 自动启)
    # 这里我们直接启动 launch，但因为 cleanup 杀得很干净，roslaunch 会启动全新的 Master

    # 2. 启动环境
    print("-> 1. 启动虚拟环境 (Fake Boat)...")
    cmd_env = ["roslaunch", "fake_planning", "fake_boat_target.launch"]
    proc_env = subprocess.Popen(cmd_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # NEW: 给环境启动留出一点领先时间
    time.sleep(3) 

    # 3. 启动规划器
    print("-> 2. 启动规划器 (Planner)...")
    cmd_planner = ["roslaunch", "planning", "simulation_tl.launch"]
    proc_planner = subprocess.Popen(cmd_planner, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    print(f"-> 等待系统初始化 ({INIT_WAIT_TIME}s)...")
    time.sleep(INIT_WAIT_TIME)

    # 4. 触发降落指令
    print("-> 3. 触发降落 (Trigger Land)...")
    trigger_script = "./sh_utils/auto_land.sh"
    
    if not os.access(trigger_script, os.X_OK):
        os.system(f"chmod +x {trigger_script}")

    try:
        subprocess.run(trigger_script, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"❌ 触发脚本执行失败: {e}")
        return

    # 5. 等待降落完成
    print(f"-> ⏳ 正在观测降落过程...")
    # 我们这里多等一会儿，确保 FSM 里的记录逻辑能写完 CSV
    time.sleep(SIM_DURATION)

    # 6. 🛑 优雅关闭
    print("-> 🛑 实验结束，正在回收进程...")
    
    # NEW: 建议先 Terminate (SIGTERM)，给节点写文件的机会，再清理
    proc_planner.terminate()
    proc_env.terminate()
    
    try:
        proc_planner.wait(timeout=3)
        proc_env.wait(timeout=3)
    except subprocess.TimeoutExpired:
        pass

    # NEW: 每一轮实验结束必须进行一次深度清理，重置“宇宙”
    cleanup_ros()

def main():
    def signal_handler(sig, frame):
        print("\n\n⚠️ 用户手动中断，清理并退出...")
        cleanup_ros()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)

    # 起始清理
    cleanup_ros()

    start_time = time.time()

    for i in range(TOTAL_RUNS):
        try:
            run_experiment(i)
        except Exception as e:
            print(f"❌ 实验 {i+1} 失败: {e}")
            cleanup_ros()
            continue

    total_time = (time.time() - start_time) / 60
    print(f"\n✅ 全部 {TOTAL_RUNS} 次实验已完成！")
    print(f"📊 总耗时: {total_time:.2f} 分钟")
    print("现在你可以去分析干净的 landing_states.csv 了。")

if __name__ == "__main__":
    main()