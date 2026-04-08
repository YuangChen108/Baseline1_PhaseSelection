import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

def run_analysis():
    # 路径配置
    base_path = "/home/Quadrotor-Landing-with-Minco/experiments_data/baseline_data_0.5_1.0.csv"
    prop_path = "/home/Quadrotor-Landing-with-Minco/experiments_data/proposed_data_0.5_1.0.csv"
    
    if not (os.path.exists(base_path) and os.path.exists(prop_path)):
        print("❌ 错误: 找不到 CSV 文件。")
        return

    cols = ["Time","UAV_X","UAV_Y","UAV_Z","Boat_X","Boat_Y","Boat_Z",
            "Dist_XY","Dist_Z","Rel_Speed","Rel_VZ","Wave_Z","Success","Reason"]
    
    # 1. 读取数据
    df_b = pd.read_csv(base_path, names=cols)
    df_p = pd.read_csv(prop_path, names=cols)
    df_b['Method'] = 'Baseline (OFF)'
    df_p['Method'] = 'Proposed (ON)'
    df_raw = pd.concat([df_b, df_p])

    # 2. 数据格式强制转换与清理
    for col in ['Dist_XY', 'Dist_Z', 'Rel_VZ', 'Success']:
        df_raw[col] = pd.to_numeric(df_raw[col], errors='coerce')
    df_raw = df_raw.dropna(subset=['Dist_XY', 'Dist_Z', 'Success'])
    
    # ==========================================
    # 📊 提前计算所有核心统计指标 (用于画图与表格)
    # ==========================================
    # 失败率与避险次数计算
    fails_b = len(df_raw[(df_raw['Method'] == 'Baseline (OFF)') & (df_raw['Success'] == 0)])
    fails_p = len(df_raw[(df_raw['Method'] == 'Proposed (ON)') & (df_raw['Success'] == 0)])
    trials_b = len(df_raw[df_raw['Method'] == 'Baseline (OFF)'])
    trials_p = len(df_raw[df_raw['Method'] == 'Proposed (ON)'])
    
    rate_b = fails_b / trials_b if trials_b > 0 else 0
    rate_p = fails_p / trials_p if trials_p > 0 else 0
    fail_reduction = ((rate_b - rate_p) / rate_b * 100) if rate_b > 0 else 0
    avoided_fails = int((rate_b * trials_p) - fails_p) if rate_b > 0 else 0

    # 剔除物理爆炸点，计算精度平均值与标准差 (Mean and Std)
    df = df_raw[(df_raw['Dist_XY'] < 10.0) & (df_raw['Dist_Z'] < 5.0) & (df_raw['Rel_VZ'] < 4.0)].copy()
    
    # 🌟 恢复 Mean 和 Std 的联合计算
    stats = df.groupby('Method').agg({
        'Dist_XY': ['mean', 'std'],
        'Dist_Z':  ['mean', 'std'],
        'Rel_VZ':  ['mean', 'std']
    })
    
    def calc_imp(base, prop): return (base - prop) / base * 100 if base != 0 else 0
    imp_xy = calc_imp(stats['Dist_XY']['mean']['Baseline (OFF)'], stats['Dist_XY']['mean']['Proposed (ON)'])
    imp_z  = calc_imp(stats['Dist_Z']['mean']['Baseline (OFF)'], stats['Dist_Z']['mean']['Proposed (ON)'])
    imp_vz = calc_imp(stats['Rel_VZ']['mean']['Baseline (OFF)'], stats['Rel_VZ']['mean']['Proposed (ON)'])

    # ==========================================
    # 🎨 图表 1：箱线图 (分布与稳定性)
    # ==========================================
    fig1 = plt.figure(figsize=(18, 6))
    sns.set(style="whitegrid", font_scale=1.1)
    
    plt.subplot(1, 3, 1)
    sns.boxplot(x='Method', y='Dist_XY', hue='Method', data=df, palette="Set2", showmeans=True, dodge=False, legend=False)
    sns.stripplot(x='Method', y='Dist_XY', hue='Method', data=df, palette="dark:gray", alpha=0.4, dodge=False, legend=False)
    plt.title("Horizontal Accuracy (Dist_XY)", fontsize=14, pad=10)
    plt.ylabel("Error (m)")

    plt.subplot(1, 3, 2)
    sns.boxplot(x='Method', y='Dist_Z', hue='Method', data=df, palette="Set3", showmeans=True, dodge=False, legend=False)
    sns.stripplot(x='Method', y='Dist_Z', hue='Method', data=df, palette="dark:gray", alpha=0.4, dodge=False, legend=False)
    plt.title("Vertical Accuracy (Dist_Z)", fontsize=14, pad=10)
    plt.ylabel("Error (m)")

    plt.subplot(1, 3, 3)
    sns.boxplot(x='Method', y='Rel_VZ', hue='Method', data=df, palette="Set1", showmeans=True, dodge=False, legend=False)
    sns.stripplot(x='Method', y='Rel_VZ', hue='Method', data=df, palette="dark:gray", alpha=0.4, dodge=False, legend=False)
    plt.title("Vertical Impact Velocity (Rel_VZ)", fontsize=14, pad=10)
    plt.ylabel("Speed (m/s)")

    plt.tight_layout()
    plt.savefig('experimental_results_boxplots.png', dpi=300, bbox_inches='tight')
    plt.close(fig1)

    # ==========================================
    # 🎨 图表 2：宏观统计柱状图 (失败率与提升比例)
    # ==========================================
    fig2 = plt.figure(figsize=(12, 6))
    sns.set(style="whitegrid", font_scale=1.1)
    
    # 子图1：失败率对比
    ax1 = plt.subplot(1, 2, 1)
    bars1 = plt.bar(['Baseline\n(OFF)', 'Proposed\n(ON)'], [rate_b*100, rate_p*100], color=['#e74c3c', '#2ecc71'], width=0.4)
    plt.title("Failure Rate Comparison", fontsize=14, pad=10)
    plt.ylabel("Failure Rate (%)")
    plt.ylim(0, max(rate_b*100, rate_p*100) + 15)
    for bar in bars1:
        yval = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2, yval + 1.5, f'{yval:.1f}%', ha='center', va='bottom', fontsize=12, fontweight='bold')

    # 子图2：各项指标提升百分比
    ax2 = plt.subplot(1, 2, 2)
    bars2 = plt.bar(['XY Accuracy', 'Z Accuracy', 'VZ Softness'], [imp_xy, imp_z, imp_vz], color=['#3498db', '#9b59b6', '#f1c40f'], width=0.5)
    plt.title("Performance Improvement over Baseline", fontsize=14, pad=10)
    plt.ylabel("Improvement (%)")
    plt.axhline(0, color='black', linewidth=1) 

    # 动态调整 Y 轴下限，防止向下的柱子碰到图表边缘
    min_y = min(imp_xy, imp_z, imp_vz, 0)
    max_y = max(imp_xy, imp_z, imp_vz, 0)
    plt.ylim(min_y - 5, max_y + 10) 

    for bar in bars2:
        yval = bar.get_height()
        
        # 🌟 核心修改：如果数值为负，强行将文字基准线设为 X 轴上方 (1.5)
        text_y = yval + 1.5 if yval >= 0 else 1.5
        
        # 统一使用 va='bottom'，确保文字永远在设定坐标的上方
        plt.text(bar.get_x() + bar.get_width()/2, text_y, f'{yval:.1f}%', 
                 ha='center', va='bottom', fontsize=12, fontweight='bold')

    plt.tight_layout()
    plt.savefig('experimental_results_barcharts.png', dpi=300, bbox_inches='tight')
    plt.close(fig2)

    # ==========================================
    # 🖨️ 终端打印：论文专用格式输出
    # ==========================================
    print("\n" + "="*75)
    print("                📝 论文专用表格数据 (Mean ± Std)")
    print("="*75)
    print(f"{'Method':<16} | {'Dist_XY (m)':<16} | {'Dist_Z (m)':<16} | {'Rel_VZ (m/s)':<16}")
    print("-" * 75)
    
    for method in ['Baseline (OFF)', 'Proposed (ON)']:
        xy_m, xy_s = stats.loc[method, ('Dist_XY', 'mean')], stats.loc[method, ('Dist_XY', 'std')]
        z_m,  z_s  = stats.loc[method, ('Dist_Z', 'mean')],  stats.loc[method, ('Dist_Z', 'std')]
        vz_m, vz_s = stats.loc[method, ('Rel_VZ', 'mean')],  stats.loc[method, ('Rel_VZ', 'std')]
        
        # 格式化为 Mean ± Std
        print(f"{method:<16} | {xy_m:.4f} ± {xy_s:.4f} | {z_m:.4f} ± {z_s:.4f} | {vz_m:.4f} ± {vz_s:.4f}")
    
    print("="*75)
    
    print("\n" + "-" * 50)
    print(f"🚀 XY 水平精度提升: {imp_xy:.1f}%")
    print(f"🚀 Z  垂直精度提升: {imp_z:.1f}%")
    print(f"🚀 VZ 垂直砸击速度降低: {imp_vz:.1f}%")
    print("-" * 50)
    
    print(f"⚠️ Baseline 失败次数: {fails_b} / {trials_b} (失败率: {rate_b*100:.1f}%)")
    print(f"✅ Proposed 失败次数: {fails_p} / {trials_p} (失败率: {rate_p*100:.1f}%)")
    
    print("\n" + "🌟"*25)
    print("          🏆 核心避险结论 🏆")
    print("🌟"*25)
    print(f"▶ 算法测试总次数: {trials_p} 次")
    print(f"▶ 等效坠机避险次数: {avoided_fails} 次")
    print(f"\n结论可以直接抄进论文正文：")
    print(f"The proposed prediction algorithm reduced the landing failure rate by a relative {fail_reduction:.1f}%!")
    print("="*50)

if __name__ == "__main__":
    run_analysis()