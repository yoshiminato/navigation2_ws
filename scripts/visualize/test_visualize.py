import pandas as pd
import matplotlib.pyplot as plt
import japanize_matplotlib  

columns = ['steps','time', 'terminal_index', 'reward', 'time_in_ep']

ws = '/home/yoshi/navigation2_ws/'

df1 = pd.read_csv(ws + 'results/test/D/dwb.log', names=columns)
df2 = pd.read_csv(ws + 'results/test/D/mppi.log', names=columns)
df3 = pd.read_csv(ws + 'results/test/D/C__sac.log', names=columns)

df1_success = df1[df1['terminal_index'] == 3]
df2_success = df2[df2['terminal_index'] == 3]
df3_success = df3[df3['terminal_index'] == 3]


success_counts = [
    df1_success.shape[0],
    df2_success.shape[0],
    df3_success.shape[0],
]

clear_time_averages = [
    df1_success['time_in_ep'].mean(),
    df2_success['time_in_ep'].mean(),
    df3_success['time_in_ep'].mean(),
]



# ラベル定義（ファイル名に対応）
labels = ['DWB', 'MPPI', 'SAC']

# 色のリストを定義
colors = ['#ff6b6b', '#4ecdc4', '#45b7d1']


# 成功率プロット
plt.figure(figsize=(8, 6))
bars = plt.bar(labels, success_counts, color=colors, width=0.6)

plt.xlabel('モデル')
plt.ylabel('成功回数')
plt.title('各モデルの総成功数比較')
plt.grid(axis='y', alpha=0.3)

# 棒グラフの上に数値を表示
for bar in bars:
    height = bar.get_height()
    plt.text(bar.get_x() + bar.get_width() / 2, height, f'{int(height)}', 
             ha='center', va='bottom', fontsize=12)

plt.show()

# クリア時間プロット
plt.figure(figsize=(8, 6))
bars = plt.bar(labels, clear_time_averages, color=colors, width=0.6)
    
plt.xlabel('モデル')
plt.ylabel('平均クリア時間')
plt.title('各モデルの平均クリア時間比較')
plt.grid(axis='y', alpha=0.3)

# 棒グラフの上に数値を表示
for bar in bars:
    height = bar.get_height()
    plt.text(bar.get_x() + bar.get_width() / 2, height, f'{height:.2f}', 
             ha='center', va='bottom', fontsize=12)

plt.show()