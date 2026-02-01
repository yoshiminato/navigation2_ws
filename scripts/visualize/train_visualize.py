import pandas as pd
import matplotlib.pyplot as plt
import japanize_matplotlib  

columns = ['steps','time', 'terminal_index', 'reward', 'time_in_ep']

ws = '/home/yoshi/navigation2_ws/'

# df1 = pd.read_csv(ws + 'simulation_logs/simulation_1.log', names=columns)
df1 = pd.read_csv(ws + 'results/train/C_/sac/sim1.log', names=columns)
df2 = pd.read_csv(ws + 'results/train/C/sac/sim1.log', names=columns)
df3 = pd.read_csv(ws + 'results/train/B/dqn/sim1.log', names=columns) 
df4 = pd.read_csv(ws + 'results/train/B/sac/sim1.log', names=columns)

r_count1 = 8
r_count2 = 8
r_count3 = 8
r_count4 = 8

df1['steps'] = df1['steps'] * r_count1
df2['steps'] = df2['steps'] * r_count2
df3['steps'] = df3['steps'] * r_count3
df4['steps'] = df4['steps'] * r_count4

window =5000
min_periods = 20

status_names = {
    'status_1': '衝突',
    'status_2': 'タイムアウト',
    'status_3': '成功'
}

# 移動平均の計算（ウィンドウサイズ50）
df1['reward_ma'] = df1['reward'].rolling(window=window, min_periods=min_periods).mean()

plt.figure(figsize=(10, 5))
plt.plot(df1['steps'], df1['reward_ma']) # 移動平均
plt.xlabel('ステップ数')
plt.ylabel(f'平均報酬({window}エピソード移動平均)')
plt.title('学習曲線')
plt.legend()
plt.show()

### ==================================================================== ###

status_dummies1 = pd.get_dummies(df1['terminal_index'], prefix='status')
temp_df1 = pd.concat([df1, status_dummies1], axis=1)

status_cols1 = [col for col in temp_df1.columns if col in status_names]
    
status_ma1 = temp_df1[status_cols1].rolling(window=window, min_periods=min_periods).mean() * 100

plt.figure(figsize=(10, 5))
for col in status_cols1:
    label_name = status_names.get(col, col)
    plt.plot(temp_df1['steps'], status_ma1[col], label=label_name)
plt.title(f'終了状態の確率分布')
plt.xlabel('ステップ数')
plt.ylabel(f'各状態の確率(移動平均 w={window}) [%]')
plt.legend()
plt.grid(True, alpha=0.3)
plt.show()

### ==================================================================== ###

df1_success = df1[df1['terminal_index'] == 3]

df1_success['time_in_ep_ma'] = df1_success['time_in_ep'].rolling(window=window, min_periods=min_periods).mean()

plt.figure(figsize=(10, 5))
plt.plot(df1_success['steps'], df1_success['time_in_ep_ma']) # 移動平均
plt.xlabel('ステップ数')
plt.ylabel(f'平均クリア時間({window}エピソード移動平均)')
plt.title('学習曲線')
plt.legend()
plt.show()
