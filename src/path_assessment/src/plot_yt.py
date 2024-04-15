import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV文件
df = pd.read_csv('data/social.csv')

# 提取 't'、'idx_o'、'eva_avg_o' 和 'eva_cur_o' 数据
t = df['t']
idx_h = df['idx_h']
eva_avg_h = df['eva_avg_h']
eva_cur_h = df['eva_cur_h']
idx_o = df['idx_o']
eva_avg_o = df['eva_avg_o']
eva_cur_o = df['eva_cur_o']
idx_h = df['idx_h']
eva_avg_h = df['eva_avg_h']
eva_cur_h = df['eva_cur_h']

# # 创建新的 figure
# plt.figure(figsize=(10, 5))
# # 绘制 'idx_o' 和 't' 图
# plt.subplot(4, 1, 2)
# plt.plot(t, idx_h, label='idx_h')
# plt.xlabel('t')
# plt.ylabel('idx_h')
# plt.title('y-t Graph')
# plt.grid(True)
# plt.ylim(-1, 100)  # 设置y轴范围
# plt.legend()

# # 绘制 'eva_avg_o' 和 'eva_cur_o' 图
# plt.subplot(4, 1, 1)
# plt.plot(t, eva_avg_h, label='eva_avg_h')
# plt.plot(t, eva_cur_h, label='eva_cur_h')
# plt.xlabel('t')
# plt.ylabel('eva')
# plt.title('eva_avg_h and eva_cur_h Graph')
# plt.grid(True)
# plt.ylim(0, 1.01)  # 设置y轴范围
# plt.legend()

# # 绘制 'idx_o' 和 't' 图
# plt.subplot(4, 1, 4)
# plt.plot(t, idx_o, label='idx_o')
# plt.xlabel('t')
# plt.ylabel('idx_o')
# plt.title('y-t Graph')
# plt.grid(True)
# plt.ylim(-1, 100)  # 设置y轴范围
# plt.legend()

# # 绘制 'eva_avg_o' 和 'eva_cur_o' 图
# plt.subplot(4, 1, 3)
# plt.plot(t, eva_avg_o, label='eva_avg_o')
# plt.plot(t, eva_cur_o, label='eva_cur_o')
# plt.xlabel('t')
# plt.ylabel('eva')
# plt.title('eva_avg_o and eva_cur_o Graph')
# plt.grid(True)
# plt.ylim(0, 1.01)  # 设置y轴范围
# plt.legend()


# 创建新的 figure
plt.figure(figsize=(10, 5))
# 绘制 'idx_o' 和 't' 图
plt.subplot(2, 1, 2)
plt.plot(t, idx_h, label='idx_h')
plt.plot(t, idx_o, label='idx_o')
plt.xlabel('t')
plt.ylabel('idx')
plt.title('y-t Graph')
plt.grid(True)
plt.ylim(-1, 100)  # 设置y轴范围
plt.legend()

# 绘制 'eva_avg_o' 和 'eva_cur_o' 图
plt.subplot(2, 1, 1)
plt.plot(t, eva_avg_h, label='eva_avg_h')
plt.plot(t, eva_cur_h, label='eva_cur_h')
plt.plot(t, eva_avg_o, label='eva_avg_o')
plt.plot(t, eva_cur_o, label='eva_cur_o')
plt.xlabel('t')
plt.ylabel('eva')
plt.title('eva_avg and eva_cur Graph')
plt.grid(True)
plt.ylim(0, 1.01)  # 设置y轴范围
plt.legend()


plt.tight_layout()
plt.show()
