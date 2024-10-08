import numpy as np
import matplotlib.pyplot as plt

# 定义参数
num_samples = 1000  # 生成的样本数量
sampingTime = 0.001
amplitude = 1  # 振幅
frequency = 1  # 频率（每个周期所包含的点的数量）
phase = np.pi*2   # 相位（弧度）

# 生成位置命令
a = 2*np.pi / sampingTime
t = np.arange(0, 2*np.pi, 0.001 )  # 时间序列（0到2π）

pos = np.zeros((len(t), 8))
# positions = 0 + amplitude * np.sin(frequency * t + phase)  # 正弦函数
positions = np.sin(t)
pos[:,0] = positions
pos[:,1] = 0
print(pos)
with open('positions.txt', 'w') as f:
    for pos in pos:
        f.write(str(pos) + '\t')
# pos.append(positions)
# print(len(positions))


# 绘制位置命令的图形（可选）
plt.plot(t, positions)
plt.xlabel('Time')
plt.ylabel('Position')
plt.title('Sine Wave Position Command')
plt.show()