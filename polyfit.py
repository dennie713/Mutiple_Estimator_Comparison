import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import ImportData, Cal

# 定义拟合函数，这里使用一次函数
def linear_func(x, a, b):
    return a * x + b

# Constant
SamplingTime = 0.001
CPI = 1600
## 讀取檔案
path1 = ['D:\ASUS_program_code/規格測試/有線/IPS650_G50_motion_1.txt'] #馬達資料.txt路徑
path2 = ['D:\ASUS_program_code/規格測試/有線/IPS650_G50_mouse_1.txt']  #滑鼠資料.txt路徑
# path1 = ['D:\ASUS_program_code\控制台增強\WIRELESS\WIRELESS/1600/force\IPS500_G10_motion_1.txt'] #馬達資料.txt路徑
# path2 = ['D:\ASUS_program_code\控制台增強\WIRELESS\WIRELESS/1600/force\IPS500_G10_mouse_1.txt']  #滑鼠資料.txt路徑
Motordata, Mousedata = ImportData.ImportData(path1, path2)
mouseX, mouseY, Pos, PosCmd, Vel, VelCmd, TorCtrl, AccCmd, mousedata_data, mouse_displacement, mouse_real_Pos = Cal.Cal(Mousedata, Motordata, SamplingTime, CPI)    
t = np.arange(0, (len(Motordata[:,0])) * SamplingTime, SamplingTime)
# 准备数据
# x_data = np.array([1, 2, 3, 4, 5])
# y_data = np.array([2.3, 3.5, 4.2, 5.0, 5.8])

x_data = t
y_data = Vel
# 拟合直线
popt, pcov = curve_fit(linear_func, x_data, y_data)

# 获取拟合参数
a_fit, b_fit = popt

# 生成拟合直线的数据
x_fit = np.linspace(min(x_data), max(x_data), 100)
y_fit = linear_func(x_fit, a_fit, b_fit)

# 绘制原始数据和拟合直线
plt.scatter(x_data, y_data, s = 1.5, label='Pos')
plt.plot(x_fit, y_fit, 'r', label='Fitted Line')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Line Fitting Example')
plt.show()