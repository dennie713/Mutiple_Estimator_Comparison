import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math
import os
from scipy.interpolate import interp1d
from scipy.signal import butter, filtfilt
from datetime import datetime, timedelta
import time

# Constants
CPI = 1600
SamplingTime = 0.001
R = 12.5
def cal(Mousedata, Motordata):
    
    # Extracting data from Motordata
    Pos = np.array(Motordata[:, 3],float)
    # print(Pos)
    PosCmd = np.array(Motordata[:, 4],float)
    Vel = np.array(Motordata[:, 5],float)
    VelCmd = np.array(Motordata[:, 6],float)
    TorCtrl = np.array(Motordata[:, 7],float)
    t = np.arange(0, (len(Motordata[:,0])+270) * SamplingTime, SamplingTime)
    # print(Motordata)
    # Processing mouse data
    mousedata_data = mousedata_add(Mousedata)
    mouseX = np.array(mousedata_data[:, 0],float)
    mouseY = np.array(mousedata_data[:, 1],float)
    # mouse_displacement = np.array(mousedata[:, 4],float)
    mouse_displacement = mousedata_data[:, 5].astype(float)
    
    # Interpolating mouse displacement
    mouse_real_Pos = interp1d(np.array(mousedata_data[:, 2],float), mouse_displacement, fill_value="extrapolate")(t) / CPI
    
    # Filtering mouse data
    Pos_est_mouse, Vel_est_mouse, Acc_est_mouse = zero_phase_filter(3, 10, mouse_real_Pos)
    # print(mouse_real_Pos)
    
    # Scaling acceleration
    Acc_est_mouse *= 0.0254
    
    # Processing motor data
    Pos_est, Vel_est, Acc_est = zero_phase_filter(3, 8, Pos)
    Pos_Cmd, Vel_Cmd, Acc_Cmd = zero_phase_filter(3, 50, PosCmd)
    
    # Scaling velocity
    Vel_est = Vel_est / (2 * np.pi) * 60
    Vel_est_mouse = Vel_est_mouse
    
    # Scaling acceleration for real mouse acceleration
    Acc_est_mouse_real = Acc_est_mouse / 9.81
    
    return mouse_real_Pos, mouse_displacement, Vel_est_mouse, Acc_est_mouse, Acc_est_mouse_real    

#zero_phase_filter
def zero_phase_filter(Order, CutoffFreq, data):
    # data = []
    SamplingTime = 0.001
    
    # Calculate sampling frequency
    fm = 1 / SamplingTime
    # Butterworth 低通滤波器，正则化：CutoffFreq / ( Sampling / 2 )，返回传递函数的系数
    b, a = butter(Order, CutoffFreq / (fm / 2), 'low')
    
    # 零相位数字滤波器：正向和反向各滤波一次，以消除零相位失真
    est1 = filtfilt(b, a, data)
    
    # 计算一阶导数
    est2 = np.concatenate(([0], (est1[2:] - est1[:-2]) / (2 * SamplingTime), [0]))
    
    # 计算二阶导数
    est3 = np.concatenate(([0], (est1[2:] - 2 * est1[1:-1] + est1[:-2]) / (SamplingTime ** 2), [0]))
    
    return est1, est2, est3

#mousedata_add
def mousedata_add(mousedata_data):
    fmt = '%H:%M:%S.%f'
    # Initialize output array
    mousedata_data = np.zeros((len(Mousedata),6))
    for ii in range(1, len(Mousedata)):
        time1 = datetime.strptime(Mousedata[ii-1, 1], fmt).time()
        time2 = datetime.strptime(Mousedata[ii, 1], fmt).time()
        # diff = time2 - time1
        diff_seconds = (datetime.combine(datetime.min, time2) - datetime.combine(datetime.min, time1)).total_seconds()
        # seconds_diff = diff.total_seconds()
        # new_time = time1 + diff_seconds  # 新的時間
        new_datetime = datetime.combine(datetime.min, time1) + timedelta(seconds=diff_seconds)
        new_time = new_datetime.strftime(fmt)
        mousedata_data [ii, 0] = Mousedata[ii, 3]
        mousedata_data [ii, 1] = Mousedata[ii, 4]
        mousedata_data [ii, 2] =  mousedata_data [ii-1, 2] + diff_seconds
        mousedata_data [ii, 3] = float(Mousedata[ii, 3]) - float(Mousedata[ii-1, 3])  # dx
        mousedata_data [ii, 4] = float(Mousedata[ii, 4]) - float(Mousedata[ii-1, 4])  # dy
        mousedata_data [ii, 5] = mousedata_data[ii-1, 5] + ((mousedata_data [ii, 3]**2 + mousedata_data [ii, 4]**2)**0.5)  # dL       
    return mousedata_data 

if __name__ == "__main__":
    # 資料讀取路徑
    folder_path = 'D:\ASUS_program_code/45度_有線'
    # IPS值
    IPS = 'IPS'
    file_names = list(range(550, 660, 20))  # 多筆ips值
    # file_names = [550] # 單筆ips值
    # G值
    G = '_G20'
    # 檔案名
    data_name1 = '_motion.txt'
    data_name2 = '_mouse.txt'
    #想繪製的資料筆數
    # final_data_num = len(file_names)
    begin_data_num = 0  # 起始資料筆數
    final_data_num = len(file_names)  # 終止資料筆數
   
    Vel_est_mouse_append = []
    Acc_est_mouse_real_append = []
    path1_append = []
    
    for i in range(begin_data_num, final_data_num):
        #Motordata
        Motordata = []
        #Motion資料路徑
        path1 = os.path.join(folder_path, IPS + str(file_names[i]) + G + data_name1) 
        print(path1)
        with open(path1, 'r')as f:
            lines = f.readlines()
            for line in lines:
                line = line.split()
                Motordata.append(line) #馬達資料

        #Mousedata
        Mousedata = []
        #Mouse資料路徑
        path2 = os.path.join(folder_path, IPS + str(file_names[i]) + G + data_name2)
        with open(path2, 'r')as f:
            lines = f.readlines()
            for line in lines:
                line = line.split()
                Mousedata.append(line) #滑鼠資料

        #Motordata ,Mousedata    
        Motordata = np.array(Motordata)    
        Mousedata = np.array(Mousedata)

        #總位移計算
        mouse_X = np.array(Mousedata[:,3], float)
        mouse_Y = np.array(Mousedata[:,4], float)
    
        mouse_real_Pos, mouse_displacement, Vel_est_mouse, Acc_est_mousek, Acc_est_mouse_real = cal(Mousedata, Motordata)
        SamplingTime = 0.001
        # t = np.arange(0, (len(Motordata[:,0])) * SamplingTime, SamplingTime)
        Vel_est_mouse_append.append(Vel_est_mouse)
        Acc_est_mouse_real_append.append(Acc_est_mouse_real)
        path1_append.append(path1)

    # velocity plot
    plt.figure(1)
    for i in range(begin_data_num, final_data_num):
        Vel_est_mouse_1 = Vel_est_mouse_append[i]
        t = np.arange(0, (len(Vel_est_mouse_1)) * SamplingTime, SamplingTime)
        # 获取文件名（不包括路径）
        label = os.path.basename(path1_append[i])
        # 去除后缀 '_G20_motion'
        label = label.replace('_G20_motion.txt', '')
        # 绘制图像，替代 'plot' 函数以适应您的数据和绘图库
        plt.plot(t, Vel_est_mouse_1, label = label)
    plt.yticks(np.arange(0, 660, 100))
    plt.axis([0, t[-1], min(Vel_est_mouse_1), max(Vel_est_mouse_1)])
    plt.grid()
    # plt.legend(loc='lower center')
    plt.legend(loc='lower center', ncol=3)
    plt.title("45 degrees-Mouse-Velocity Plot")
    plt.xlabel("Time (sec)")
    plt.ylabel("IPS")

    # Acceleration plot
    plt.figure(2)
    for i in range(begin_data_num, final_data_num):
        Acc_est_mouse_real_1 = Acc_est_mouse_real_append[i]
        t = np.arange(0, (len(Acc_est_mouse_real_1)) * SamplingTime, SamplingTime)
        # 获取文件名（不包括路径）
        label = os.path.basename(path1_append[i])
        # 去除后缀 '_G20_motion'
        label = label.replace('_G20_motion.txt', '')
        # 绘制图像，替代 'plot' 函数以适应您的数据和绘图库
        plt.plot(t, Acc_est_mouse_real_1, label = label)
    plt.yticks(np.arange(-25, 25, 5))
    plt.grid()
    plt.axis([0, t[-1], min(Acc_est_mouse_real_1), max(Acc_est_mouse_real_1)])
    # plt.legend(loc='lower center')
    plt.legend(loc='lower center', ncol=3)
    plt.title("45 degrees-Mouse-Acceleration Plot at Different Speeds")
    plt.xlabel("Time (sec)")
    plt.ylabel("G")

    plt.figure(3)
    for i in range(begin_data_num, final_data_num):
        # Acc_est_mouse_real_1 = Acc_est_mouse_real_append[i]
        t = np.arange(0, (len(mouse_real_Pos)) * SamplingTime, SamplingTime)
        # 获取文件名（不包括路径）
        label = os.path.basename(path1_append[i])
        # 去除后缀 '_G20_motion'
        label = label.replace('_G20_motion.txt', '')
        # 绘制图像，替代 'plot' 函数以适应您的数据和绘图库
        mouse_displacement = mouse_displacement/CPI
        plt.plot(t, mouse_real_Pos, label = label)
    # plt.yticks(np.arange(-25, 25, 5))
    plt.grid()
    plt.axis([0, t[-1], min(mouse_real_Pos), max(mouse_real_Pos)])
    # plt.legend(loc='lower center')
    plt.legend(loc='lower center', ncol=3)
    plt.title("Mouse Displacement Plot at Different Speeds")
    plt.xlabel("Time (sec)")
    plt.ylabel("Displacement (inch)")
    
    plt.show()

