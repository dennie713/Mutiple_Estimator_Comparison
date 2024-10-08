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
def cal(Mousedata, Motordata):
    # Extracting data from Motordata
    Pos = np.array(Motordata[:, 3],float)
    PosCmd = np.array(Motordata[:, 4],float)
    Vel = np.array(Motordata[:, 5],float)
    VelCmd = np.array(Motordata[:, 6],float)
    TorCtrl = np.array(Motordata[:, 7],float)
    t = np.arange(0, len(Motordata[:,0]) * SamplingTime, SamplingTime)
    # print(Motordata)
    # Processing mouse data
    mousedata_data = mousedata_add(Mousedata)
    mouseX = np.array(mousedata_data[:, 0],float)
    mouseY = np.array(mousedata_data[:, 1],float)
    # mouse_displacement = np.array(mousedata[:, 4],float)
    mouse_displacement = mousedata_data[:, 5].astype(float)
    # print(mouse_displacement)
    # print(mousedata_data)
    # Interpolating mouse displacement
    mouse_real_Pos = interp1d(np.array(mousedata_data[:, 2],float), mouse_displacement, fill_value="extrapolate")(t) / CPI
    
    # Processing motor data
    Pos_est, Vel_est, Acc_est = zero_phase_filter(3, 17, Pos)
    Pos_Cmd, Vel_Cmd, Acc_Cmd = zero_phase_filter(3, 50, PosCmd)

    # Filtering mouse data
    Pos_est_mouse, Vel_est_mouse, Acc_est_mouse = zero_phase_filter(3, 14, mouse_real_Pos)

    # Scaling acceleration
    Acc_est_mouse *= 0.0254
    
    # Scaling velocity
    Vel_est = Vel_est / (2 * np.pi) * 60
    Vel_est_mouse = Vel_est_mouse
    
    # Scaling acceleration for real mouse acceleration
    Acc_est_mouse_real = Acc_est_mouse / 9.81
    
    return Pos, Vel, Pos_Cmd, Vel_Cmd, Acc_Cmd, TorCtrl, Pos_est, Vel_est, Acc_est, Pos_est_mouse, Vel_est_mouse, Acc_est_mouse, Acc_est_mouse_real, mousedata_data

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
    path1 = ['D:\ASUS_program_code/規格測試/有線/IPS650_G50_motion_1.txt'] #馬達資料.txt路徑
    path2 = ['D:\ASUS_program_code/規格測試/有線/IPS650_G50_mouse_1.txt']  #滑鼠資料.txt路徑
    for i in range (len(path1)):
        #Motordata
        Motordata = []
        with open(path1[i], 'r')as f:
            lines = f.readlines()
            for line in lines:
                line = line.split()
                Motordata.append(line) #馬達資料

        #Mousedata
        Mousedata = []
        with open(path2[i], 'r')as f:
            lines = f.readlines()
            for line in lines:
                line = line.split()
                Mousedata.append(line) #滑鼠資料

        #Motordata ,Mousedata    
        Motordata = np.array(Motordata)    
        Mousedata = np.array(Mousedata)

        #總位移計算
        mouse_X = abs(np.array(Mousedata[:,3], float))
        mouse_Y = np.array(Mousedata[:,4], float)

    Pos, Vel, Pos_Cmd, Vel_Cmd, Acc_Cmd, TorCtrl, Pos_est, Vel_est, Acc_est, Pos_est_mouse, Vel_est_mouse, Acc_est_mouse, Acc_est_mouse_real, mousedata_data = cal(Mousedata, Motordata)
    SamplingTime = 0.001
    t = np.arange(0, (len(Motordata[:,0])) * SamplingTime, SamplingTime)

    # plt.figure(1)
    # plt.plot(t, Pos, label='Pos')
    # plt.plot(t, Pos_Cmd, label='PosCmd')
    # plt.title("Pos")
    # plt.legend()
    # plt.xlabel("Time(sec)")
    # plt.ylabel("rad")

    # # Plot Poserr
    # plt.figure(2)
    # plt.plot(t, Pos_Cmd - Pos)
    # plt.title("Poserr")

    # # Plot Vel and VelCmd
    # plt.figure(3)
    # plt.plot(t, Vel, label='Vel')
    # plt.plot(t, Vel_Cmd, label='VelCmd')
    # plt.title("Vel")
    # plt.legend()
    # plt.xlabel("Time(sec)")
    # plt.ylabel("rad/s")

    # # Plot Velerr
    # plt.figure(4)
    # plt.plot(t, Vel_Cmd - Vel)
    # plt.title("Velerr")

    # # Plot TorCtrl
    # plt.figure(5)
    # plt.plot(t, TorCtrl)
    # plt.title("TorCtrl")

    # # Plot mousex
    # plt.figure(6)
    # plt.plot(mousedata_data[:,2], mouse_X/CPI)
    # plt.title("mousex")
    # plt.xlabel("Time(sec)")
    # plt.ylabel("in")

    # # Plot mousey
    # plt.figure(7)
    # plt.plot(mousedata_data[:,2], mouse_Y/CPI)
    # plt.title("mousey")
    # plt.xlabel("Time(sec)")
    # plt.ylabel("in")

    # # Plot mousex^2+mousey^2
    # plt.figure(8)
    # plt.plot(mousedata_data[:,2], np.sqrt(mouse_X**2 + mouse_Y**2)/CPI)
    # plt.title("mousex^2+mousey^2")
    # plt.xlabel("Time(sec)")
    # plt.ylabel("in")

    # Plot mouse path
    plt.figure(10)
    plt.plot(mouse_X/CPI, mouse_Y/CPI)
    r = (mouse_Y[len(Mousedata)-1]-mouse_Y[0])/(mouse_X[len(Mousedata)-1]-mouse_X[0])
    plt.text(3, 8, r, fontsize=12, color='blue')
    plt.title("Mouse Path")
    plt.xlabel("Mouse X Displacement (in)")
    plt.ylabel("Mouse Y Displacement (in)")

    # 位移比较图
    plt.figure(11)
    plt.plot(t, Pos_est, label="Filtered Motor Displacement")
    plt.title("Filtered Motor and Mouse Displacement Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Displacement (rad)")
    plt.axis([0, t[-1], min(Pos_est), max(Pos_est)])
    plt.legend(loc="upper center")

    plt.twinx()
    plt.plot(t, Pos_est_mouse, 'r', label="Filtered Mouse Displacement")
    plt.ylabel("Mouse Displacement (pixel)")
    plt.legend(loc="lower center")

    # 速度比较图
    plt.figure(12)
    plt.plot(t, Vel_est/60*12.5*2*3.14/2.54, label="Filtered Motor Speed")
    plt.title("Filtered Motor and Mouse Speed Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Velocity (IPS)")
    # plt.axis([0, t[-1], min(Vel_est/60*12.5*2*3.14/2.54), max(Vel_est/60*12.5*2*3.14/2.54)])
    plt.axis([0, t[-1], 0, max(max(Vel_est/60*12.5*2*3.14/2.54),max(Vel_est_mouse))+50])
    # plt.legend(loc="center")
    # 获取第一个图例的标签
    handles1, labels1 = plt.gca().get_legend_handles_labels()

    plt.twinx()
    plt.plot(t, Vel_est_mouse, 'r', label="Filtered Mouse Speed")
    plt.ylabel("Mouse Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_est/60*12.5*2*3.14/2.54),max(Vel_est_mouse))+50])
    # plt.legend(loc="lower center")
    # 获取第二个图例的标签
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    # 合并两个标签列表
    handles = handles1 + handles2
    labels = labels1 + labels2
    # 添加图例
    plt.legend(handles, labels, loc="lower center")

    # 加速度比较图
    plt.figure(13)
    plt.plot(t, Acc_est*0.125/9.81, label="Filtered Motor Acceleration")
    # plt.plot(t, Acc_est*0.125/9.81, 'DisplayName', 'Filtered Motor Acceleration')
    plt.title("Filtered Motor and Mouse Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Acceleration (G)")
    # plt.axis([0, t[-1], min(Acc_est), max(Acc_est)])
    plt.axis([0, t[-1], min(min(Acc_est*0.125/9.81),min(Acc_est_mouse_real))-5, max(max(Acc_est*0.125/9.81),max(Acc_est_mouse_real))+5])
    # plt.legend(loc="upper center")
    # 获取第一个图例的标签
    handles1, labels1 = plt.gca().get_legend_handles_labels()

    plt.twinx()
    plt.plot(t, Acc_est_mouse_real, 'r', label="Filtered Mouse Acceleration")
    # plt.plot(t, Acc_est_mouse_real, 'r', 'DisplayName', 'Filtered Mouse Acceleration')
    plt.ylabel("Mouse Acceleration (G)")
    plt.axis([0, t[-1], min(min(Acc_est*0.125/9.81),min(Acc_est_mouse_real))-5, max(max(Acc_est*0.125/9.81),max(Acc_est_mouse_real))+5])
    # plt.legend(loc="lower center")
    # 获取第二个图例的标签
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    # 合并两个标签列表
    handles = handles1 + handles2
    labels = labels1 + labels2
    # 添加图例
    plt.legend(handles, labels, loc="lower center")

    # 速度误差
    plt.figure(14)
    denominator =  Vel_est * 12.5 * 2 * np.pi / 60 / 2.5
    deviation = np.abs(Vel_est_mouse - denominator) / np.where(denominator != 0, denominator, 1) * 100
    plt.plot(t, deviation)
    plt.title("Filtered Velocity Error")
    plt.xlabel("Time (sec)")
    plt.ylabel("Error (%)")

    #加速度誤差
    # plt.figure(15)
    # denominator = Acc_est * 12.5 * 0.01 / 9.81
    # deviation = np.abs(Acc_est_mouse_real - denominator) / np.where(denominator != 0, denominator, 1) * 100
    # plt.plot(t, deviation)
    # plt.title("Filtered Acceleration Error")
    # plt.xlabel("Time (sec)")
    # plt.ylabel("Error (%)")
    plt.show()

# fig, axs = plt.subplots(4, 4, figsize=(15, 15))
# # # Plot Pos and PosCmd
# axs[0, 0].plot(t, Pos, label='Pos')
# axs[0, 0].plot(t, Pos_Cmd, label='PosCmd')
# axs[0, 0].set_title("Pos")
# axs[0, 0].legend()
# axs[0, 0].set_xlabel("Time(sec)")
# axs[0, 0].set_ylabel("rad")

# # Plot Poserr
# axs[0, 1].plot(t, Pos_Cmd - Pos)
# axs[0, 1].set_title("Poserr")

# # Plot Vel and VelCmd
# axs[1, 0].plot(t, Vel, label='Vel')
# axs[1, 0].plot(t, Vel_Cmd, label='VelCmd')
# axs[1, 0].set_title("Vel")
# axs[1, 0].legend()
# axs[1, 0].set_xlabel("Time(sec)")
# axs[1, 0].set_ylabel("rad/s")

# # Plot Velerr
# axs[1, 1].plot(t, Vel_Cmd - Vel)
# axs[1, 1].set_title("Velerr")

# # Plot TorCtrl
# axs[2, 0].plot(t, TorCtrl)
# axs[2, 0].set_title("TorCtrl")

# # Plot mousex
# axs[2, 1].plot(mousedata_data[:, 2], mouse_X / CPI)
# axs[2, 1].set_title("mousex")
# axs[2, 1].set_xlabel("Time(sec)")
# axs[2, 1].set_ylabel("in")

# # Plot mousey
# axs[3, 0].plot(mousedata_data[:, 2], mouse_Y / CPI)
# axs[3, 0].set_title("mousey")
# axs[3, 0].set_xlabel("Time(sec)")
# axs[3, 0].set_ylabel("in")

# # Plot mousex^2+mousey^2
# axs[3, 1].plot(mousedata_data[:, 2], np.sqrt(mouse_X ** 2 + mouse_Y ** 2) / CPI)
# axs[3, 1].set_title("mousex^2+mousey^2")
# axs[3, 1].set_xlabel("Time(sec)")
# axs[3, 1].set_ylabel("in")

# # Plot mouse path
# axs[0, 2].plot(mouse_X / CPI, mouse_Y / CPI)
# r = (mouse_Y[len(Mousedata) - 1] - mouse_Y[0]) / (mouse_X[len(Mousedata) - 1] - mouse_X[0])
# axs[0, 2].text(3, 8, str(r), fontsize=12, color='blue')
# axs[0, 2].set_title("Mouse Path")
# axs[0, 2].set_xlabel("Mouse X Displacement (in)")
# axs[0, 2].set_ylabel("Mouse Y Displacement (in)")

# # 位移比较图
# axs[0, 3].plot(t, Pos_est, label="Filtered Motor Displacement")
# axs[0, 3].set_title("Filtered Motor and Mouse Displacement Comparison")
# axs[0, 3].set_xlabel("Time (sec)")
# axs[0, 3].set_ylabel("Motor Displacement (rad)")
# axs[0, 3].axis([0, t[-1], min(Pos_est), max(Pos_est)])
# axs[0, 3].legend(loc="upper center")

# axs[0, 3].twinx()
# axs[0, 3].plot(t, Pos_est_mouse, 'r', label="Filtered Mouse Displacement")
# axs[0, 3].set_ylabel("Mouse Displacement (pixel)")
# axs[0, 3].legend(loc="lower center")

# # 速度比较图
# axs[1, 2].plot(t, Vel_est, label="Filtered Motor Speed")
# axs[1, 2].set_title("Filtered Motor and Mouse Speed Comparison")
# axs[1, 2].set_xlabel("Time (sec)")
# axs[1, 2].set_ylabel("Motor Velocity (rpm)")
# axs[1, 2].axis([0, t[-1], min(Vel_est), max(Vel_est)])
# axs[1, 2].legend(loc="center")

# axs[1, 2].twinx()
# axs[1, 2].plot(t, Vel_est_mouse, 'r', label="Filtered Mouse Speed")
# axs[1, 2].set_ylabel("Mouse Velocity (IPS)")
# axs[1, 2].legend(loc="lower center")

# # 加速度比较图
# axs[1, 3].plot(t, Acc_est, label="Filtered Motor Acceleration")
# axs[1, 3].set_title("Filtered Motor and Mouse Acceleration Comparison")
# axs[1, 3].set_xlabel("Time (sec)")
# axs[1, 3].set_ylabel("Motor Acceleration (rad/s^2)")
# axs[1, 3].axis([0, t[-1], min(Acc_est), max(Acc_est)])
# axs[1, 3].legend(loc="upper center")

# axs[1, 3].twinx()
# axs[1, 3].plot(t, Acc_est_mouse_real, 'r', label="Filtered Mouse Acceleration")
# axs[1, 3].set_ylabel("Mouse Acceleration (G)")
# axs[1, 3].legend(loc="lower center")

# # 速度误差
# denominator =  Vel_est * 12.5 * 2 * np.pi / 60 / 2.5
# deviation = np.abs(Vel_est_mouse - denominator) / np.where(denominator != 0, denominator, 1) * 100
# axs[2, 2].plot(t, deviation)
# axs[2, 2].set_title("Filtered Velocity Error")
# axs[2, 2].set_xlabel("Time (sec)")
# axs[2, 2].set_ylabel("Error (%)")

# # 加速度誤差
# denominator = Acc_est * 12.5 * 0.01 / 9.81
# deviation = np.abs(Acc_est_mouse_real - denominator) / np.where(denominator != 0, denominator, 1) * 100
# axs[2, 3].plot(t, deviation)
# axs[2, 3].set_title("Filtered Acceleration Error")
# axs[2, 3].set_xlabel("Time (sec)")
# axs[2, 3].set_ylabel("Error (%)")

# # plt.tight_layout()
# plt.show()