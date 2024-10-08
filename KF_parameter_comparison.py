import matplotlib.pyplot as plt
import numpy as np
import mousedata_add,  ImportData
import Cal, CFD, LAE, TSE, TTD, LSF, BDE, KF, zero_phase_filter
import AddNoice, PlotFig, KF_modify

SamplingTime = 0.001
CPI = 1600
R = 12.5 # 半徑
# 讀取檔案
path1 = ['D:\ASUS_program_code/規格測試/有線/IPS650_G50_motion_1.txt'] #馬達資料.txt路徑
path2 = ['D:\ASUS_program_code/規格測試/有線/IPS650_G50_mouse_1.txt']  #滑鼠資料.txt路徑
# path1 = ['D:\ASUS_program_code\低速命令/IPS10_G1_motion.txt'] #馬達資料.txt路徑
# path2 = ['D:\ASUS_program_code\低速命令/IPS10_G1_mouse.txt']  #滑鼠資料.txt路徑
Motordata, Mousedata = ImportData.ImportData(path1, path2)
mouseX, mouseY, Pos, PosCmd, Vel, VelCmd, TorCtrl, AccCmd, mousedata_data, mouse_displacement, mouse_real_Pos = Cal.Cal(Mousedata, Motordata, SamplingTime, CPI)    
t = np.arange(0, (len(Motordata[:,0])) * SamplingTime, SamplingTime)

## 資料中加入雜訊
Pos_added_noice, PosCmd_added_noice, mouse_real_Pos_added_noice, noice_record = AddNoice.AddNoice(Pos, PosCmd, mouse_real_Pos)
Pos = Pos_added_noice 
PosCmd = PosCmd # 位置命令未加雜訊
PosCmd_added_noice = PosCmd_added_noice # 位置命令加雜訊
mouse_real_Pos = mouse_real_Pos_added_noice 

## 未加入雜訊
# PosCmd = PosCmd
# PosCmd_added_noice = PosCmd

## 馬達命令
Vel = [x*R/2.54 for x in Vel] # rad to inch
VelCmd = [x*R/2.54 for x in VelCmd] # rad to inch
AccCmd = [x*0.01*R/9.81 for x in AccCmd] # rad to m to G

acce_max = []
acce_min = []
vele_max = []
vele_min = []

# kf vel 調整 a 比較
a = [ 6, 5, 4, 3, 2, 1, 0, -1, -2, -3]
b = 2000
plt.figure()
for i in range(len(a)):
    pos, vel, acc = KF_modify.mod_KF(SamplingTime, PosCmd_added_noice, a[i], b)
    acc = [x * 0.01 * R / 9.81 for x in acc] # rad to G
    acce_max.append(np.max(acc))
    acce_min.append(np.min(acc))
    label  = f"a = {a[i]}"
    plt.plot(t, acc, label = label)
plt.plot(t, AccCmd, label = 'AccCmd')
plt.title("a of 10^a comparison")
plt.xlabel("Time (sec)")
plt.ylabel("Acc (G)")
plt.axis([0, t[-1], min(min(AccCmd),min(acce_max))-25, max(max(AccCmd),max(acce_min))+25])
plt.legend(loc='lower center', ncol=3)

plt.figure()
for i in range(len(a)):
    pos, vel, acc = KF_modify.mod_KF(SamplingTime, PosCmd_added_noice, a[i], b)
    vel = [x * R / 2.54 for x in vel]
    vele_max.append(np.max(vel))
    vele_min.append(np.min(vel))
    label  = f"a = {a[i]}"
    plt.plot(t, vel, label = label)
plt.plot(t, VelCmd, label = 'AccCmd')
plt.title("a of 10^a comparison")
plt.xlabel("Time (sec)")
plt.ylabel("Vel (IPS)")
plt.axis([0, t[-1], 0, max(max(VelCmd),max(vele_max))+25])
plt.legend(loc='lower center', ncol=3)

# kf vel 調整 R 比較
a = 5
b = [0.001, 0.01, 0.1, 1, 10, 100, 1000]
# b = [ 1000, 1500, 2000, 3000, 5000, 10000, 15000, 20000]
plt.figure()
for i in range(len(b)):
    pos, vel, acc = KF_modify.mod_KF(SamplingTime, PosCmd_added_noice, a, b[i])
    acc = [x * 0.01 * R / 9.81 for x in acc] # rad to G
    acce_max.append(np.max(acc))
    acce_min.append(np.min(acc))
    label  = f"b = {b[i]}"
    plt.plot(t, acc, label = label)
plt.plot(t, AccCmd, label = 'AccCmd')
plt.title("R of matrix comparison")
plt.xlabel("Time (sec)")
plt.ylabel("Acc (G)")
# plt.axis([0, t[-1], min(min(AccCmd),min(acce_max))-25, max(max(AccCmd),max(acce_min))+25])
plt.legend(loc='lower center', ncol=3)

plt.figure()
for i in range(len(b)):
    pos, vel, acc = KF_modify.mod_KF(SamplingTime, PosCmd_added_noice, a, b[i])
    vel = [x * R / 2.54 for x in vel]
    vele_max.append(np.max(vel))
    vele_min.append(np.min(vel))
    label  = f"b = {b[i]}"
    plt.plot(t, vel, label = label)
plt.plot(t, VelCmd, label = 'AccCmd')
plt.title("R of matrix comparison")
plt.xlabel("Time (sec)")
plt.ylabel("Vel (IPS)")
plt.axis([0, t[-1], 0, max(max(VelCmd),max(vele_max))+25])
plt.legend(loc='lower center', ncol=3)
plt.show()