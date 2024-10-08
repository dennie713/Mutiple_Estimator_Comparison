import matplotlib.pyplot as plt
import numpy as np
import mousedata_add,  ImportData
import Cal, CFD, LAE, TSE, TTD, LSF, BDE, KF, zero_phase_filter
import AddNoice

# MAIN
if __name__ == "__main__":
    # Constant
    SamplingTime = 0.001
    CPI = 1600
    ## 讀取檔案
    path1 = ['D:\ASUS_program_code/規格測試/有線/IPS650_G50_motion_1.txt'] #馬達資料.txt路徑
    path2 = ['D:\ASUS_program_code/規格測試/有線/IPS650_G50_mouse_1.txt']  #滑鼠資料.txt路徑
    # path1 = ['D:\ASUS_program_code\控制台增強\WIRELESS\WIRELESS/1600/force\IPS500_G10_motion_1.txt'] #馬達資料.txt路徑
    # path2 = ['D:\ASUS_program_code\控制台增強\WIRELESS\WIRELESS/1600/force\IPS500_G10_mouse_1.txt']  #滑鼠資料.txt路徑
    # path1 = ['D:\ASUS_program_code\控制台增強\WIRELESS\WIRELESS/1600/force\IPS500_G10_motion_1.txt'] #馬達資料.txt路徑
    # path2 = ['D:\ASUS_program_code\控制台增強\WIRELESS\WIRELESS/1600/force\IPS500_G10_mouse_1.txt']  #滑鼠資料.txt路徑
    Motordata, Mousedata = ImportData.ImportData(path1, path2)
    mouseX, mouseY, Pos, PosCmd, Vel, VelCmd, TorCtrl, AccCmd, mousedata_data, mouse_displacement, mouse_real_Pos = Cal.Cal(Mousedata, Motordata, SamplingTime, CPI)    
    t = np.arange(0, (len(Motordata[:,0])) * SamplingTime, SamplingTime)

    ## 馬達命令
    Vel = [x*12.5/2.54 for x in Vel]
    VelCmd = [x*12.5/2.54 for x in VelCmd]

    ## 將資料加入雜訊
    Pos_added_noice, PosCmd_added_noice, mouse_real_Pos_added_noice, noice_record = AddNoice.AddNoice(Pos, PosCmd, mouse_real_Pos)
    # Pos = Pos_added_noice 
    # PosCmd = PosCmd_added_noice
    # mouse_real_Pos = mouse_real_Pos_added_noice 

    # 將資料濾波
    # filtered_Pos = zero_phase_filter.zero_phase_filter(3, 17, Pos)
    # filtered_PosCmd = zero_phase_filter.zero_phase_filter(3, 50, PosCmd)
    # filtered_mouse_real_Pos = zero_phase_filter.zero_phase_filter(3, 14, mouse_real_Pos)
    # Pos = filtered_Pos
    # PosCmd = filtered_PosCmd
    # mouse_real_Pos = filtered_mouse_real_Pos

    ## CFD 速度&加速度
    Pos_CFD_est, Vel_CFD_est, Acc_CFD_est = CFD.CFD(Pos) #濾波
    Pos_CFD_Cmd, Vel_CFD_Cmd, Acc_CFD_Cmd = CFD.CFD(PosCmd) #濾波
    Pos_CFD_est_mouse, Vel_CFD_est_mouse, Acc_CFD_est_mouse = CFD.CFD(mouse_real_Pos) #濾波

    ## TTD 加速度
    Acc_TTD_est, Vel_TTD_est = TTD.TTD(Pos, PosCmd) #濾波
    PosCmd_cvt = PosCmd*12.5/2.54 #rad轉換成inch
    Acc_TTD_est_mouse, Vel_TTD_est_mouse = TTD.TTD(mouse_real_Pos, PosCmd_cvt) #濾波#得到inch
    # Pos_cvt = Pos*12.5/2.54 #rad轉換成inch
    # acc_est_mouse, vel_est_mouse = TTD.TTD(mouse_real_Pos, Pos_cvt) #得到inch

    ## LAE 加速度
    Acc_LAE_est = LAE.LAE(Pos, SamplingTime)
    Acc_LAE_est_mouse = LAE.LAE(mouse_real_Pos, SamplingTime)

    ## LSF2/8 加速度
    Acc_LSF28_est = LSF.LSF28_Acc(Pos)
    Acc_LSF28_est_mouse = LSF.LSF28_Acc(mouse_real_Pos)

    ## LSF1/4 速度
    Vel_LSF14_est = LSF.LSF14(Pos)
    Vel_LSF14_est_mouse = LSF.LSF14(mouse_real_Pos)

    ## LSF2/8 速度
    Vel_LSF28_est = LSF.LSF28(Pos)
    Vel_LSF28_est_mouse = LSF.LSF28(mouse_real_Pos)

    ## LSF3/8 速度
    Vel_LSF38_est = LSF.LSF38(Pos)
    Vel_LSF38_est_mouse = LSF.LSF38(mouse_real_Pos)

    ## TSE2 速度
    Vel_TSE_est = TSE.TSE2(Pos)
    Vel_TSE_est_mouse = TSE.TSE2(mouse_real_Pos)

    ## TSE3 速度
    Vel_TSE3_est = TSE.TSE3(Pos)
    Vel_TSE3_est_mouse = TSE.TSE3(mouse_real_Pos)

    ## BDE3 速度
    Vel_BDE3_est = BDE.BDE3(Pos)
    Vel_BDE3_est_mouse = BDE.BDE3(mouse_real_Pos)

    ## KF 速度
    Pos_KF_est, Vel_KF_est = KF.KalmanFilter(0.001, Pos) #無濾波
    # Pos_KF_Cmd, Vel_KF_Cmd = KF.KalmanFilter(0.001, PosCmd) #無濾波
    Pos_KF_est_mouse, Vel_KF_est_mouse = KF.KalmanFilter(0.001, mouse_real_Pos) #無濾波

   ## CFD速度比较图
    plt.figure()
    Vel_CFD_est = [x*12.5/2.54 for x in Vel_CFD_est]
    Vel_CFD_est_mouse_real = Vel_CFD_est_mouse
    plt.plot(t, Vel_CFD_est, label="CFD Estimator Motor Velocity")
    plt.title("CFD Estimator Motor and Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_CFD_est),max(Vel_CFD_est_mouse))+50])
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Vel_CFD_est_mouse_real, 'r', label="CFD Estimator Mouse Velocity")
    plt.ylabel("Mouse Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_CFD_est),max(Vel_CFD_est_mouse))+50])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")

    ## LSF 1/4速度比較圖 
    plt.figure()
    Vel_LSF14_est = [x*12.5/2.54 for x in Vel_LSF14_est]
    Vel_LSF14_est_mouse_real = Vel_LSF14_est_mouse
    plt.plot(t, Vel_LSF14_est, label="LSF1/4 Estimator Motor Velocity")
    plt.title("LSF1/4 Estimator Motor and Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_LSF14_est),max(Vel_LSF14_est_mouse))+50])
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Vel_LSF14_est_mouse_real, 'r', label="LSF1/4 Estimator Mouse Velocity")
    plt.ylabel("Mouse Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_LSF14_est),max(Vel_LSF14_est_mouse_real))+50])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")

    ## LSF 2/8速度比較圖 
    plt.figure()
    Vel_LSF28_est = [x*12.5/2.54 for x in Vel_LSF28_est]
    Vel_LSF28_est_mouse_real = Vel_LSF28_est_mouse
    plt.plot(t, Vel_LSF28_est, label="LSF2/8 Estimator Motor Velocity")
    plt.title("LSF2/8 Estimator Motor and Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_LSF28_est),max(Vel_LSF28_est_mouse))+50])
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Vel_LSF28_est_mouse_real, 'r', label="LSF2/8 Estimator Mouse Velocity")
    plt.ylabel("Mouse Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_LSF28_est),max(Vel_LSF28_est_mouse_real))+50])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")

    ## LSF 3/8速度比較圖 
    plt.figure()
    Vel_LSF38_est = [x*12.5/2.54 for x in Vel_LSF38_est]
    Vel_LSF38_est_mouse_real = Vel_LSF38_est_mouse
    plt.plot(t, Vel_LSF38_est, label="LSF3/8 Estimator Motor Velocity")
    plt.title("LSF3/8 Estimator Motor and Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_LSF38_est),max(Vel_LSF38_est_mouse))+50])
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Vel_LSF38_est_mouse_real, 'r', label="LSF3/8 Estimator Mouse Velocity")
    plt.ylabel("Mouse Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_LSF38_est),max(Vel_LSF38_est_mouse_real))+50])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")

    ## TSE2 速度比較圖
    plt.figure()
    Vel_TSE_est = [x*12.5/2.54 for x in Vel_TSE_est]
    Vel_TSE_est_mouse_real = Vel_TSE_est_mouse
    plt.plot(t, Vel_TSE_est, label="TSE2nd Estimator Motor Velocity")
    plt.title("TSE2nd Estimator Motor and Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_TSE_est),max(Vel_TSE_est_mouse))+50])
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Vel_TSE_est_mouse_real, 'r', label="TSE2nd Estimator Mouse Velocity")
    plt.ylabel("Mouse Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_TSE_est),max(Vel_TSE_est_mouse_real))+50])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")

    ## TSE3 速度比較圖
    plt.figure()
    Vel_TSE3_est = [x*12.5/2.54 for x in Vel_TSE3_est]
    Vel_TSE3_est_mouse_real = Vel_TSE3_est_mouse
    plt.plot(t, Vel_TSE3_est, label="TSE3rd Estimator Motor Velocity")
    plt.title("TSE3rd Estimator Motor and Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_TSE3_est),max(Vel_TSE3_est_mouse))+50])
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Vel_TSE3_est_mouse_real, 'r', label="TSE3rd Estimator Mouse Velocity")
    plt.ylabel("Mouse Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_TSE3_est),max(Vel_TSE3_est_mouse_real))+50])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")

    ## BDE3 速度比較圖
    plt.figure()
    Vel_BDE3_est = [x*12.5/2.54 for x in Vel_BDE3_est]
    Vel_BDE3_est_mouse_real = Vel_BDE3_est_mouse
    plt.plot(t, Vel_BDE3_est, label="BDE3rd Estimator Motor Velocity")
    plt.title("BDE3rd Estimator Motor and Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_BDE3_est),max(Vel_BDE3_est_mouse))+50])
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Vel_BDE3_est_mouse_real, 'r', label="BDE3rd Estimator Mouse Velocity")
    plt.ylabel("Mouse Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_BDE3_est),max(Vel_BDE3_est_mouse_real))+50])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")

    ## KF 速度比較圖
    plt.figure()
    Vel_KF_est = [x*12.5/2.54 for x in Vel_KF_est]
    Vel_KF_est_mouse_real = Vel_KF_est_mouse
    plt.plot(t, Vel_KF_est, label="KF Estimator Motor Velocity")
    plt.title("KF Estimator Motor and Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_KF_est),max(Vel_KF_est_mouse))+50])
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Vel_KF_est_mouse_real, 'r', label="KF Estimator Mouse Velocity")
    plt.ylabel("Mouse Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_KF_est),max(Vel_KF_est_mouse))+50])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")

    ## 所有方法速度比較
    # 馬達比較
    plt.figure()
    plt.plot(t, Vel_BDE3_est, label="BDE3rd Estimator Motor Velocity")
    plt.plot(t, Vel_TSE3_est, label="TSE3rd Estimator Motor Velocity")
    plt.plot(t, Vel_LSF38_est, label="LSF3/8 Estimator Motor Velocity")
    plt.plot(t, Vel_KF_est, label="KF Estimator Motor Velocity")
    plt.plot(t, Vel_LSF28_est, label="LSF2/8 Estimator Motor Velocity")
    plt.plot(t, Vel_CFD_est, label="CFD Estimator Motor Velocity")
    plt.plot(t, Vel_LSF14_est, label="LSF1/4 Estimator Motor Velocity")
    plt.plot(t, Vel, label="Motor Velocity ")
    plt.title("Estimators Motor Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_CFD_est),max(Vel_LSF14_est),max(Vel_LSF14_est),max(Vel_LSF28_est),max(Vel_LSF38_est),max(Vel_TSE3_est),max(Vel_BDE3_est),max(Vel_KF_est))+50])
    plt.legend(loc="lower center")
    # 滑鼠比較
    plt.figure()
    plt.plot(t, Vel_BDE3_est_mouse_real, label="BDE3rd Estimator Motor Velocity")
    plt.plot(t, Vel_TSE3_est_mouse_real, label="TSE3rd Estimator Motor Velocity")
    plt.plot(t, Vel_LSF38_est_mouse_real, label="LSF3/8 Estimator Motor Velocity")
    plt.plot(t, Vel_KF_est_mouse_real, label="KF Estimator Motor Velocity")
    plt.plot(t, Vel_LSF28_est_mouse_real, label="LSF2/8 Estimator Motor Velocity")
    plt.plot(t, Vel_CFD_est_mouse_real, label="CFD Estimator Motor Velocity")
    plt.plot(t, Vel_LSF14_est_mouse_real, label="LSF1/4 Estimator Motor Velocity")
    plt.title("Estimators Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_CFD_est_mouse_real),max(Vel_LSF14_est_mouse_real),max(Vel_LSF28_est_mouse_real),max(Vel_LSF38_est_mouse_real),max(Vel_TSE3_est_mouse_real),max(Vel_BDE3_est_mouse_real),max(Vel_KF_est_mouse_real))+50])
    plt.legend(loc="lower center")

    # CFD加速度比较图
    plt.figure()
    Acc_CFD_est = [x*0.125/9.81 for x in Acc_CFD_est]
    Acc_CFD_est_mouse = [x*0.0254/ 9.81 for x in Acc_CFD_est_mouse]
    Acc_CFD_est_mouse_real = Acc_CFD_est_mouse 
    plt.plot(t, Acc_CFD_est, label="CFD Estimator Motor Acceleration")
    plt.title("CFD Estimator Motor and Mouse Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Acceleration (G)")
    plt.axis([0, t[-1], min(min(Acc_CFD_est),min(Acc_CFD_est_mouse_real))-5, max(max(Acc_CFD_est),max(Acc_CFD_est_mouse_real))+5])
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Acc_CFD_est_mouse_real, 'r', label="CFD Estimator Motor Acceleration")
    plt.ylabel("Mouse Acceleration (G)")
    plt.axis([0, t[-1], min(min(Acc_CFD_est),min(Acc_CFD_est_mouse_real))-5, max(max(Acc_CFD_est),max(Acc_CFD_est_mouse_real))+5])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")

    # TTD加速度比较图
    plt.figure()
    Acc_TTD_est = [x * 0.125/9.81 for x in Acc_TTD_est]  # 假设您想要获取第一个元素
    Acc_TTD_est_mouse = [y * 0.0254 / 9.81 for y in Acc_TTD_est_mouse] #得到G
    Acc_TTD_est_mouse_real = Acc_TTD_est_mouse
    plt.plot(t, Acc_TTD_est, label="TTD Estimator Motor Acceleration")
    plt.title("TTD Estimator Motor and Mouse Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Acceleration (G)")
    plt.axis([0, t[-1], min(min(Acc_TTD_est),min(Acc_TTD_est_mouse_real))-5, max(max(Acc_TTD_est),max(Acc_TTD_est_mouse_real))+5])
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Acc_TTD_est_mouse_real , 'r', label="TTD Estimator Mouse Acceleration")
    plt.ylabel("Mouse Acceleration (G)")
    plt.axis([0, t[-1], min(min(Acc_TTD_est),min(Acc_TTD_est_mouse_real))-5, max(max(Acc_TTD_est),max(Acc_TTD_est_mouse_real))+5])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")

    # LAE ACC
    plt.figure()
    Acc_LAE_est = [x * 0.125/9.81 for x in  Acc_LAE_est]  # 假设您想要获取第一个元素
    Acc_LAE_est_mouse = [y * 0.0254 / 9.81 for y in Acc_LAE_est_mouse]
    Acc_LAE_est_mouse_real = Acc_LAE_est_mouse
    plt.plot(t, Acc_LAE_est, label="LAE Estimator Motor Acceleration")
    plt.title("LAE Estimator Motor and Mouse Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Acceleration (G)")
    plt.axis([0, t[-1], min(Acc_LAE_est)-5, max(Acc_LAE_est)+5])
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Acc_LAE_est_mouse_real , 'r', label="LAE Estimator Mouse Acceleration")
    plt.ylabel("Mouse Acceleration (G)")
    plt.axis([0, t[-1], min(min(Acc_LAE_est),min(Acc_LAE_est_mouse_real))-5, max(max(Acc_LAE_est),max(Acc_LAE_est_mouse_real))+5])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")

    # LSF2/8 ACC
    plt.figure()
    Acc_LSF28_est = [x * 0.125/9.81 for x in  Acc_LSF28_est]  # 假设您想要获取第一个元素
    Acc_LSF28_est_mouse = [y * 0.0254 / 9.81 for y in Acc_LSF28_est_mouse]
    Acc_LSF28_est_mouse_real = Acc_LSF28_est_mouse
    plt.plot(t, Acc_LSF28_est, label="LSF2/8 Estimator Motor Acceleration")
    plt.title("LSF2/8 Estimator Motor and Mouse Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Acceleration (G)")
    plt.axis([0, t[-1], -max(Acc_LSF28_est)-5, max(Acc_LSF28_est)+5])
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Acc_LSF28_est_mouse_real , 'r', label="LSF2/8 Estimator Mouse Acceleration")
    plt.ylabel("Mouse Acceleration (G)")
    plt.axis([0, t[-1], min(min(Acc_LSF28_est),min(Acc_LSF28_est_mouse_real))-5, max(max(Acc_LSF28_est),max(Acc_LSF28_est_mouse_real))+5])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")

    ## 所有方法加速度比較
    # 馬達比較
    plt.figure()
    plt.plot(t, Acc_CFD_est, label="CFD Estimator Motor Acceleration")
    plt.plot(t, Acc_TTD_est, label="TTD Estimator Motor Acceleration")
    plt.plot(t, Acc_LAE_est, label="LAE Estimator Motor Acceleration")
    plt.plot(t, Acc_LSF28_est, label="LSF28 Estimator Motor Acceleration")
    plt.title("Estimators Motor Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Acceleration (G)")
    plt.axis([0, t[-1], min(min(Acc_CFD_est),min(Acc_TTD_est),min(Acc_LAE_est),min(Acc_LSF28_est))-5, max(max(Acc_CFD_est),max(Acc_TTD_est),max(Acc_LAE_est),max(Acc_LSF28_est))+5])
    plt.legend()
    # 滑鼠比較
    plt.figure()
    plt.plot(t, Acc_CFD_est_mouse_real, label="CFD Estimator Motor Acceleration")
    plt.plot(t, Acc_TTD_est_mouse_real, label="TTD Estimator Motor Acceleration")
    plt.plot(t, Acc_LAE_est_mouse_real, label="LAE Estimator Motor Acceleration")
    plt.plot(t, Acc_LSF28_est_mouse_real, label="LSF28 Estimator Motor Acceleration")
    plt.title("Estimators Motor Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Acceleration (G)")
    plt.axis([0, t[-1], min(min(Acc_CFD_est_mouse_real),min(Acc_TTD_est_mouse_real),min(Acc_LAE_est_mouse_real),min(Acc_LSF28_est_mouse_real))-5, max(max(Acc_CFD_est_mouse_real),max(Acc_TTD_est_mouse_real),max(Acc_LAE_est_mouse_real),max(Acc_LSF28_est_mouse_real))+5])
    plt.legend()

    plt.show()