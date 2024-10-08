import matplotlib.pyplot as plt
import numpy as np
import ImportData
import Cal, CFD, LAE, TSE, TTD, LSF, BDE
import AddNoice, zero_phase_filter, PlotFig, KF_modify, Err

# MAIN
if __name__ == "__main__":
    # Constant
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
    # Pos_added_noice, PosCmd_added_noice, mouse_real_Pos_added_noice, noice_record = AddNoice.AddNoice(Pos, PosCmd, mouse_real_Pos)
    # Pos = Pos_added_noice 
    # PosCmd = PosCmd_added_noice
    # mouse_real_Pos = mouse_real_Pos_added_noice 

    # 將資料濾波
    # filtered_Pos = zero_phase_filter.zero_phase_filter(3, 18, Pos)
    # filtered_PosCmd = zero_phase_filter.zero_phase_filter(3, 50, PosCmd)
    # filtered_mouse_real_Pos = zero_phase_filter.zero_phase_filter(3, 16, mouse_real_Pos) #17
    # Pos = filtered_Pos
    # PosCmd = filtered_PosCmd
    # mouse_real_Pos = filtered_mouse_real_Pos
    

    ## 馬達命令
    Vel = [x*R/2.54 for x in Vel]
    VelCmd = [x*R/2.54 for x in VelCmd]

    ## CFD 速度&加速度
    Pos_CFD_est, Vel_CFD_est, Acc_CFD_est = CFD.CFD(Pos) #濾波
    Pos_CFD_Cmd, Vel_CFD_Cmd, Acc_CFD_Cmd = CFD.CFD(PosCmd) #濾波
    Pos_CFD_est_mouse, Vel_CFD_est_mouse, Acc_CFD_est_mouse = CFD.CFD(mouse_real_Pos) #濾波

    ## TTD 加速度
    Acc_TTD_est, Vel_TTD_est = TTD.TTD(Pos, PosCmd) #濾波
    PosCmd_cvt = PosCmd*R/2.54 # rad轉換成inch
    # mouse_real_Pos = [x *2.54/12.5 for x in mouse_real_Pos] #inch轉rad
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
    Pos_KF_est, Vel_KF_est, Acc_KF_est = KF_modify.mod_KalmanFilter(0.001, Pos, PosCmd, VelCmd, AccCmd)
    # Pos_KF_est, Vel_KF_est, Acc_KF_est = KF_modify.mod_KalmanFilter(0.001, Pos, AccCmd)
    Pos_KF_est_mouse, Vel_KF_est_mouse, Acc_KF_est_mouse = KF_modify.mod_KalmanFilter(0.001, mouse_real_Pos, PosCmd, VelCmd, AccCmd)
    # Pos_KF_est_mouse, Vel_KF_est_mouse, Acc_KF_est_mouse = KF_modify.ModKF_mouse(0.001, mouse_real_Pos)
    # with open('Vel_KF_est.txt', 'w') as file:
    #     for data in Vel_KF_est:
    #         file.write(f'{data:.8f}\n')

    ## CFD速度比较图
    PlotFig.VelPlotFig(Vel_CFD_est, Vel_CFD_est_mouse, t, R, 'CFD')
    # Err. VelErr_Cmd_1(Vel_CFD_est, VelCmd, R) # 取中間段
    # Err. VelErr_Cmd_2(Vel_CFD_est_mouse, VelCmd, R) # 取中間段

    ## LSF 1/4速度比較圖 
    PlotFig.VelPlotFig(Vel_LSF14_est, Vel_LSF14_est_mouse, t, R, 'LSF14')
    # Err. VelErr_Cmd_1(Vel_LSF14_est, VelCmd, R) # 取中間段
    # Err. VelErr_Cmd_2(Vel_LSF14_est_mouse, VelCmd, R) # 取中間段

    ## LSF 2/8速度比較圖 
    PlotFig.VelPlotFig(Vel_LSF28_est, Vel_LSF28_est_mouse, t, R, 'LSF28')
    # Err. VelErr_Cmd_1(Vel_LSF28_est, VelCmd, R) # 取中間段
    # Err. VelErr_Cmd_2(Vel_LSF28_est_mouse, VelCmd, R) # 取中間段

    ## LSF 3/8速度比較圖 
    PlotFig.VelPlotFig(Vel_LSF38_est, Vel_LSF38_est_mouse, t, R, 'LSF38')
    # Err. VelErr_Cmd_1(Vel_LSF38_est, VelCmd, R) # 取中間段
    # Err. VelErr_Cmd_2(Vel_LSF38_est_mouse, VelCmd, R) # 取中間段

    ## TSE2 速度比較圖
    PlotFig.VelPlotFig(Vel_TSE_est, Vel_TSE_est_mouse, t, R, 'TSE2')
    # Err. VelErr_Cmd_1(Vel_TSE_est, VelCmd, R) # 取中間段
    # Err. VelErr_Cmd_2(Vel_TSE_est_mouse, VelCmd, R) # 取中間段

    ## TSE3 速度比較圖
    PlotFig.VelPlotFig(Vel_TSE3_est, Vel_TSE3_est_mouse, t, R, 'TSE3')
    # Err. VelErr_Cmd_1(Vel_TSE3_est, VelCmd, R) # 取中間段
    # Err. VelErr_Cmd_2(Vel_TSE3_est_mouse, VelCmd, R) # 取中間段

    ## BDE3 速度比較圖
    PlotFig.VelPlotFig(Vel_BDE3_est, Vel_BDE3_est_mouse, t, R,  'BDE3')
    # Err. VelErr_Cmd_1(Vel_BDE3_est, VelCmd, R) # 取中間段
    # Err. VelErr_Cmd_2(Vel_BDE3_est_mouse, VelCmd, R) # 取中間段

    ## KF 速度比較圖
    PlotFig.VelPlotFig(Vel_KF_est, Vel_KF_est_mouse, t, R, 'KF')
    # PlotFig.VelPlotFig(Vel_KF_est, VelCmd, t, R, 'KF')
    # Err. VelErr_Cmd_1(Vel_KF_est, VelCmd, R) # 取中間段
    # Err. VelErr_Cmd_2(Vel_KF_est_mouse, VelCmd, R) # 取中間段
    # PlotFig.VelPlotFig(Vel_TTD_est, Vel_TTD_est_mouse, t, 'TTD')

    ## 所有方法速度比較
    # 馬達比較
    VelCmd = [x/R*2.54 for x in VelCmd] # inch to rad
    DataName = [Vel_BDE3_est, Vel_TSE3_est, Vel_LSF38_est, Vel_LSF28_est, Vel_CFD_est, Vel_LSF14_est, Vel_KF_est, VelCmd]
    labels = ['BDE3', 'TSE3', 'LSF38', 'LSF28', 'CFD', 'LSF14', 'KF', 'VelCmd']
    for i in range(len(DataName)):
        DataName[i] = [x * R / 2.54 for x in DataName[i]]
    PlotFig.VelComp(DataName, t, labels)
    Err.ErrComp(DataName, t, labels)
    # Err.VelErr_Cmd_multi(DataName, labels, R)
    with open('Vel_KF_est.txt', 'w') as file:
        for data in Vel_KF_est:
            file.write(f'{data:.8f}\n')
   
    ## 滑鼠比較
    VelCmd = [x*R/2.54 for x in VelCmd] # rad to inch
    DataName = [Vel_BDE3_est_mouse, Vel_TSE3_est_mouse, Vel_LSF38_est_mouse, Vel_LSF28_est_mouse, Vel_CFD_est_mouse, Vel_LSF14_est_mouse, Vel_KF_est_mouse, VelCmd]
    labels = ['BDE3', 'TSE3', 'LSF38', 'LSF28', 'CFD', 'LSF14', 'KF', 'VelCmd']
    PlotFig.VelComp1(DataName, t, labels)
    # DataName = [Vel_BDE3_est_mouse, Vel_TSE3_est_mouse, Vel_LSF38_est_mouse, Vel_LSF28_est_mouse, Vel_CFD_est_mouse, Vel_LSF14_est_mouse, Vel_KF_est_mouse, VelCmd]
    # labels = ['BDE3', 'TSE3', 'LSF38', 'LSF28', 'CFD', 'LSF14', 'KF', 'VelCmd']
    Err.ErrComp(DataName, t, labels)
    # Err.VelErr_Cmd_multi(DataName, labels, R)

    # CFD加速度比较图
    PlotFig.AccPlotFig(Acc_CFD_est, Acc_CFD_est_mouse, t, R, 'CFD')

    # TTD加速度比较图
    PlotFig.AccPlotFig(Acc_TTD_est, Acc_TTD_est_mouse, t, R, 'TTD')
    # AccCmd = [x*12.5*0.01/9.81 for x in AccCmd]
    # PlotFig.AccPlotFig_Cmd(Acc_TTD_est_mouse, AccCmd, t, R, 'TTD')

    # LSF2/8 ACC
    PlotFig.AccPlotFig(Acc_LSF28_est, Acc_LSF28_est_mouse, t, R, 'LSF28')

    # LAE ACC
    PlotFig.AccPlotFig(Acc_LAE_est, Acc_LAE_est_mouse, t, R, 'LAE')
    # Err.AccErr_Cmd(Acc_LAE_est, AccCmd, t ,R)
    # Err.AccErr_Cmd(Acc_LAE_est_mouse, AccCmd, t ,R)

    # KF ACC
    PlotFig.AccPlotFig(Acc_KF_est, Acc_KF_est_mouse, t, R, 'KF')
    # Err.AccErr_Cmd(Acc_KF_est, AccCmd, t ,R)
    # Err.AccErr_Cmd(Acc_KF_est_mouse, AccCmd, t ,R)

    # LAE vs KF mouse 
    AccCmd = [x*12.5/2.54 for x in AccCmd] # rad to inch
    DataName = [Acc_LAE_est_mouse, Acc_KF_est_mouse, AccCmd]
    labels = ['LAE', 'KF', 'AccCmd']
    for i in range(len(DataName)):
        DataName[i] = [x * 0.0254/ 9.81 for x in DataName[i]] # INCH to G
    PlotFig.AccComp(DataName, t, labels)  
    # shift&error
    Acc_LAE_est_mouse = [x * 0.0254/ 9.81 for x in Acc_LAE_est_mouse]
    Acc_KF_est_mouse = [x * 0.0254/ 9.81 for x in Acc_KF_est_mouse]
    AccCmd = [x * 0.0254/ 9.81 for x in AccCmd]
    t = np.arange(0, (len(Motordata[:,0])-46) * SamplingTime, SamplingTime)
    plt.figure()
    plt.plot(t, Acc_LAE_est_mouse[46:],label = 'LAE')
    plt.plot(t, Acc_KF_est_mouse[46:],label ='KF')
    plt.plot(t, AccCmd[:1902],label ='AccCmd')
    plt.title("Estimators Mouse Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Acceleration (G)")
    plt.axis([0, t[-1]-0.046, min(min(Acc_LAE_est_mouse),min(Acc_KF_est_mouse),min(AccCmd))-5, max(max(Acc_LAE_est_mouse),max(Acc_KF_est_mouse),max(AccCmd))+5])
    plt.legend() 
    t = np.arange(0, (len(Motordata[:,0])) * SamplingTime, SamplingTime)

    # print('-------- MOUSE RMS err --------')
    # Err.RMS_err(Acc_CFD_est_mouse, AccCmd, 'Acc_CFD_est_mouse')
    # Err.RMS_err(Acc_LSF28_est_mouse, AccCmd, 'Acc_LSF28_est_mouse')
    # Err.RMS_err(Acc_TTD_est_mouse, AccCmd, 'Acc_TTD_est_mouse')
    # Err.RMS_err(Acc_LAE_est_mouse, AccCmd, 'Acc_LAE_est_mouse')
    # Err.RMS_err(Acc_KF_est_mouse, AccCmd, 'Acc_KF_est_mouse')
    ##
    #
    AccCmd = [x / 0.0254* 9.81 for x in AccCmd]
    #

    # LAE vs KF motor
    AccCmd = [x*0.01*2.54/ 9.81 for x in AccCmd] # INCH to G
    DataName = [Acc_LAE_est, Acc_KF_est, AccCmd]
    labels = ['LAE', 'KF', 'AccCmd']
    for i in range(len(DataName)-1):
        DataName[i] = [x * 0.01*R/ 9.81 for x in DataName[i]] #RAD to G
    PlotFig.AccComp(DataName, t, labels)
   
    # shift&error
    Acc_LAE_est = [x * 0.01*R/ 9.81 for x in Acc_LAE_est]
    Acc_KF_est = [x * 0.01*R/ 9.81 for x in Acc_KF_est]
    # AccCmd = [x * 0.01*2.54/ 9.81 for x in AccCmd]
    t = np.arange(0, (len(Motordata[:,0])-22) * SamplingTime, SamplingTime)
    plt.figure()
    plt.plot(t, Acc_LAE_est[22:],label = 'LAE')
    plt.plot(t, Acc_KF_est[22:],label ='KF')
    plt.plot(t, AccCmd[:1926],label ='AccCmd')
    plt.title("Estimators Motor Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Acceleration (G)")
    plt.axis([0, t[-1]-0.022, min(min(Acc_LAE_est_mouse),min(Acc_KF_est_mouse),min(AccCmd))-5, max(max(Acc_LAE_est_mouse),max(Acc_KF_est_mouse),max(AccCmd))+5])
    plt.legend() 
    t = np.arange(0, (len(Motordata[:,0])) * SamplingTime, SamplingTime)

    # print('-------- MOTOR RMS err --------')
    # Err.RMS_err(Acc_CFD_est, AccCmd, 'Acc_CFD_est')
    # Err.RMS_err(Acc_LSF28_est, AccCmd, 'Acc_LSF28_est')
    # Err.RMS_err(Acc_TTD_est, AccCmd, 'Acc_TTD_est')
    # Err.RMS_err(Acc_LAE_est, AccCmd, 'Acc_LAE_est')
    # Err.RMS_err(Acc_KF_est, AccCmd, 'Acc_KF_est')
    ##
    #
    AccCmd = [x*9.81*100/R for x in AccCmd] #G to rad
    #

    ## 所有方法加速度比較
    # 馬達比較
    DataName = [Acc_CFD_est, Acc_LSF28_est, Acc_TTD_est, Acc_LAE_est, Acc_KF_est, AccCmd]
    labels = ['CFD', 'LSF28', 'TTD', 'LAE', 'KF', 'AccCmd']
    # DataName = [Acc_CFD_est, Acc_KF_est, AccCmd]
    # labels = ['CFD', 'KF', 'AccCmd']
    for i in range(len(DataName)):
        DataName[i] = [x * 0.01*R/9.81 for x in DataName[i]] # rad to m 
    PlotFig.AccComp(DataName, t, labels)   
    Err.Acc_ErrComp(DataName, t ,labels) 
    
    ## 滑鼠比較
    # AccCmd = [x*12.5/2.54 for x in AccCmd] # rad to inch
    DataName = [Acc_CFD_est_mouse, Acc_LSF28_est_mouse, Acc_TTD_est_mouse, Acc_LAE_est_mouse, Acc_KF_est_mouse, AccCmd]
    labels = ['CFD', 'LSF28', 'TTD', 'LAE', 'KF', 'AccCmd']
    for i in range(len(DataName)):
        DataName[i] = [x * 0.0254/ 9.81 for x in DataName[i]] # inch to m 
    PlotFig.AccComp1(DataName, t, labels) 
    Err.Acc_ErrComp(DataName, t ,labels) 
 
    plt.show()