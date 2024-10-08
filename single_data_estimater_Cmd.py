import matplotlib.pyplot as plt
import numpy as np
import mousedata_add,  ImportData
import Cal, CFD, LAE, TSE, TTD, LSF, BDE, KF_modify, zero_phase_filter
import AddNoice, PlotFig, Err

# MAIN
if __name__ == "__main__":
    # Constant
    SamplingTime = 0.001
    CPI = 1600
    R = 12.5 # 半徑 
    ## 讀取檔案
    path1 = ['D:\ASUS_program_code/規格測試/有線/IPS650_G50_motion_1.txt'] #馬達資料.txt路徑
    path2 = ['D:\ASUS_program_code/規格測試/有線/IPS650_G50_mouse_1.txt']  #滑鼠資料.txt路徑
    # path1 = ['D:\ASUS_program_code\低速命令/IPS10_G1_motion.txt'] #馬達資料.txt路徑
    # path2 = ['D:\ASUS_program_code\低速命令/IPS10_G1_mouse.txt']  #滑鼠資料.txt路徑
    Motordata, Mousedata = ImportData.ImportData(path1, path2)
    mouseX, mouseY, Pos, PosCmd, Vel, VelCmd, TorCtrl, AccCmd, mousedata_data, mouse_displacement, mouse_real_Pos = Cal.Cal(Mousedata, Motordata, SamplingTime, CPI)    
    t = np.arange(0, (len(Motordata[:,0])) * SamplingTime, SamplingTime)
    # t=t/10

    ## 資料中加入雜訊
    Pos_added_noice, PosCmd_added_noice, mouse_real_Pos_added_noice, noice_record = AddNoice.AddNoice(Pos, PosCmd, mouse_real_Pos)
    Pos = Pos_added_noice 
    PosCmd = PosCmd # 位置命令未加雜訊
    PosCmd_added_noice = PosCmd_added_noice # 位置命令加雜訊
    mouse_real_Pos = mouse_real_Pos_added_noice 

    ## 未加入雜訊
    # PosCmd = PosCmd
    # PosCmd_added_noice = PosCmd
    
    # ## KF 速度&加速度
    Pos_KF_est, Vel_KF_est, Acc_KF_est = KF_modify.mod_KalmanFilter(0.001, PosCmd_added_noice, PosCmd, VelCmd, AccCmd)

    ## 馬達命令
    Vel = [x * R / 2.54 for x in Vel] # rad to inch
    VelCmd = [x * R / 2.54 for x in VelCmd] # rad to inch
    AccCmd = [x * 0.01 * R / 9.81 for x in AccCmd] # rad to m to G

    ## CFD 速度&加速度
    Pos_CFD_est, Vel_CFD_est, Acc_CFD_est = CFD.CFD(PosCmd_added_noice) #濾波
    # Pos_CFD_Cmd, Vel_CFD_Cmd, Acc_CFD_Cmd = CFD.CFD(PosCmd) #濾波
    # Pos_CFD_Cmd, Vel_CFD_Cmd, Acc_CFD_Cmd = CFD.CFD(PosCmd_added_noice) #濾波
    # Pos_CFD_est_mouse, Vel_CFD_est_mouse, Acc_CFD_est_mouse = CFD.CFD(mouse_real_Pos) #濾波

    ## TTD 加速度
    # Acc_TTD_est, Vel_TTD_est = TTD.TTD(Pos, PosCmd) #濾波 _added_noice
    Acc_TTD_est, Vel_TTD_est = TTD.TTD(PosCmd_added_noice, PosCmd)
    # PosCmd_cvt = PosCmd*12.5/2.54 #rad轉換成inch
    # Acc_TTD_est_mouse, Vel_TTD_est_mouse = TTD.TTD(mouse_real_Pos, PosCmd_cvt) #濾波#得到inch
    # Pos_cvt = Pos*12.5/2.54 #rad轉換成inch
    # acc_est_mouse, vel_est_mouse = TTD.TTD(mouse_real_Pos, Pos_cvt) #得到inch

    ## LAE 加速度
    Acc_LAE_est = LAE.LAE(PosCmd_added_noice, SamplingTime)
    # Acc_LAE_est_mouse = LAE.LAE(mouse_real_Pos, SamplingTime)

    ## LSF2/8 加速度
    Acc_LSF28_est = LSF.LSF28_Acc(PosCmd_added_noice)
    # Acc_LSF28_est_mouse = LSF.LSF28_Acc(mouse_real_Pos)

    # ## KF 速度&加速度
    # Pos_KF_est, Vel_KF_est, Acc_KF_est = KF_modify.mod_KalmanFilter(0.001, PosCmd_added_noice, PosCmd, VelCmd, AccCmd)

    ## LSF1/4 速度
    Vel_LSF14_est = LSF.LSF14(PosCmd_added_noice)

    ## LSF2/8 速度
    Vel_LSF28_est = LSF.LSF28(PosCmd_added_noice)

    ## LSF3/8 速度
    Vel_LSF38_est = LSF.LSF38(PosCmd_added_noice)

    ## TSE2 速度
    Vel_TSE_est = TSE.TSE2(PosCmd_added_noice)

    ## TSE3 速度
    Vel_TSE3_est = TSE.TSE3(PosCmd_added_noice)

    ## BDE3 速度
    Vel_BDE3_est = BDE.BDE3(PosCmd_added_noice)

    ## 
    plt.figure()
    plt.plot(t, PosCmd_added_noice, label="PosCmd_added_noice")
    plt.plot(t, PosCmd, label="PosCmd")
    plt.legend()

    ## CFD速度比较图
    PlotFig.VelPlotFig(Vel_CFD_est, VelCmd, t, R, 'CFD')
    # Err.VelErr_Cmd(Vel_CFD_est, VelCmd, t, R)

    ## LSF 1/4速度比較圖 
    PlotFig.VelPlotFig(Vel_LSF14_est, VelCmd, t, R, 'LSF14')
    # Err.VelErr_Cmd(Vel_LSF14_est, VelCmd, t, R)
    
    ## LSF 2/8速度比較圖 
    PlotFig.VelPlotFig(Vel_LSF28_est, VelCmd, t, R, 'LSF28')
    # Err.VelErr_Cmd(Vel_LSF28_est, VelCmd, t, R)
    
    ## LSF 3/8速度比較圖 
    PlotFig.VelPlotFig(Vel_LSF38_est, VelCmd, t, R, 'LSF38')
    # Err.VelErr_Cmd(Vel_LSF38_est, VelCmd, t, R)
    
    ## TSE2 速度比較圖
    PlotFig.VelPlotFig(Vel_TSE_est, VelCmd, t, R, 'TSE2')
    # Err.VelErr_Cmd(Vel_TSE_est, VelCmd, t, R)
    
    ## TSE3 速度比較圖
    PlotFig.VelPlotFig(Vel_TSE3_est, VelCmd, t, R, 'TSE3')
    # Err.VelErr_Cmd(Vel_TSE3_est, VelCmd, t, R)
   
    ## BDE3 速度比較圖
    PlotFig.VelPlotFig(Vel_BDE3_est, VelCmd, t, R, 'BDE3')
    # Err.VelErr_Cmd(Vel_BDE3_est, VelCmd, t, R)
   
    ## KF 速度比較圖
    PlotFig.VelPlotFig(Vel_KF_est, VelCmd, t, R, 'KF')
    Err. VelErr_Cmd(Vel_KF_est, VelCmd, t, R)
    
    ## 所有方法速度比較
    # 馬達命令加雜訊比較
    VelCmd = [x/R*2.54 for x in VelCmd] # inch to rad
    DataName = [Vel_BDE3_est, Vel_TSE3_est, Vel_LSF38_est, Vel_LSF28_est, Vel_CFD_est, Vel_LSF14_est, Vel_KF_est, VelCmd]
    labels = ['BDE3', 'TSE3', 'LSF38', 'LSF28', 'CFD', 'LSF14', 'KF', 'VelCmd']
    for i in range(len(DataName)):
        DataName[i] = [x * R / 2.54 for x in DataName[i]]
    PlotFig.VelComp(DataName, t, labels)
    ## 所有方法誤差比較
    Err.ErrComp(DataName, t, labels)

    ##
    # CFD加速度比较图
    PlotFig.AccPlotFig_Cmd(Acc_CFD_est, AccCmd, t, R, 'CFD')
    # Err.Err_Cmd(Acc_CFD_est, AccCmd, t)
   
    # TTD加速度比较图
    PlotFig.AccPlotFig_Cmd(Acc_TTD_est, AccCmd, t, R, 'TTD')
    # Err.Err_Cmd(Acc_TTD_est, AccCmd, t)
   
    # LSF2/8 ACC
    PlotFig.AccPlotFig_Cmd(Acc_LSF28_est, AccCmd, t, R, 'LSF28')
    # Err.Err_Cmd(Acc_LSF28_est, AccCmd, t)

    # LAE ACC
    PlotFig.AccPlotFig_Cmd(Acc_LAE_est, AccCmd, t, R, 'LAE')
    # Err.Err_Cmd(Acc_LAE_est, AccCmd, t)
   
    # KF ACC
    PlotFig.AccPlotFig_Cmd(Acc_KF_est, AccCmd, t, R, 'KF')
    # Err.AccErr_Cmd(Acc_KF_est, AccCmd, t ,R)
   
    ## 所有方法加速度比較
    # 馬達比較
    AccCmd = [x/0.01/R*9.81 for x in AccCmd] # G to rad
    DataName = [Acc_CFD_est, Acc_LSF28_est, Acc_TTD_est, Acc_LAE_est, Acc_KF_est, AccCmd]
    labels = ['CFD', 'LSF28', 'TTD', 'LAE', 'KF', 'AccCmd']
    # DataName = [ Acc_TTD_est, Acc_LAE_est, Acc_KF_est, AccCmd]
    # labels = [ 'TTD', 'LAE', 'KF', 'AccCmd']
    for i in range(len(DataName)):
        DataName[i] = [x * 0.01*R/9.81 for x in DataName[i]] # rad to G
    PlotFig.AccComp(DataName, t, labels) 
    ## 所有方法誤差比較 
    # Err.ErrComp(DataName, t, labels)  

    plt.show()