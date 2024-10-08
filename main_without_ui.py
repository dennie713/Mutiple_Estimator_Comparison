import matplotlib.pyplot as plt
import numpy as np
import sys
import mousedata_add,  ImportData
import Cal, CFD, KF_v2, zero_phase_filter

# MAIN
if __name__ == "__main__":
    # Constant
    SamplingTime = 0.001
    CPI = 1600
    ## 木盤半徑12.5 壓克力盤半徑12.53
    wood = 12.5
    plastic = 11.945 #11.62 #11.14 # 12.53
    
    ## 量測半徑
    r = 11.6287
    ## 讀取檔案 "D:\ASUS_program_code\規格測試\有線\IPS650_G50_motion.txt"
    # path1 = ['D:/ASUS_program_code/asus_code_backup\cmake_mouse_boundary_v9_1/build/IPS750_G50_F_motion.txt'] #馬達資料.txt路徑
    # path2 = ['D:/ASUS_program_code/asus_code_backup\cmake_mouse_boundary_v9_1/build/IPS750_G50_F_mouse.txt']  #滑鼠資料.txt路徑
    path1 = ['D:/ASUS_program_code/規格測試\有線\IPS650_G50_motion_1.txt'] #馬達資料.txt路徑
    path2 = ['D:/ASUS_program_code/規格測試\有線\IPS650_G50_mouse_1.txt']  #滑鼠資料.txt路徑
    # path1 = ['build/IPS200_G26_FBFB_motion.txt'] #馬達資料.txt路徑
    # path2 = ['build/IPS200_G26_FBFB_mouse.txt']  #滑鼠資料.txt路徑
    
    Motordata, Mousedata = ImportData.ImportData(path1, path2)
    MouseTime, MotorTime, mouseX, mouseY, Pos, PosCmd, Vel, VelCmd, AccCmd, TorCtrl, mousedata_data, mouse_displacement, mouse_real_Pos = Cal.Cal(Mousedata, Motordata, SamplingTime, CPI) 
    t = np.arange(0, (len(Motordata[:,0])) * SamplingTime, SamplingTime)
    print("Pos.shape = ",Pos.shape)

    # print(mouse_displacement)

    # 將資料濾波
    filtered_Pos = zero_phase_filter.zero_phase_filter(3, 17, Pos)
    filtered_PosCmd = zero_phase_filter.zero_phase_filter(3, 50, PosCmd)
    filtered_mouse_real_Pos = zero_phase_filter.zero_phase_filter(3, 20, mouse_real_Pos)
    Pos = filtered_Pos
    PosCmd = filtered_PosCmd
    mouse_real_Pos = filtered_mouse_real_Pos

    ## CFD 速度&加速度
    Pos_CFD_est, Vel_CFD_est, Acc_CFD_est = CFD.CFD(Pos) 
    Pos_CFD_Cmd, Vel_CFD_Cmd, Acc_CFD_Cmd = CFD.CFD(PosCmd) 
    Pos_CFD_est_mouse, Vel_CFD_est_mouse, Acc_CFD_est_mouse = CFD.CFD(mouse_real_Pos) 
    # Acc_CFD_est_mouse = CFD_2.CFD_2(Vel_CFD_est_mouse, Pos)

    ## KF 速度&加速度
    Pos_KF_est, Vel_KF_est, Acc_KF_est = KF_v2.KF(0.001, Pos, AccCmd)
    Pos_KF_est_mouse, Vel_KF_est_mouse, Acc_KF_est_mouse = KF_v2.KF(0.001, mouse_real_Pos, AccCmd)
    # print(len(mouse_real_Pos))

    ## AKF 速度&加速度
    mouse_real_Pos = [x*2.54/r for x in mouse_real_Pos] # inch to rad
    # Pos_AKF_est, Vel_AKF_est, Acc_AKF_est, Q_acc, Q_vel = KF_v2.AKF_2(0.001, Pos, Vel, Acc_CFD_est)
    # Pos_AKF_est_mouse, Vel_AKF_est_mouse, Acc_AKF_est_mouse, Q_acc, Q_vel = KF_v2.AKF_2(0.001, mouse_real_Pos, Vel, Acc_CFD_est)
    Pos_AKF_est, Vel_AKF_est, Acc_AKF_est, Q_acc, Q_vel = KF_v2.AKF_3(0.001, Pos, VelCmd, AccCmd)
    Pos_AKF_est_mouse, Vel_AKF_est_mouse, Acc_AKF_est_mouse, Q_acc, Q_vel = KF_v2.AKF_3(0.001, mouse_real_Pos, VelCmd, AccCmd)


    ##選擇估測方法
    Pos_est = Pos_KF_est
    Vel_est = Vel_KF_est
    Acc_est = Acc_KF_est
    Pos_est_mouse = Pos_KF_est_mouse
    Vel_est_mouse = Vel_KF_est_mouse
    Acc_est_mouse = Acc_KF_est_mouse

    ##為了正反轉方向計算
    #檢查path中是否包含 "FB","BF","FBFB"，找出有正反轉(方向有變化的)
    keywords = ["B", "FB", "BF", "FBFB"]
    contains_string = any(keyword in path for path in path1 for keyword in keywords)
    if contains_string:
        print("有反轉方向")
        for i in range(1, len(Vel_est)):
            if Vel_est[i] < 0: 
                Vel_est_mouse[i] = -Vel_est_mouse[i]
        Vel_est_mouse = zero_phase_filter.zero_phase_filter(3, 19, Vel_est_mouse)
    else:
        print("無反轉方向")

    # # Plot Pos&PosCmd
    # plt.figure(1)
    # plt.plot(t, Pos, label='Pos', linewidth=4)
    # plt.plot(t, PosCmd, label='PosCmd', linewidth=2)
    # plt.title("Pos")
    # plt.legend()
    # plt.xlabel("Time(sec)")
    # plt.ylabel("rad")

    # # Plot Poserr
    # plt.figure(2)
    # plt.plot(t, PosCmd - Pos)
    # plt.title("Poserr")

    # # Plot Vel and VelCmd
    # plt.figure(3)
    # plt.plot(t, Vel, label='Vel', linewidth=4)
    # plt.plot(t, VelCmd, label='VelCmd', linewidth=2)
    # plt.title("Vel")
    # plt.legend()
    # plt.xlabel("Time(sec)")
    # plt.ylabel("rad/s")

    # # Plot Velerr
    # plt.figure(4)
    # plt.plot(t, VelCmd - Vel)
    # plt.title("Velerr")

    # # Plot TorCtrl
    # plt.figure(5)
    # plt.plot(t, TorCtrl)
    # plt.title("TorCtrl")

    # # Plot mousex
    # plt.figure(6)
    # plt.plot(mousedata_data[:,2], mouseX/CPI)
    # plt.title("mousex")
    # plt.xlabel("Time(sec)")
    # plt.ylabel("in")

    # # Plot mousey
    # plt.figure(7)
    # plt.plot(mousedata_data[:,2], mouseY/CPI)
    # plt.title("mousey")
    # plt.xlabel("Time(sec)")
    # plt.ylabel("in")

    # # Plot mousex^2+mousey^2
    # plt.figure(8)
    # plt.plot(mousedata_data[:,2], np.sqrt(mouseX**2 + mouseY**2)/CPI)
    # plt.title("mousex^2+mousey^2")
    # plt.xlabel("Time(sec)")
    # plt.ylabel("in")

    # # Plot mouse path
    # plt.figure(9)
    # plt.plot(mouseX/CPI, mouseY/CPI)
    # slope = abs((mouseY[len(Mousedata)-1]-mouseY[0])/(mouseX[len(Mousedata)-1]-mouseX[0]))
    # plt.text(3, 8, slope, fontsize=12, color='blue')
    # plt.title("Mouse Path")
    # plt.xlabel("Mouse X Displacement (in)")
    # plt.ylabel("Mouse Y Displacement (in)")

    # 位移比較圖
    plt.figure(10)
    displacement_motor = np.zeros(len(Pos_est))
    displacement_motor[0] = 0
    Pos_est[0] = 0
    for i in range(1, len(Pos_est)):
        displacement_motor[i] = abs(Pos_est[i]-Pos_est[i-1]) + displacement_motor[i-1]
    plt.plot(t, displacement_motor, label="Motor displacement", linewidth=4)
    plt.title("Motor and Mouse displacement Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Displacement (rad)")
    Pos_est_mouse = [x*2.54/r for x in Pos_est_mouse]
    plt.plot(t, Pos_est_mouse, 'r', label="Mouse Displacement", linewidth=1)
    plt.legend(loc="lower center")

    # 位置比较图
    plt.figure(11)
    plt.plot(t, Pos_est, label="Motor Position", linewidth=2)
    plt.title("Motor and Mouse Position Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Position (rad)")
    plt.plot(t, Pos_est_mouse, 'r', label="Mouse Position", linewidth=1)
    plt.legend(loc="lower center")

    # 速度比较图
    plt.figure(12)
    Vel_est = [x*r/2.54 for x in Vel_est] 
    Vel_est_mouse_real = Vel_est_mouse
    plt.plot(t, Vel_est, label="Motor Velocity", linewidth=2)
    plt.title("Motor and Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.yticks(np.arange(-1000, 1000, 50))
    plt.plot(t, Vel_est_mouse_real, 'r', label="Mouse Velocity", linewidth=1)
    plt.ylabel("Mouse Velocity (IPS)")
    plt.legend(loc="lower center")
    plt.grid()

    # 加速度比较图
    plt.figure(13)
    Acc_est = [x*r*0.01/9.81 for x in Acc_est]
    if contains_string:
        Acc_est_mouse = CFD.CFD_2(Vel_est_mouse)
    Acc_est_mouse[(Acc_est_mouse < -70 / 0.0254 * 9.81) | (Acc_est_mouse >  70 / 0.0254 * 9.81)] = 0
    Acc_est_mouse = [x*0.0254/ 9.81 for x in Acc_est_mouse]
    Acc_est_mouse_real = Acc_est_mouse 
    plt.plot(t, Acc_est, label="Motor Acceleration", linewidth=2)
    plt.title("Motor and Mouse Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.yticks(np.arange(-100, 100, 10))
    plt.plot(t, Acc_est_mouse_real, 'r', label="Mouse Acceleration", linewidth=1)
    plt.ylabel("Mouse Acceleration (G)")
    plt.legend(loc="lower center")
    plt.grid()

    # # 速度误差
    # plt.figure(14)
    # deviation = np.abs(Vel_est_mouse - Vel_est) / np.where(Vel_est != 0, Vel_est, 1) * 100
    # deviation[(deviation < -100) | (deviation > 100)] = 0
    # plt.plot(t, np.abs(deviation))
    # plt.title("Velocity Error")
    # plt.xlabel("Time (sec)")
    # plt.ylabel("Error (%)")

    # # 延遲時間比較圖
    # plt.figure(15)
    # plt.plot(MotorTime[0:len(MotorTime)-1], Pos_est[0:len(Pos_est)-1], label="Motor Position", linewidth=2)
    # plt.title("Motor and Mouse Original Time vs. Position Comparison(latency)")
    # plt.xlabel("Original Time (s)")
    # plt.ylabel("Position (rad)")
    # # plt.xticks(0, len(MouseTime)-1, 10)/8
    # mouse_displacement = [x/CPI*2.54/r for x in mouse_displacement]
    # plt.scatter(MouseTime[0:len(MouseTime)-1], mouse_displacement[0:len(mouse_displacement)-1], color='r', label="Mouse Position", s=4)
    # plt.legend(loc="lower center")

    ## AKF
    # 位移比較圖
    plt.figure(10)
    displacement_motor = np.zeros(len(Pos_AKF_est))
    displacement_motor[0] = 0
    Pos_AKF_est[0] = 0
    for i in range(1, len(Pos_AKF_est)):
        displacement_motor[i] = abs(Pos_AKF_est[i]-Pos_AKF_est[i-1]) + displacement_motor[i-1]
    plt.plot(t, displacement_motor, 'green', label="AKF Motor displacement", linewidth=4)
    plt.title("Motor and Mouse displacement Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Displacement (rad)")
    # Pos_AKF_est_mouse = [x*2.54/r for x in Pos_AKF_est_mouse]
    plt.plot(t, Pos_AKF_est_mouse, 'orange', label="AKF Mouse Displacement", linewidth=1)
    plt.legend(loc="lower center")

    # 位置比较图
    plt.figure(11)
    plt.plot(t, Pos_AKF_est, 'green', label="AKF Motor Position", linewidth=2)
    plt.title("Motor and Mouse Position Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Position (rad)")
    plt.plot(t, Pos_AKF_est_mouse, 'orange', label="AKF Mouse Position", linewidth=1)
    plt.legend(loc="lower center")

    # 速度比较图
    plt.figure(12)
    Vel_AKF_est = [x*r/2.54 for x in Vel_AKF_est] 
    VelCmd = [x*r/2.54 for x in VelCmd] 
    # rad to inch
    Vel_est_mouse_real = [x*r/2.54 for x in Vel_AKF_est_mouse]
    plt.plot(t, Vel_AKF_est, 'green', label="AKF Motor Velocity", linewidth=2)
    plt.title("Motor and Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.yticks(np.arange(-1000, 1000, 50))
    plt.plot(t, Vel_est_mouse_real, 'orange', label="AKF Mouse Velocity", linewidth=1)
    plt.plot(t, VelCmd, 'black', label='Motor Velocity Cmd', linewidth=1)
    plt.ylabel("Mouse Velocity (IPS)")
    plt.legend(loc="lower center")
    plt.grid()

    # 加速度比较图
    plt.figure(13)
    Acc_AKF_est = [x*r*0.01/9.81 for x in Acc_AKF_est]
    AccCmd = [x*r*0.01/9.81 for x in AccCmd]
    # if contains_string:
    #     Acc_AFK_est_mouse = CFD.CFD_2(Vel_AKF_est_mouse)
    # Acc_AKF_est_mouse[(Acc_AKF_est_mouse < -70 / 0.0254 * 9.81) | (Acc_AKF_est_mouse >  70 / 0.0254 * 9.81)] = 0
    Acc_AKF_est_mouse = [x*0.0254/ 9.81 for x in Acc_AKF_est_mouse]
    Acc_est_mouse_real = [x*r/2.54 for x in Acc_AKF_est_mouse]
    plt.plot(t, Acc_AKF_est, 'green', label="AKF Motor Acceleration", linewidth=2)
    plt.title("Motor and Mouse Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.yticks(np.arange(-100, 100, 10))
    plt.plot(t, Acc_est_mouse_real, 'orange', label="AKF Mouse Acceleration", linewidth=1)
    plt.plot(t, AccCmd, 'black', label="Motor Acceleration Cmd", linewidth=1)
    plt.ylabel("Mouse Acceleration (G)")
    plt.legend(loc="lower center")
    plt.grid()

    # 速度误差
    plt.figure(14)
    Vel_est_mouse_real = np.array(Vel_est_mouse_real)
    Vel_AKF_est = np.array(Vel_AKF_est)
    deviation = np.abs(Vel_est_mouse_real - Vel_AKF_est) / np.where(Vel_AKF_est != 0, Vel_AKF_est, 1) * 100
    deviation[(deviation < -100) | (deviation > 100)] = 0
    print("mean error = ", np.mean(np.abs(deviation)))
    plt.plot(t, np.abs(deviation))
    plt.title("Velocity Error")
    plt.xlabel("Time (sec)")
    plt.ylabel("Error (%)")

    # Plot Q
    plt.figure(15)
    # plt.scatter(range(1, len(Q_vel) + 1), Q_vel, color='b', label="VEL_Q", s=4)
    plt.plot(range(1, len(Q_vel) + 1), Q_vel, color='b', linestyle='-', marker='x', label="VEL_Q")
    plt.xlabel('Iteration')
    plt.ylabel('Q Values')
    plt.title('Q_vel Values over Iterations')
    plt.legend(loc="upper right")

    plt.figure(16)
    # plt.scatter(range(1, len(Q_acc) + 1), Q_acc, color='r', label="ACC_Q", s=4)
    plt.plot(range(1, len(Q_acc) + 1), Q_acc, color='r', linestyle='-', marker='o', label="ACC_Q")
    plt.xlabel('Iteration')
    plt.ylabel('Q_acc Values')
    plt.title('Q_acc Values over Iterations')
    plt.legend(loc="upper right")

    print("mean Q_vel Values = ", np.mean(Q_vel))
    # print(len(Q_vel))
    print("mean Q_acc Values = ", np.mean(Q_acc))
    # print(len(Q_acc))

    plt.show()
