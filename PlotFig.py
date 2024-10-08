import matplotlib.pyplot as plt
import numpy as np

## 速度圖比較
def VelPlotFig(Vel_xxx_est, Vel_xxx_est_mouse, t, R, type_name):
    plt.figure()
    Vel_xxx_est = [x * R / 2.54 for x in Vel_xxx_est] # rad to inch
    Vel_xxx_est_mouse_real = Vel_xxx_est_mouse
    type = str(type_name)
    plt.plot(t, Vel_xxx_est, label= type + " Estimator Motor Velocity")
    plt.title(type + " Estimator Motor and Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Velocity (IPS)")
    plt.yticks(np.arange(0, max(max(Vel_xxx_est),max(Vel_xxx_est_mouse_real))+50, 100))
    plt.axis([0, t[-1], 0, max(max(Vel_xxx_est),max(Vel_xxx_est_mouse_real))+50])
    plt.grid()
    handles1, labels1 = plt.gca().get_legend_handles_labels()
    plt.twinx()
    plt.plot(t, Vel_xxx_est_mouse_real, 'r', label=type + " Estimator Mouse Velocity")
    plt.ylabel("Mouse Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(max(Vel_xxx_est),max(Vel_xxx_est_mouse_real))+50])
    handles2, labels2 = plt.gca().get_legend_handles_labels()
    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.legend(handles, labels, loc="lower center")
    err_1 = []
    err = np.array(Vel_xxx_est_mouse[200:1748]) - np.array(Vel_xxx_est[200:1748])
    err_1.append(abs(err))
    print( type_name, 'RMS mouse/motor Vel err :', np.sqrt(np.mean(np.square(err_1))))
    
    # plt.show()

## 加速度圖比較
def AccPlotFig(Acc_xxx_est, Acc_xxx_est_mouse, t, R, type_name):
    plt.figure()
    Acc_xxx_est = [x*0.01*R/9.81 for x in Acc_xxx_est] # rad to G
    Acc_xxx_est_mouse = [x*0.0254/ 9.81 for x in Acc_xxx_est_mouse] # inch to G
    Acc_xxx_est_mouse_real = Acc_xxx_est_mouse 
    type = str(type_name)
    plt.plot(t, Acc_xxx_est, label= type + " Estimator Motor Acceleration")
    plt.title(type +" Estimator Motor and Mouse Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Acceleration (G)")
    plt.yticks(np.arange(round(-max(max(Acc_xxx_est_mouse_real),max(Acc_xxx_est))/10)*10-5, round(max(max(Acc_xxx_est_mouse_real),max(Acc_xxx_est))/10)*10+5, 10))
    # plt.axis([0, t[-1], min(min(Acc_xxx_est),min(Acc_xxx_est_mouse_real))-5, max(max(Acc_xxx_est),max(Acc_xxx_est_mouse_real))+5])
    err_1 = []
    err = np.array(Acc_xxx_est_mouse) - np.array(Acc_xxx_est)
    err_1.append(abs(err))
    print( type_name, 'RMS mouse/motor Acc err :', np.sqrt(np.mean(np.square(err_1))))
    # handles1, labels1 = plt.gca().get_legend_handles_labels()
    # plt.twinx()
    plt.plot(t, Acc_xxx_est_mouse_real, 'r', label=type +" Estimator Mouse Acceleration")
    plt.axis([0, t[-1]+0.1, -max(max(Acc_xxx_est_mouse_real),max(Acc_xxx_est))-5, max(max(Acc_xxx_est_mouse_real),max(Acc_xxx_est))+5])
    
    # plt.ylabel("Mouse Acceleration (G)")
    # plt.axis([0, t[-1], min(min(Acc_xxx_est),min(Acc_xxx_est_mouse_real))-5, max(max(Acc_xxx_est),max(Acc_xxx_est_mouse_real))+5])
    # handles2, labels2 = plt.gca().get_legend_handles_labels()
    # handles = handles1 + handles2
    # labels = labels1 + labels2
    # plt.legend(handles, labels, loc="lower center")
    plt.grid()
    plt.legend(loc="lower center")
    # plt.show()

## 加速度圖比較
def AccPlotFig_Cmd(Acc_xxx_est, Acc_Cmd, t, R, type_name):
    plt.figure()
    Acc_xxx_est = [x*0.01*R/9.81 for x in Acc_xxx_est] # rad to G
    # Acc_xxx_est = [x*2.54*0.01/9.81 for x in Acc_xxx_est] # inch to G 
    # Acc_xxx_est_mouse = [x*0.0254/ 9.81 for x in Acc_xxx_est_mouse] # inch to m
    # Acc_xxx_est_mouse_real = Acc_Cmd 
    type = str(type_name)
    plt.plot(t, Acc_xxx_est, label= type + " Estimator Motor Acceleration")
    plt.title(type +" Estimator Motor and Mouse Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Motor Acceleration (G)")
    # plt.axis([0, t[-1], min(min(Acc_xxx_est),min(Acc_xxx_est_mouse_real))-5, max(max(Acc_xxx_est),max(Acc_xxx_est_mouse_real))+5])
    # handles1, labels1 = plt.gca().get_legend_handles_labels()
    # plt.twinx()
    plt.plot(t, Acc_Cmd, 'r', label=type +" Estimator Mouse Acceleration")
    plt.axis([0, t[-1], -max(max(Acc_Cmd),max(Acc_xxx_est))-5, max(max(Acc_Cmd),max(Acc_xxx_est))+5])
    # plt.ylabel("Mouse Acceleration (G)")
    # plt.axis([0, t[-1], min(min(Acc_xxx_est),min(Acc_xxx_est_mouse_real))-5, max(max(Acc_xxx_est),max(Acc_xxx_est_mouse_real))+5])
    # handles2, labels2 = plt.gca().get_legend_handles_labels()
    # handles = handles1 + handles2
    # labels = labels1 + labels2
    # plt.legend(handles, labels, loc="lower center")
    plt.legend(loc="lower center")
    # plt.show()

## 所有速度圖比較
def VelComp(DataName, t, labels):
    plt.figure()
    a = []
    for i in range(len(DataName)):
        a.append(max(DataName[i]))
        plt.plot(t, DataName[i], label = labels[i])
    plt.title("Estimators Motor Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(a)+50])
    plt.legend()   

def VelComp1(DataName, t, labels):
    plt.figure()
    a = []
    for i in range(len(DataName)):
        a.append(max(DataName[i]))
        plt.plot(t, DataName[i], label = labels[i])
    plt.title("Estimators Mouse Velocity Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Velocity (IPS)")
    plt.axis([0, t[-1], 0, max(a)+50])
    plt.legend()

## 所有加速度圖比較
def AccComp(DataName, t, labels):   
    plt.figure()
    a = []
    b = []
    for i in range(len(DataName)):
        a.append(max(DataName[i]))
        b.append(min(DataName[i]))
        plt.plot(t, DataName[i], label = labels[i])
    plt.title("Estimators Motor Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Acceleration (G)")
    plt.axis([0, t[-1], min(b)-5, max(a)+5])
    plt.legend()   

def AccComp1(DataName, t, labels):   
    plt.figure()
    a = []
    b = []
    # print(DataName[1])
    for i in range(len(DataName)):
        a.append(max(DataName[i]))
        b.append(min(DataName[i]))
        plt.plot(t, DataName[i], label = labels[i])
    plt.title("Estimators Mouse Acceleration Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Acceleration (G)")
    plt.axis([0, t[-1], min(b)-5, max(a)+5])
    plt.legend() 

