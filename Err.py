import numpy as np
import matplotlib.pyplot as plt

# 取中間段數據
def VelErr_Cmd_1(a, b, R):
    t = np.arange(0, (len(a)-300) * 0.001, 0.001)
    err_data = []
    plt.figure()
    a = np.array(a)
    b = np.array(b)
    a = [x * R / 2.54 for x in a] # rad to inch
    for i in range(150, 1798, 1):
        err = np.abs(a[i] - b[i]) / np.where(b[i] != 0, b[i], 1) * 100
        err_data.append(np.abs(err))
    plt.plot(t, err_data)
    plt.axis([0, t[-1], 0, 150])
    plt.title("Velocity Error")
    plt.xlabel("Time (sec)")
    plt.ylabel("Error (%)")
    print('error_ave_percent :', np.mean(np.abs(err_data)))

def VelErr_Cmd_2(a, b, R):
    t = np.arange(0, (len(a)-400) * 0.001, 0.001)
    err_data = []
    plt.figure()
    a = np.array(a)
    b = np.array(b)
    # a = [x * R / 2.54 for x in a] # rad to inch
    for i in range(200, 1748, 1):
        err = (a[i] - b[i]) / np.where(b[i] != 0, b[i], 1) * 100
        err_data.append(np.abs(err))
    plt.plot(t, err_data)
    plt.axis([0, t[-1], 0, 150])
    plt.title("Velocity Error")
    plt.xlabel("Time (sec)")
    plt.ylabel("Error (%)")
    print('error_ave_percent :', np.mean(np.abs(err_data)))
    print('---------')

def VelErr_Cmd(a, b, t, R):
    plt.figure()
    a = np.array(a)
    b = np.array(b)
    a = [x * R / 2.54 for x in a] # rad to inch
    err = (a - b) / np.where(b != 0, b, 1) * 100
    plt.plot(t, np.abs(err))
    plt.axis([0, t[-1], 0, 150])
    plt.title("Velocity Error")
    plt.xlabel("Time (sec)")
    plt.ylabel("Error (%)")
    # print('error_ave_percent :', np.mean(np.abs(err)))

def AccErr_Cmd(a, b, t, R):
    err_1 = []
    plt.figure()
    a = np.array(a)
    b = np.array(b)
    a = [x*0.01*R/9.81 for x in a] # rad to G
    err = a - b
    err_1.append(abs(err))
    plt.plot(t, np.abs(err))
    plt.axis([0, t[-1], 0, 200])
    plt.title("Acceleration Error")
    plt.xlabel("Time (sec)")
    plt.ylabel("Error (G)")
    plt.show
    # print('error_ave :', np.mean(np.abs(err_1)))

def ErrComp(a, t, labels):
    err_1 = []
    plt.figure()
    b = a[len(a)-1]
    for i in range(len(a)-1):
        a[i] = np.array(a[i])
        err = (a[i] - b) / np.where(b != 0, b, 1) * 100
        err_1.append(abs(err))
        plt.plot(t, np.abs(err), label = labels[i])
        # print(labels[i],'ave_err = ',np.mean(np.abs(err_1)))
    plt.axis([0, t[-1], 0, 300])
    plt.title("Error Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Error (%)")
    plt.legend()   

# 取中間段數據
def VelErr_Cmd_multi(a, labels, R):
    t = np.arange(0, (len(a)-400) * 0.001, 0.001)
    err_1 = []
    a = np.array(a)
    b = a[len(a)-1]
    for i in range(len(a)-1):
        for j in range (200, 1748, 1):
            a[i][j] = np.array(a[i][j])
            err = a[i][j] - b[j]
            err_percent = np.abs(err)/np.where(b[j] != 0, b[j], 1) * 100
            err_1.append(np.abs(err_percent))
        print(labels[i],'ave_err = ',np.mean(np.abs(err_1[i])))

def Acc_ErrComp(a, t, labels):
    err_1 = []
    plt.figure()
    b = a[len(a)-1]
    for i in range(len(a)-1):
        a[i] = np.array(a[i])
        err = a[i] - b
        err_1.append(abs(err))
        plt.plot(t, np.abs(err), label = labels[i])
        # print(labels[i], 'rms err :', np.sqrt(np.mean(np.square(err_1))))
        # print(labels[i],'ave_err = ',np.mean(np.abs(err_1)))
    plt.axis([0, t[-1], 0, 300])
    plt.title("Error Comparison")
    plt.xlabel("Time (sec)")
    plt.ylabel("Error (G)")
    plt.legend()  

def RMS_err(Acc_XXX_est_mouse, AccCmd, labels):
    err_1 = []
    # err = np.array(Acc_XXX_est_mouse[46:]) - np.array(AccCmd[:1902])
    err = np.array(Acc_XXX_est_mouse) - np.array(AccCmd)
    err_1.append(abs(err))
    print(labels, 'rms mouse err :', np.sqrt(np.mean(np.square(err_1))))
    print('-------------------------------------------')
    