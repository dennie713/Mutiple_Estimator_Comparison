import numpy as np
import matplotlib.pyplot as plt
import sys
import mousedata_add,  ImportData
import Cal, CFD, KF_v2, zero_phase_filter


def objective_function(Q_flat, dt, pos, AccCmd):
    Q = np.array(Q_flat).reshape((3, 3))  # 将扁平化的Q矩阵恢复为二维数组
    
    # 设置卡尔曼滤波器的初始状态
    A = np.array([[1, dt, 0.5*dt**2],
                  [0, 1, dt],
                  [0, 0, 1 ]])
    B = np.array([[0.5*dt**2],
                  [dt],
                  [1]])
    C = np.array([[1, 0, 0]])
    R = 0.00126*2
    P = np.array([[1e-4, 0, 0],
                  [0, 1e-4, 0],
                  [0, 0, 1e-4]])
    xm = np.zeros((3, 1))
    Pm = P

    for i in range(len(pos)):
        Pp = np.dot(np.dot(A, Pm), A.T) + Q
        xp = np.dot(A, xm)
        Km = np.dot(Pp, C.T) / (np.dot(np.dot(C, Pp), C.T) + R)
        y = pos[i] - np.dot(C, xp)
        xm = xp + np.dot(Km, y)
        Pm = np.dot((np.eye(3) - np.dot(Km, C)), Pp)

    # 计算均方根误差 (RMSE)
    estimated_pos = xm[0].flatten()
    rmse = np.sqrt(np.mean((pos - estimated_pos)**2))
    return rmse

from pyswarm import pso

def optimize_q(dt, pos, AccCmd):
    def objective(Q_flat):
        return objective_function(Q_flat, dt, pos, AccCmd)
    
    # 设置 Q 的搜索空间
    MIN_Q_VALUE = np.array([[1e-6, 0, 0],
                            [0, 0.05501*10**-2, 0],
                            [0, 0, 5501*289*10**-5]])
    MAX_Q_VALUE = np.array([[1e-5, 0, 0],
                            [0, 0.05501*10**2, 0],
                            [0, 0, 5501*289*10**-1]])

    lb = MIN_Q_VALUE.flatten()  # 将矩阵展开为一维数组
    ub = MAX_Q_VALUE.flatten()  # 将矩阵展开为一维数组
    # assert np.all(MAX_Q_VALUE >= MIN_Q_VALUE), 'All upper-bound values must be greater than lower-bound values'

    # 执行 PSO
    best_Q_flat, best_rmse = pso(objective, lb, ub, swarmsize=100, maxiter=50)
    
    best_Q = np.array(best_Q_flat).reshape((3, 3))  # 恢复为二维数组
    return best_Q, best_rmse

if __name__ == "__main__":
    SamplingTime = 0.01
    CPI = 1600
    ## 讀取檔案
    path1 = ['D:/ASUS_program_code/asus_code_backup\cmake_mouse_boundary_v9_1/build/IPS750_G50_F_motion.txt'] #馬達資料.txt路徑
    path2 = ['D:/ASUS_program_code/asus_code_backup\cmake_mouse_boundary_v9_1/build/IPS750_G50_F_mouse.txt']  #滑鼠資料.txt路徑
    # path1 = ['build/IPS200_G26_FBFB_motion.txt'] #馬達資料.txt路徑
    # path2 = ['build/IPS200_G26_FBFB_mouse.txt']  #滑鼠資料.txt路徑
    
    Motordata, Mousedata = ImportData.ImportData(path1, path2)
    MouseTime, MotorTime, mouseX, mouseY, Pos, PosCmd, Vel, VelCmd, AccCmd, TorCtrl, mousedata_data, mouse_displacement, mouse_real_Pos = Cal.Cal(Mousedata, Motordata, SamplingTime, CPI) 
    t = np.arange(0, (len(Motordata[:,0])) * SamplingTime, SamplingTime)

    # print(mouse_displacement)

    # 將資料濾波
    filtered_Pos = zero_phase_filter.zero_phase_filter(3, 17, Pos)
    filtered_PosCmd = zero_phase_filter.zero_phase_filter(3, 50, PosCmd)
    filtered_mouse_real_Pos = zero_phase_filter.zero_phase_filter(3, 20, mouse_real_Pos)
    Pos = filtered_Pos
    PosCmd = filtered_PosCmd
    mouse_real_Pos = filtered_mouse_real_Pos
    best_Q, best_rmse = optimize_q(SamplingTime, Pos, AccCmd)