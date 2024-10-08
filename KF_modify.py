import numpy as np

# a = 50*9.81*100/12.5
a = 0
def mod_KalmanFilter(dt, pos, PosCmd, VelCmd, AccCmd):
    # A = np.array([[1, dt, 0],
    #               [0, 1, dt],
    #               [0, 0, 1 ]])
    A = np.array([[1, dt, 0.5*dt**2],
                  [0, 1, dt],
                  [0, 0, 1 ]])
    # B = np.array([[1, dt, 0.5*dt**2],
    #               [0, 1, dt],
    #               [0, 0, 1 ]])
    B = np.array([[0.5*dt**2],
                  [dt],
                  [1]])
    # u = np.array([PosCmd, VelCmd, AccCmd])
    u = AccCmd
    C = np.array([[1, 0, 0]]) # 1/cpi 
    # Q = np.array([[1e-6, 0, 0],
    #               [0, 5501, 0],
    #               [0, 0, 5501*289*10**3]]) #5501*289*100 #5501.379519887754為PosCmd的變異數 #289為velcmd的變異數 #e的次方數與暫態時間有關，5的效果最好
    Q = np.array([[1e-6, 0, 0],
                  [0, 0.05501, 0],
                  [0, 0, 5501*289*10**-3]]) 
    # Q = np.array([[1e-6, 0, 0],
    #               [0, 0.00222, 0],
    #               [0, 0, 0.00222*8.629]]) 
    R = 0.00126*2 # 3*10e-4 # 3000 # 5000 # 5000 #150 #500 #1000 #100 #10 #1.5 #1 #與誤差有關 -> 影響平滑度
    P = np.array([[1e-4, 0, 0],
                  [0, 1e-4, 0],
                  [0, 0, 1e-4]])
    
    # Wt = np.array([[0.5*a*dt**5],
    #                [a*dt],
    #                [a*1]])
    Wt = 0
    pose = np.zeros(len(pos))
    vele = np.zeros(len(pos))
    acce = np.zeros(len(pos))
    xm = np.zeros((3, 1))  
    Pm = P
    # print(u)
    # print(u[:,1])
    # print(u.T[:,1])
    # print(u[:,1].shape)
    # print(u[:,1].T.shape)
    for i in range(len(pos)): # m = measurement;p = predict
        Pp = np.dot(np.dot(A, Pm), A.T) + Q
        xp = np.dot(A, xm) + Wt
        # Wt = np.array([[u[i]*0.5*dt**2],
        #                [u[i]*dt],
        #                [u[i]*1]])
        # xp = np.dot(A, xm) + 1e0 * np.dot(B, u[i]) + Wt
        # xp = np.dot(A, xm) + np.dot(B, u[:,i]) + Wt  
        Km = np.dot(Pp, C.T) / (np.dot(np.dot(C, Pp), C.T) + R)
        y = (pos[i] - np.dot(C, xp))
        xm = xp + np.dot(Km, y)
        Pm = np.dot((np.eye(3) - np.dot(Km, C)), Pp)
        pose[i] = xm[0, 0]
        vele[i] = xm[1, 0]
        acce[i] = xm[2, 0]
    return pose, vele, acce

# 滑鼠專用
def ModKF_mouse(dt, pos):   
    A = np.array([[1, dt, 0],
                  [0, 1, dt],
                  [0, 0, 1 ]])
    B = np.array([[0],
                  [dt],
                  [0]])
    C = np.array([[1, 0, 0]]) # 1/cpi 
    Q = np.array([[1e-6, 0, 0],
                  [0, 5405, 0],
                  [0, 0, 5405*277*10**3]]) #5501*289*100 #5405為mouse_real_Pos的變異數 #277為vel的變異數 #e的次方數與暫態時間有關，5的效果最好
    R = 3000 # 5000 #150 #500 #1000 #100 #10 #1.5 #1 #與誤差有關 -> 影響平滑度
    P = np.array([[1e-4, 0, 0],
                  [0, 1e-4, 0],
                  [0, 0, 1e-4]])
    Wt = np.array([[0.5*a*dt**5],
                   [a*dt],
                   [1]])
    # Wt = 0
    pose = np.zeros(len(pos))
    vele = np.zeros(len(pos))
    acce = np.zeros(len(pos))
    xm = np.array([[pose[0]],
                   [vele[0]],
                   [acce[0]]])
    Pm = P
    for i in range(len(pos)):
        Pp = np.dot(np.dot(A, Pm), A.T) + Q
        xp = np.dot(A, xm)
        Km = np.dot(Pp, C.T) / (np.dot(np.dot(C, Pp), C.T) + R)
        y = (pos[i] - np.dot(C, xp))
        xm = xp + np.dot(Km, y)
        Pm = np.dot((np.eye(3) - np.dot(Km, C)), Pp)
        pose[i] = xm[0]
        vele[i] = xm[1]
        acce[i] = xm[2]
    return pose, vele, acce

# 調整變數比較用的 a, R
def mod_KF(dt, pos, a, R):
    A = np.array([[1, dt, 0],
                  [0, 1, dt],
                  [0, 0, 1 ]])
    B = np.array([[0],
                  [dt],
                  [0]]) # 因為控制項u設為0，所以不會影響結果
    C = np.array([[1, 0, 0]]) # 1/cpi 
    Q = np.array([[1e-6, 0, 0],
                  [0, 5501, 0],
                  [0, 0, 5501*289*10**a]]) #5501*289*100 #5501.379519887754為Pos的變異數 #289為velcmd的變異數 #e的次方數與暫態時間有關，5的效果最好
    R = R #5000 #150 #500 #1000 #100 #10 #1.5 #1 #與誤差有關 -> 影響平滑度
    P = np.array([[1e-4, 0, 0],
                  [0, 1e-4, 0],
                  [0, 0, 1e-4]])
    # Wt = np.array([[0.5*dt**2],
    #                [dt],
    #                [0]])
    Wt = 0
    pose = np.zeros(len(pos))
    vele = np.zeros(len(pos))
    acce = np.zeros(len(pos))
    xm = np.array([[pose[0]],
                   [vele[0]],
                   [acce[0]]])
    Pm = P
    for i in range(len(pos)):
        Pp = np.dot(np.dot(A, Pm), A.T) + Q
        xp = np.dot(A, xm) + Wt
        Km = np.dot(Pp, C.T) / (np.dot(np.dot(C, Pp), C.T) + R)
        y = (pos[i] - np.dot(C, xp))
        xm = xp + np.dot(Km, y)
        Pm = np.dot((np.eye(3) - np.dot(Km, C)), Pp)
        pose[i] = xm[0]
        vele[i] = xm[1]
        acce[i] = xm[2]
    return pose, vele, acce