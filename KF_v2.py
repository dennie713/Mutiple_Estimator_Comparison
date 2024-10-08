import numpy as np
import matplotlib.pyplot as plt

a = 0
def KF(dt, pos, AccCmd):
    A = np.array([[1, dt, 0.5*dt**2],
                  [0, 1, dt],
                  [0, 0, 1 ]])
    B = np.array([[0.5*dt**2],
                  [dt],
                  [1]])
    u = AccCmd
    C = np.array([[1, 0, 0]]) # 1/cpi 
    Q = np.array([[1e-6, 0, 0],
                  [0, 0.05501, 0],
                  [0, 0, 5501*289*10**-3]]) 
    # Q = np.array([[8.601050321038729e-29, 0, 0],
    #               [0, 16.711145344706146, 0],
    #               [0, 0, 66722977.441158645]]) 
    # MIN_Q_VALUE = np.array([[1e-6, 0, 0],
    #                         [0, 0.05501*10**-2, 0],
    #                         [0, 0, 5501*289*10**-5]])
    # MAX_Q_VALUE = np.array([[1e-6, 0, 0],
    #                         [0, 0.05501*10**2, 0],
    #                         [0, 0, 5501*289*10**-1]])
    R = 0.00126*2 # 3*10e-4 # 3000 # 5000 # 5000 #150 #500 #1000 #100 #10 #1.5 #1 #與誤差有關 -> 影響平滑度
    P = np.array([[1e-4, 0, 0],
                  [0, 1e-4, 0],
                  [0, 0, 1e-4]])
    Wt = 0
    pose = np.zeros(len(pos))
    vele = np.zeros(len(pos))
    acce = np.zeros(len(pos))
    xm = np.zeros((3, 1))  
    Pm = P
    for i in range(len(pos)): # m = measurement;p = predict
        Pp = np.dot(np.dot(A, Pm), A.T) + Q
        xp = np.dot(A, xm) + Wt
        # xp = np.dot(A, xm) + 1e0 * np.dot(B, u[i]) + Wt 
        Km = np.dot(Pp, C.T) / (np.dot(np.dot(C, Pp), C.T) + R)
        y = (pos[i] - np.dot(C, xp))
        xm = xp + np.dot(Km, y)
        Pm = np.dot((np.eye(3) - np.dot(Km, C)), Pp)
        # Q_new = Q
        # Q_new[2, 2] = np.random.uniform(MAX_Q_VALUE[2, 2], MIN_Q_VALUE[2, 2])
        # Q_new[1, 1] = np.random.uniform(MAX_Q_VALUE[1, 1], MIN_Q_VALUE[1, 1])
        # Q = Q_new
        pose[i] = xm[0, 0]
        vele[i] = xm[1, 0]
        acce[i] = xm[2, 0]
    return pose, vele, acce

def KF_2(dt, pos, vel, AccCmd):
    A = np.array([[1, dt, 0.5*dt**2],
                  [0, 1, dt],
                  [0, 0, 1 ]])
    B = np.array([[0.5*dt**2],
                  [dt],
                  [1]])
    u = AccCmd
    C = np.array([[1, 0, 0]]) # 1/cpi 
    Q = np.array([[ 4.03005280e-06, 0, 0],
                  [0,  1.73656793e+01,  0],
                  [0,  0,  6.67572647e+05]]) 
    # Q = np.array([[8.601050321038729e-29, 0, 0],
    #               [0, 16.711145344706146, 0],
    #               [0, 0, 66722977.441158645]]) 
    # MIN_Q_VALUE = np.array([[1e-6, 0, 0],
    #                         [0, 0.05501*10**-2, 0],
    #                         [0, 0, 5501*289*10**-5]])
    # MAX_Q_VALUE = np.array([[1e-6, 0, 0],
    #                         [0, 0.05501*10**2, 0],
    #                         [0, 0, 5501*289*10**-1]])
    R = 0.00126*2 # 3*10e-4 # 3000 # 5000 # 5000 #150 #500 #1000 #100 #10 #1.5 #1 #與誤差有關 -> 影響平滑度
    P = np.array([[1e-4, 0, 0],
                  [0, 1e-4, 0],
                  [0, 0, 1e-4]])
    Wt = 0
    pose = np.zeros(len(pos))
    vele = np.zeros(len(pos))
    acce = np.zeros(len(pos))
    xm = np.zeros((3, 1))  
    Pm = P
    for i in range(len(pos)): # m = measurement;p = predict
        Pp = np.dot(np.dot(A, Pm), A.T) + Q
        xp = np.dot(A, xm) + Wt
        # xp = np.dot(A, xm) + 1e0 * np.dot(B, u[i]) + Wt 
        Km = np.dot(Pp, C.T) / (np.dot(np.dot(C, Pp), C.T) + R)
        y = (pos[i] - np.dot(C, xp))
        xm = xp + np.dot(Km, y)
        Pm = np.dot((np.eye(3) - np.dot(Km, C)), Pp)
        # Q_new = Q
        # Q_new[2, 2] = np.random.uniform(MAX_Q_VALUE[2, 2], MIN_Q_VALUE[2, 2])
        # Q_new[1, 1] = np.random.uniform(MAX_Q_VALUE[1, 1], MIN_Q_VALUE[1, 1])
        # Q = Q_new
        pose[i] = xm[0, 0]
        vele[i] = xm[1, 0]
        acce[i] = xm[2, 0]
        z=0
        x=0
    return pose, vele, acce, z, x

def AKF(dt, pos, AccCmd):
    A = np.array([[1, dt, 0.5*dt**2],
                  [0, 1, dt],
                  [0, 0, 1 ]])
    B = np.array([[0.5*dt**2],
                  [dt],
                  [1]])
    u = AccCmd
    C = np.array([[1, 0, 0]]) # 1/cpi 
    Q = np.array([[1e-6, 0, 0],
                  [0, 0.05501, 0],
                  [0, 0, 5501*289*10**-3]]) 
    MIN_Q_VALUE = np.array([[1e-6, 0, 0],
                  [0, 0.05501, 0],
                  [0, 0, 5501*289*10**-3]])
    MAX_Q_VALUE = np.array([[1e-6, 0, 0],
                  [0, 0.05501, 0],
                  [0, 0, 5501*289*10**0]])
    # Q = np.array([[1, 0, 0],
    #               [0, 1, 0],
    #               [0, 0, 1]]) 
    R = 0.00126*2 # 3*10e-4 # 3000 # 5000 # 5000 #150 #500 #1000 #100 #10 #1.5 #1 #與誤差有關 -> 影響平滑度
    P = np.array([[1e-4, 0, 0],
                  [0, 1e-4, 0],
                  [0, 0, 1e-4]])
    Wt = 0
    pose = np.zeros(len(pos))
    vele = np.zeros(len(pos))
    acce = np.zeros(len(pos))
    xm = np.zeros((3, 1))  
    Pm = P
    y_values = []
    delta_x_values = []
    for i in range(len(pos)): # m = measurement;p = predict len(pos)
        Pp = np.dot(np.dot(A, Pm), A.T) + Q
        xp = np.dot(A, xm) + Wt
        # xp = np.dot(A, xm) + 1e0 * np.dot(B, u[i]) + Wt 
        Km = np.dot(Pp, C.T) / (np.dot(np.dot(C, Pp), C.T) + R)
        y = (pos[i] - np.dot(C, xp))
        xm = xp + np.dot(Km, y)
        Pm = np.dot((np.eye(3) - np.dot(Km, C)), Pp)
        ## 求M
        y_values.append(y)
        n = i+1
        y_sqr = [val**2 for val in y_values[:n]]
        # if len(y_sqr) < n:
        #     n = len(y_sqr)  
        M = sum(y_sqr[:n])/n
        print("M = ", M)
        ## 求G_telda
        Y = np.dot(np.dot(C, Pp), C.T) + R
        print("Km = ", Km)
        G_tel = np.dot(np.dot(Km.T, Km), Y)
        print("G_tel = ", G_tel)
        ## 求G_hat
        delta_x = xm - xp # x - x-
        delta_x_values.append(delta_x)
        print("delta_x = ", delta_x)
        m = i+1
        delta_x_values_sqr = [np.dot(val, val.T) for val in delta_x_values[:m]]
        # delta_x_values_sqr = [np.array([[val[0]**2], [val[1]**2], [val[2]**2]]) for val in delta_x_values[:m]]
        # delta_x_values_arr = np.array([val.flatten() for val in delta_x_values])
        # delta_x_values_sqr = np.sum(delta_x_values_arr**2, axis=0)
        G_hat = sum(delta_x_values_sqr[:m]) / m
        # G_hat = np.reshape(G_hat, (3, 1))
        # if len(delta_x_values_sqr) < m:
        #     m = len(delta_x_values_sqr)  
        # print("delta_x_values_sqr = ", delta_x_values_sqr[1])
        # G_hat = np.array([np.sum(delta_x_values_sqr[0, m])/m], [np.sum(delta_x_values_sqr[1, m])/m], [np.sum(delta_x_values_sqr[2, m])/m])
        print("G_hat = ", G_hat)
        ## 求S
        G = G_hat/G_tel
        S = np.maximum(1, G_hat/G_tel)
        # if np.issubdtype(G_hat.dtype, np.number):  # Check if G_hat is a scalar
        #     S = max(1, G_hat / G_tel)
        # else:
        #     S = np.maximum(1, G_hat / G_tel)
        # S = [[np.maximum(1, G[0])],
        #      [np.maximum(1, G[1])],
        #      [np.maximum(1, G[2])]]
        print("G_hat/G_tel = ", G_hat/G_tel)
        print("S = ", S)
        ## 求Q_hat
        # Q_hat = np.dot(S, M)
        Q_hat = S * M
        print("Q_hat = ", Q_hat)
        ## 更新Q值
        # Q = np.dot(Q, Q_hat)
        # Q_hat_flattened = np.array([Q_hat[i, 0, 0] for i in range(Q_hat.shape[0])])
        # Q_diag =  np.diag(Q_hat_flattened)
        # print("Q_diag = ", Q_diag)
        print("Q = ", Q)
        # Q_new  = np.dot(Q_hat, Q)
        Q_new  = Q_hat * Q
        print("Q_new = ", Q_new)
        Q_new = np.maximum(Q_new, MIN_Q_VALUE)
        Q_new = np.minimum(Q_new, MAX_Q_VALUE)
        diag_elements = np.diag(Q_new)
        Q_new = np.diag(diag_elements)
        # Q_new = Q_hat * Q
        print("Q_new = ", Q_new)

        Q = Q_new
        # Q_new = np.dot(Q, Q_diag)
    
        
        # Q = np.array([Q_hat[0], 0, 0],
        #              [0, Q_hat[1], 0],
        #              [0, 0, Q_hat[2]])
        # Q = np.eye(3)*Q_hat
        # if np.issubdtype(G_hat.dtype, np.number):  # Check if G_hat is a scalar
        #     Q_hat = S * M
        #     Q = np.eye(3) * Q_hat  # Create a diagonal matrix with Q_hat as diagonal elements
        # else:
        #     Q_hat = S * M
        #     Q = np.diag(Q_hat)  # Use Q_hat as diagonal elements if it's an array
        print("--------------------------------------------------")

        pose[i] = xm[0, 0]
        vele[i] = xm[1, 0]
        acce[i] = xm[2, 0]
    return pose, vele, acce, Q_hat

## 速度\加速度Q值分開調整
def AKF_2(dt, Pos, Vel, Acc_CFD_est):
    A = np.array([[1, dt, 0.5*dt**2],
                  [0, 1, dt],
                  [0, 0, 1 ]])
    B = np.array([[0.5*dt**2],
                  [dt],
                  [1]])
    u = Acc_CFD_est
    C = np.array([[1, 0, 0]]) 
    # C = np.array([[1, 1, 0]])
    # Q = np.array([[1e-6, 0, 0],
    #               [0, 0, 0],
    #               [0, 0, 0]]) 
    # Q = np.array([[ 4.03005280e-06, 0, 0],
    #               [0,  1.73656793e+01,  0],
    #               [0,  0,  6.67572647e+05]]) 
    # MIN_Q_VALUE = np.array([[1e-6, 0, 0],
    #                         [0, 0.0001, 0],
    #                         [0, 0, 0.0001]])
    # MAX_Q_VALUE = np.array([[1e-6, 0, 0],
    #                         [0, 1000, 0],
    #                         [0, 0, 5000]])
    Q = np.array([[1e-6, 0, 0],
                  [0, 0.05501, 0],
                  [0, 0, 5501*289*10**-3]]) 
    MIN_Q_VALUE = np.array([[1e-6, 0, 0],
                            [0, 0.05501*10**-3, 0],
                            [0, 0, 5501*289*10**-6]])
    MAX_Q_VALUE = np.array([[1e-6, 0, 0],
                            [0, 0.05501*10**3, 0],
                            [0, 0, 5501*289*10**0]])
    # Q = np.array([[ 4.03005280e-06, 0, 0],
    #               [0,  1.73656793e+01,  0],
    #               [0,  0,  6.67572647e+05]]) 
    # Q = np.array([[ 4.03005280e-06, 0, 0],
    #               [0,  1.73656793e+01,  0],
    #               [0,  0,  6.67572647e+05]]) 
    # MIN_Q_VALUE = np.array([[ 4.03005280e-08, 0, 0],
    #               [0,  1.73656793e-01,  0],
    #               [0,  0,  6.67572647e+03]])
    # MAX_Q_VALUE = np.array([[ 4.03005280e-04, 0, 0],
    #               [0,  1.73656793e+03,  0],
    #               [0,  0,  6.67572647e+07]])
    R = 0.00126*2 # 3*10e-4 # 3000 # 5000 # 5000 #150 #500 #1000 #100 #10 #1.5 #1 #與誤差有關 -> 影響平滑度
    P = np.array([[1e-4, 0, 0],
                  [0, 1e-4, 0],
                  [0, 0, 1e-4]])
    Wt = 0
    pose = np.zeros(len(Pos))
    vele = np.zeros(len(Pos))
    acce = np.zeros(len(Pos))
    xm = np.zeros((3, 1))  
    Pm = P
    u_values = []
    y_values = []
    delta_x_values = []
    Q_acc = []
    Q_vel = []
    Q_pos = []
    count_1 = 0
    count_2 = 0
    count_3 = 0
    
    for i in range(len(Pos)): # m = measurement;p = predict len(pos)
        Pp = np.dot(np.dot(A, Pm), A.T) + Q
        xp = np.dot(A, xm) + Wt
        # xp = A@xm + Wt
        # Wt = xp - np.dot(A, xm)
        # xp = np.dot(A, xm) + 1e0 * np.dot(B, u[i]) + Wt 
        Km = np.dot(Pp, C.T) / (np.dot(np.dot(C, Pp), C.T) + R)
        # z = np.array([[Pos[i]],
        #               [Vel[i]],
        #               [Acc_CFD_est[i]]])
        # y = (z - np.dot(C, xp))
        y = (Pos[i] - np.dot(C, xp))
        xm = xp + np.dot(Km, y) # =x_hat
        Pm = np.dot((np.eye(3) - np.dot(Km, C)), Pp)

        ##-------加速度求Q-------##
        print("-------加速度求Q-------")
        ## 求M
        u = (Pos[i] - xm[0]) # y(k)-x_hat(k)
        # u = (Acc_CFD_est[i] - xm[2]) 
        print("u = ", u)
        # u = Acc_CFD_est[i] - xm[2]
        print("xm[0] = ", xm[0])
        u_values.append(u)
        # y_values.append(y)
        n = i+1
        u_sqr = [val**2 for val in u_values[:n]]
        M = sum(u_sqr[:n])/n
        print("M = ", M)
        ## 求G_telda
        Y = np.dot(np.dot(C, Pp), C.T) + R
        print("Y = ", Y)
        print("Km = ", Km)
        G_tel = Km[2]**2 * Y
        # G_tel = np.dot(np.dot(Km.T, Km), Y)
        print("G_tel = ", G_tel)
        ## 求G_hat
        delta_x = xm[2] - xp[2] # x - x-
        delta_x_values.append(delta_x)
        print("delta_x = ", delta_x)
        m = i+1
        # delta_x_values_sqr = [np.dot(val.T, val) for val in delta_x_values[:m]]
        delta_x_values_sqr = [val**2 for val in delta_x_values[:m]]
        # print("sum(delta_x_values_sqr[:m]) = ", sum(delta_x_values_sqr[:m]))
        G_hat = sum(delta_x_values_sqr[:m]) / m
        print("G_hat = ", G_hat)
        ## 求S
        G = G_hat/G_tel
        S = np.maximum(1, G_hat/G_tel)
        print("G_hat/G_tel = ", G_hat/G_tel)
        print("S = ", S)
        ## 求Q_hat
        # Q_hat = np.dot(S, M)
        Q_hat = S * M
        a = Q_hat
        print("Q_hat = ", Q_hat)
        Q_hat_array = np.array([1, 1, Q_hat[0, 0]])  # Extracting the scalar value from 2-D array
        Q_hat_matrix = np.diag(Q_hat_array)
        print("Q_hat_matrix = ", Q_hat_matrix)
        ## 更新Q值
        print("Q = ", Q)
        Q_new = np.dot(Q, Q_hat_matrix)
        if (Q_new[2, 2] > MAX_Q_VALUE[2, 2] or Q_new[2, 2] < MIN_Q_VALUE[2, 2]):
            Q_new[2, 2] = np.random.uniform(MAX_Q_VALUE[2, 2], MIN_Q_VALUE[2, 2])
            print("Q_new[2, 2] = ", Q_new[2, 2])
            count_1 = count_1 + 1
        print("count_1 = ", count_1)
        print("Q_new = ", Q_new)
        # udate_factor = 0.5
        # Q_new = Q + (Q_new - Q) * udate_factor
        Q = Q_new
        Q_acc.append(Q_new[2, 2])
        # MIN_Q_VALUE[2, 2] = np.mean(Q_acc)
        print("--------------------------------------------------")

        ##-------速度求Q-------##
        print("-------速度求Q-------")
        ## 求M
        u = (Pos[i]- xm[0])
        # u = Vel[i]- xm[1]
        print("u = ", u)
        print("xm[1] = ", xm[1])
        u_values.append(u)
        # y_values.append(y)
        n = i+1
        u_sqr = [val**2 for val in u_values[:n]]
        M = sum(u_sqr[:n])/n
        print("M = ", M)
        ## 求G_tel
        G_tel = Km[1]**2 * Y
        # G_tel = np.dot(np.dot(Km.T, Km), Y)
        print("G_tel = ", G_tel)
        ## 求G_hat
        delta_x = xm[1] - xp[1] # x - x-
        delta_x_values.append(delta_x)
        print("delta_x = ", delta_x)
        m = i+1
        # delta_x_values_sqr = [np.dot(val.T, val) for val in delta_x_values[:m]]
        delta_x_values_sqr = [val**2 for val in delta_x_values[:m]]
        # print("sum(delta_x_values_sqr[:m]) = ", sum(delta_x_values_sqr[:m]))
        G_hat = sum(delta_x_values_sqr[:m]) / m
        print("G_hat = ", G_hat)
        ## 求S
        G = G_hat/G_tel
        S = np.maximum(1, G_hat/G_tel)
        print("G_hat/G_tel = ", G_hat/G_tel)
        print("S = ", S)
        ## 求Q_hat
        Q_hat = S * M
        a = Q_hat
        print("Q_hat = ", Q_hat)
        Q_hat_array = np.array([1, Q_hat[0, 0], 1])  # Extracting the scalar value from 2-D array
        Q_hat_matrix = np.diag(Q_hat_array)
        print("Q_hat_matrix = ", Q_hat_matrix)
        ## 更新Q值
        print("Q = ", Q)
        Q_new = np.dot(Q, Q_hat_matrix)
        if (Q_new[1, 1] > MAX_Q_VALUE[1, 1] or Q_new[1, 1] < MIN_Q_VALUE[1, 1]):
            Q_new[1, 1] = np.random.uniform(MAX_Q_VALUE[1, 1], MIN_Q_VALUE[1, 1])
            print("Q_new[1, 1] = ", Q_new[1, 1])
            count_2 = count_2 + 1
        print("count_2 = ", count_2)
        print("Q_new = ", Q_new)
        # update_factor = 0.5
        # Q_new = Q + (Q_new - Q) * update_factor
        Q = Q_new
        Q_vel.append(Q_new[1, 1])
        # MIN_Q_VALUE[1, 1] = np.mean(Q_vel)
        print("--------------------------------------------------")

        ##-------位置求Q-------##
        # print("-------位置求Q-------")
        # ## 求M
        # u = Pos[i]- xm[0]
        # print("u = ", u)
        # # u = Vel[i]- xm[1]
        # print("xm[1] = ", xm[0])
        # u_values.append(u)
        # # y_values.append(y)
        # n = i+1
        # u_sqr = [val**2 for val in u_values[:n]]
        # M = sum(u_sqr[:n])/n
        # print("M = ", M)
        # ## 求G_tel
        # G_tel = Km[0]**2 * Y
        # # G_tel = np.dot(np.dot(Km.T, Km), Y)
        # print("G_tel = ", G_tel)
        # ## 求G_hat
        # delta_x = xm[0] - xp[0] # x - x-
        # delta_x_values.append(delta_x)
        # print("delta_x = ", delta_x)
        # m = i+1
        # # delta_x_values_sqr = [np.dot(val.T, val) for val in delta_x_values[:m]]
        # delta_x_values_sqr = [val**2 for val in delta_x_values[:m]]
        # # print("sum(delta_x_values_sqr[:m]) = ", sum(delta_x_values_sqr[:m]))
        # G_hat = sum(delta_x_values_sqr[:m]) / m
        # print("G_hat = ", G_hat)
        # ## 求S
        # G = G_hat/G_tel
        # S = np.maximum(1, G_hat/G_tel)
        # print("G_hat/G_tel = ", G_hat/G_tel)
        # print("S = ", S)
        # ## 求Q_hat
        # Q_hat = S * M
        # a = Q_hat
        # print("Q_hat = ", Q_hat)
        # Q_hat_array = np.array([Q_hat[0, 0], 1, 1])  # Extracting the scalar value from 2-D array
        # Q_hat_matrix = np.diag(Q_hat_array)
        # print("Q_hat_matrix = ", Q_hat_matrix)
        # ## 更新Q值
        # print("Q = ", Q)
        # Q_new = np.dot(Q, Q_hat_matrix)
        # if (Q_new[0, 0] > MAX_Q_VALUE[0, 0] or Q_new[0, 0] < MIN_Q_VALUE[0, 0]):
        #     Q_new[0, 0] = np.random.uniform(MAX_Q_VALUE[0, 0], MIN_Q_VALUE[0, 0])
        #     print("Q_new[0, 0] = ", Q_new[0, 0])
        #     count_3 = count_3 + 1
        # print("count_3 = ", count_3)
        # print("Q_new = ", Q_new)
        # # update_factor = 0.01
        # # Q_new = Q + (Q_new - Q) * update_factor
        # Q = Q_new
        # Q_pos.append(Q_new[0, 0])
        # # MIN_Q_VALUE[1, 1] = np.mean(Q_vel)
        # print("--------------------------------------------------")

        pose[i] = xm[0]
        vele[i] = xm[1]
        acce[i] = xm[2]
    # # Plot Q
    # plt.figure()
    # # plt.plot(range(1, len(Q_acc) + 1), Q_acc, color='r', label="ACC_Q")
    # plt.scatter(range(1, len(Q_vel) + 1), Q_vel, color='b', label="VEL_Q")
    # plt.xlabel('Iteration')
    # plt.ylabel('Q Values')
    # plt.title('Q Values over Iterations')
    # plt.legend(loc="upper right")
    # plt.grid()

    # plt.figure()
    # plt.scatter(range(1, len(Q_acc) + 1), Q_acc, color='r', label="ACC_Q")
    # # plt.plot(range(1, len(Q_vel) + 1), Q_vel, color='b', label="VEL_Q")
    # plt.xlabel('Iteration')
    # plt.ylabel('Q Values')
    # plt.title('Q Values over Iterations')
    # plt.legend(loc="upper right")
    # plt.grid()
    # plt.show()
    return pose, vele, acce, Q_acc, Q_vel

def AKF_3(dt, Pos, VelCmd, AccCmd):
    A = np.array([[1, dt, 0.5*dt**2],
                  [0, 1, dt],
                  [0, 0, 1 ]])
    B = np.array([[0.5*dt**2],
                  [dt],
                  [1]])
    u = AccCmd
    C = np.array([[1, 0, 0]]) 
    C1 = np.array([[1, 0, 0]]) 
    C2 = np.array([[0, 1, 0]]) 
    C3 = np.array([[0, 0, 1]]) 
    D = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])
    Q = np.array([[1e-6, 0, 0],
                  [0, 0, 0],
                  [0, 0, 0]]) 
    # Q = np.array([[ 4.03005280e-06, 0, 0],
    #               [0,  1.73656793e+01,  0],
    #               [0,  0,  6.67572647e+05]]) 
    # MIN_Q_VALUE = np.array([[1e-6, 0, 0],
    #                         [0, 0.0001, 0],
    #                         [0, 0, 0.0001]])
    # MAX_Q_VALUE = np.array([[1e-6, 0, 0],
    #                         [0, 1000, 0],
    #                         [0, 0, 5000]])
    # Q = np.array([[1e-6, 0, 0],
    #               [0, 0.05501, 0],
    #               [0, 0, 5501*289*10**-3]]) 
    MIN_Q_VALUE = np.array([[1e-6, 0, 0],
                            [0, 0.05501*10**-3, 0],
                            [0, 0, 5501*289*10**-6]])
    MAX_Q_VALUE = np.array([[1e-6, 0, 0],
                            [0, 0.05501*10**3, 0],
                            [0, 0, 5501*289*10**0]])
    # Q = np.array([[ 4.03005280e-06, 0, 0],
    #               [0,  1.73656793e+01,  0],
    #               [0,  0,  6.67572647e+05]]) 
    # Q = np.array([[ 4.03005280e-06, 0, 0],
    #               [0,  1.73656793e+01,  0],
    #               [0,  0,  6.67572647e+05]]) 
    # MIN_Q_VALUE = np.array([[ 4.03005280e-08, 0, 0],
    #               [0,  1.73656793e-01,  0],
    #               [0,  0,  6.67572647e+03]])
    # MAX_Q_VALUE = np.array([[ 4.03005280e-04, 0, 0],
    #               [0,  1.73656793e+03,  0],
    #               [0,  0,  6.67572647e+07]])
    R = 0.00126*2 # 3*10e-4 # 3000 # 5000 # 5000 #150 #500 #1000 #100 #10 #1.5 #1 #與誤差有關 -> 影響平滑度
    P = np.array([[1e-4, 0, 0],
                  [0, 1e-4, 0],
                  [0, 0, 1e-4]])
    Wt = 0
    pose = np.zeros(len(Pos))
    vele = np.zeros(len(Pos))
    acce = np.zeros(len(Pos))
    xm = np.zeros((3, 1))  
    Pm = P
    u_values = []
    v_values = []
    delta_x_values = []
    Q_acc = []
    Q_vel = []
    Q_pos = []
    count_1 = 0
    count_2 = 0
    count_3 = 0
    
    for i in range(len(Pos)): # m = measurement;p = predict len(pos)
    # for i in range(1): # m = measurement;p = predict len(pos)

        Pp = np.dot(np.dot(A, Pm), A.T) + Q
        xp = np.dot(A, xm) + Wt
        # Wt = xp - np.dot(A, xm)
        # xp = np.dot(A, xm) + 1e0 * np.dot(B, u[i]) + Wt 
        # Km = np.dot(Pp, C.T) / (np.dot(np.dot(C, Pp), C.T) + R)
        Kmp = Pp[0][0] / (np.dot(np.dot(C1, Pp), C1.T) + R)
        Kmv = Pp[1][1] / (np.dot(np.dot(C2, Pp), C2.T) + R)
        Kma = Pp[2][2] / (np.dot(np.dot(C3, Pp), C3.T) + R)
        z = np.array([[Pos[i]],
                      [VelCmd[i]],
                      [AccCmd[i]]])
        print("z=",z)
        # y = (z - D @ xp)
        # y = (z - np.dot(D, xp))
        y = (Pos[i] - np.dot(C, xp))
        # xm = xp + np.dot(Km, y) # =x_hat
        # Km[i]*y[i]
        print('xp = ',xp)
        print("xm = ", xm)
        print('z = ',z)
        print('np.dot(D, xp) = ',D @ xp)
        # print('Km = ',Km)
        print('Kmp = ',Kmp)
        print('Kmv = ',Kmv)
        print('Kma = ',Kma)
        print('y = ',y)
        # Km_y = np.array([[Km[0]@y[0]],
        #                 [Km[1]@y[1]],
        #                 [Km[2]@y[2]]])
        Km_y = np.array([[Kmp[0, 0]*y[0, 0]],
                        [Kmv[0, 0]*y[1, 0]],
                        [Kma[0, 0]*y[2, 0]]])
        print('Kmy = ',Km_y)
        xm = xp + Km_y
        # Km_c = np.array([[Km[0, 0], 0, 0],
        #                 [0, Km[1, 0], 0],
        #                 [0, 0, Km[2, 0]]])
        Km_c = np.array([[Kmp[0, 0], 0, 0],
                        [0, Kmv[0, 0], 0],
                        [0, 0, Kma[0, 0]]])
        print("Km_c = ", Km_c)
        Pm = np.dot((np.eye(3) - Km_c), Pp)
        # Pm = np.dot((np.eye(3) - np.dot(Km, C)), Pp)

        ##-------加速度求Q-------##
        print("-------加速度求Q-------")
        ## 求M
        u = (Pos[i] - xm[0]) # y(k)-x_hat(k)
        # print("xm[2] = ", xm[2, 0])
        # u = (AccCmd[i] - xm[2, 0]) 
        print("u = ", u)
        # u = Acc_CFD_est[i] - xm[2]
        # print("xm[0] = ", xm[0])
        u_values.append(u)
        # y_values.append(y)
        n = i+1
        u_sqr = [val**2 for val in u_values[:n]]
        M = sum(u_sqr[:n])/n
        print("M = ", M)
        ## 求G_telda
        Y = np.dot(np.dot(C2, Pp), C2.T) + R
        print("Y = ", Y)
        print("Km = ", Kma)
        # G_tel = Km[2]**2 * Y
        G_tel = Kma**2 * Y
        # G_tel = np.dot(np.dot(Km.T, Km), Y)
        print("G_tel = ", G_tel)
        ## 求G_hat
        delta_x = xm[2, 0] - xp[2] # x - x-
        # delta_x = xm[0, 0] - xp[0] # x - x-
        delta_x_values.append(delta_x)
        print("delta_x = ", delta_x)
        m = i+1
        # delta_x_values_sqr = [np.dot(val.T, val) for val in delta_x_values[:m]]
        delta_x_values_sqr = [val**2 for val in delta_x_values[:m]]
        # print("sum(delta_x_values_sqr[:m]) = ", sum(delta_x_values_sqr[:m]))
        G_hat = sum(delta_x_values_sqr[:m]) / m
        print("G_hat = ", G_hat)
        ## 求S
        G = G_hat/G_tel
        S = np.maximum(1, G_hat/G_tel)
        print("G_hat/G_tel = ", G_hat/G_tel)
        print("S = ", S)
        ## 求Q_hat
        # Q_hat = np.dot(S, M)
        Q_hat = S * M
        a = Q_hat
        print("Q_hat = ", Q_hat)
        Q_hat_array = np.array([1, 1, Q_hat[0, 0]])  # Extracting the scalar value from 2-D array
        Q_hat_matrix = np.diag(Q_hat_array)
        print("Q_hat_matrix = ", Q_hat_matrix)
        ## 更新Q值
        print("Q = ", Q)
        # Q_new = Q_hat_matrix 
        # Q_new = np.dot(Q, Q_hat_matrix)
        Q[2, 2] = Q_hat 
        Q_new = Q
        # if (Q_new[2, 2] > MAX_Q_VALUE[2, 2] or Q_new[2, 2] < MIN_Q_VALUE[2, 2]):
        #     Q_new[2, 2] = np.random.uniform(MAX_Q_VALUE[2, 2], MIN_Q_VALUE[2, 2])
        #     print("Q_new[2, 2] = ", Q_new[2, 2])
        #     count_1 = count_1 + 1
        print("count_1 = ", count_1)
        print("Q_new = ", Q_new)
        # udate_factor = 0.5
        # Q_new = Q + (Q_new - Q) * udate_factor
        Q = Q_new
        Q_acc.append(Q_new[2, 2])
        # MIN_Q_VALUE[2, 2] = np.mean(Q_acc)
        print("--------------------------------------------------")

        ##-------速度求Q-------##
        print("-------速度求Q-------")
        ## 求M
        v = (Pos[i]- xm[0])
        # v = (VelCmd[i] - xm[1, 0])
        # print("v = ", v)
        # print("xm[1] = ", xm[1])
        v_values.append(v)
        # y_values.append(y)
        n = i+1
        v_sqr = [val**2 for val in v_values[:n]]
        M = sum(v_sqr[:n])/n
        print("M = ", M)
        ## 求G_tel
        # G_tel = Km[1]**2 * Y
        G_tel = Kmv**2 * Y
        # G_tel = np.dot(np.dot(Km.T, Km), Y)
        print("G_tel = ", G_tel)
        ## 求G_hat
        delta_x = xm[1, 0] - xp[1] # x - x-
        
        delta_x_values.append(delta_x)
        print("delta_x = ", delta_x)
        m = i+1
        # delta_x_values_sqr = [np.dot(val.T, val) for val in delta_x_values[:m]]
        delta_x_values_sqr = [val**2 for val in delta_x_values[:m]]
        # print("sum(delta_x_values_sqr[:m]) = ", sum(delta_x_values_sqr[:m]))
        G_hat = sum(delta_x_values_sqr[:m]) / m
        print("G_hat = ", G_hat)
        ## 求S
        G = G_hat/G_tel
        S = np.maximum(1, G_hat/G_tel)
        print("G_hat/G_tel = ", G_hat/G_tel)
        print("S = ", S)
        ## 求Q_hat
        Q_hat = S * M
        a = Q_hat
        print("Q_hat = ", Q_hat)
        Q_hat_array = np.array([1, Q_hat[0, 0], 1])  # Extracting the scalar value from 2-D array
        Q_hat_matrix = np.diag(Q_hat_array)
        print("Q_hat_matrix = ", Q_hat_matrix)
        ## 更新Q值
        # print("Q = ", Q)
        # Q_new = np.dot(Q, Q_hat_matrix)
        # Q_new = Q_hat_matrix
        print("Q = ", Q)
        Q[1, 1] = Q_hat
        Q_new = Q
        # if (Q_new[1, 1] > MAX_Q_VALUE[1, 1] or Q_new[1, 1] < MIN_Q_VALUE[1, 1]):
        #     Q_new[1, 1] = np.random.uniform(MAX_Q_VALUE[1, 1], MIN_Q_VALUE[1, 1])
        #     print("Q_new[1, 1] = ", Q_new[1, 1])
        #     count_2 = count_2 + 1
        print("count_2 = ", count_2)
        print("Q_new = ", Q_new)
        # update_factor = 0.5
        # Q_new = Q + (Q_new - Q) * update_factor
        Q = Q_new
        Q_vel.append(Q_new[1, 1])
        # MIN_Q_VALUE[1, 1] = np.mean(Q_vel)
        print("--------------------------------------------------")

        # #-------位置求Q-------##
        print("-------位置求Q-------")
        ## 求M
        u = Pos[i]- xm[0, 0]
        print("u = ", u)
        # # u = Vel[i]- xm[1]
        # print("xm[1] = ", xm[0])
        # u_values.append(u)
        # # y_values.append(y)
        # n = i+1
        # u_sqr = [val**2 for val in u_values[:n]]
        # M = sum(u_sqr[:n])/n
        # print("M = ", M)
        # ## 求G_tel
        # G_tel = Km[0]**2 * Y
        # # G_tel = np.dot(np.dot(Km.T, Km), Y)
        # print("G_tel = ", G_tel)
        # ## 求G_hat
        # delta_x = xm[0] - xp[0] # x - x-
        # delta_x_values.append(delta_x)
        # print("delta_x = ", delta_x)
        # m = i+1
        # # delta_x_values_sqr = [np.dot(val.T, val) for val in delta_x_values[:m]]
        # delta_x_values_sqr = [val**2 for val in delta_x_values[:m]]
        # # print("sum(delta_x_values_sqr[:m]) = ", sum(delta_x_values_sqr[:m]))
        # G_hat = sum(delta_x_values_sqr[:m]) / m
        # print("G_hat = ", G_hat)
        # ## 求S
        # G = G_hat/G_tel
        # S = np.maximum(1, G_hat/G_tel)
        # print("G_hat/G_tel = ", G_hat/G_tel)
        # print("S = ", S)
        # ## 求Q_hat
        # Q_hat = S * M
        # a = Q_hat
        # print("Q_hat = ", Q_hat)
        # Q_hat_array = np.array([Q_hat[0, 0], 1, 1])  # Extracting the scalar value from 2-D array
        # Q_hat_matrix = np.diag(Q_hat_array)
        # print("Q_hat_matrix = ", Q_hat_matrix)
        # ## 更新Q值
        # print("Q = ", Q)
        # Q_new = np.dot(Q, Q_hat_matrix)
        # if (Q_new[0, 0] > MAX_Q_VALUE[0, 0] or Q_new[0, 0] < MIN_Q_VALUE[0, 0]):
        #     Q_new[0, 0] = np.random.uniform(MAX_Q_VALUE[0, 0], MIN_Q_VALUE[0, 0])
        #     print("Q_new[0, 0] = ", Q_new[0, 0])
        #     count_3 = count_3 + 1
        # print("count_3 = ", count_3)
        # print("Q_new = ", Q_new)
        # # update_factor = 0.01
        # # Q_new = Q + (Q_new - Q) * update_factor
        # Q = Q_new
        # Q_pos.append(Q_new[0, 0])
        # # MIN_Q_VALUE[1, 1] = np.mean(Q_vel)
        print("--------------------------------------------------")

        pose[i] = xm[0 ,0]
        vele[i] = xm[1, 0]
        acce[i] = xm[2, 0]
    # # Plot Q
    # plt.figure()
    # # plt.plot(range(1, len(Q_acc) + 1), Q_acc, color='r', label="ACC_Q")
    # plt.scatter(range(1, len(Q_vel) + 1), Q_vel, color='b', label="VEL_Q")
    # plt.xlabel('Iteration')
    # plt.ylabel('Q Values')
    # plt.title('Q Values over Iterations')
    # plt.legend(loc="upper right")
    # plt.grid()

    # plt.figure()
    # plt.scatter(range(1, len(Q_acc) + 1), Q_acc, color='r', label="ACC_Q")
    # # plt.plot(range(1, len(Q_vel) + 1), Q_vel, color='b', label="VEL_Q")
    # plt.xlabel('Iteration')
    # plt.ylabel('Q Values')
    # plt.title('Q Values over Iterations')
    # plt.legend(loc="upper right")
    # plt.grid()
    # plt.show()
    return pose, vele, acce, Q_acc, Q_vel