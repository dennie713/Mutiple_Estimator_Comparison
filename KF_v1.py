import numpy as np

def KalmanFilter(dt, pos):
    
    A = np.array([[1, dt],
                  [0, 1]])
    B = np.array([[0],
                  [dt]])
    C = np.array([[1, 0]])
    Q = np.array([[1e-7, 0],
                  [0, 5501]]) #5501.379519887754為Pos的變異數
    R = 1 # 與誤差有關
    P = np.array([[1e-4, 0],
                  [0, 1e-4]])
    pose = np.zeros(len(pos))
    vele = np.zeros(len(pos))
    xm = np.array([[pose[0]],
                   [vele[0]]])
    Pm = P
    for i in range(len(pos)):
        Pp = np.dot(np.dot(A, Pm), A.T) + Q
        xp = np.dot(A, xm)
        Km = np.dot(Pp, C.T) / (np.dot(np.dot(C, Pp), C.T) + R)
        xm = xp + np.dot(Km, (pos[i] - np.dot(C, xp)))
        Pm = np.dot((np.eye(2) - np.dot(Km, C)), Pp)
        pose[i] = xm[0]
        vele[i] = xm[1]
    return pose, vele

# 示例用法：
# dt = 0.1
# pos = np.array([1, 2, 3, 4, 5])
# vele, pose = KalmanFilter(dt, pos)
# print("Velocity:", vele)
# print("Position:", pose)