import numpy as np
import Cal, ImportData

class KalmanFilter(object):
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):

        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(I - np.dot(K, self.H), self.P)

def example():
    # Constant
    SamplingTime = 0.001
    CPI = 1600
    ## 讀取檔案
    path1 = ['D:\ASUS_program_code/規格測試/有線/IPS650_G50_motion_1.txt'] #馬達資料.txt路徑
    path2 = ['D:\ASUS_program_code/規格測試/有線/IPS650_G50_mouse_1.txt']  #滑鼠資料.txt路徑
    Motordata, Mousedata = ImportData.ImportData(path1, path2)
    Pos = np.array(Motordata[:, 3],float)
    PosCmd = np.array(Motordata[:, 4],float)
    Vel = np.array(Motordata[:, 5],float)
    VelCmd = np.array(Motordata[:, 6],float)
    t = np.arange(0, (len(Motordata[:,0])) * SamplingTime, SamplingTime)

    dt = 0.001
    F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
    H = np.array([1, 0, 0]).reshape(1, 3)
    Q = np.array([[1e-6, 1e-6, 0.0], [5501, 5501, 0.0], [0.0, 0.0, 0.0]])
    R = np.array([1]).reshape(1, 1)
    
    itr  = 200
    
    def f(x):
        return np.dot(F,x)+np.random.normal(0,5,3)
    
    # real_state = []
    real_state = Pos
    real_state1 = np.zeros(len(Pos))
    x = np.array([0,0,0])
    
    for i in range(itr):
        # real_state1.append(x[0])
        real_state1[i] = x[0]
        x = f(x)
    real_state1[itr] = x[0]
    
    measurements = [x-1+np.random.normal(0,1) for x in real_state]

    kf = KalmanFilter(F = F, H = H, Q = Q, R = R)
    predictions = []
    for z in measurements:
        predictions.append(kf.predict()[0])
        kf.update(z)

    import matplotlib.pyplot as plt
    plt.plot(range(len(measurements)), measurements, label = 'Measurements')
    plt.plot(range(len(predictions)), np.array(predictions), label = 'Kalman Filter Prediction')
    plt.plot(range(len(real_state)), real_state, label = 'Real statement' )
    plt.legend()
    plt.show()

if __name__ == '__main__':
    example()