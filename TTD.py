import numpy as np

def TTD(z1, r):
    # SamplingTime = 0.001
    acc_est = np.zeros(len(z1))
    acc_est[0] = 0
    vel_est = np.zeros(len(z1))
    vel_est[0] = 0
    for i in range(len(z1)):
        if i > 0:
            dz3 = acc_estimator(r[i-1], z1[i-1], vel_est[i-1], acc_est[i-1])
            acc_est[i] = integrate(acc_est[i-1], dz3)
            vel_est[i] = integrate(vel_est[i-1], acc_est[i])
    return acc_est, vel_est

def acc_estimator(r, z1, z2, z3):
    # 可調整參數
    R = 30
    l1 = 25
    l2 = 15
    l3 = 10
    k1 = 20
    k2 = 1
    k3 = 10
    e = z1 - r
    dz3 = -R**3*(l1*e+sat(k1*e)+l2*z2/R+sat(k2*z2/R)+l3*z3/R**2+sat(k3*z3/R**2))
    return dz3

# ρ: 影響z1(t)，z2(t)和z3(t)的收斂和準確性特性
# 較小的ρ，有助於快速收斂，尤其是在暫態和高精度時；
# 但ρ太小可能會導致更多的干擾，特別是在考慮雜訊時z3(t)。
# R:
# 較大的R有助於加速暫態收斂，但損失了雜訊抑制能力。
# 𝑙_1 、 𝑘_1:
# 較大的𝑙_1 、 𝑘_1有助於暫態響應和高穩態估計精度。
# 𝑙_2 、 𝑘_2:確保最平滑的估計
# 較大的𝑙_2 、 𝑘_2有助於最平滑的估計，但對暫態有害。
# 𝑙_3 、 𝑘_3:
# 較大的𝑙_3 、 𝑘_3顯然對改善z3(t)的最平滑估計能力很重要，但暫態可能較慢。

def integrate(x, dx):
    dt = 0.001
    x = x + dx*dt
    return x

def sat(x):
    rho = 0.1
    sat = x/((rho+x**2)**0.5)
    return sat