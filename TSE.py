import numpy as np

def TSE2(yTemp):
    # 定义静态变量
    global dy, SamplingTime
    SamplingTime = 0.001
    # try:
    #     # 如果变量未定义，则进行初始化
    #     xTemp
    # except NameError:
    #     xTemp = np.zeros(2)
    #     dx = np.zeros(2)
    #     SamplingTime = 0.001
    
    # 更新历史值
    # 存储新值
    y = []
    dy = np.zeros(len(yTemp))
    dy[0] = 0
    for i in range(len(yTemp)):
        if i >=1 :
            dy[i] = yTemp[i] - yTemp[i-1]
            # xTemp[j] = xTemp[j + 1]
            # 计算y值
            temp = (dy[i] + (dy[i] - dy[i-1]) / 2) / SamplingTime
        else :
            temp = 0
        y.append(temp)
    return y

def TSE3(xTemp):
    # 定义静态变量
    global dx, SamplingTime
    SamplingTime = 0.001
    # try:
    #     # 如果变量未定义，则进行初始化
    #     xTemp
    # except NameError:
    #     xTemp = np.zeros(2)
    #     dx = np.zeros(3)
    #     SamplingTime = 0.001
    
    # 更新历史值
    # 存储新值
    x = []
    dx = np.zeros(len(xTemp))
    dx[0] = 0
    for i in range(len(xTemp)):
        if i >=2 :
            dx[i] = xTemp[i] - xTemp[i-1]
            # xTemp[j] = xTemp[j + 1]
            # 计算y值
            temp = (dx[i] + (dx[i] - dx[i-1]) / 2 + (dx[i] - 2 * dx[i-1] + dx[i-2]) / 8) / SamplingTime
        else :
            temp = 0
        x.append(temp)
    
    return x