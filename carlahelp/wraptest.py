
import numpy as np
import math
import matplotlib.pyplot as plt

def angle_wrap(unwrapped):
    '''
    wraps angle between -pi to pi
    input & output in degrees
    '''
    wrapped = (unwrapped + 180) % 360
    if(wrapped<0):
        wrapped += 360
    return wrapped - 180


wrapped = []
for i in range(1440):
    i = i-720
    wave = angle_wrap(i)
    wrapped.append([i, wave])
wrapped = np.asarray(wrapped)
print(angle_wrap(0))

plt.plot(wrapped[:,0],wrapped[:,1])
plt.show()
