import numpy as np
from matplotlib import pyplot as plt # type: ignore

data = np.loadtxt('tmp.csv', delimiter=',')

print(data[1])

plt.figure()
plt.plot(data[:,0], data[:,1], label='Position')
plt.plot(data[:,0], data[:,2], label='Velocity')
plt.plot(data[:,0], data[:,3], label='Acceleration')
plt.plot(data[:,0], data[:,4], label='Jerk')
plt.legend()
plt.show()