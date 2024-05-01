import numpy as np
from matplotlib import pyplot as plt

q_Robotran = np.loadtxt("/home/matthieu/Documents/LMECA2802/Projects/pendulum/resultsR/dirdyn_q.res")
print(q_Robotran[0])
q_selfmade = np.loadtxt("dirdyn_q.res")

plt.plot(q_Robotran.T[0],q_Robotran.T[1] - q_selfmade.T[1], label = 'q[1]')
plt.plot(q_Robotran.T[0],q_Robotran.T[2] - q_selfmade.T[2], label = 'q[2]')
plt.plot(q_Robotran.T[0],q_Robotran.T[3] - q_selfmade.T[3], label = 'q[3]')
plt.legend()
plt.show()