from MBS_design import *
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#MBS.integrate(5.0, 0.001)
MBS.read_q()


MBS.plot()
plt.show()

MBS.set_q(MBS.q[0][1:])
MBS.plot()
plt.xlim(-0.2,0.2)
plt.ylim(8.5,8.7)
plt.show()

MBS.set_q(MBS.q[500][1:])
MBS.plot()
plt.xlim(-0.2,0.2)
plt.ylim(8.5,8.7)
plt.show()

MBS.set_q(MBS.q[-1][1:])
MBS.plot()
plt.xlim(-0.2,0.2)
plt.ylim(8.5,8.7)

plt.show()
