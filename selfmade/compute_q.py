from MBS_design import *

MBS.integrate(20.0, 0.001)
t = np.linspace(0,len(MBS.q)/1000, len(MBS.q))
plt.plot(t, MBS.q.T[0])
plt.plot(t, MBS.q.T[1])
plt.show()
