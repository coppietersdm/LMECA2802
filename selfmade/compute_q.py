from MBS_design import *

dt = 1e-4
MBS.integrate(10, dt)

for i in range(1,len(MBS.q.T)):
    plt.plot(MBS.t, MBS.q.T[i], label = 'q['+ str(i) + ']')
plt.grid(True)
plt.legend()
# plt.show()
