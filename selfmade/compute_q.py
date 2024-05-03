from MBS_design import *

dt = 1e-3
MBS.integrate(10.0, dt)

for i in range(1,len(MBS.q.T)):
    plt.plot(MBS.t, MBS.q.T[i], label = 'q['+ str(i) + ']')
print(MBS.t)
plt.grid(True)
plt.legend()
plt.show()
