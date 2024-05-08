from MBS_design import *
import matplotlib.pyplot as plt

#MBS.integrate(5.0, 0.001)
MBS.read_q()


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
F1 = (MBS.spring(5,1,plot = True))
F2 = (MBS.spring(5,3,plot = True))
F3 = (MBS.spring(4,7,plot = True))

print(F1 + F2 + F3- np.array([0,0,9.81])*50*5)

F1 = (MBS.spring(6,0,plot = True))
F2 = (MBS.spring(6,2,plot = True))
F3 = (MBS.spring(7,4,plot = True))

print(F1 + F2 + F3- np.array([0,0,9.81])*50*5)

F1 = (MBS.spring(2,6,plot = True))
F2 = (MBS.spring(3,5,plot = True))

print(F1 + F2 - np.array([0,0,9.81])*50*5)
plt.show()
