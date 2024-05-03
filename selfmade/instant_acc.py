from MBS_design import *

y = np.zeros(1+2*len(MBS.bodies))

def derivative_test(t,y):
    MBS.set_q(MBS.get_q())
    MBS.second_derivative(t)
    return np.array(list(MBS.get_qd()) + list(MBS.get_qdd()))

derivative_test(0,y)
print(MBS.get_q())
print(MBS.get_qd())
print(MBS.get_qdd())
