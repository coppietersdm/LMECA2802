from MBS_class import *

# draw your own Multi body system



MBS = mbs()

MBS.add_body(MBS.base,      "R2", m=1.0, I=I2, dii=np.array([0.0,0.0,-1.0]),    dhi=np.array([0.0,0.0,-2.0]),   q0=np.array([1.0,0,0]))
MBS.add_body(MBS.bodies[-1],"T3", m=1.0, I=I2, dii=np.array([0.0,0.0,-1.0]),    dhi=np.array([0.0,0.0,-2.0]),   Q = lambda t,q,qd:-100.0*q)
MBS.add_body(MBS.bodies[-1],"R2", m=1.0, I=I2, dii=np.array([0.0,0.0,-1.0]),    dhi=np.array([0.0,0.0,0.0]),    q0=np.array([0.0,0.0,0.0]))