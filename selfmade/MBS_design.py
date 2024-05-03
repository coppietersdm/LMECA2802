from MBS_class import *

# draw your own Multi body system

MBS = mbs()

# MBS.add_body(MBS.base,"T1",   q0=np.array([0.0,0.0,0.0]))
# MBS.add_body(MBS.bodies[-1],"T3",   q0=np.array([0.0,0.0,0.0]))
# MBS.add_body(MBS.bodies[-1],"R2", m=1.0, I=I1,   q0=np.array([0.0,1.0,0.0]), anchor_points=[np.array([0,0,-1.0])])

"""
MBS.add_body(MBS.base,      "R2", m=1.0, I=I1, dii=np.array([0.0,0.0,-1.0]),    dhi=np.array([0.0,0.0,5.0]),   q0=np.array([1.0,0,0]))
MBS.add_body(MBS.bodies[-1],"T3", m=1.0, I=I1, dii=np.array([0.0,0.0,-1.0]),    dhi=np.array([0.0,0.0,-2.0]),   Q = lambda t,q,qd:-10.0*q)
MBS.add_body(MBS.bodies[-1],"R2", m=1.0, I=I1, dii=np.array([0.0,0.0,-1.0]),    dhi=np.array([0.0,0.0,0.0]),    q0=np.array([0.0,0.0,0.0]))

"""


MBS.add_body(MBS.base,                              joint = 'T2', q0 = np.array([0.0,0.0,0.0]))
MBS.add_body(MBS.bodies[-1],                        joint = 'T3', q0 = np.array([0.0,0.0,0.0]))
MBS.add_body(MBS.bodies[-1], m=50.0, I = I1*50.0,   joint = 'R1', q0 = np.array([0.0,0.0,0.0]))

MBS.add_body(MBS.bodies[-1], m=50.0, I = I1*50.0,   joint = 'R1',
             dii = np.array([0.0,1.0,0.0]), dhi = np.array([0.0,1.0,0.0]),
             q0 = np.array([0.0,0.0,0.0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd)

MBS.add_body(MBS.bodies[-1], m=50.0, I = I1*50.0,   joint = 'R1',
             dii = np.array([0.0,1.0,0.0]), dhi = np.array([0.0,2.0,0.0]),
             q0 = np.array([0.0,0.0,0.0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd, 
             anchor_points=[np.array([0.0,2.0,0.0])])
MBS.add_body(MBS.bodies[-3], m=50.0, I = I1*50.0,   joint = 'R1',
             dii = np.array([0.0,-1.0,0.0]), dhi = np.array([0.0,-1.0,0.0]),
             q0 = np.array([0.0,0.0,0.0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd)


MBS.add_body(MBS.bodies[-1], m=50.0, I = I1*50.0,   joint = 'R1',
             dii = np.array([0.0,-1.0,0.0]), dhi = np.array([0.0,-2.0,0.0]),
             q0 = np.array([0.0,0.0,0.0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd,
             anchor_points=[np.array([0.0,-2.0,0.0])])


MBS.add_body(MBS.base,                              joint = 'T2', q0 = np.array([-10/4.0,0,0]))
MBS.add_body(MBS.bodies[-1],                        joint = 'T3', q0 = np.array([-10*np.sqrt(3)/4,0,0]))
MBS.add_body(MBS.bodies[-1], m=50.0, I = I1*50.0,   joint = 'R1', q0 = np.array([-np.pi/3,0,0]))


MBS.add_body(MBS.bodies[-1], m=50.0, I = I1*50.0,   joint = 'R1',
             dii = np.array([0.0,1.0,0]), dhi = np.array([0.0,1.0,0.0]),
             q0 = np.array([0.0,0,0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd)
MBS.add_body(MBS.bodies[-1], m=50.0, I = I1*50.0,   joint = 'R1',
             dii = np.array([0.0,1.0,0]), dhi = np.array([0.0,2.0,0.0]),
             q0 = np.array([0.0,0.0,0.0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd, 
             anchor_points=[np.array([0,2.0,0])])
MBS.add_body(MBS.bodies[-3], m=50.0, I = I1*50.0,   joint = 'R1',
             dii = np.array([0.0,-1.0,0]), dhi = np.array([0.0,-1.0,0.0]),
             q0 = np.array([0.0,0,0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd)
MBS.add_body(MBS.bodies[-1], m=50.0, I = I1*50.0,   joint = 'R1',
             dii = np.array([0.0,-1.0,0]), dhi = np.array([0.0,-2.0,0.0]),
             q0 = np.array([0.0,0,0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd,
             anchor_points=[np.array([0,-2.0,0])])

MBS.add_body(MBS.base,                              joint = 'T2', q0 = np.array([10/4.0,0,0]))
MBS.add_body(MBS.bodies[-1],                        joint = 'T3', q0 = np.array([-10*np.sqrt(3)/4,0,0]))
MBS.add_body(MBS.bodies[-1], m=50.0, I = I1*50.0,   joint = 'R1', q0 = np.array([np.pi/3,0,0]))


MBS.add_body(MBS.bodies[-1], m=50.0, I = I1*50.0,   joint = 'R1',
             dii = np.array([0.0,1.0,0]), dhi = np.array([0.0,1.0,0.0]),
             q0 = np.array([0.0,0,0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd)
MBS.add_body(MBS.bodies[-1], m=50.0, I = I1*50.0,   joint = 'R1',
             dii = np.array([0.0,1.0,0]), dhi = np.array([0.0,2.0,0.0]),
             q0 = np.array([0.0,0.0,0.0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd, 
             anchor_points=[np.array([0,2.0,0])])
MBS.add_body(MBS.bodies[-3], m=50.0, I = I1*50.0,   joint = 'R1',
             dii = np.array([0.0,-1.0,0]), dhi = np.array([0.0,-1.0,0.0]),
             q0 = np.array([0.0,0,0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd)
MBS.add_body(MBS.bodies[-1], m=50.0, I = I1*50.0,   joint = 'R1',
             dii = np.array([0.0,-1.0,0]), dhi = np.array([0.0,-2.0,0.0]),
             q0 = np.array([0.0,0,0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd,
             anchor_points=[np.array([0,-2.0,0])])

             
