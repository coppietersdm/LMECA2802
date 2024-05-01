from MBS_class import *

# draw your own Multi body system

MBS = mbs()

# MBS.add_body(MBS.base,"T1",   q0=np.array([0.0,0.0,0.0]))
# MBS.add_body(MBS.bodies[-1],"T3",   q0=np.array([0.0,0.0,0.0]))
# MBS.add_body(MBS.bodies[-1],"R2", m=1.0, I=I2,   q0=np.array([0.0,1.0,0.0]), anchor_points=[np.array([0,0,-1.0])])

"""
MBS.add_body(MBS.base,      "R2", m=1.0, I=I2, dii=np.array([0.0,0.0,-1.0]),    dhi=np.array([0.0,0.0,5.0]),   q0=np.array([1.0,0,0]))
MBS.add_body(MBS.bodies[-1],"T3", m=1.0, I=I2, dii=np.array([0.0,0.0,-1.0]),    dhi=np.array([0.0,0.0,-2.0]),   Q = lambda t,q,qd:-10.0*q)
MBS.add_body(MBS.bodies[-1],"R2", m=1.0, I=I2, dii=np.array([0.0,0.0,-1.0]),    dhi=np.array([0.0,0.0,0.0]),    q0=np.array([0.0,0.0,0.0]))


"""
MBS.add_body(MBS.base,          joint = 'T3', q0 = np.array([0.0,0.0,0.0]))
MBS.add_body(MBS.bodies[-1],    joint = 'T1', q0 = np.array([0.0,0.0,0.0]))
MBS.add_body(MBS.bodies[-1], m=50.0, I = I2*50.0, joint = 'R2', q0 = np.array([0.0,0.0,0.0]))

MBS.add_body(MBS.bodies[-1], m=50.0, I = I2*50.0, joint = 'R2',
             dii = np.array([1.0,0.0,0.0]), dhi = np.array([1.0,0.0,0.0]),
             q0 = np.array([0.0,0.0,0.0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd)
MBS.add_body(MBS.bodies[-1], m=50.0, I = I2*50.0, joint = 'R2',
             dii = np.array([1.0,0.0,0.0]), dhi = np.array([2.0,0.0,0.0]),
             q0 = np.array([0.0,0.0,0.0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd, 
             anchor_points=[np.array([2.0,0.0,0.0])])
MBS.add_body(MBS.bodies[-3], m=50.0, I = I2*50.0, joint = 'R2',
             dii = np.array([1.0,0.0,0.0]), dhi = np.array([-1.0,0.0,0.0]),
             q0 = np.array([np.pi,0,0]), Q = lambda t,q,qd: -8.5e6*(q-np.pi) - 1.0e4*qd)
MBS.add_body(MBS.bodies[-1], m=50.0, I = I2*50, joint = 'R2',
             dii = np.array([1.0,0.0,0.0]), dhi = np.array([2.0,0.0,0.0]),
             q0 = np.array([0.0,0.0,0.0]), Q = lambda t,q,qd: -8.5e6*q - 1.0e4*qd,
             anchor_points=[np.array([2.0,0.0,0.0])])

MBS.add_body(MBS.base,          joint = 'T3', q0 = np.array([10*np.sqrt(3)/4,0,0]))
MBS.add_body(MBS.bodies[-1],    joint = 'T1', q0 = np.array([10/4.0,0,0]))
MBS.add_body(MBS.bodies[-1], m=50.0, I = I2*50, joint = 'R2', q0 = np.array([np.pi/3,0,0]))

MBS.add_body(MBS.bodies[-1], m=50.0, I = I2*50, joint = 'R2',
             dii = np.array([1.0,0,0]), dhi = np.array([1.0,0.0,0.0]),
             q0 = np.array([0.0,0,0]), Q = lambda t,q,qd: -8.5e6*q - 1e4*qd)
MBS.add_body(MBS.bodies[-1], m=50.0, I = I2*50, joint = 'R2',
             dii = np.array([1.0,0,0]), dhi = np.array([2.0,0.0,0.0]),
             q0 = np.array([0.0,0,0]), Q = lambda t,q,qd: -8.5e6*q - 1e4*qd, 
             anchor_points=[np.array([2.0,0,0])])
MBS.add_body(MBS.bodies[-3], m=50.0, I = I2*50, joint = 'R2',
             dii = np.array([1.0,0,0]), dhi = np.array([-1.0,0.0,0.0]),
             q0 = np.array([np.pi,0,0]), Q = lambda t,q,qd: -8.5e6*(q-np.pi) - 1e4*qd)
MBS.add_body(MBS.bodies[-1], m=50.0, I = I2*50, joint = 'R2',
             dii = np.array([1.0,0,0]), dhi = np.array([2.0,0.0,0.0]),
             q0 = np.array([0.0,0,0]), Q = lambda t,q,qd: -8.5e6*q - 1e4*qd,
             anchor_points=[np.array([2.0,0,0])])

MBS.add_body(MBS.base,          joint = 'T3', q0 = np.array([10*np.sqrt(3)/4,0,0]))
MBS.add_body(MBS.bodies[-1],    joint = 'T1', q0 = np.array([-10/4.0,0,0]))
MBS.add_body(MBS.bodies[-1], m=50.0, I = I2*50, joint = 'R2', q0 = np.array([-np.pi/3,0,0]))

MBS.add_body(MBS.bodies[-1], m=50.0, I = I2*50, joint = 'R2',
             dii = np.array([1.0,0,0]), dhi = np.array([1.0,0.0,0.0]),
             q0 = np.array([0.0,0,0]), Q = lambda t,q,qd: -8.5e6*q - 1e4*qd)
MBS.add_body(MBS.bodies[-1], m=50.0, I = I2*50, joint = 'R2',
             dii = np.array([1.0,0,0]), dhi = np.array([2.0,0.0,0.0]),
             q0 = np.array([0.0,0,0]), Q = lambda t,q,qd: -8.5e6*q - 1e4*qd, 
             anchor_points=[np.array([2.0,0,0])])
MBS.add_body(MBS.bodies[-3], m=50.0, I = I2*50, joint = 'R2',
             dii = np.array([1.0,0,0]), dhi = np.array([-1.0,0.0,0.0]),
             q0 = np.array([np.pi,0,0]), Q = lambda t,q,qd: -8.5e6*(q-np.pi) - 1e4*qd)
MBS.add_body(MBS.bodies[-1], m=50.0, I = I2*50, joint = 'R2',
             dii = np.array([1.0,0,0]), dhi = np.array([2.0,0.0,0.0]),
             q0 = np.array([0.0,0,0]), Q = lambda t,q,qd: -8.5e6*q - 1e4*qd,
             anchor_points=[np.array([2.0,0,0])])
             
