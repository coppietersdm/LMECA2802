#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Fri Mar 15 17:11:52 2024
#
#	==> Project name: pendulum
#
#	==> Number of joints: 5
#
#	==> Function: F7 - Link Forces (1D)
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos, sqrt

def link(frc, trq, Flink, Z, Zd, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S1 = sin(q[1])
    C1 = cos(q[1])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk2_12 = s.dpt[3,4]*S1
    RLlnk2_32 = s.dpt[3,4]*C1
    ORlnk2_12 = qd[1]*RLlnk2_32
    ORlnk2_32 = -qd[1]*RLlnk2_12
    Plnk11 = RLlnk2_12-s.dpt[1,1]
    PPlnk1 = Plnk11*Plnk11+RLlnk2_32*RLlnk2_32
    Z1 = sqrt(PPlnk1)
    e11 = Plnk11/Z1
    e31 = RLlnk2_32/Z1
    Zd1 = ORlnk2_12*e11+ORlnk2_32*e31

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
 
# Link Dynamics: forces projection on body-fixed frames

    fSlnk11 = Flink1*(e11*C1-e31*S1)
    fSlnk31 = Flink1*(e11*S1+e31*C1)
    trqlnk1_1_2 = -fSlnk11*(s.dpt[3,4]-s.l[3,1])
 
# Symbolic model output

    frc[1,1] = s.frc[1,1]-fSlnk11
    frc[3,1] = s.frc[3,1]-fSlnk31
    trq[2,1] = s.trq[2,1]+trqlnk1_1_2
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1

# Number of continuation lines = 0


