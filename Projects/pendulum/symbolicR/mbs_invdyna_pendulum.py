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
#	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def invdyna(phi,s,tsim):
    Qq = phi  # compatibility with symbolic generation
    q = s.q
    qd = s.qd
    qdd = s.qdd
 
# Trigonometric functions

    S1 = sin(q[1])
    C1 = cos(q[1])
    S2 = sin(q[2])
    C2 = cos(q[2])
    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
 
# Augmented Joint Position Vectors

 
# Forward Kinematics

    BS91 = -qd[1]*qd[1]
    ALPHA11 = s.g[3]*S1
    ALPHA31 = -s.g[3]*C1
    OM12 = qd[1]*S2
    OM22 = qd[1]*C2
    OMp12 = qd[1]*qd[2]*C2+qdd[1]*S2
    OMp22 = -qd[1]*qd[2]*S2+qdd[1]*C2
    BS32 = qd[2]*OM12
    BS62 = qd[2]*OM22
    BS92 = -OM12*OM12-OM22*OM22
    BETA32 = BS32+OMp22
    BETA62 = BS62-OMp12
    ALPHA12 = C2*(ALPHA11+qdd[1]*s.dpt[3,3])
    ALPHA22 = -S2*(ALPHA11+qdd[1]*s.dpt[3,3])
    ALPHA32 = ALPHA31+BS91*s.dpt[3,3]
    ALPHA13 = ALPHA12+q[3]*BETA32+(2.0)*qd[3]*OM22
    ALPHA23 = ALPHA22+q[3]*BETA62-(2.0)*qd[3]*OM12
    ALPHA33 = qdd[3]+ALPHA32+q[3]*BS92
    BS94 = -qd[4]*qd[4]
    ALPHA14 = s.g[3]*S4
    ALPHA34 = -s.g[3]*C4
    OM25 = qd[4]+qd[5]
    OMp25 = qdd[4]+qdd[5]
    BS95 = -OM25*OM25
    ALPHA15 = C5*(ALPHA14+qdd[4]*s.dpt[3,6])-S5*(ALPHA34+BS94*s.dpt[3,6])
    ALPHA35 = C5*(ALPHA34+BS94*s.dpt[3,6])+S5*(ALPHA14+qdd[4]*s.dpt[3,6])
 
# Backward Dynamics

    Fs15 = -s.frc[1,5]+s.m[5]*(ALPHA15+OMp25*s.l[3,5])
    Fs35 = -s.frc[3,5]+s.m[5]*(ALPHA35+BS95*s.l[3,5])
    Cq25 = -s.trq[2,5]+s.In[5,5]*OMp25+Fs15*s.l[3,5]
    Fs14 = -s.frc[1,4]+s.m[4]*(ALPHA14+qdd[4]*s.l[3,4])
    Cq24 = -s.trq[2,4]+Cq25+qdd[4]*s.In[5,4]+Fs14*s.l[3,4]+s.dpt[3,6]*(Fs15*C5+Fs35*S5)
    Fs13 = -s.frc[1,3]+s.m[3]*ALPHA13
    Fs23 = -s.frc[2,3]+s.m[3]*ALPHA23
    Fs33 = -s.frc[3,3]+s.m[3]*ALPHA33
    Cq12 = -s.trq[1,3]-q[3]*Fs23
    Cq22 = -s.trq[2,3]+q[3]*Fs13
    Fs11 = -s.frc[1,1]+s.m[1]*(ALPHA11+qdd[1]*s.l[3,1])
    Cq21 = -s.trq[2,1]+qdd[1]*s.In[5,1]+Cq12*S2+Cq22*C2+Fs11*s.l[3,1]+s.dpt[3,3]*(Fs13*C2-Fs23*S2)
 
# Symbolic model output

    Qq[1] = Cq21
    Qq[2] = -s.trq[3,3]
    Qq[3] = Fs33
    Qq[4] = Cq24
    Qq[5] = Cq25

# Number of continuation lines = 0


