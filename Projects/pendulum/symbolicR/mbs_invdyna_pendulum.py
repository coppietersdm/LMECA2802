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
#	==> Generation Date: Mon Apr  1 14:00:13 2024
#
#	==> Project name: pendulum
#
#	==> Number of joints: 3
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
    S3 = sin(q[3])
    C3 = cos(q[3])
 
# Augmented Joint Position Vectors

    Dz23 = q[2]+s.dpt[3,1]
 
# Augmented Joint Position Vectors

 
# Forward Kinematics

    BS91 = -qd[1]*qd[1]
    ALPHA11 = s.g[3]*S1
    ALPHA31 = -s.g[3]*C1
    BS92 = -qd[1]*qd[1]
    ALPHA12 = ALPHA11+(2.0)*qd[1]*qd[2]+qdd[1]*Dz23
    ALPHA32 = qdd[2]+ALPHA31+BS91*Dz23
    OM23 = qd[1]+qd[3]
    OMp23 = qdd[1]+qdd[3]
    BS93 = -OM23*OM23
    ALPHA13 = ALPHA12*C3-ALPHA32*S3
    ALPHA33 = ALPHA12*S3+ALPHA32*C3
 
# Backward Dynamics

    Fs13 = -s.frc[1,3]+s.m[3]*(ALPHA13+OMp23*s.l[3,3])
    Fs33 = -s.frc[3,3]+s.m[3]*(ALPHA33+BS93*s.l[3,3])
    Cq23 = -s.trq[2,3]+s.In[5,3]*OMp23+Fs13*s.l[3,3]
    Fs12 = -s.frc[1,2]+s.m[2]*(ALPHA12+qdd[1]*s.l[3,2])
    Fs32 = -s.frc[3,2]+s.m[2]*(ALPHA32+BS92*s.l[3,2])
    Fq12 = Fs12+Fs13*C3+Fs33*S3
    Fq32 = Fs32-Fs13*S3+Fs33*C3
    Cq22 = -s.trq[2,2]+Cq23+qdd[1]*s.In[5,2]+Fs12*s.l[3,2]
    Fs11 = -s.frc[1,1]+s.m[1]*(ALPHA11+qdd[1]*s.l[3,1])
    Cq21 = -s.trq[2,1]+Cq22+qdd[1]*s.In[5,1]+Fq12*Dz23+Fs11*s.l[3,1]
 
# Symbolic model output

    Qq[1] = Cq21
    Qq[2] = Fq32
    Qq[3] = Cq23

# Number of continuation lines = 0


