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
#	==> Function: F1 - Recursive Direct Dynamics of tree-like MBS
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def dirdyna(M, c, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S1 = sin(q[1])
    C1 = cos(q[1])
    S3 = sin(q[3])
    C3 = cos(q[3])
 
# Augmented Joint Position Vectors

    Dz23 = q[2]+s.dpt[3,1]
 
# Forward Kinematics

    BS91 = -qd[1]*qd[1]
    AF11 = s.g[3]*S1
    AF31 = -s.g[3]*C1
    BS92 = -qd[1]*qd[1]
    AF12 = AF11+(2.0)*qd[1]*qd[2]
    AF32 = AF31+BS91*Dz23
    OM23 = qd[1]+qd[3]
    BS93 = -OM23*OM23
    AF13 = AF12*C3-AF32*S3
    AF33 = AF12*S3+AF32*C3
    AM13_1 = Dz23*C3
    AM33_1 = Dz23*S3
 
# Backward Dynamics

    FA13 = -s.frc[1,3]+s.m[3]*AF13
    FA33 = -s.frc[3,3]+s.m[3]*(AF33+BS93*s.l[3,3])
    CF23 = -s.trq[2,3]+FA13*s.l[3,3]
    FB13_1 = s.m[3]*(AM13_1+s.l[3,3])
    FB33_1 = s.m[3]*AM33_1
    CM23_1 = s.In[5,3]+FB13_1*s.l[3,3]
    FB13_2 = -s.m[3]*S3
    FB33_2 = s.m[3]*C3
    CM23_2 = FB13_2*s.l[3,3]
    FB13_3 = s.m[3]*s.l[3,3]
    CM23_3 = s.In[5,3]+FB13_3*s.l[3,3]
    FA12 = -s.frc[1,2]+s.m[2]*AF12
    FA32 = -s.frc[3,2]+s.m[2]*(AF32+BS92*s.l[3,2])
    FF12 = FA12+FA13*C3+FA33*S3
    FF32 = FA32-FA13*S3+FA33*C3
    CF22 = -s.trq[2,2]+CF23+FA12*s.l[3,2]
    FB12_1 = s.m[2]*(s.l[3,2]+Dz23)
    FM12_1 = FB12_1+FB13_1*C3+FB33_1*S3
    FM32_1 = -FB13_1*S3+FB33_1*C3
    CM22_1 = s.In[5,2]+CM23_1+FB12_1*s.l[3,2]
    FM32_2 = s.m[2]-FB13_2*S3+FB33_2*C3
    FA11 = -s.frc[1,1]+s.m[1]*AF11
    CF21 = -s.trq[2,1]+CF22+FA11*s.l[3,1]+FF12*Dz23
    FB11_1 = s.m[1]*s.l[3,1]
    CM21_1 = s.In[5,1]+CM22_1+FB11_1*s.l[3,1]+FM12_1*Dz23
 
# Symbolic model output

    c[1] = CF21
    c[2] = FF32
    c[3] = CF23
    M[1,1] = CM21_1
    M[1,2] = FM32_1
    M[1,3] = CM23_1
    M[2,1] = FM32_1
    M[2,2] = FM32_2
    M[2,3] = CM23_2
    M[3,1] = CM23_1
    M[3,2] = CM23_2
    M[3,3] = CM23_3

# Number of continuation lines = 0


