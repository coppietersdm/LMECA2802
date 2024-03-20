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
    S2 = sin(q[2])
    C2 = cos(q[2])
    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
 
# Forward Kinematics

    BS91 = -qd[1]*qd[1]
    AF11 = s.g[3]*S1
    AF31 = -s.g[3]*C1
    OM12 = qd[1]*S2
    OM22 = qd[1]*C2
    OA12 = qd[1]*qd[2]*C2
    OA22 = -qd[1]*qd[2]*S2
    BS32 = qd[2]*OM12
    BS62 = qd[2]*OM22
    BS92 = -OM12*OM12-OM22*OM22
    BEF32 = BS32+OA22
    BEF62 = BS62-OA12
    AF12 = AF11*C2
    AF22 = -AF11*S2
    AF32 = AF31+BS91*s.dpt[3,3]
    AM12_1 = s.dpt[3,3]*C2
    AM22_1 = -s.dpt[3,3]*S2
    AF13 = AF12+q[3]*BEF32+(2.0)*qd[3]*OM22
    AF23 = AF22+q[3]*BEF62-(2.0)*qd[3]*OM12
    AF33 = AF32+q[3]*BS92
    AM13_1 = AM12_1+q[3]*C2
    AM23_1 = AM22_1-q[3]*S2
    BS94 = -qd[4]*qd[4]
    AF14 = s.g[3]*S4
    AF34 = -s.g[3]*C4
    OM25 = qd[4]+qd[5]
    BS95 = -OM25*OM25
    AF15 = AF14*C5-S5*(AF34+BS94*s.dpt[3,6])
    AF35 = AF14*S5+C5*(AF34+BS94*s.dpt[3,6])
    AM15_4 = s.dpt[3,6]*C5
    AM35_4 = s.dpt[3,6]*S5
 
# Backward Dynamics

    FA15 = -s.frc[1,5]+s.m[5]*AF15
    FA35 = -s.frc[3,5]+s.m[5]*(AF35+BS95*s.l[3,5])
    CF25 = -s.trq[2,5]+FA15*s.l[3,5]
    FB15_4 = s.m[5]*(AM15_4+s.l[3,5])
    FB35_4 = s.m[5]*AM35_4
    CM25_4 = s.In[5,5]+FB15_4*s.l[3,5]
    FB15_5 = s.m[5]*s.l[3,5]
    CM25_5 = s.In[5,5]+FB15_5*s.l[3,5]
    FA14 = -s.frc[1,4]+s.m[4]*AF14
    CF24 = -s.trq[2,4]+CF25+FA14*s.l[3,4]+s.dpt[3,6]*(FA15*C5+FA35*S5)
    FB14_4 = s.m[4]*s.l[3,4]
    CM24_4 = s.In[5,4]+CM25_4+FB14_4*s.l[3,4]+s.dpt[3,6]*(FB15_4*C5+FB35_4*S5)
    FA13 = -s.frc[1,3]+s.m[3]*AF13
    FA23 = -s.frc[2,3]+s.m[3]*AF23
    FA33 = -s.frc[3,3]+s.m[3]*AF33
    FB13_1 = s.m[3]*AM13_1
    FB23_1 = s.m[3]*AM23_1
    CF12 = -s.trq[1,3]-q[3]*FA23
    CF22 = -s.trq[2,3]+q[3]*FA13
    CM12_1 = -q[3]*FB23_1
    CM22_1 = q[3]*FB13_1
    FA11 = -s.frc[1,1]+s.m[1]*AF11
    CF21 = -s.trq[2,1]+CF12*S2+CF22*C2+FA11*s.l[3,1]+s.dpt[3,3]*(FA13*C2-FA23*S2)
    FB11_1 = s.m[1]*s.l[3,1]
    CM21_1 = s.In[5,1]+CM12_1*S2+CM22_1*C2+FB11_1*s.l[3,1]+s.dpt[3,3]*(FB13_1*C2-FB23_1*S2)
 
# Symbolic model output

    c[1] = CF21
    c[2] = -s.trq[3,3]
    c[3] = FA33
    c[4] = CF24
    c[5] = CF25
    M[1,1] = CM21_1
    M[3,3] = s.m[3]
    M[4,4] = CM24_4
    M[4,5] = CM25_4
    M[5,4] = CM25_4
    M[5,5] = CM25_5

# Number of continuation lines = 0


