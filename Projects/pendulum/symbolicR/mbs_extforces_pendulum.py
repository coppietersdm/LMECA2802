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
#	==> Function: F19 - External Forces
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos, sqrt
from numpy import zeros

def extforces(frc, trq, s, tsim):
    q = s.q
    qd = s.qd
    qdd = s.qdd
    frc = s.frc
    trq = s.trq
    PxF1 = zeros(4)
    RxF1 = zeros((4, 4))
    VxF1 = zeros(4)
    OMxF1 = zeros(4)
    AxF1 = zeros(4)
    OMPxF1 = zeros(4)

 
# Trigonometric functions

    S1 = sin(q[1])
    C1 = cos(q[1])
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics

    RLcp1_12 = s.dpt[3,5]*S1
    RLcp1_32 = s.dpt[3,5]*C1
    ORcp1_12 = qd[1]*RLcp1_32
    ORcp1_32 = -qd[1]*RLcp1_12
    ACcp1_12 = qd[1]*ORcp1_32+qdd[1]*RLcp1_32
    ACcp1_32 = -qd[1]*ORcp1_12-qdd[1]*RLcp1_12
    PxF1[1] = RLcp1_12
    PxF1[2] = 0
    PxF1[3] = RLcp1_32
    RxF1[1,1] = C1
    RxF1[1,3] = -S1
    RxF1[2,2] = (1.0)
    RxF1[3,1] = S1
    RxF1[3,3] = C1
    VxF1[1] = ORcp1_12
    VxF1[2] = 0
    VxF1[3] = ORcp1_32
    OMxF1[1] = 0
    OMxF1[2] = qd[1]
    OMxF1[3] = 0
    AxF1[1] = ACcp1_12
    AxF1[2] = 0
    AxF1[3] = ACcp1_32
    OMPxF1[1] = 0
    OMPxF1[2] = qdd[1]
    OMPxF1[3] = 0
 
# Sensor Forces 

    SWr1 = s.user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1)
    xfrc11 = RxF1[1,1]*SWr1[1]+RxF1[1,3]*SWr1[3]
    xfrc21 = RxF1[2,2]*SWr1[2]
    xfrc31 = RxF1[3,1]*SWr1[1]+RxF1[3,3]*SWr1[3]
    xtrq11 = RxF1[1,1]*SWr1[4]+RxF1[1,3]*SWr1[6]
    xtrq21 = RxF1[2,2]*SWr1[5]
    xtrq31 = RxF1[3,1]*SWr1[4]+RxF1[3,3]*SWr1[6]
    trqext_1_1_0 = xtrq11-xfrc21*(SWr1[9]-s.l[3,1])+xfrc31*SWr1[8]
    trqext_2_1_0 = xtrq21+xfrc11*(SWr1[9]-s.l[3,1])-xfrc31*SWr1[7]
    trqext_3_1_0 = xtrq31-xfrc11*SWr1[8]+xfrc21*SWr1[7]
 
# Symbolic model output

    frc[1,1] = s.frc[1,1]+xfrc11
    frc[2,1] = s.frc[2,1]+xfrc21
    frc[3,1] = s.frc[3,1]+xfrc31
    trq[1,1] = s.trq[1,1]+trqext_1_1_0
    trq[2,1] = s.trq[2,1]+trqext_2_1_0
    trq[3,1] = s.trq[3,1]+trqext_3_1_0

# Number of continuation lines = 0


