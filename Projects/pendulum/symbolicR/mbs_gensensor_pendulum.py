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
#	==> Function: F6 - Sensors Kinematics
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos, sqrt

def sensor(sens, s, isens):
  q = s.q
  qd = s.qd
  qdd = s.qdd

  dpt = s.dpt
 
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

 
# Sensor Kinematics


  if (isens == 1): 

    sens.P[1] = 0
    sens.P[2] = 0
    sens.P[3] = 0
    sens.R[1,1] = C1
    sens.R[1,3] = -S1
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S1
    sens.R[3,3] = C1
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = qd[1]
    sens.OM[3] = 0
    sens.J[5,1] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[1]
    sens.OMP[3] = 0

  if (isens == 2): 

    ROcp2_12 = C1*C2
    ROcp2_32 = -S1*C2
    ROcp2_42 = -C1*S2
    ROcp2_62 = S1*S2
    RLcp2_12 = s.dpt[3,3]*S1
    RLcp2_32 = s.dpt[3,3]*C1
    OMcp2_12 = qd[2]*S1
    OMcp2_32 = qd[2]*C1
    ORcp2_12 = RLcp2_32*qd[1]
    ORcp2_32 = -RLcp2_12*qd[1]
    OPcp2_12 = qdd[2]*S1+qd[1]*qd[2]*C1
    OPcp2_32 = qdd[2]*C1-qd[1]*qd[2]*S1
    ACcp2_12 = ORcp2_32*qd[1]+RLcp2_32*qdd[1]
    ACcp2_32 = -ORcp2_12*qd[1]-RLcp2_12*qdd[1]
    sens.P[1] = RLcp2_12
    sens.P[2] = 0
    sens.P[3] = RLcp2_32
    sens.R[1,1] = ROcp2_12
    sens.R[1,2] = S2
    sens.R[1,3] = ROcp2_32
    sens.R[2,1] = ROcp2_42
    sens.R[2,2] = C2
    sens.R[2,3] = ROcp2_62
    sens.R[3,1] = S1
    sens.R[3,3] = C1
    sens.V[1] = ORcp2_12
    sens.V[2] = 0
    sens.V[3] = ORcp2_32
    sens.OM[1] = OMcp2_12
    sens.OM[2] = qd[1]
    sens.OM[3] = OMcp2_32
    sens.J[1,1] = RLcp2_32
    sens.J[3,1] = -RLcp2_12
    sens.J[4,2] = S1
    sens.J[5,1] = (1.0)
    sens.J[6,2] = C1
    sens.A[1] = ACcp2_12
    sens.A[2] = 0
    sens.A[3] = ACcp2_32
    sens.OMP[1] = OPcp2_12
    sens.OMP[2] = qdd[1]
    sens.OMP[3] = OPcp2_32

  if (isens == 3): 

    ROcp3_12 = C1*C2
    ROcp3_32 = -S1*C2
    ROcp3_42 = -C1*S2
    ROcp3_62 = S1*S2
    RLcp3_12 = s.dpt[3,3]*S1
    RLcp3_32 = s.dpt[3,3]*C1
    OMcp3_12 = qd[2]*S1
    OMcp3_32 = qd[2]*C1
    ORcp3_12 = RLcp3_32*qd[1]
    ORcp3_32 = -RLcp3_12*qd[1]
    OPcp3_12 = qdd[2]*S1+qd[1]*qd[2]*C1
    OPcp3_32 = qdd[2]*C1-qd[1]*qd[2]*S1
    ACcp3_12 = ORcp3_32*qd[1]+RLcp3_32*qdd[1]
    ACcp3_32 = -ORcp3_12*qd[1]-RLcp3_12*qdd[1]
    RLcp3_13 = q[3]*S1
    RLcp3_33 = q[3]*C1
    POcp3_13 = RLcp3_12+RLcp3_13
    POcp3_33 = RLcp3_32+RLcp3_33
    JTcp3_13_1 = RLcp3_32+RLcp3_33
    JTcp3_33_1 = -RLcp3_12-RLcp3_13
    JTcp3_23_2 = RLcp3_13*C1-RLcp3_33*S1
    ORcp3_13 = RLcp3_33*qd[1]
    ORcp3_23 = -OMcp3_12*RLcp3_33+OMcp3_32*RLcp3_13
    ORcp3_33 = -RLcp3_13*qd[1]
    VIcp3_13 = ORcp3_12+ORcp3_13+qd[3]*S1
    VIcp3_33 = ORcp3_32+ORcp3_33+qd[3]*C1
    ACcp3_13 = ACcp3_12-OMcp3_32*ORcp3_23+ORcp3_33*qd[1]+RLcp3_33*qdd[1]+qdd[3]*S1+(2.0)*qd[1]*qd[3]*C1
    ACcp3_23 = -OMcp3_12*ORcp3_33+OMcp3_32*ORcp3_13-OPcp3_12*RLcp3_33+OPcp3_32*RLcp3_13+(2.0)*qd[3]*(-OMcp3_12*C1+OMcp3_32*S1)
    ACcp3_33 = ACcp3_32+OMcp3_12*ORcp3_23-ORcp3_13*qd[1]-RLcp3_13*qdd[1]+qdd[3]*C1-(2.0)*qd[1]*qd[3]*S1
    sens.P[1] = POcp3_13
    sens.P[2] = 0
    sens.P[3] = POcp3_33
    sens.R[1,1] = ROcp3_12
    sens.R[1,2] = S2
    sens.R[1,3] = ROcp3_32
    sens.R[2,1] = ROcp3_42
    sens.R[2,2] = C2
    sens.R[2,3] = ROcp3_62
    sens.R[3,1] = S1
    sens.R[3,3] = C1
    sens.V[1] = VIcp3_13
    sens.V[2] = ORcp3_23
    sens.V[3] = VIcp3_33
    sens.OM[1] = OMcp3_12
    sens.OM[2] = qd[1]
    sens.OM[3] = OMcp3_32
    sens.J[1,1] = JTcp3_13_1
    sens.J[1,3] = S1
    sens.J[2,2] = JTcp3_23_2
    sens.J[3,1] = JTcp3_33_1
    sens.J[3,3] = C1
    sens.J[4,2] = S1
    sens.J[5,1] = (1.0)
    sens.J[6,2] = C1
    sens.A[1] = ACcp3_13
    sens.A[2] = ACcp3_23
    sens.A[3] = ACcp3_33
    sens.OMP[1] = OPcp3_12
    sens.OMP[2] = qdd[1]
    sens.OMP[3] = OPcp3_32

  if (isens == 4): 

    sens.P[1] = s.dpt[1,2]
    sens.P[2] = 0
    sens.P[3] = 0
    sens.R[1,1] = C4
    sens.R[1,3] = -S4
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S4
    sens.R[3,3] = C4
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = qd[4]
    sens.OM[3] = 0
    sens.J[5,4] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[4]
    sens.OMP[3] = 0

  if (isens == 5): 

    ROcp5_15 = C4*C5-S4*S5
    ROcp5_35 = -C4*S5-S4*C5
    ROcp5_75 = C4*S5+S4*C5
    ROcp5_95 = C4*C5-S4*S5
    RLcp5_12 = s.dpt[3,6]*S4
    RLcp5_32 = s.dpt[3,6]*C4
    POcp5_12 = RLcp5_12+s.dpt[1,2]
    OMcp5_22 = qd[4]+qd[5]
    ORcp5_12 = RLcp5_32*qd[4]
    ORcp5_32 = -RLcp5_12*qd[4]
    OPcp5_22 = qdd[4]+qdd[5]
    ACcp5_12 = ORcp5_32*qd[4]+RLcp5_32*qdd[4]
    ACcp5_32 = -ORcp5_12*qd[4]-RLcp5_12*qdd[4]
    sens.P[1] = POcp5_12
    sens.P[2] = 0
    sens.P[3] = RLcp5_32
    sens.R[1,1] = ROcp5_15
    sens.R[1,3] = ROcp5_35
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp5_75
    sens.R[3,3] = ROcp5_95
    sens.V[1] = ORcp5_12
    sens.V[2] = 0
    sens.V[3] = ORcp5_32
    sens.OM[1] = 0
    sens.OM[2] = OMcp5_22
    sens.OM[3] = 0
    sens.J[1,4] = RLcp5_32
    sens.J[3,4] = -RLcp5_12
    sens.J[5,4] = (1.0)
    sens.J[5,5] = (1.0)
    sens.A[1] = ACcp5_12
    sens.A[2] = 0
    sens.A[3] = ACcp5_32
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp5_22
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


