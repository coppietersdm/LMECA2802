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
  S3 = sin(q[3])
  C3 = cos(q[3])
 
# Augmented Joint Position Vectors

  Dz23 = q[2]+s.dpt[3,1]
 
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

    RLcp2_12 = Dz23*S1
    RLcp2_32 = Dz23*C1
    ORcp2_12 = RLcp2_32*qd[1]
    ORcp2_32 = -RLcp2_12*qd[1]
    VIcp2_12 = ORcp2_12+qd[2]*S1
    VIcp2_32 = ORcp2_32+qd[2]*C1
    ACcp2_12 = ORcp2_32*qd[1]+RLcp2_32*qdd[1]+qdd[2]*S1+(2.0)*qd[1]*qd[2]*C1
    ACcp2_32 = -ORcp2_12*qd[1]-RLcp2_12*qdd[1]+qdd[2]*C1-(2.0)*qd[1]*qd[2]*S1
    sens.P[1] = RLcp2_12
    sens.P[2] = 0
    sens.P[3] = RLcp2_32
    sens.R[1,1] = C1
    sens.R[1,3] = -S1
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S1
    sens.R[3,3] = C1
    sens.V[1] = VIcp2_12
    sens.V[2] = 0
    sens.V[3] = VIcp2_32
    sens.OM[1] = 0
    sens.OM[2] = qd[1]
    sens.OM[3] = 0
    sens.J[1,1] = RLcp2_32
    sens.J[1,2] = S1
    sens.J[3,1] = -RLcp2_12
    sens.J[3,2] = C1
    sens.J[5,1] = (1.0)
    sens.A[1] = ACcp2_12
    sens.A[2] = 0
    sens.A[3] = ACcp2_32
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[1]
    sens.OMP[3] = 0

  if (isens == 3): 

    ROcp3_13 = C1*C3-S1*S3
    ROcp3_33 = -C1*S3-S1*C3
    ROcp3_73 = C1*S3+S1*C3
    ROcp3_93 = C1*C3-S1*S3
    RLcp3_12 = Dz23*S1
    RLcp3_32 = Dz23*C1
    ORcp3_12 = RLcp3_32*qd[1]
    ORcp3_32 = -RLcp3_12*qd[1]
    VIcp3_12 = ORcp3_12+qd[2]*S1
    VIcp3_32 = ORcp3_32+qd[2]*C1
    ACcp3_12 = ORcp3_32*qd[1]+RLcp3_32*qdd[1]+qdd[2]*S1+(2.0)*qd[1]*qd[2]*C1
    ACcp3_32 = -ORcp3_12*qd[1]-RLcp3_12*qdd[1]+qdd[2]*C1-(2.0)*qd[1]*qd[2]*S1
    OMcp3_23 = qd[1]+qd[3]
    OPcp3_23 = qdd[1]+qdd[3]
    sens.P[1] = RLcp3_12
    sens.P[2] = 0
    sens.P[3] = RLcp3_32
    sens.R[1,1] = ROcp3_13
    sens.R[1,3] = ROcp3_33
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp3_73
    sens.R[3,3] = ROcp3_93
    sens.V[1] = VIcp3_12
    sens.V[2] = 0
    sens.V[3] = VIcp3_32
    sens.OM[1] = 0
    sens.OM[2] = OMcp3_23
    sens.OM[3] = 0
    sens.J[1,1] = RLcp3_32
    sens.J[1,2] = S1
    sens.J[3,1] = -RLcp3_12
    sens.J[3,2] = C1
    sens.J[5,1] = (1.0)
    sens.J[5,3] = (1.0)
    sens.A[1] = ACcp3_12
    sens.A[2] = 0
    sens.A[3] = ACcp3_32
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp3_23
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


