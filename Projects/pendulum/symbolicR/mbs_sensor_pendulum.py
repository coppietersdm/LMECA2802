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
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics


  if (isens == 1): 

    RLcp1_12 = s.dpt[3,5]*S1
    RLcp1_32 = s.dpt[3,5]*C1
    ORcp1_12 = RLcp1_32*qd[1]
    ORcp1_32 = -RLcp1_12*qd[1]
    ACcp1_12 = ORcp1_32*qd[1]+RLcp1_32*qdd[1]
    ACcp1_32 = -ORcp1_12*qd[1]-RLcp1_12*qdd[1]
    sens.P[1] = RLcp1_12
    sens.P[2] = 0
    sens.P[3] = RLcp1_32
    sens.R[1,1] = C1
    sens.R[1,3] = -S1
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S1
    sens.R[3,3] = C1
    sens.V[1] = ORcp1_12
    sens.V[2] = 0
    sens.V[3] = ORcp1_32
    sens.OM[1] = 0
    sens.OM[2] = qd[1]
    sens.OM[3] = 0
    sens.A[1] = ACcp1_12
    sens.A[2] = 0
    sens.A[3] = ACcp1_32
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[1]
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


