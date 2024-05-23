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
#	==> Generation Date: Thu May 23 21:22:39 2024
#
#	==> Project name: PreProject
#
#	==> Number of joints: 19
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

  S3 = sin(q[3])
  C3 = cos(q[3])
  S4 = sin(q[4])
  C4 = cos(q[4])
  S5 = sin(q[5])
  C5 = cos(q[5])
  S8 = sin(q[8])
  C8 = cos(q[8])
  S9 = sin(q[9])
  C9 = cos(q[9])
  S10 = sin(q[10])
  C10 = cos(q[10])
  S11 = sin(q[11])
  C11 = cos(q[11])
  S12 = sin(q[12])
  C12 = cos(q[12])
  S15 = sin(q[15])
  C15 = cos(q[15])
  S16 = sin(q[16])
  C16 = cos(q[16])
  S17 = sin(q[17])
  C17 = cos(q[17])
  S18 = sin(q[18])
  C18 = cos(q[18])
  S19 = sin(q[19])
  C19 = cos(q[19])
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics


  if (isens == 1): 

    sens.P[1] = 0
    sens.P[2] = q[1]
    sens.P[3] = 0
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[1]
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,1] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[1]
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 2): 

    sens.P[1] = 0
    sens.P[2] = q[1]
    sens.P[3] = q[2]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[1]
    sens.V[3] = qd[2]
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,1] = (1.0)
    sens.J[3,2] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[1]
    sens.A[3] = qdd[2]
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 3): 

    sens.P[1] = 0
    sens.P[2] = q[1]
    sens.P[3] = q[2]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = C3
    sens.R[2,3] = S3
    sens.R[3,2] = -S3
    sens.R[3,3] = C3
    sens.V[1] = 0
    sens.V[2] = qd[1]
    sens.V[3] = qd[2]
    sens.OM[1] = qd[3]
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,1] = (1.0)
    sens.J[3,2] = (1.0)
    sens.J[4,3] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[1]
    sens.A[3] = qdd[2]
    sens.OMP[1] = qdd[3]
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 4): 

    ROcp4_54 = C3*C4-S3*S4
    ROcp4_64 = C3*S4+S3*C4
    ROcp4_84 = -C3*S4-S3*C4
    ROcp4_94 = C3*C4-S3*S4
    RLcp4_24 = s.dpt[2,1]*C3
    RLcp4_34 = s.dpt[2,1]*S3
    POcp4_24 = RLcp4_24+q[1]
    POcp4_34 = RLcp4_34+q[2]
    OMcp4_14 = qd[3]+qd[4]
    ORcp4_24 = -RLcp4_34*qd[3]
    ORcp4_34 = RLcp4_24*qd[3]
    VIcp4_24 = ORcp4_24+qd[1]
    VIcp4_34 = ORcp4_34+qd[2]
    OPcp4_14 = qdd[3]+qdd[4]
    ACcp4_24 = qdd[1]-ORcp4_34*qd[3]-RLcp4_34*qdd[3]
    ACcp4_34 = qdd[2]+ORcp4_24*qd[3]+RLcp4_24*qdd[3]
    sens.P[1] = 0
    sens.P[2] = POcp4_24
    sens.P[3] = POcp4_34
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp4_54
    sens.R[2,3] = ROcp4_64
    sens.R[3,2] = ROcp4_84
    sens.R[3,3] = ROcp4_94
    sens.V[1] = 0
    sens.V[2] = VIcp4_24
    sens.V[3] = VIcp4_34
    sens.OM[1] = OMcp4_14
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,1] = (1.0)
    sens.J[2,3] = -RLcp4_34
    sens.J[3,2] = (1.0)
    sens.J[3,3] = RLcp4_24
    sens.J[4,3] = (1.0)
    sens.J[4,4] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp4_24
    sens.A[3] = ACcp4_34
    sens.OMP[1] = OPcp4_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 5): 

    ROcp5_55 = C3*C5-S3*S5
    ROcp5_65 = C3*S5+S3*C5
    ROcp5_85 = -C3*S5-S3*C5
    ROcp5_95 = C3*C5-S3*S5
    RLcp5_24 = s.dpt[2,2]*C3
    RLcp5_34 = s.dpt[2,2]*S3
    POcp5_24 = RLcp5_24+q[1]
    POcp5_34 = RLcp5_34+q[2]
    OMcp5_14 = qd[3]+qd[5]
    ORcp5_24 = -RLcp5_34*qd[3]
    ORcp5_34 = RLcp5_24*qd[3]
    VIcp5_24 = ORcp5_24+qd[1]
    VIcp5_34 = ORcp5_34+qd[2]
    OPcp5_14 = qdd[3]+qdd[5]
    ACcp5_24 = qdd[1]-ORcp5_34*qd[3]-RLcp5_34*qdd[3]
    ACcp5_34 = qdd[2]+ORcp5_24*qd[3]+RLcp5_24*qdd[3]
    sens.P[1] = 0
    sens.P[2] = POcp5_24
    sens.P[3] = POcp5_34
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp5_55
    sens.R[2,3] = ROcp5_65
    sens.R[3,2] = ROcp5_85
    sens.R[3,3] = ROcp5_95
    sens.V[1] = 0
    sens.V[2] = VIcp5_24
    sens.V[3] = VIcp5_34
    sens.OM[1] = OMcp5_14
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,1] = (1.0)
    sens.J[2,3] = -RLcp5_34
    sens.J[3,2] = (1.0)
    sens.J[3,3] = RLcp5_24
    sens.J[4,3] = (1.0)
    sens.J[4,5] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp5_24
    sens.A[3] = ACcp5_34
    sens.OMP[1] = OPcp5_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 6): 

    sens.P[1] = 0
    sens.P[2] = q[6]
    sens.P[3] = 0
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[6]
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,6] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[6]
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 7): 

    sens.P[1] = 0
    sens.P[2] = q[6]
    sens.P[3] = q[7]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[6]
    sens.V[3] = qd[7]
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,6] = (1.0)
    sens.J[3,7] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[6]
    sens.A[3] = qdd[7]
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 8): 

    sens.P[1] = 0
    sens.P[2] = q[6]
    sens.P[3] = q[7]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = C8
    sens.R[2,3] = S8
    sens.R[3,2] = -S8
    sens.R[3,3] = C8
    sens.V[1] = 0
    sens.V[2] = qd[6]
    sens.V[3] = qd[7]
    sens.OM[1] = qd[8]
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,6] = (1.0)
    sens.J[3,7] = (1.0)
    sens.J[4,8] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[6]
    sens.A[3] = qdd[7]
    sens.OMP[1] = qdd[8]
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 9): 

    ROcp9_59 = C8*C9-S8*S9
    ROcp9_69 = C8*S9+S8*C9
    ROcp9_89 = -C8*S9-S8*C9
    ROcp9_99 = C8*C9-S8*S9
    RLcp9_24 = s.dpt[2,5]*C8
    RLcp9_34 = s.dpt[2,5]*S8
    POcp9_24 = RLcp9_24+q[6]
    POcp9_34 = RLcp9_34+q[7]
    OMcp9_14 = qd[8]+qd[9]
    ORcp9_24 = -RLcp9_34*qd[8]
    ORcp9_34 = RLcp9_24*qd[8]
    VIcp9_24 = ORcp9_24+qd[6]
    VIcp9_34 = ORcp9_34+qd[7]
    OPcp9_14 = qdd[8]+qdd[9]
    ACcp9_24 = qdd[6]-ORcp9_34*qd[8]-RLcp9_34*qdd[8]
    ACcp9_34 = qdd[7]+ORcp9_24*qd[8]+RLcp9_24*qdd[8]
    sens.P[1] = 0
    sens.P[2] = POcp9_24
    sens.P[3] = POcp9_34
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp9_59
    sens.R[2,3] = ROcp9_69
    sens.R[3,2] = ROcp9_89
    sens.R[3,3] = ROcp9_99
    sens.V[1] = 0
    sens.V[2] = VIcp9_24
    sens.V[3] = VIcp9_34
    sens.OM[1] = OMcp9_14
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,6] = (1.0)
    sens.J[2,8] = -RLcp9_34
    sens.J[3,7] = (1.0)
    sens.J[3,8] = RLcp9_24
    sens.J[4,8] = (1.0)
    sens.J[4,9] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp9_24
    sens.A[3] = ACcp9_34
    sens.OMP[1] = OPcp9_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 10): 

    ROcp10_59 = C8*C9-S8*S9
    ROcp10_69 = C8*S9+S8*C9
    ROcp10_89 = -C8*S9-S8*C9
    ROcp10_99 = C8*C9-S8*S9
    ROcp10_510 = ROcp10_59*C10+ROcp10_89*S10
    ROcp10_610 = ROcp10_69*C10+ROcp10_99*S10
    ROcp10_810 = -ROcp10_59*S10+ROcp10_89*C10
    ROcp10_910 = -ROcp10_69*S10+ROcp10_99*C10
    RLcp10_24 = s.dpt[2,5]*C8
    RLcp10_34 = s.dpt[2,5]*S8
    POcp10_24 = RLcp10_24+q[6]
    POcp10_34 = RLcp10_34+q[7]
    OMcp10_14 = qd[8]+qd[9]
    ORcp10_24 = -RLcp10_34*qd[8]
    ORcp10_34 = RLcp10_24*qd[8]
    VIcp10_24 = ORcp10_24+qd[6]
    VIcp10_34 = ORcp10_34+qd[7]
    OPcp10_14 = qdd[8]+qdd[9]
    ACcp10_24 = qdd[6]-ORcp10_34*qd[8]-RLcp10_34*qdd[8]
    ACcp10_34 = qdd[7]+ORcp10_24*qd[8]+RLcp10_24*qdd[8]
    RLcp10_25 = ROcp10_59*s.dpt[2,7]
    RLcp10_35 = ROcp10_69*s.dpt[2,7]
    POcp10_25 = POcp10_24+RLcp10_25
    POcp10_35 = POcp10_34+RLcp10_35
    JTcp10_25_3 = -RLcp10_34-RLcp10_35
    JTcp10_35_3 = RLcp10_24+RLcp10_25
    OMcp10_15 = OMcp10_14+qd[10]
    ORcp10_25 = -OMcp10_14*RLcp10_35
    ORcp10_35 = OMcp10_14*RLcp10_25
    VIcp10_25 = ORcp10_25+VIcp10_24
    VIcp10_35 = ORcp10_35+VIcp10_34
    OPcp10_15 = OPcp10_14+qdd[10]
    ACcp10_25 = ACcp10_24-OMcp10_14*ORcp10_35-OPcp10_14*RLcp10_35
    ACcp10_35 = ACcp10_34+OMcp10_14*ORcp10_25+OPcp10_14*RLcp10_25
    sens.P[1] = 0
    sens.P[2] = POcp10_25
    sens.P[3] = POcp10_35
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp10_510
    sens.R[2,3] = ROcp10_610
    sens.R[3,2] = ROcp10_810
    sens.R[3,3] = ROcp10_910
    sens.V[1] = 0
    sens.V[2] = VIcp10_25
    sens.V[3] = VIcp10_35
    sens.OM[1] = OMcp10_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,6] = (1.0)
    sens.J[2,8] = JTcp10_25_3
    sens.J[2,9] = -RLcp10_35
    sens.J[3,7] = (1.0)
    sens.J[3,8] = JTcp10_35_3
    sens.J[3,9] = RLcp10_25
    sens.J[4,8] = (1.0)
    sens.J[4,9] = (1.0)
    sens.J[4,10] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp10_25
    sens.A[3] = ACcp10_35
    sens.OMP[1] = OPcp10_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 11): 

    ROcp11_511 = C11*C8-S11*S8
    ROcp11_611 = C11*S8+S11*C8
    ROcp11_811 = -C11*S8-S11*C8
    ROcp11_911 = C11*C8-S11*S8
    RLcp11_24 = s.dpt[2,6]*C8
    RLcp11_34 = s.dpt[2,6]*S8
    POcp11_24 = RLcp11_24+q[6]
    POcp11_34 = RLcp11_34+q[7]
    OMcp11_14 = qd[11]+qd[8]
    ORcp11_24 = -RLcp11_34*qd[8]
    ORcp11_34 = RLcp11_24*qd[8]
    VIcp11_24 = ORcp11_24+qd[6]
    VIcp11_34 = ORcp11_34+qd[7]
    OPcp11_14 = qdd[11]+qdd[8]
    ACcp11_24 = qdd[6]-ORcp11_34*qd[8]-RLcp11_34*qdd[8]
    ACcp11_34 = qdd[7]+ORcp11_24*qd[8]+RLcp11_24*qdd[8]
    sens.P[1] = 0
    sens.P[2] = POcp11_24
    sens.P[3] = POcp11_34
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp11_511
    sens.R[2,3] = ROcp11_611
    sens.R[3,2] = ROcp11_811
    sens.R[3,3] = ROcp11_911
    sens.V[1] = 0
    sens.V[2] = VIcp11_24
    sens.V[3] = VIcp11_34
    sens.OM[1] = OMcp11_14
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,6] = (1.0)
    sens.J[2,8] = -RLcp11_34
    sens.J[3,7] = (1.0)
    sens.J[3,8] = RLcp11_24
    sens.J[4,8] = (1.0)
    sens.J[4,11] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp11_24
    sens.A[3] = ACcp11_34
    sens.OMP[1] = OPcp11_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 12): 

    ROcp12_511 = C11*C8-S11*S8
    ROcp12_611 = C11*S8+S11*C8
    ROcp12_811 = -C11*S8-S11*C8
    ROcp12_911 = C11*C8-S11*S8
    ROcp12_512 = ROcp12_511*C12+ROcp12_811*S12
    ROcp12_612 = ROcp12_611*C12+ROcp12_911*S12
    ROcp12_812 = -ROcp12_511*S12+ROcp12_811*C12
    ROcp12_912 = -ROcp12_611*S12+ROcp12_911*C12
    RLcp12_24 = s.dpt[2,6]*C8
    RLcp12_34 = s.dpt[2,6]*S8
    POcp12_24 = RLcp12_24+q[6]
    POcp12_34 = RLcp12_34+q[7]
    OMcp12_14 = qd[11]+qd[8]
    ORcp12_24 = -RLcp12_34*qd[8]
    ORcp12_34 = RLcp12_24*qd[8]
    VIcp12_24 = ORcp12_24+qd[6]
    VIcp12_34 = ORcp12_34+qd[7]
    OPcp12_14 = qdd[11]+qdd[8]
    ACcp12_24 = qdd[6]-ORcp12_34*qd[8]-RLcp12_34*qdd[8]
    ACcp12_34 = qdd[7]+ORcp12_24*qd[8]+RLcp12_24*qdd[8]
    RLcp12_25 = ROcp12_511*s.dpt[2,9]
    RLcp12_35 = ROcp12_611*s.dpt[2,9]
    POcp12_25 = POcp12_24+RLcp12_25
    POcp12_35 = POcp12_34+RLcp12_35
    JTcp12_25_3 = -RLcp12_34-RLcp12_35
    JTcp12_35_3 = RLcp12_24+RLcp12_25
    OMcp12_15 = OMcp12_14+qd[12]
    ORcp12_25 = -OMcp12_14*RLcp12_35
    ORcp12_35 = OMcp12_14*RLcp12_25
    VIcp12_25 = ORcp12_25+VIcp12_24
    VIcp12_35 = ORcp12_35+VIcp12_34
    OPcp12_15 = OPcp12_14+qdd[12]
    ACcp12_25 = ACcp12_24-OMcp12_14*ORcp12_35-OPcp12_14*RLcp12_35
    ACcp12_35 = ACcp12_34+OMcp12_14*ORcp12_25+OPcp12_14*RLcp12_25
    sens.P[1] = 0
    sens.P[2] = POcp12_25
    sens.P[3] = POcp12_35
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp12_512
    sens.R[2,3] = ROcp12_612
    sens.R[3,2] = ROcp12_812
    sens.R[3,3] = ROcp12_912
    sens.V[1] = 0
    sens.V[2] = VIcp12_25
    sens.V[3] = VIcp12_35
    sens.OM[1] = OMcp12_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,6] = (1.0)
    sens.J[2,8] = JTcp12_25_3
    sens.J[2,11] = -RLcp12_35
    sens.J[3,7] = (1.0)
    sens.J[3,8] = JTcp12_35_3
    sens.J[3,11] = RLcp12_25
    sens.J[4,8] = (1.0)
    sens.J[4,11] = (1.0)
    sens.J[4,12] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp12_25
    sens.A[3] = ACcp12_35
    sens.OMP[1] = OPcp12_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 13): 

    sens.P[1] = 0
    sens.P[2] = q[13]
    sens.P[3] = 0
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[13]
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,13] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[13]
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 14): 

    sens.P[1] = 0
    sens.P[2] = q[13]
    sens.P[3] = q[14]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[13]
    sens.V[3] = qd[14]
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,13] = (1.0)
    sens.J[3,14] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[13]
    sens.A[3] = qdd[14]
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 15): 

    sens.P[1] = 0
    sens.P[2] = q[13]
    sens.P[3] = q[14]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = C15
    sens.R[2,3] = S15
    sens.R[3,2] = -S15
    sens.R[3,3] = C15
    sens.V[1] = 0
    sens.V[2] = qd[13]
    sens.V[3] = qd[14]
    sens.OM[1] = qd[15]
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,13] = (1.0)
    sens.J[3,14] = (1.0)
    sens.J[4,15] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[13]
    sens.A[3] = qdd[14]
    sens.OMP[1] = qdd[15]
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 16): 

    ROcp16_516 = C15*C16-S15*S16
    ROcp16_616 = C15*S16+S15*C16
    ROcp16_816 = -C15*S16-S15*C16
    ROcp16_916 = C15*C16-S15*S16
    RLcp16_24 = s.dpt[2,11]*C15
    RLcp16_34 = s.dpt[2,11]*S15
    POcp16_24 = RLcp16_24+q[13]
    POcp16_34 = RLcp16_34+q[14]
    OMcp16_14 = qd[15]+qd[16]
    ORcp16_24 = -RLcp16_34*qd[15]
    ORcp16_34 = RLcp16_24*qd[15]
    VIcp16_24 = ORcp16_24+qd[13]
    VIcp16_34 = ORcp16_34+qd[14]
    OPcp16_14 = qdd[15]+qdd[16]
    ACcp16_24 = qdd[13]-ORcp16_34*qd[15]-RLcp16_34*qdd[15]
    ACcp16_34 = qdd[14]+ORcp16_24*qd[15]+RLcp16_24*qdd[15]
    sens.P[1] = 0
    sens.P[2] = POcp16_24
    sens.P[3] = POcp16_34
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp16_516
    sens.R[2,3] = ROcp16_616
    sens.R[3,2] = ROcp16_816
    sens.R[3,3] = ROcp16_916
    sens.V[1] = 0
    sens.V[2] = VIcp16_24
    sens.V[3] = VIcp16_34
    sens.OM[1] = OMcp16_14
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,13] = (1.0)
    sens.J[2,15] = -RLcp16_34
    sens.J[3,14] = (1.0)
    sens.J[3,15] = RLcp16_24
    sens.J[4,15] = (1.0)
    sens.J[4,16] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp16_24
    sens.A[3] = ACcp16_34
    sens.OMP[1] = OPcp16_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 17): 

    ROcp17_516 = C15*C16-S15*S16
    ROcp17_616 = C15*S16+S15*C16
    ROcp17_816 = -C15*S16-S15*C16
    ROcp17_916 = C15*C16-S15*S16
    ROcp17_517 = ROcp17_516*C17+ROcp17_816*S17
    ROcp17_617 = ROcp17_616*C17+ROcp17_916*S17
    ROcp17_817 = -ROcp17_516*S17+ROcp17_816*C17
    ROcp17_917 = -ROcp17_616*S17+ROcp17_916*C17
    RLcp17_24 = s.dpt[2,11]*C15
    RLcp17_34 = s.dpt[2,11]*S15
    POcp17_24 = RLcp17_24+q[13]
    POcp17_34 = RLcp17_34+q[14]
    OMcp17_14 = qd[15]+qd[16]
    ORcp17_24 = -RLcp17_34*qd[15]
    ORcp17_34 = RLcp17_24*qd[15]
    VIcp17_24 = ORcp17_24+qd[13]
    VIcp17_34 = ORcp17_34+qd[14]
    OPcp17_14 = qdd[15]+qdd[16]
    ACcp17_24 = qdd[13]-ORcp17_34*qd[15]-RLcp17_34*qdd[15]
    ACcp17_34 = qdd[14]+ORcp17_24*qd[15]+RLcp17_24*qdd[15]
    RLcp17_25 = ROcp17_516*s.dpt[2,13]
    RLcp17_35 = ROcp17_616*s.dpt[2,13]
    POcp17_25 = POcp17_24+RLcp17_25
    POcp17_35 = POcp17_34+RLcp17_35
    JTcp17_25_3 = -RLcp17_34-RLcp17_35
    JTcp17_35_3 = RLcp17_24+RLcp17_25
    OMcp17_15 = OMcp17_14+qd[17]
    ORcp17_25 = -OMcp17_14*RLcp17_35
    ORcp17_35 = OMcp17_14*RLcp17_25
    VIcp17_25 = ORcp17_25+VIcp17_24
    VIcp17_35 = ORcp17_35+VIcp17_34
    OPcp17_15 = OPcp17_14+qdd[17]
    ACcp17_25 = ACcp17_24-OMcp17_14*ORcp17_35-OPcp17_14*RLcp17_35
    ACcp17_35 = ACcp17_34+OMcp17_14*ORcp17_25+OPcp17_14*RLcp17_25
    sens.P[1] = 0
    sens.P[2] = POcp17_25
    sens.P[3] = POcp17_35
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp17_517
    sens.R[2,3] = ROcp17_617
    sens.R[3,2] = ROcp17_817
    sens.R[3,3] = ROcp17_917
    sens.V[1] = 0
    sens.V[2] = VIcp17_25
    sens.V[3] = VIcp17_35
    sens.OM[1] = OMcp17_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,13] = (1.0)
    sens.J[2,15] = JTcp17_25_3
    sens.J[2,16] = -RLcp17_35
    sens.J[3,14] = (1.0)
    sens.J[3,15] = JTcp17_35_3
    sens.J[3,16] = RLcp17_25
    sens.J[4,15] = (1.0)
    sens.J[4,16] = (1.0)
    sens.J[4,17] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp17_25
    sens.A[3] = ACcp17_35
    sens.OMP[1] = OPcp17_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 18): 

    ROcp18_518 = C15*C18-S15*S18
    ROcp18_618 = C15*S18+S15*C18
    ROcp18_818 = -C15*S18-S15*C18
    ROcp18_918 = C15*C18-S15*S18
    RLcp18_24 = s.dpt[2,12]*C15
    RLcp18_34 = s.dpt[2,12]*S15
    POcp18_24 = RLcp18_24+q[13]
    POcp18_34 = RLcp18_34+q[14]
    OMcp18_14 = qd[15]+qd[18]
    ORcp18_24 = -RLcp18_34*qd[15]
    ORcp18_34 = RLcp18_24*qd[15]
    VIcp18_24 = ORcp18_24+qd[13]
    VIcp18_34 = ORcp18_34+qd[14]
    OPcp18_14 = qdd[15]+qdd[18]
    ACcp18_24 = qdd[13]-ORcp18_34*qd[15]-RLcp18_34*qdd[15]
    ACcp18_34 = qdd[14]+ORcp18_24*qd[15]+RLcp18_24*qdd[15]
    sens.P[1] = 0
    sens.P[2] = POcp18_24
    sens.P[3] = POcp18_34
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp18_518
    sens.R[2,3] = ROcp18_618
    sens.R[3,2] = ROcp18_818
    sens.R[3,3] = ROcp18_918
    sens.V[1] = 0
    sens.V[2] = VIcp18_24
    sens.V[3] = VIcp18_34
    sens.OM[1] = OMcp18_14
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,13] = (1.0)
    sens.J[2,15] = -RLcp18_34
    sens.J[3,14] = (1.0)
    sens.J[3,15] = RLcp18_24
    sens.J[4,15] = (1.0)
    sens.J[4,18] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp18_24
    sens.A[3] = ACcp18_34
    sens.OMP[1] = OPcp18_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 19): 

    ROcp19_518 = C15*C18-S15*S18
    ROcp19_618 = C15*S18+S15*C18
    ROcp19_818 = -C15*S18-S15*C18
    ROcp19_918 = C15*C18-S15*S18
    ROcp19_519 = ROcp19_518*C19+ROcp19_818*S19
    ROcp19_619 = ROcp19_618*C19+ROcp19_918*S19
    ROcp19_819 = -ROcp19_518*S19+ROcp19_818*C19
    ROcp19_919 = -ROcp19_618*S19+ROcp19_918*C19
    RLcp19_24 = s.dpt[2,12]*C15
    RLcp19_34 = s.dpt[2,12]*S15
    POcp19_24 = RLcp19_24+q[13]
    POcp19_34 = RLcp19_34+q[14]
    OMcp19_14 = qd[15]+qd[18]
    ORcp19_24 = -RLcp19_34*qd[15]
    ORcp19_34 = RLcp19_24*qd[15]
    VIcp19_24 = ORcp19_24+qd[13]
    VIcp19_34 = ORcp19_34+qd[14]
    OPcp19_14 = qdd[15]+qdd[18]
    ACcp19_24 = qdd[13]-ORcp19_34*qd[15]-RLcp19_34*qdd[15]
    ACcp19_34 = qdd[14]+ORcp19_24*qd[15]+RLcp19_24*qdd[15]
    RLcp19_25 = ROcp19_518*s.dpt[2,15]
    RLcp19_35 = ROcp19_618*s.dpt[2,15]
    POcp19_25 = POcp19_24+RLcp19_25
    POcp19_35 = POcp19_34+RLcp19_35
    JTcp19_25_3 = -RLcp19_34-RLcp19_35
    JTcp19_35_3 = RLcp19_24+RLcp19_25
    OMcp19_15 = OMcp19_14+qd[19]
    ORcp19_25 = -OMcp19_14*RLcp19_35
    ORcp19_35 = OMcp19_14*RLcp19_25
    VIcp19_25 = ORcp19_25+VIcp19_24
    VIcp19_35 = ORcp19_35+VIcp19_34
    OPcp19_15 = OPcp19_14+qdd[19]
    ACcp19_25 = ACcp19_24-OMcp19_14*ORcp19_35-OPcp19_14*RLcp19_35
    ACcp19_35 = ACcp19_34+OMcp19_14*ORcp19_25+OPcp19_14*RLcp19_25
    sens.P[1] = 0
    sens.P[2] = POcp19_25
    sens.P[3] = POcp19_35
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp19_519
    sens.R[2,3] = ROcp19_619
    sens.R[3,2] = ROcp19_819
    sens.R[3,3] = ROcp19_919
    sens.V[1] = 0
    sens.V[2] = VIcp19_25
    sens.V[3] = VIcp19_35
    sens.OM[1] = OMcp19_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,13] = (1.0)
    sens.J[2,15] = JTcp19_25_3
    sens.J[2,18] = -RLcp19_35
    sens.J[3,14] = (1.0)
    sens.J[3,15] = JTcp19_35_3
    sens.J[3,18] = RLcp19_25
    sens.J[4,15] = (1.0)
    sens.J[4,18] = (1.0)
    sens.J[4,19] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp19_25
    sens.A[3] = ACcp19_35
    sens.OMP[1] = OPcp19_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


