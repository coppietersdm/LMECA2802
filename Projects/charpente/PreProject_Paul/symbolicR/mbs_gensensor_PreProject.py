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
#	==> Generation Date: Wed May  1 12:12:23 2024
#
#	==> Project name: PreProject
#
#	==> Number of joints: 21
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
  S6 = sin(q[6])
  C6 = cos(q[6])
  S7 = sin(q[7])
  C7 = cos(q[7])
  S10 = sin(q[10])
  C10 = cos(q[10])
  S11 = sin(q[11])
  C11 = cos(q[11])
  S12 = sin(q[12])
  C12 = cos(q[12])
  S13 = sin(q[13])
  C13 = cos(q[13])
  S14 = sin(q[14])
  C14 = cos(q[14])
  S17 = sin(q[17])
  C17 = cos(q[17])
  S18 = sin(q[18])
  C18 = cos(q[18])
  S19 = sin(q[19])
  C19 = cos(q[19])
  S20 = sin(q[20])
  C20 = cos(q[20])
  S21 = sin(q[21])
  C21 = cos(q[21])
 
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

    ROcp5_54 = C3*C4-S3*S4
    ROcp5_64 = C3*S4+S3*C4
    ROcp5_84 = -C3*S4-S3*C4
    ROcp5_94 = C3*C4-S3*S4
    ROcp5_55 = ROcp5_54*C5+ROcp5_84*S5
    ROcp5_65 = ROcp5_64*C5+ROcp5_94*S5
    ROcp5_85 = -ROcp5_54*S5+ROcp5_84*C5
    ROcp5_95 = -ROcp5_64*S5+ROcp5_94*C5
    RLcp5_24 = s.dpt[2,1]*C3
    RLcp5_34 = s.dpt[2,1]*S3
    POcp5_24 = RLcp5_24+q[1]
    POcp5_34 = RLcp5_34+q[2]
    OMcp5_14 = qd[3]+qd[4]
    ORcp5_24 = -RLcp5_34*qd[3]
    ORcp5_34 = RLcp5_24*qd[3]
    VIcp5_24 = ORcp5_24+qd[1]
    VIcp5_34 = ORcp5_34+qd[2]
    OPcp5_14 = qdd[3]+qdd[4]
    ACcp5_24 = qdd[1]-ORcp5_34*qd[3]-RLcp5_34*qdd[3]
    ACcp5_34 = qdd[2]+ORcp5_24*qd[3]+RLcp5_24*qdd[3]
    RLcp5_25 = ROcp5_54*s.dpt[2,3]
    RLcp5_35 = ROcp5_64*s.dpt[2,3]
    POcp5_25 = POcp5_24+RLcp5_25
    POcp5_35 = POcp5_34+RLcp5_35
    JTcp5_25_3 = -RLcp5_34-RLcp5_35
    JTcp5_35_3 = RLcp5_24+RLcp5_25
    OMcp5_15 = OMcp5_14+qd[5]
    ORcp5_25 = -OMcp5_14*RLcp5_35
    ORcp5_35 = OMcp5_14*RLcp5_25
    VIcp5_25 = ORcp5_25+VIcp5_24
    VIcp5_35 = ORcp5_35+VIcp5_34
    OPcp5_15 = OPcp5_14+qdd[5]
    ACcp5_25 = ACcp5_24-OMcp5_14*ORcp5_35-OPcp5_14*RLcp5_35
    ACcp5_35 = ACcp5_34+OMcp5_14*ORcp5_25+OPcp5_14*RLcp5_25
    sens.P[1] = 0
    sens.P[2] = POcp5_25
    sens.P[3] = POcp5_35
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp5_55
    sens.R[2,3] = ROcp5_65
    sens.R[3,2] = ROcp5_85
    sens.R[3,3] = ROcp5_95
    sens.V[1] = 0
    sens.V[2] = VIcp5_25
    sens.V[3] = VIcp5_35
    sens.OM[1] = OMcp5_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,1] = (1.0)
    sens.J[2,3] = JTcp5_25_3
    sens.J[2,4] = -RLcp5_35
    sens.J[3,2] = (1.0)
    sens.J[3,3] = JTcp5_35_3
    sens.J[3,4] = RLcp5_25
    sens.J[4,3] = (1.0)
    sens.J[4,4] = (1.0)
    sens.J[4,5] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp5_25
    sens.A[3] = ACcp5_35
    sens.OMP[1] = OPcp5_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 6): 

    ROcp6_56 = C3*C6-S3*S6
    ROcp6_66 = C3*S6+S3*C6
    ROcp6_86 = -C3*S6-S3*C6
    ROcp6_96 = C3*C6-S3*S6
    RLcp6_24 = s.dpt[2,2]*C3
    RLcp6_34 = s.dpt[2,2]*S3
    POcp6_24 = RLcp6_24+q[1]
    POcp6_34 = RLcp6_34+q[2]
    OMcp6_14 = qd[3]+qd[6]
    ORcp6_24 = -RLcp6_34*qd[3]
    ORcp6_34 = RLcp6_24*qd[3]
    VIcp6_24 = ORcp6_24+qd[1]
    VIcp6_34 = ORcp6_34+qd[2]
    OPcp6_14 = qdd[3]+qdd[6]
    ACcp6_24 = qdd[1]-ORcp6_34*qd[3]-RLcp6_34*qdd[3]
    ACcp6_34 = qdd[2]+ORcp6_24*qd[3]+RLcp6_24*qdd[3]
    sens.P[1] = 0
    sens.P[2] = POcp6_24
    sens.P[3] = POcp6_34
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp6_56
    sens.R[2,3] = ROcp6_66
    sens.R[3,2] = ROcp6_86
    sens.R[3,3] = ROcp6_96
    sens.V[1] = 0
    sens.V[2] = VIcp6_24
    sens.V[3] = VIcp6_34
    sens.OM[1] = OMcp6_14
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,1] = (1.0)
    sens.J[2,3] = -RLcp6_34
    sens.J[3,2] = (1.0)
    sens.J[3,3] = RLcp6_24
    sens.J[4,3] = (1.0)
    sens.J[4,6] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp6_24
    sens.A[3] = ACcp6_34
    sens.OMP[1] = OPcp6_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 7): 

    ROcp7_56 = C3*C6-S3*S6
    ROcp7_66 = C3*S6+S3*C6
    ROcp7_86 = -C3*S6-S3*C6
    ROcp7_96 = C3*C6-S3*S6
    ROcp7_57 = ROcp7_56*C7+ROcp7_86*S7
    ROcp7_67 = ROcp7_66*C7+ROcp7_96*S7
    ROcp7_87 = -ROcp7_56*S7+ROcp7_86*C7
    ROcp7_97 = -ROcp7_66*S7+ROcp7_96*C7
    RLcp7_24 = s.dpt[2,2]*C3
    RLcp7_34 = s.dpt[2,2]*S3
    POcp7_24 = RLcp7_24+q[1]
    POcp7_34 = RLcp7_34+q[2]
    OMcp7_14 = qd[3]+qd[6]
    ORcp7_24 = -RLcp7_34*qd[3]
    ORcp7_34 = RLcp7_24*qd[3]
    VIcp7_24 = ORcp7_24+qd[1]
    VIcp7_34 = ORcp7_34+qd[2]
    OPcp7_14 = qdd[3]+qdd[6]
    ACcp7_24 = qdd[1]-ORcp7_34*qd[3]-RLcp7_34*qdd[3]
    ACcp7_34 = qdd[2]+ORcp7_24*qd[3]+RLcp7_24*qdd[3]
    RLcp7_25 = ROcp7_56*s.dpt[2,5]
    RLcp7_35 = ROcp7_66*s.dpt[2,5]
    POcp7_25 = POcp7_24+RLcp7_25
    POcp7_35 = POcp7_34+RLcp7_35
    JTcp7_25_3 = -RLcp7_34-RLcp7_35
    JTcp7_35_3 = RLcp7_24+RLcp7_25
    OMcp7_15 = OMcp7_14+qd[7]
    ORcp7_25 = -OMcp7_14*RLcp7_35
    ORcp7_35 = OMcp7_14*RLcp7_25
    VIcp7_25 = ORcp7_25+VIcp7_24
    VIcp7_35 = ORcp7_35+VIcp7_34
    OPcp7_15 = OPcp7_14+qdd[7]
    ACcp7_25 = ACcp7_24-OMcp7_14*ORcp7_35-OPcp7_14*RLcp7_35
    ACcp7_35 = ACcp7_34+OMcp7_14*ORcp7_25+OPcp7_14*RLcp7_25
    sens.P[1] = 0
    sens.P[2] = POcp7_25
    sens.P[3] = POcp7_35
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp7_57
    sens.R[2,3] = ROcp7_67
    sens.R[3,2] = ROcp7_87
    sens.R[3,3] = ROcp7_97
    sens.V[1] = 0
    sens.V[2] = VIcp7_25
    sens.V[3] = VIcp7_35
    sens.OM[1] = OMcp7_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,1] = (1.0)
    sens.J[2,3] = JTcp7_25_3
    sens.J[2,6] = -RLcp7_35
    sens.J[3,2] = (1.0)
    sens.J[3,3] = JTcp7_35_3
    sens.J[3,6] = RLcp7_25
    sens.J[4,3] = (1.0)
    sens.J[4,6] = (1.0)
    sens.J[4,7] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp7_25
    sens.A[3] = ACcp7_35
    sens.OMP[1] = OPcp7_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 8): 

    sens.P[1] = 0
    sens.P[2] = q[8]
    sens.P[3] = 0
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[8]
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,8] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[8]
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 9): 

    sens.P[1] = 0
    sens.P[2] = q[8]
    sens.P[3] = q[9]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[8]
    sens.V[3] = qd[9]
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,8] = (1.0)
    sens.J[3,9] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[8]
    sens.A[3] = qdd[9]
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 10): 

    sens.P[1] = 0
    sens.P[2] = q[8]
    sens.P[3] = q[9]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = C10
    sens.R[2,3] = S10
    sens.R[3,2] = -S10
    sens.R[3,3] = C10
    sens.V[1] = 0
    sens.V[2] = qd[8]
    sens.V[3] = qd[9]
    sens.OM[1] = qd[10]
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,8] = (1.0)
    sens.J[3,9] = (1.0)
    sens.J[4,10] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[8]
    sens.A[3] = qdd[9]
    sens.OMP[1] = qdd[10]
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 11): 

    ROcp11_511 = C10*C11-S10*S11
    ROcp11_611 = C10*S11+S10*C11
    ROcp11_811 = -C10*S11-S10*C11
    ROcp11_911 = C10*C11-S10*S11
    RLcp11_24 = s.dpt[2,7]*C10
    RLcp11_34 = s.dpt[2,7]*S10
    POcp11_24 = RLcp11_24+q[8]
    POcp11_34 = RLcp11_34+q[9]
    OMcp11_14 = qd[10]+qd[11]
    ORcp11_24 = -RLcp11_34*qd[10]
    ORcp11_34 = RLcp11_24*qd[10]
    VIcp11_24 = ORcp11_24+qd[8]
    VIcp11_34 = ORcp11_34+qd[9]
    OPcp11_14 = qdd[10]+qdd[11]
    ACcp11_24 = qdd[8]-ORcp11_34*qd[10]-RLcp11_34*qdd[10]
    ACcp11_34 = qdd[9]+ORcp11_24*qd[10]+RLcp11_24*qdd[10]
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
    sens.J[2,8] = (1.0)
    sens.J[2,10] = -RLcp11_34
    sens.J[3,9] = (1.0)
    sens.J[3,10] = RLcp11_24
    sens.J[4,10] = (1.0)
    sens.J[4,11] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp11_24
    sens.A[3] = ACcp11_34
    sens.OMP[1] = OPcp11_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 12): 

    ROcp12_511 = C10*C11-S10*S11
    ROcp12_611 = C10*S11+S10*C11
    ROcp12_811 = -C10*S11-S10*C11
    ROcp12_911 = C10*C11-S10*S11
    ROcp12_512 = ROcp12_511*C12+ROcp12_811*S12
    ROcp12_612 = ROcp12_611*C12+ROcp12_911*S12
    ROcp12_812 = -ROcp12_511*S12+ROcp12_811*C12
    ROcp12_912 = -ROcp12_611*S12+ROcp12_911*C12
    RLcp12_24 = s.dpt[2,7]*C10
    RLcp12_34 = s.dpt[2,7]*S10
    POcp12_24 = RLcp12_24+q[8]
    POcp12_34 = RLcp12_34+q[9]
    OMcp12_14 = qd[10]+qd[11]
    ORcp12_24 = -RLcp12_34*qd[10]
    ORcp12_34 = RLcp12_24*qd[10]
    VIcp12_24 = ORcp12_24+qd[8]
    VIcp12_34 = ORcp12_34+qd[9]
    OPcp12_14 = qdd[10]+qdd[11]
    ACcp12_24 = qdd[8]-ORcp12_34*qd[10]-RLcp12_34*qdd[10]
    ACcp12_34 = qdd[9]+ORcp12_24*qd[10]+RLcp12_24*qdd[10]
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
    sens.J[2,8] = (1.0)
    sens.J[2,10] = JTcp12_25_3
    sens.J[2,11] = -RLcp12_35
    sens.J[3,9] = (1.0)
    sens.J[3,10] = JTcp12_35_3
    sens.J[3,11] = RLcp12_25
    sens.J[4,10] = (1.0)
    sens.J[4,11] = (1.0)
    sens.J[4,12] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp12_25
    sens.A[3] = ACcp12_35
    sens.OMP[1] = OPcp12_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 13): 

    ROcp13_513 = C10*C13-S10*S13
    ROcp13_613 = C10*S13+S10*C13
    ROcp13_813 = -C10*S13-S10*C13
    ROcp13_913 = C10*C13-S10*S13
    RLcp13_24 = s.dpt[2,8]*C10
    RLcp13_34 = s.dpt[2,8]*S10
    POcp13_24 = RLcp13_24+q[8]
    POcp13_34 = RLcp13_34+q[9]
    OMcp13_14 = qd[10]+qd[13]
    ORcp13_24 = -RLcp13_34*qd[10]
    ORcp13_34 = RLcp13_24*qd[10]
    VIcp13_24 = ORcp13_24+qd[8]
    VIcp13_34 = ORcp13_34+qd[9]
    OPcp13_14 = qdd[10]+qdd[13]
    ACcp13_24 = qdd[8]-ORcp13_34*qd[10]-RLcp13_34*qdd[10]
    ACcp13_34 = qdd[9]+ORcp13_24*qd[10]+RLcp13_24*qdd[10]
    sens.P[1] = 0
    sens.P[2] = POcp13_24
    sens.P[3] = POcp13_34
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp13_513
    sens.R[2,3] = ROcp13_613
    sens.R[3,2] = ROcp13_813
    sens.R[3,3] = ROcp13_913
    sens.V[1] = 0
    sens.V[2] = VIcp13_24
    sens.V[3] = VIcp13_34
    sens.OM[1] = OMcp13_14
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,8] = (1.0)
    sens.J[2,10] = -RLcp13_34
    sens.J[3,9] = (1.0)
    sens.J[3,10] = RLcp13_24
    sens.J[4,10] = (1.0)
    sens.J[4,13] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp13_24
    sens.A[3] = ACcp13_34
    sens.OMP[1] = OPcp13_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 14): 

    ROcp14_513 = C10*C13-S10*S13
    ROcp14_613 = C10*S13+S10*C13
    ROcp14_813 = -C10*S13-S10*C13
    ROcp14_913 = C10*C13-S10*S13
    ROcp14_514 = ROcp14_513*C14+ROcp14_813*S14
    ROcp14_614 = ROcp14_613*C14+ROcp14_913*S14
    ROcp14_814 = -ROcp14_513*S14+ROcp14_813*C14
    ROcp14_914 = -ROcp14_613*S14+ROcp14_913*C14
    RLcp14_24 = s.dpt[2,8]*C10
    RLcp14_34 = s.dpt[2,8]*S10
    POcp14_24 = RLcp14_24+q[8]
    POcp14_34 = RLcp14_34+q[9]
    OMcp14_14 = qd[10]+qd[13]
    ORcp14_24 = -RLcp14_34*qd[10]
    ORcp14_34 = RLcp14_24*qd[10]
    VIcp14_24 = ORcp14_24+qd[8]
    VIcp14_34 = ORcp14_34+qd[9]
    OPcp14_14 = qdd[10]+qdd[13]
    ACcp14_24 = qdd[8]-ORcp14_34*qd[10]-RLcp14_34*qdd[10]
    ACcp14_34 = qdd[9]+ORcp14_24*qd[10]+RLcp14_24*qdd[10]
    RLcp14_25 = ROcp14_513*s.dpt[2,11]
    RLcp14_35 = ROcp14_613*s.dpt[2,11]
    POcp14_25 = POcp14_24+RLcp14_25
    POcp14_35 = POcp14_34+RLcp14_35
    JTcp14_25_3 = -RLcp14_34-RLcp14_35
    JTcp14_35_3 = RLcp14_24+RLcp14_25
    OMcp14_15 = OMcp14_14+qd[14]
    ORcp14_25 = -OMcp14_14*RLcp14_35
    ORcp14_35 = OMcp14_14*RLcp14_25
    VIcp14_25 = ORcp14_25+VIcp14_24
    VIcp14_35 = ORcp14_35+VIcp14_34
    OPcp14_15 = OPcp14_14+qdd[14]
    ACcp14_25 = ACcp14_24-OMcp14_14*ORcp14_35-OPcp14_14*RLcp14_35
    ACcp14_35 = ACcp14_34+OMcp14_14*ORcp14_25+OPcp14_14*RLcp14_25
    sens.P[1] = 0
    sens.P[2] = POcp14_25
    sens.P[3] = POcp14_35
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp14_514
    sens.R[2,3] = ROcp14_614
    sens.R[3,2] = ROcp14_814
    sens.R[3,3] = ROcp14_914
    sens.V[1] = 0
    sens.V[2] = VIcp14_25
    sens.V[3] = VIcp14_35
    sens.OM[1] = OMcp14_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,8] = (1.0)
    sens.J[2,10] = JTcp14_25_3
    sens.J[2,13] = -RLcp14_35
    sens.J[3,9] = (1.0)
    sens.J[3,10] = JTcp14_35_3
    sens.J[3,13] = RLcp14_25
    sens.J[4,10] = (1.0)
    sens.J[4,13] = (1.0)
    sens.J[4,14] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp14_25
    sens.A[3] = ACcp14_35
    sens.OMP[1] = OPcp14_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 15): 

    sens.P[1] = 0
    sens.P[2] = q[15]
    sens.P[3] = 0
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[15]
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,15] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[15]
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 16): 

    sens.P[1] = 0
    sens.P[2] = q[15]
    sens.P[3] = q[16]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = qd[15]
    sens.V[3] = qd[16]
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,15] = (1.0)
    sens.J[3,16] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[15]
    sens.A[3] = qdd[16]
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 17): 

    sens.P[1] = 0
    sens.P[2] = q[15]
    sens.P[3] = q[16]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = C17
    sens.R[2,3] = S17
    sens.R[3,2] = -S17
    sens.R[3,3] = C17
    sens.V[1] = 0
    sens.V[2] = qd[15]
    sens.V[3] = qd[16]
    sens.OM[1] = qd[17]
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,15] = (1.0)
    sens.J[3,16] = (1.0)
    sens.J[4,17] = (1.0)
    sens.A[1] = 0
    sens.A[2] = qdd[15]
    sens.A[3] = qdd[16]
    sens.OMP[1] = qdd[17]
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 18): 

    ROcp18_518 = C17*C18-S17*S18
    ROcp18_618 = C17*S18+S17*C18
    ROcp18_818 = -C17*S18-S17*C18
    ROcp18_918 = C17*C18-S17*S18
    RLcp18_24 = s.dpt[2,13]*C17
    RLcp18_34 = s.dpt[2,13]*S17
    POcp18_24 = RLcp18_24+q[15]
    POcp18_34 = RLcp18_34+q[16]
    OMcp18_14 = qd[17]+qd[18]
    ORcp18_24 = -RLcp18_34*qd[17]
    ORcp18_34 = RLcp18_24*qd[17]
    VIcp18_24 = ORcp18_24+qd[15]
    VIcp18_34 = ORcp18_34+qd[16]
    OPcp18_14 = qdd[17]+qdd[18]
    ACcp18_24 = qdd[15]-ORcp18_34*qd[17]-RLcp18_34*qdd[17]
    ACcp18_34 = qdd[16]+ORcp18_24*qd[17]+RLcp18_24*qdd[17]
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
    sens.J[2,15] = (1.0)
    sens.J[2,17] = -RLcp18_34
    sens.J[3,16] = (1.0)
    sens.J[3,17] = RLcp18_24
    sens.J[4,17] = (1.0)
    sens.J[4,18] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp18_24
    sens.A[3] = ACcp18_34
    sens.OMP[1] = OPcp18_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 19): 

    ROcp19_518 = C17*C18-S17*S18
    ROcp19_618 = C17*S18+S17*C18
    ROcp19_818 = -C17*S18-S17*C18
    ROcp19_918 = C17*C18-S17*S18
    ROcp19_519 = ROcp19_518*C19+ROcp19_818*S19
    ROcp19_619 = ROcp19_618*C19+ROcp19_918*S19
    ROcp19_819 = -ROcp19_518*S19+ROcp19_818*C19
    ROcp19_919 = -ROcp19_618*S19+ROcp19_918*C19
    RLcp19_24 = s.dpt[2,13]*C17
    RLcp19_34 = s.dpt[2,13]*S17
    POcp19_24 = RLcp19_24+q[15]
    POcp19_34 = RLcp19_34+q[16]
    OMcp19_14 = qd[17]+qd[18]
    ORcp19_24 = -RLcp19_34*qd[17]
    ORcp19_34 = RLcp19_24*qd[17]
    VIcp19_24 = ORcp19_24+qd[15]
    VIcp19_34 = ORcp19_34+qd[16]
    OPcp19_14 = qdd[17]+qdd[18]
    ACcp19_24 = qdd[15]-ORcp19_34*qd[17]-RLcp19_34*qdd[17]
    ACcp19_34 = qdd[16]+ORcp19_24*qd[17]+RLcp19_24*qdd[17]
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
    sens.J[2,15] = (1.0)
    sens.J[2,17] = JTcp19_25_3
    sens.J[2,18] = -RLcp19_35
    sens.J[3,16] = (1.0)
    sens.J[3,17] = JTcp19_35_3
    sens.J[3,18] = RLcp19_25
    sens.J[4,17] = (1.0)
    sens.J[4,18] = (1.0)
    sens.J[4,19] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp19_25
    sens.A[3] = ACcp19_35
    sens.OMP[1] = OPcp19_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 20): 

    ROcp20_520 = C17*C20-S17*S20
    ROcp20_620 = C17*S20+S17*C20
    ROcp20_820 = -C17*S20-S17*C20
    ROcp20_920 = C17*C20-S17*S20
    RLcp20_24 = s.dpt[2,14]*C17
    RLcp20_34 = s.dpt[2,14]*S17
    POcp20_24 = RLcp20_24+q[15]
    POcp20_34 = RLcp20_34+q[16]
    OMcp20_14 = qd[17]+qd[20]
    ORcp20_24 = -RLcp20_34*qd[17]
    ORcp20_34 = RLcp20_24*qd[17]
    VIcp20_24 = ORcp20_24+qd[15]
    VIcp20_34 = ORcp20_34+qd[16]
    OPcp20_14 = qdd[17]+qdd[20]
    ACcp20_24 = qdd[15]-ORcp20_34*qd[17]-RLcp20_34*qdd[17]
    ACcp20_34 = qdd[16]+ORcp20_24*qd[17]+RLcp20_24*qdd[17]
    sens.P[1] = 0
    sens.P[2] = POcp20_24
    sens.P[3] = POcp20_34
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp20_520
    sens.R[2,3] = ROcp20_620
    sens.R[3,2] = ROcp20_820
    sens.R[3,3] = ROcp20_920
    sens.V[1] = 0
    sens.V[2] = VIcp20_24
    sens.V[3] = VIcp20_34
    sens.OM[1] = OMcp20_14
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,15] = (1.0)
    sens.J[2,17] = -RLcp20_34
    sens.J[3,16] = (1.0)
    sens.J[3,17] = RLcp20_24
    sens.J[4,17] = (1.0)
    sens.J[4,20] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp20_24
    sens.A[3] = ACcp20_34
    sens.OMP[1] = OPcp20_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 21): 

    ROcp21_520 = C17*C20-S17*S20
    ROcp21_620 = C17*S20+S17*C20
    ROcp21_820 = -C17*S20-S17*C20
    ROcp21_920 = C17*C20-S17*S20
    ROcp21_521 = ROcp21_520*C21+ROcp21_820*S21
    ROcp21_621 = ROcp21_620*C21+ROcp21_920*S21
    ROcp21_821 = -ROcp21_520*S21+ROcp21_820*C21
    ROcp21_921 = -ROcp21_620*S21+ROcp21_920*C21
    RLcp21_24 = s.dpt[2,14]*C17
    RLcp21_34 = s.dpt[2,14]*S17
    POcp21_24 = RLcp21_24+q[15]
    POcp21_34 = RLcp21_34+q[16]
    OMcp21_14 = qd[17]+qd[20]
    ORcp21_24 = -RLcp21_34*qd[17]
    ORcp21_34 = RLcp21_24*qd[17]
    VIcp21_24 = ORcp21_24+qd[15]
    VIcp21_34 = ORcp21_34+qd[16]
    OPcp21_14 = qdd[17]+qdd[20]
    ACcp21_24 = qdd[15]-ORcp21_34*qd[17]-RLcp21_34*qdd[17]
    ACcp21_34 = qdd[16]+ORcp21_24*qd[17]+RLcp21_24*qdd[17]
    RLcp21_25 = ROcp21_520*s.dpt[2,17]
    RLcp21_35 = ROcp21_620*s.dpt[2,17]
    POcp21_25 = POcp21_24+RLcp21_25
    POcp21_35 = POcp21_34+RLcp21_35
    JTcp21_25_3 = -RLcp21_34-RLcp21_35
    JTcp21_35_3 = RLcp21_24+RLcp21_25
    OMcp21_15 = OMcp21_14+qd[21]
    ORcp21_25 = -OMcp21_14*RLcp21_35
    ORcp21_35 = OMcp21_14*RLcp21_25
    VIcp21_25 = ORcp21_25+VIcp21_24
    VIcp21_35 = ORcp21_35+VIcp21_34
    OPcp21_15 = OPcp21_14+qdd[21]
    ACcp21_25 = ACcp21_24-OMcp21_14*ORcp21_35-OPcp21_14*RLcp21_35
    ACcp21_35 = ACcp21_34+OMcp21_14*ORcp21_25+OPcp21_14*RLcp21_25
    sens.P[1] = 0
    sens.P[2] = POcp21_25
    sens.P[3] = POcp21_35
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp21_521
    sens.R[2,3] = ROcp21_621
    sens.R[3,2] = ROcp21_821
    sens.R[3,3] = ROcp21_921
    sens.V[1] = 0
    sens.V[2] = VIcp21_25
    sens.V[3] = VIcp21_35
    sens.OM[1] = OMcp21_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[2,15] = (1.0)
    sens.J[2,17] = JTcp21_25_3
    sens.J[2,20] = -RLcp21_35
    sens.J[3,16] = (1.0)
    sens.J[3,17] = JTcp21_35_3
    sens.J[3,20] = RLcp21_25
    sens.J[4,17] = (1.0)
    sens.J[4,20] = (1.0)
    sens.J[4,21] = (1.0)
    sens.A[1] = 0
    sens.A[2] = ACcp21_25
    sens.A[3] = ACcp21_35
    sens.OMP[1] = OPcp21_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


