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
#	==> Generation Date: Mon Apr 22 01:26:51 2024
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

    ROcp1_54 = C3*C4-S3*S4
    ROcp1_64 = C3*S4+S3*C4
    ROcp1_84 = -C3*S4-S3*C4
    ROcp1_94 = C3*C4-S3*S4
    ROcp1_55 = ROcp1_54*C5+ROcp1_84*S5
    ROcp1_65 = ROcp1_64*C5+ROcp1_94*S5
    ROcp1_85 = -ROcp1_54*S5+ROcp1_84*C5
    ROcp1_95 = -ROcp1_64*S5+ROcp1_94*C5
    RLcp1_24 = s.dpt[2,1]*C3
    RLcp1_34 = s.dpt[2,1]*S3
    POcp1_24 = RLcp1_24+q[1]
    POcp1_34 = RLcp1_34+q[2]
    OMcp1_14 = qd[3]+qd[4]
    ORcp1_24 = -RLcp1_34*qd[3]
    ORcp1_34 = RLcp1_24*qd[3]
    VIcp1_24 = ORcp1_24+qd[1]
    VIcp1_34 = ORcp1_34+qd[2]
    OPcp1_14 = qdd[3]+qdd[4]
    ACcp1_24 = qdd[1]-ORcp1_34*qd[3]-RLcp1_34*qdd[3]
    ACcp1_34 = qdd[2]+ORcp1_24*qd[3]+RLcp1_24*qdd[3]
    RLcp1_25 = ROcp1_54*s.dpt[2,3]
    RLcp1_35 = ROcp1_64*s.dpt[2,3]
    POcp1_25 = POcp1_24+RLcp1_25
    POcp1_35 = POcp1_34+RLcp1_35
    OMcp1_15 = OMcp1_14+qd[5]
    ORcp1_25 = -OMcp1_14*RLcp1_35
    ORcp1_35 = OMcp1_14*RLcp1_25
    VIcp1_25 = ORcp1_25+VIcp1_24
    VIcp1_35 = ORcp1_35+VIcp1_34
    OPcp1_15 = OPcp1_14+qdd[5]
    ACcp1_25 = ACcp1_24-OMcp1_14*ORcp1_35-OPcp1_14*RLcp1_35
    ACcp1_35 = ACcp1_34+OMcp1_14*ORcp1_25+OPcp1_14*RLcp1_25
    RLcp1_26 = ROcp1_55*s.dpt[2,4]
    RLcp1_36 = ROcp1_65*s.dpt[2,4]
    POcp1_26 = POcp1_25+RLcp1_26
    POcp1_36 = POcp1_35+RLcp1_36
    ORcp1_26 = -OMcp1_15*RLcp1_36
    ORcp1_36 = OMcp1_15*RLcp1_26
    VIcp1_26 = ORcp1_26+VIcp1_25
    VIcp1_36 = ORcp1_36+VIcp1_35
    ACcp1_26 = ACcp1_25-OMcp1_15*ORcp1_36-OPcp1_15*RLcp1_36
    ACcp1_36 = ACcp1_35+OMcp1_15*ORcp1_26+OPcp1_15*RLcp1_26
    sens.P[1] = 0
    sens.P[2] = POcp1_26
    sens.P[3] = POcp1_36
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp1_55
    sens.R[2,3] = ROcp1_65
    sens.R[3,2] = ROcp1_85
    sens.R[3,3] = ROcp1_95
    sens.V[1] = 0
    sens.V[2] = VIcp1_26
    sens.V[3] = VIcp1_36
    sens.OM[1] = OMcp1_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.A[1] = 0
    sens.A[2] = ACcp1_26
    sens.A[3] = ACcp1_36
    sens.OMP[1] = OPcp1_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 2): 

    ROcp2_56 = C3*C6-S3*S6
    ROcp2_66 = C3*S6+S3*C6
    ROcp2_86 = -C3*S6-S3*C6
    ROcp2_96 = C3*C6-S3*S6
    ROcp2_57 = ROcp2_56*C7+ROcp2_86*S7
    ROcp2_67 = ROcp2_66*C7+ROcp2_96*S7
    ROcp2_87 = -ROcp2_56*S7+ROcp2_86*C7
    ROcp2_97 = -ROcp2_66*S7+ROcp2_96*C7
    RLcp2_24 = s.dpt[2,2]*C3
    RLcp2_34 = s.dpt[2,2]*S3
    POcp2_24 = RLcp2_24+q[1]
    POcp2_34 = RLcp2_34+q[2]
    OMcp2_14 = qd[3]+qd[6]
    ORcp2_24 = -RLcp2_34*qd[3]
    ORcp2_34 = RLcp2_24*qd[3]
    VIcp2_24 = ORcp2_24+qd[1]
    VIcp2_34 = ORcp2_34+qd[2]
    OPcp2_14 = qdd[3]+qdd[6]
    ACcp2_24 = qdd[1]-ORcp2_34*qd[3]-RLcp2_34*qdd[3]
    ACcp2_34 = qdd[2]+ORcp2_24*qd[3]+RLcp2_24*qdd[3]
    RLcp2_25 = ROcp2_56*s.dpt[2,5]
    RLcp2_35 = ROcp2_66*s.dpt[2,5]
    POcp2_25 = POcp2_24+RLcp2_25
    POcp2_35 = POcp2_34+RLcp2_35
    OMcp2_15 = OMcp2_14+qd[7]
    ORcp2_25 = -OMcp2_14*RLcp2_35
    ORcp2_35 = OMcp2_14*RLcp2_25
    VIcp2_25 = ORcp2_25+VIcp2_24
    VIcp2_35 = ORcp2_35+VIcp2_34
    OPcp2_15 = OPcp2_14+qdd[7]
    ACcp2_25 = ACcp2_24-OMcp2_14*ORcp2_35-OPcp2_14*RLcp2_35
    ACcp2_35 = ACcp2_34+OMcp2_14*ORcp2_25+OPcp2_14*RLcp2_25
    RLcp2_26 = ROcp2_57*s.dpt[2,6]
    RLcp2_36 = ROcp2_67*s.dpt[2,6]
    POcp2_26 = POcp2_25+RLcp2_26
    POcp2_36 = POcp2_35+RLcp2_36
    ORcp2_26 = -OMcp2_15*RLcp2_36
    ORcp2_36 = OMcp2_15*RLcp2_26
    VIcp2_26 = ORcp2_26+VIcp2_25
    VIcp2_36 = ORcp2_36+VIcp2_35
    ACcp2_26 = ACcp2_25-OMcp2_15*ORcp2_36-OPcp2_15*RLcp2_36
    ACcp2_36 = ACcp2_35+OMcp2_15*ORcp2_26+OPcp2_15*RLcp2_26
    sens.P[1] = 0
    sens.P[2] = POcp2_26
    sens.P[3] = POcp2_36
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp2_57
    sens.R[2,3] = ROcp2_67
    sens.R[3,2] = ROcp2_87
    sens.R[3,3] = ROcp2_97
    sens.V[1] = 0
    sens.V[2] = VIcp2_26
    sens.V[3] = VIcp2_36
    sens.OM[1] = OMcp2_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.A[1] = 0
    sens.A[2] = ACcp2_26
    sens.A[3] = ACcp2_36
    sens.OMP[1] = OPcp2_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 3): 

    ROcp3_511 = C10*C11-S10*S11
    ROcp3_611 = C10*S11+S10*C11
    ROcp3_811 = -C10*S11-S10*C11
    ROcp3_911 = C10*C11-S10*S11
    ROcp3_512 = ROcp3_511*C12+ROcp3_811*S12
    ROcp3_612 = ROcp3_611*C12+ROcp3_911*S12
    ROcp3_812 = -ROcp3_511*S12+ROcp3_811*C12
    ROcp3_912 = -ROcp3_611*S12+ROcp3_911*C12
    RLcp3_24 = s.dpt[2,7]*C10
    RLcp3_34 = s.dpt[2,7]*S10
    POcp3_24 = RLcp3_24+q[8]
    POcp3_34 = RLcp3_34+q[9]
    OMcp3_14 = qd[10]+qd[11]
    ORcp3_24 = -RLcp3_34*qd[10]
    ORcp3_34 = RLcp3_24*qd[10]
    VIcp3_24 = ORcp3_24+qd[8]
    VIcp3_34 = ORcp3_34+qd[9]
    OPcp3_14 = qdd[10]+qdd[11]
    ACcp3_24 = qdd[8]-ORcp3_34*qd[10]-RLcp3_34*qdd[10]
    ACcp3_34 = qdd[9]+ORcp3_24*qd[10]+RLcp3_24*qdd[10]
    RLcp3_25 = ROcp3_511*s.dpt[2,9]
    RLcp3_35 = ROcp3_611*s.dpt[2,9]
    POcp3_25 = POcp3_24+RLcp3_25
    POcp3_35 = POcp3_34+RLcp3_35
    OMcp3_15 = OMcp3_14+qd[12]
    ORcp3_25 = -OMcp3_14*RLcp3_35
    ORcp3_35 = OMcp3_14*RLcp3_25
    VIcp3_25 = ORcp3_25+VIcp3_24
    VIcp3_35 = ORcp3_35+VIcp3_34
    OPcp3_15 = OPcp3_14+qdd[12]
    ACcp3_25 = ACcp3_24-OMcp3_14*ORcp3_35-OPcp3_14*RLcp3_35
    ACcp3_35 = ACcp3_34+OMcp3_14*ORcp3_25+OPcp3_14*RLcp3_25
    RLcp3_26 = ROcp3_512*s.dpt[2,10]
    RLcp3_36 = ROcp3_612*s.dpt[2,10]
    POcp3_26 = POcp3_25+RLcp3_26
    POcp3_36 = POcp3_35+RLcp3_36
    ORcp3_26 = -OMcp3_15*RLcp3_36
    ORcp3_36 = OMcp3_15*RLcp3_26
    VIcp3_26 = ORcp3_26+VIcp3_25
    VIcp3_36 = ORcp3_36+VIcp3_35
    ACcp3_26 = ACcp3_25-OMcp3_15*ORcp3_36-OPcp3_15*RLcp3_36
    ACcp3_36 = ACcp3_35+OMcp3_15*ORcp3_26+OPcp3_15*RLcp3_26
    sens.P[1] = 0
    sens.P[2] = POcp3_26
    sens.P[3] = POcp3_36
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp3_512
    sens.R[2,3] = ROcp3_612
    sens.R[3,2] = ROcp3_812
    sens.R[3,3] = ROcp3_912
    sens.V[1] = 0
    sens.V[2] = VIcp3_26
    sens.V[3] = VIcp3_36
    sens.OM[1] = OMcp3_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.A[1] = 0
    sens.A[2] = ACcp3_26
    sens.A[3] = ACcp3_36
    sens.OMP[1] = OPcp3_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 4): 

    ROcp4_513 = C10*C13-S10*S13
    ROcp4_613 = C10*S13+S10*C13
    ROcp4_813 = -C10*S13-S10*C13
    ROcp4_913 = C10*C13-S10*S13
    ROcp4_514 = ROcp4_513*C14+ROcp4_813*S14
    ROcp4_614 = ROcp4_613*C14+ROcp4_913*S14
    ROcp4_814 = -ROcp4_513*S14+ROcp4_813*C14
    ROcp4_914 = -ROcp4_613*S14+ROcp4_913*C14
    RLcp4_24 = s.dpt[2,8]*C10
    RLcp4_34 = s.dpt[2,8]*S10
    POcp4_24 = RLcp4_24+q[8]
    POcp4_34 = RLcp4_34+q[9]
    OMcp4_14 = qd[10]+qd[13]
    ORcp4_24 = -RLcp4_34*qd[10]
    ORcp4_34 = RLcp4_24*qd[10]
    VIcp4_24 = ORcp4_24+qd[8]
    VIcp4_34 = ORcp4_34+qd[9]
    OPcp4_14 = qdd[10]+qdd[13]
    ACcp4_24 = qdd[8]-ORcp4_34*qd[10]-RLcp4_34*qdd[10]
    ACcp4_34 = qdd[9]+ORcp4_24*qd[10]+RLcp4_24*qdd[10]
    RLcp4_25 = ROcp4_513*s.dpt[2,11]
    RLcp4_35 = ROcp4_613*s.dpt[2,11]
    POcp4_25 = POcp4_24+RLcp4_25
    POcp4_35 = POcp4_34+RLcp4_35
    OMcp4_15 = OMcp4_14+qd[14]
    ORcp4_25 = -OMcp4_14*RLcp4_35
    ORcp4_35 = OMcp4_14*RLcp4_25
    VIcp4_25 = ORcp4_25+VIcp4_24
    VIcp4_35 = ORcp4_35+VIcp4_34
    OPcp4_15 = OPcp4_14+qdd[14]
    ACcp4_25 = ACcp4_24-OMcp4_14*ORcp4_35-OPcp4_14*RLcp4_35
    ACcp4_35 = ACcp4_34+OMcp4_14*ORcp4_25+OPcp4_14*RLcp4_25
    RLcp4_26 = ROcp4_514*s.dpt[2,12]
    RLcp4_36 = ROcp4_614*s.dpt[2,12]
    POcp4_26 = POcp4_25+RLcp4_26
    POcp4_36 = POcp4_35+RLcp4_36
    ORcp4_26 = -OMcp4_15*RLcp4_36
    ORcp4_36 = OMcp4_15*RLcp4_26
    VIcp4_26 = ORcp4_26+VIcp4_25
    VIcp4_36 = ORcp4_36+VIcp4_35
    ACcp4_26 = ACcp4_25-OMcp4_15*ORcp4_36-OPcp4_15*RLcp4_36
    ACcp4_36 = ACcp4_35+OMcp4_15*ORcp4_26+OPcp4_15*RLcp4_26
    sens.P[1] = 0
    sens.P[2] = POcp4_26
    sens.P[3] = POcp4_36
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp4_514
    sens.R[2,3] = ROcp4_614
    sens.R[3,2] = ROcp4_814
    sens.R[3,3] = ROcp4_914
    sens.V[1] = 0
    sens.V[2] = VIcp4_26
    sens.V[3] = VIcp4_36
    sens.OM[1] = OMcp4_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.A[1] = 0
    sens.A[2] = ACcp4_26
    sens.A[3] = ACcp4_36
    sens.OMP[1] = OPcp4_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 5): 

    ROcp5_518 = C17*C18-S17*S18
    ROcp5_618 = C17*S18+S17*C18
    ROcp5_818 = -C17*S18-S17*C18
    ROcp5_918 = C17*C18-S17*S18
    ROcp5_519 = ROcp5_518*C19+ROcp5_818*S19
    ROcp5_619 = ROcp5_618*C19+ROcp5_918*S19
    ROcp5_819 = -ROcp5_518*S19+ROcp5_818*C19
    ROcp5_919 = -ROcp5_618*S19+ROcp5_918*C19
    RLcp5_24 = s.dpt[2,13]*C17
    RLcp5_34 = s.dpt[2,13]*S17
    POcp5_24 = RLcp5_24+q[15]
    POcp5_34 = RLcp5_34+q[16]
    OMcp5_14 = qd[17]+qd[18]
    ORcp5_24 = -RLcp5_34*qd[17]
    ORcp5_34 = RLcp5_24*qd[17]
    VIcp5_24 = ORcp5_24+qd[15]
    VIcp5_34 = ORcp5_34+qd[16]
    OPcp5_14 = qdd[17]+qdd[18]
    ACcp5_24 = qdd[15]-ORcp5_34*qd[17]-RLcp5_34*qdd[17]
    ACcp5_34 = qdd[16]+ORcp5_24*qd[17]+RLcp5_24*qdd[17]
    RLcp5_25 = ROcp5_518*s.dpt[2,15]
    RLcp5_35 = ROcp5_618*s.dpt[2,15]
    POcp5_25 = POcp5_24+RLcp5_25
    POcp5_35 = POcp5_34+RLcp5_35
    OMcp5_15 = OMcp5_14+qd[19]
    ORcp5_25 = -OMcp5_14*RLcp5_35
    ORcp5_35 = OMcp5_14*RLcp5_25
    VIcp5_25 = ORcp5_25+VIcp5_24
    VIcp5_35 = ORcp5_35+VIcp5_34
    OPcp5_15 = OPcp5_14+qdd[19]
    ACcp5_25 = ACcp5_24-OMcp5_14*ORcp5_35-OPcp5_14*RLcp5_35
    ACcp5_35 = ACcp5_34+OMcp5_14*ORcp5_25+OPcp5_14*RLcp5_25
    RLcp5_26 = ROcp5_519*s.dpt[2,16]
    RLcp5_36 = ROcp5_619*s.dpt[2,16]
    POcp5_26 = POcp5_25+RLcp5_26
    POcp5_36 = POcp5_35+RLcp5_36
    ORcp5_26 = -OMcp5_15*RLcp5_36
    ORcp5_36 = OMcp5_15*RLcp5_26
    VIcp5_26 = ORcp5_26+VIcp5_25
    VIcp5_36 = ORcp5_36+VIcp5_35
    ACcp5_26 = ACcp5_25-OMcp5_15*ORcp5_36-OPcp5_15*RLcp5_36
    ACcp5_36 = ACcp5_35+OMcp5_15*ORcp5_26+OPcp5_15*RLcp5_26
    sens.P[1] = 0
    sens.P[2] = POcp5_26
    sens.P[3] = POcp5_36
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp5_519
    sens.R[2,3] = ROcp5_619
    sens.R[3,2] = ROcp5_819
    sens.R[3,3] = ROcp5_919
    sens.V[1] = 0
    sens.V[2] = VIcp5_26
    sens.V[3] = VIcp5_36
    sens.OM[1] = OMcp5_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.A[1] = 0
    sens.A[2] = ACcp5_26
    sens.A[3] = ACcp5_36
    sens.OMP[1] = OPcp5_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 6): 

    ROcp6_520 = C17*C20-S17*S20
    ROcp6_620 = C17*S20+S17*C20
    ROcp6_820 = -C17*S20-S17*C20
    ROcp6_920 = C17*C20-S17*S20
    ROcp6_521 = ROcp6_520*C21+ROcp6_820*S21
    ROcp6_621 = ROcp6_620*C21+ROcp6_920*S21
    ROcp6_821 = -ROcp6_520*S21+ROcp6_820*C21
    ROcp6_921 = -ROcp6_620*S21+ROcp6_920*C21
    RLcp6_24 = s.dpt[2,14]*C17
    RLcp6_34 = s.dpt[2,14]*S17
    POcp6_24 = RLcp6_24+q[15]
    POcp6_34 = RLcp6_34+q[16]
    OMcp6_14 = qd[17]+qd[20]
    ORcp6_24 = -RLcp6_34*qd[17]
    ORcp6_34 = RLcp6_24*qd[17]
    VIcp6_24 = ORcp6_24+qd[15]
    VIcp6_34 = ORcp6_34+qd[16]
    OPcp6_14 = qdd[17]+qdd[20]
    ACcp6_24 = qdd[15]-ORcp6_34*qd[17]-RLcp6_34*qdd[17]
    ACcp6_34 = qdd[16]+ORcp6_24*qd[17]+RLcp6_24*qdd[17]
    RLcp6_25 = ROcp6_520*s.dpt[2,17]
    RLcp6_35 = ROcp6_620*s.dpt[2,17]
    POcp6_25 = POcp6_24+RLcp6_25
    POcp6_35 = POcp6_34+RLcp6_35
    OMcp6_15 = OMcp6_14+qd[21]
    ORcp6_25 = -OMcp6_14*RLcp6_35
    ORcp6_35 = OMcp6_14*RLcp6_25
    VIcp6_25 = ORcp6_25+VIcp6_24
    VIcp6_35 = ORcp6_35+VIcp6_34
    OPcp6_15 = OPcp6_14+qdd[21]
    ACcp6_25 = ACcp6_24-OMcp6_14*ORcp6_35-OPcp6_14*RLcp6_35
    ACcp6_35 = ACcp6_34+OMcp6_14*ORcp6_25+OPcp6_14*RLcp6_25
    RLcp6_26 = ROcp6_521*s.dpt[2,18]
    RLcp6_36 = ROcp6_621*s.dpt[2,18]
    POcp6_26 = POcp6_25+RLcp6_26
    POcp6_36 = POcp6_35+RLcp6_36
    ORcp6_26 = -OMcp6_15*RLcp6_36
    ORcp6_36 = OMcp6_15*RLcp6_26
    VIcp6_26 = ORcp6_26+VIcp6_25
    VIcp6_36 = ORcp6_36+VIcp6_35
    ACcp6_26 = ACcp6_25-OMcp6_15*ORcp6_36-OPcp6_15*RLcp6_36
    ACcp6_36 = ACcp6_35+OMcp6_15*ORcp6_26+OPcp6_15*RLcp6_26
    sens.P[1] = 0
    sens.P[2] = POcp6_26
    sens.P[3] = POcp6_36
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp6_521
    sens.R[2,3] = ROcp6_621
    sens.R[3,2] = ROcp6_821
    sens.R[3,3] = ROcp6_921
    sens.V[1] = 0
    sens.V[2] = VIcp6_26
    sens.V[3] = VIcp6_36
    sens.OM[1] = OMcp6_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.A[1] = 0
    sens.A[2] = ACcp6_26
    sens.A[3] = ACcp6_36
    sens.OMP[1] = OPcp6_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


