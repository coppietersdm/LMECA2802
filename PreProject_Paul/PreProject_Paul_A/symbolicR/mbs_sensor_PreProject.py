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

    ROcp1_54 = C3*C4-S3*S4
    ROcp1_64 = C3*S4+S3*C4
    ROcp1_84 = -C3*S4-S3*C4
    ROcp1_94 = C3*C4-S3*S4
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
    ORcp1_25 = -OMcp1_14*RLcp1_35
    ORcp1_35 = OMcp1_14*RLcp1_25
    VIcp1_25 = ORcp1_25+VIcp1_24
    VIcp1_35 = ORcp1_35+VIcp1_34
    ACcp1_25 = ACcp1_24-OMcp1_14*ORcp1_35-OPcp1_14*RLcp1_35
    ACcp1_35 = ACcp1_34+OMcp1_14*ORcp1_25+OPcp1_14*RLcp1_25
    sens.P[1] = 0
    sens.P[2] = POcp1_25
    sens.P[3] = POcp1_35
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp1_54
    sens.R[2,3] = ROcp1_64
    sens.R[3,2] = ROcp1_84
    sens.R[3,3] = ROcp1_94
    sens.V[1] = 0
    sens.V[2] = VIcp1_25
    sens.V[3] = VIcp1_35
    sens.OM[1] = OMcp1_14
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.A[1] = 0
    sens.A[2] = ACcp1_25
    sens.A[3] = ACcp1_35
    sens.OMP[1] = OPcp1_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 2): 

    ROcp2_55 = C3*C5-S3*S5
    ROcp2_65 = C3*S5+S3*C5
    ROcp2_85 = -C3*S5-S3*C5
    ROcp2_95 = C3*C5-S3*S5
    RLcp2_24 = s.dpt[2,2]*C3
    RLcp2_34 = s.dpt[2,2]*S3
    POcp2_24 = RLcp2_24+q[1]
    POcp2_34 = RLcp2_34+q[2]
    OMcp2_14 = qd[3]+qd[5]
    ORcp2_24 = -RLcp2_34*qd[3]
    ORcp2_34 = RLcp2_24*qd[3]
    VIcp2_24 = ORcp2_24+qd[1]
    VIcp2_34 = ORcp2_34+qd[2]
    OPcp2_14 = qdd[3]+qdd[5]
    ACcp2_24 = qdd[1]-ORcp2_34*qd[3]-RLcp2_34*qdd[3]
    ACcp2_34 = qdd[2]+ORcp2_24*qd[3]+RLcp2_24*qdd[3]
    RLcp2_25 = ROcp2_55*s.dpt[2,4]
    RLcp2_35 = ROcp2_65*s.dpt[2,4]
    POcp2_25 = POcp2_24+RLcp2_25
    POcp2_35 = POcp2_34+RLcp2_35
    ORcp2_25 = -OMcp2_14*RLcp2_35
    ORcp2_35 = OMcp2_14*RLcp2_25
    VIcp2_25 = ORcp2_25+VIcp2_24
    VIcp2_35 = ORcp2_35+VIcp2_34
    ACcp2_25 = ACcp2_24-OMcp2_14*ORcp2_35-OPcp2_14*RLcp2_35
    ACcp2_35 = ACcp2_34+OMcp2_14*ORcp2_25+OPcp2_14*RLcp2_25
    sens.P[1] = 0
    sens.P[2] = POcp2_25
    sens.P[3] = POcp2_35
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp2_55
    sens.R[2,3] = ROcp2_65
    sens.R[3,2] = ROcp2_85
    sens.R[3,3] = ROcp2_95
    sens.V[1] = 0
    sens.V[2] = VIcp2_25
    sens.V[3] = VIcp2_35
    sens.OM[1] = OMcp2_14
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.A[1] = 0
    sens.A[2] = ACcp2_25
    sens.A[3] = ACcp2_35
    sens.OMP[1] = OPcp2_14
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 3): 

    RLcp3_24 = s.dpt[2,6]*C8
    RLcp3_34 = s.dpt[2,6]*S8
    POcp3_24 = RLcp3_24+q[6]
    POcp3_34 = RLcp3_34+q[7]
    ORcp3_24 = -RLcp3_34*qd[8]
    ORcp3_34 = RLcp3_24*qd[8]
    VIcp3_24 = ORcp3_24+qd[6]
    VIcp3_34 = ORcp3_34+qd[7]
    ACcp3_24 = qdd[6]-ORcp3_34*qd[8]-RLcp3_34*qdd[8]
    ACcp3_34 = qdd[7]+ORcp3_24*qd[8]+RLcp3_24*qdd[8]
    sens.P[1] = 0
    sens.P[2] = POcp3_24
    sens.P[3] = POcp3_34
    sens.R[1,1] = (1.0)
    sens.R[2,2] = C8
    sens.R[2,3] = S8
    sens.R[3,2] = -S8
    sens.R[3,3] = C8
    sens.V[1] = 0
    sens.V[2] = VIcp3_24
    sens.V[3] = VIcp3_34
    sens.OM[1] = qd[8]
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.A[1] = 0
    sens.A[2] = ACcp3_24
    sens.A[3] = ACcp3_34
    sens.OMP[1] = qdd[8]
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 4): 

    ROcp4_59 = C8*C9-S8*S9
    ROcp4_69 = C8*S9+S8*C9
    ROcp4_89 = -C8*S9-S8*C9
    ROcp4_99 = C8*C9-S8*S9
    ROcp4_510 = ROcp4_59*C10+ROcp4_89*S10
    ROcp4_610 = ROcp4_69*C10+ROcp4_99*S10
    ROcp4_810 = -ROcp4_59*S10+ROcp4_89*C10
    ROcp4_910 = -ROcp4_69*S10+ROcp4_99*C10
    RLcp4_24 = s.dpt[2,5]*C8
    RLcp4_34 = s.dpt[2,5]*S8
    POcp4_24 = RLcp4_24+q[6]
    POcp4_34 = RLcp4_34+q[7]
    OMcp4_14 = qd[8]+qd[9]
    ORcp4_24 = -RLcp4_34*qd[8]
    ORcp4_34 = RLcp4_24*qd[8]
    VIcp4_24 = ORcp4_24+qd[6]
    VIcp4_34 = ORcp4_34+qd[7]
    OPcp4_14 = qdd[8]+qdd[9]
    ACcp4_24 = qdd[6]-ORcp4_34*qd[8]-RLcp4_34*qdd[8]
    ACcp4_34 = qdd[7]+ORcp4_24*qd[8]+RLcp4_24*qdd[8]
    RLcp4_25 = ROcp4_59*s.dpt[2,7]
    RLcp4_35 = ROcp4_69*s.dpt[2,7]
    POcp4_25 = POcp4_24+RLcp4_25
    POcp4_35 = POcp4_34+RLcp4_35
    OMcp4_15 = OMcp4_14+qd[10]
    ORcp4_25 = -OMcp4_14*RLcp4_35
    ORcp4_35 = OMcp4_14*RLcp4_25
    VIcp4_25 = ORcp4_25+VIcp4_24
    VIcp4_35 = ORcp4_35+VIcp4_34
    OPcp4_15 = OPcp4_14+qdd[10]
    ACcp4_25 = ACcp4_24-OMcp4_14*ORcp4_35-OPcp4_14*RLcp4_35
    ACcp4_35 = ACcp4_34+OMcp4_14*ORcp4_25+OPcp4_14*RLcp4_25
    RLcp4_26 = ROcp4_510*s.dpt[2,8]
    RLcp4_36 = ROcp4_610*s.dpt[2,8]
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
    sens.R[2,2] = ROcp4_510
    sens.R[2,3] = ROcp4_610
    sens.R[3,2] = ROcp4_810
    sens.R[3,3] = ROcp4_910
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

    ROcp5_511 = C11*C8-S11*S8
    ROcp5_611 = C11*S8+S11*C8
    ROcp5_811 = -C11*S8-S11*C8
    ROcp5_911 = C11*C8-S11*S8
    ROcp5_512 = ROcp5_511*C12+ROcp5_811*S12
    ROcp5_612 = ROcp5_611*C12+ROcp5_911*S12
    ROcp5_812 = -ROcp5_511*S12+ROcp5_811*C12
    ROcp5_912 = -ROcp5_611*S12+ROcp5_911*C12
    RLcp5_24 = s.dpt[2,6]*C8
    RLcp5_34 = s.dpt[2,6]*S8
    POcp5_24 = RLcp5_24+q[6]
    POcp5_34 = RLcp5_34+q[7]
    OMcp5_14 = qd[11]+qd[8]
    ORcp5_24 = -RLcp5_34*qd[8]
    ORcp5_34 = RLcp5_24*qd[8]
    VIcp5_24 = ORcp5_24+qd[6]
    VIcp5_34 = ORcp5_34+qd[7]
    OPcp5_14 = qdd[11]+qdd[8]
    ACcp5_24 = qdd[6]-ORcp5_34*qd[8]-RLcp5_34*qdd[8]
    ACcp5_34 = qdd[7]+ORcp5_24*qd[8]+RLcp5_24*qdd[8]
    RLcp5_25 = ROcp5_511*s.dpt[2,9]
    RLcp5_35 = ROcp5_611*s.dpt[2,9]
    POcp5_25 = POcp5_24+RLcp5_25
    POcp5_35 = POcp5_34+RLcp5_35
    OMcp5_15 = OMcp5_14+qd[12]
    ORcp5_25 = -OMcp5_14*RLcp5_35
    ORcp5_35 = OMcp5_14*RLcp5_25
    VIcp5_25 = ORcp5_25+VIcp5_24
    VIcp5_35 = ORcp5_35+VIcp5_34
    OPcp5_15 = OPcp5_14+qdd[12]
    ACcp5_25 = ACcp5_24-OMcp5_14*ORcp5_35-OPcp5_14*RLcp5_35
    ACcp5_35 = ACcp5_34+OMcp5_14*ORcp5_25+OPcp5_14*RLcp5_25
    RLcp5_26 = ROcp5_512*s.dpt[2,10]
    RLcp5_36 = ROcp5_612*s.dpt[2,10]
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
    sens.R[2,2] = ROcp5_512
    sens.R[2,3] = ROcp5_612
    sens.R[3,2] = ROcp5_812
    sens.R[3,3] = ROcp5_912
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

    RLcp6_24 = s.dpt[2,11]*C15
    RLcp6_34 = s.dpt[2,11]*S15
    POcp6_24 = RLcp6_24+q[13]
    POcp6_34 = RLcp6_34+q[14]
    ORcp6_24 = -RLcp6_34*qd[15]
    ORcp6_34 = RLcp6_24*qd[15]
    VIcp6_24 = ORcp6_24+qd[13]
    VIcp6_34 = ORcp6_34+qd[14]
    ACcp6_24 = qdd[13]-ORcp6_34*qd[15]-RLcp6_34*qdd[15]
    ACcp6_34 = qdd[14]+ORcp6_24*qd[15]+RLcp6_24*qdd[15]
    sens.P[1] = 0
    sens.P[2] = POcp6_24
    sens.P[3] = POcp6_34
    sens.R[1,1] = (1.0)
    sens.R[2,2] = C15
    sens.R[2,3] = S15
    sens.R[3,2] = -S15
    sens.R[3,3] = C15
    sens.V[1] = 0
    sens.V[2] = VIcp6_24
    sens.V[3] = VIcp6_34
    sens.OM[1] = qd[15]
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.A[1] = 0
    sens.A[2] = ACcp6_24
    sens.A[3] = ACcp6_34
    sens.OMP[1] = qdd[15]
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 7): 

    ROcp7_516 = C15*C16-S15*S16
    ROcp7_616 = C15*S16+S15*C16
    ROcp7_816 = -C15*S16-S15*C16
    ROcp7_916 = C15*C16-S15*S16
    ROcp7_517 = ROcp7_516*C17+ROcp7_816*S17
    ROcp7_617 = ROcp7_616*C17+ROcp7_916*S17
    ROcp7_817 = -ROcp7_516*S17+ROcp7_816*C17
    ROcp7_917 = -ROcp7_616*S17+ROcp7_916*C17
    RLcp7_24 = s.dpt[2,11]*C15
    RLcp7_34 = s.dpt[2,11]*S15
    POcp7_24 = RLcp7_24+q[13]
    POcp7_34 = RLcp7_34+q[14]
    OMcp7_14 = qd[15]+qd[16]
    ORcp7_24 = -RLcp7_34*qd[15]
    ORcp7_34 = RLcp7_24*qd[15]
    VIcp7_24 = ORcp7_24+qd[13]
    VIcp7_34 = ORcp7_34+qd[14]
    OPcp7_14 = qdd[15]+qdd[16]
    ACcp7_24 = qdd[13]-ORcp7_34*qd[15]-RLcp7_34*qdd[15]
    ACcp7_34 = qdd[14]+ORcp7_24*qd[15]+RLcp7_24*qdd[15]
    RLcp7_25 = ROcp7_516*s.dpt[2,13]
    RLcp7_35 = ROcp7_616*s.dpt[2,13]
    POcp7_25 = POcp7_24+RLcp7_25
    POcp7_35 = POcp7_34+RLcp7_35
    OMcp7_15 = OMcp7_14+qd[17]
    ORcp7_25 = -OMcp7_14*RLcp7_35
    ORcp7_35 = OMcp7_14*RLcp7_25
    VIcp7_25 = ORcp7_25+VIcp7_24
    VIcp7_35 = ORcp7_35+VIcp7_34
    OPcp7_15 = OPcp7_14+qdd[17]
    ACcp7_25 = ACcp7_24-OMcp7_14*ORcp7_35-OPcp7_14*RLcp7_35
    ACcp7_35 = ACcp7_34+OMcp7_14*ORcp7_25+OPcp7_14*RLcp7_25
    RLcp7_26 = ROcp7_517*s.dpt[2,14]
    RLcp7_36 = ROcp7_617*s.dpt[2,14]
    POcp7_26 = POcp7_25+RLcp7_26
    POcp7_36 = POcp7_35+RLcp7_36
    ORcp7_26 = -OMcp7_15*RLcp7_36
    ORcp7_36 = OMcp7_15*RLcp7_26
    VIcp7_26 = ORcp7_26+VIcp7_25
    VIcp7_36 = ORcp7_36+VIcp7_35
    ACcp7_26 = ACcp7_25-OMcp7_15*ORcp7_36-OPcp7_15*RLcp7_36
    ACcp7_36 = ACcp7_35+OMcp7_15*ORcp7_26+OPcp7_15*RLcp7_26
    sens.P[1] = 0
    sens.P[2] = POcp7_26
    sens.P[3] = POcp7_36
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp7_517
    sens.R[2,3] = ROcp7_617
    sens.R[3,2] = ROcp7_817
    sens.R[3,3] = ROcp7_917
    sens.V[1] = 0
    sens.V[2] = VIcp7_26
    sens.V[3] = VIcp7_36
    sens.OM[1] = OMcp7_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.A[1] = 0
    sens.A[2] = ACcp7_26
    sens.A[3] = ACcp7_36
    sens.OMP[1] = OPcp7_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 8): 

    ROcp8_518 = C15*C18-S15*S18
    ROcp8_618 = C15*S18+S15*C18
    ROcp8_818 = -C15*S18-S15*C18
    ROcp8_918 = C15*C18-S15*S18
    ROcp8_519 = ROcp8_518*C19+ROcp8_818*S19
    ROcp8_619 = ROcp8_618*C19+ROcp8_918*S19
    ROcp8_819 = -ROcp8_518*S19+ROcp8_818*C19
    ROcp8_919 = -ROcp8_618*S19+ROcp8_918*C19
    RLcp8_24 = s.dpt[2,12]*C15
    RLcp8_34 = s.dpt[2,12]*S15
    POcp8_24 = RLcp8_24+q[13]
    POcp8_34 = RLcp8_34+q[14]
    OMcp8_14 = qd[15]+qd[18]
    ORcp8_24 = -RLcp8_34*qd[15]
    ORcp8_34 = RLcp8_24*qd[15]
    VIcp8_24 = ORcp8_24+qd[13]
    VIcp8_34 = ORcp8_34+qd[14]
    OPcp8_14 = qdd[15]+qdd[18]
    ACcp8_24 = qdd[13]-ORcp8_34*qd[15]-RLcp8_34*qdd[15]
    ACcp8_34 = qdd[14]+ORcp8_24*qd[15]+RLcp8_24*qdd[15]
    RLcp8_25 = ROcp8_518*s.dpt[2,15]
    RLcp8_35 = ROcp8_618*s.dpt[2,15]
    POcp8_25 = POcp8_24+RLcp8_25
    POcp8_35 = POcp8_34+RLcp8_35
    OMcp8_15 = OMcp8_14+qd[19]
    ORcp8_25 = -OMcp8_14*RLcp8_35
    ORcp8_35 = OMcp8_14*RLcp8_25
    VIcp8_25 = ORcp8_25+VIcp8_24
    VIcp8_35 = ORcp8_35+VIcp8_34
    OPcp8_15 = OPcp8_14+qdd[19]
    ACcp8_25 = ACcp8_24-OMcp8_14*ORcp8_35-OPcp8_14*RLcp8_35
    ACcp8_35 = ACcp8_34+OMcp8_14*ORcp8_25+OPcp8_14*RLcp8_25
    RLcp8_26 = ROcp8_519*s.dpt[2,16]
    RLcp8_36 = ROcp8_619*s.dpt[2,16]
    POcp8_26 = POcp8_25+RLcp8_26
    POcp8_36 = POcp8_35+RLcp8_36
    ORcp8_26 = -OMcp8_15*RLcp8_36
    ORcp8_36 = OMcp8_15*RLcp8_26
    VIcp8_26 = ORcp8_26+VIcp8_25
    VIcp8_36 = ORcp8_36+VIcp8_35
    ACcp8_26 = ACcp8_25-OMcp8_15*ORcp8_36-OPcp8_15*RLcp8_36
    ACcp8_36 = ACcp8_35+OMcp8_15*ORcp8_26+OPcp8_15*RLcp8_26
    sens.P[1] = 0
    sens.P[2] = POcp8_26
    sens.P[3] = POcp8_36
    sens.R[1,1] = (1.0)
    sens.R[2,2] = ROcp8_519
    sens.R[2,3] = ROcp8_619
    sens.R[3,2] = ROcp8_819
    sens.R[3,3] = ROcp8_919
    sens.V[1] = 0
    sens.V[2] = VIcp8_26
    sens.V[3] = VIcp8_36
    sens.OM[1] = OMcp8_15
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.A[1] = 0
    sens.A[2] = ACcp8_26
    sens.A[3] = ACcp8_36
    sens.OMP[1] = OPcp8_15
    sens.OMP[2] = 0
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


