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

    PxF2 = zeros(4)
    RxF2 = zeros((4, 4))
    VxF2 = zeros(4)
    OMxF2 = zeros(4)
    AxF2 = zeros(4)
    OMPxF2 = zeros(4)

    PxF3 = zeros(4)
    RxF3 = zeros((4, 4))
    VxF3 = zeros(4)
    OMxF3 = zeros(4)
    AxF3 = zeros(4)
    OMPxF3 = zeros(4)

    PxF4 = zeros(4)
    RxF4 = zeros((4, 4))
    VxF4 = zeros(4)
    OMxF4 = zeros(4)
    AxF4 = zeros(4)
    OMPxF4 = zeros(4)

    PxF5 = zeros(4)
    RxF5 = zeros((4, 4))
    VxF5 = zeros(4)
    OMxF5 = zeros(4)
    AxF5 = zeros(4)
    OMPxF5 = zeros(4)

    PxF6 = zeros(4)
    RxF6 = zeros((4, 4))
    VxF6 = zeros(4)
    OMxF6 = zeros(4)
    AxF6 = zeros(4)
    OMPxF6 = zeros(4)

 
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
    POcp1_24 = q[1]+RLcp1_24
    POcp1_34 = q[2]+RLcp1_34
    OMcp1_14 = qd[3]+qd[4]
    ORcp1_24 = -qd[3]*RLcp1_34
    ORcp1_34 = qd[3]*RLcp1_24
    VIcp1_24 = qd[1]+ORcp1_24
    VIcp1_34 = qd[2]+ORcp1_34
    OPcp1_14 = qdd[3]+qdd[4]
    ACcp1_24 = qdd[1]-qd[3]*ORcp1_34-qdd[3]*RLcp1_34
    ACcp1_34 = qdd[2]+qd[3]*ORcp1_24+qdd[3]*RLcp1_24
    RLcp1_25 = ROcp1_54*s.dpt[2,3]
    RLcp1_35 = ROcp1_64*s.dpt[2,3]
    POcp1_25 = POcp1_24+RLcp1_25
    POcp1_35 = POcp1_34+RLcp1_35
    OMcp1_15 = qd[5]+OMcp1_14
    ORcp1_25 = -OMcp1_14*RLcp1_35
    ORcp1_35 = OMcp1_14*RLcp1_25
    VIcp1_25 = ORcp1_25+VIcp1_24
    VIcp1_35 = ORcp1_35+VIcp1_34
    OPcp1_15 = qdd[5]+OPcp1_14
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
    PxF1[1] = 0
    PxF1[2] = POcp1_26
    PxF1[3] = POcp1_36
    RxF1[1,1] = (1.0)
    RxF1[2,2] = ROcp1_55
    RxF1[2,3] = ROcp1_65
    RxF1[3,2] = ROcp1_85
    RxF1[3,3] = ROcp1_95
    VxF1[1] = 0
    VxF1[2] = VIcp1_26
    VxF1[3] = VIcp1_36
    OMxF1[1] = OMcp1_15
    OMxF1[2] = 0
    OMxF1[3] = 0
    AxF1[1] = 0
    AxF1[2] = ACcp1_26
    AxF1[3] = ACcp1_36
    OMPxF1[1] = OPcp1_15
    OMPxF1[2] = 0
    OMPxF1[3] = 0
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
    POcp2_24 = q[1]+RLcp2_24
    POcp2_34 = q[2]+RLcp2_34
    OMcp2_14 = qd[3]+qd[6]
    ORcp2_24 = -qd[3]*RLcp2_34
    ORcp2_34 = qd[3]*RLcp2_24
    VIcp2_24 = qd[1]+ORcp2_24
    VIcp2_34 = qd[2]+ORcp2_34
    OPcp2_14 = qdd[3]+qdd[6]
    ACcp2_24 = qdd[1]-qd[3]*ORcp2_34-qdd[3]*RLcp2_34
    ACcp2_34 = qdd[2]+qd[3]*ORcp2_24+qdd[3]*RLcp2_24
    RLcp2_25 = ROcp2_56*s.dpt[2,5]
    RLcp2_35 = ROcp2_66*s.dpt[2,5]
    POcp2_25 = POcp2_24+RLcp2_25
    POcp2_35 = POcp2_34+RLcp2_35
    OMcp2_15 = qd[7]+OMcp2_14
    ORcp2_25 = -OMcp2_14*RLcp2_35
    ORcp2_35 = OMcp2_14*RLcp2_25
    VIcp2_25 = ORcp2_25+VIcp2_24
    VIcp2_35 = ORcp2_35+VIcp2_34
    OPcp2_15 = qdd[7]+OPcp2_14
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
    PxF2[1] = 0
    PxF2[2] = POcp2_26
    PxF2[3] = POcp2_36
    RxF2[1,1] = (1.0)
    RxF2[2,2] = ROcp2_57
    RxF2[2,3] = ROcp2_67
    RxF2[3,2] = ROcp2_87
    RxF2[3,3] = ROcp2_97
    VxF2[1] = 0
    VxF2[2] = VIcp2_26
    VxF2[3] = VIcp2_36
    OMxF2[1] = OMcp2_15
    OMxF2[2] = 0
    OMxF2[3] = 0
    AxF2[1] = 0
    AxF2[2] = ACcp2_26
    AxF2[3] = ACcp2_36
    OMPxF2[1] = OPcp2_15
    OMPxF2[2] = 0
    OMPxF2[3] = 0
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
    POcp3_24 = q[8]+RLcp3_24
    POcp3_34 = q[9]+RLcp3_34
    OMcp3_14 = qd[10]+qd[11]
    ORcp3_24 = -qd[10]*RLcp3_34
    ORcp3_34 = qd[10]*RLcp3_24
    VIcp3_24 = qd[8]+ORcp3_24
    VIcp3_34 = qd[9]+ORcp3_34
    OPcp3_14 = qdd[10]+qdd[11]
    ACcp3_24 = qdd[8]-qd[10]*ORcp3_34-qdd[10]*RLcp3_34
    ACcp3_34 = qdd[9]+qd[10]*ORcp3_24+qdd[10]*RLcp3_24
    RLcp3_25 = ROcp3_511*s.dpt[2,9]
    RLcp3_35 = ROcp3_611*s.dpt[2,9]
    POcp3_25 = POcp3_24+RLcp3_25
    POcp3_35 = POcp3_34+RLcp3_35
    OMcp3_15 = qd[12]+OMcp3_14
    ORcp3_25 = -OMcp3_14*RLcp3_35
    ORcp3_35 = OMcp3_14*RLcp3_25
    VIcp3_25 = ORcp3_25+VIcp3_24
    VIcp3_35 = ORcp3_35+VIcp3_34
    OPcp3_15 = qdd[12]+OPcp3_14
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
    PxF3[1] = 0
    PxF3[2] = POcp3_26
    PxF3[3] = POcp3_36
    RxF3[1,1] = (1.0)
    RxF3[2,2] = ROcp3_512
    RxF3[2,3] = ROcp3_612
    RxF3[3,2] = ROcp3_812
    RxF3[3,3] = ROcp3_912
    VxF3[1] = 0
    VxF3[2] = VIcp3_26
    VxF3[3] = VIcp3_36
    OMxF3[1] = OMcp3_15
    OMxF3[2] = 0
    OMxF3[3] = 0
    AxF3[1] = 0
    AxF3[2] = ACcp3_26
    AxF3[3] = ACcp3_36
    OMPxF3[1] = OPcp3_15
    OMPxF3[2] = 0
    OMPxF3[3] = 0
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
    POcp4_24 = q[8]+RLcp4_24
    POcp4_34 = q[9]+RLcp4_34
    OMcp4_14 = qd[10]+qd[13]
    ORcp4_24 = -qd[10]*RLcp4_34
    ORcp4_34 = qd[10]*RLcp4_24
    VIcp4_24 = qd[8]+ORcp4_24
    VIcp4_34 = qd[9]+ORcp4_34
    OPcp4_14 = qdd[10]+qdd[13]
    ACcp4_24 = qdd[8]-qd[10]*ORcp4_34-qdd[10]*RLcp4_34
    ACcp4_34 = qdd[9]+qd[10]*ORcp4_24+qdd[10]*RLcp4_24
    RLcp4_25 = ROcp4_513*s.dpt[2,11]
    RLcp4_35 = ROcp4_613*s.dpt[2,11]
    POcp4_25 = POcp4_24+RLcp4_25
    POcp4_35 = POcp4_34+RLcp4_35
    OMcp4_15 = qd[14]+OMcp4_14
    ORcp4_25 = -OMcp4_14*RLcp4_35
    ORcp4_35 = OMcp4_14*RLcp4_25
    VIcp4_25 = ORcp4_25+VIcp4_24
    VIcp4_35 = ORcp4_35+VIcp4_34
    OPcp4_15 = qdd[14]+OPcp4_14
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
    PxF4[1] = 0
    PxF4[2] = POcp4_26
    PxF4[3] = POcp4_36
    RxF4[1,1] = (1.0)
    RxF4[2,2] = ROcp4_514
    RxF4[2,3] = ROcp4_614
    RxF4[3,2] = ROcp4_814
    RxF4[3,3] = ROcp4_914
    VxF4[1] = 0
    VxF4[2] = VIcp4_26
    VxF4[3] = VIcp4_36
    OMxF4[1] = OMcp4_15
    OMxF4[2] = 0
    OMxF4[3] = 0
    AxF4[1] = 0
    AxF4[2] = ACcp4_26
    AxF4[3] = ACcp4_36
    OMPxF4[1] = OPcp4_15
    OMPxF4[2] = 0
    OMPxF4[3] = 0
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
    POcp5_24 = q[15]+RLcp5_24
    POcp5_34 = q[16]+RLcp5_34
    OMcp5_14 = qd[17]+qd[18]
    ORcp5_24 = -qd[17]*RLcp5_34
    ORcp5_34 = qd[17]*RLcp5_24
    VIcp5_24 = qd[15]+ORcp5_24
    VIcp5_34 = qd[16]+ORcp5_34
    OPcp5_14 = qdd[17]+qdd[18]
    ACcp5_24 = qdd[15]-qd[17]*ORcp5_34-qdd[17]*RLcp5_34
    ACcp5_34 = qdd[16]+qd[17]*ORcp5_24+qdd[17]*RLcp5_24
    RLcp5_25 = ROcp5_518*s.dpt[2,15]
    RLcp5_35 = ROcp5_618*s.dpt[2,15]
    POcp5_25 = POcp5_24+RLcp5_25
    POcp5_35 = POcp5_34+RLcp5_35
    OMcp5_15 = qd[19]+OMcp5_14
    ORcp5_25 = -OMcp5_14*RLcp5_35
    ORcp5_35 = OMcp5_14*RLcp5_25
    VIcp5_25 = ORcp5_25+VIcp5_24
    VIcp5_35 = ORcp5_35+VIcp5_34
    OPcp5_15 = qdd[19]+OPcp5_14
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
    PxF5[1] = 0
    PxF5[2] = POcp5_26
    PxF5[3] = POcp5_36
    RxF5[1,1] = (1.0)
    RxF5[2,2] = ROcp5_519
    RxF5[2,3] = ROcp5_619
    RxF5[3,2] = ROcp5_819
    RxF5[3,3] = ROcp5_919
    VxF5[1] = 0
    VxF5[2] = VIcp5_26
    VxF5[3] = VIcp5_36
    OMxF5[1] = OMcp5_15
    OMxF5[2] = 0
    OMxF5[3] = 0
    AxF5[1] = 0
    AxF5[2] = ACcp5_26
    AxF5[3] = ACcp5_36
    OMPxF5[1] = OPcp5_15
    OMPxF5[2] = 0
    OMPxF5[3] = 0
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
    POcp6_24 = q[15]+RLcp6_24
    POcp6_34 = q[16]+RLcp6_34
    OMcp6_14 = qd[17]+qd[20]
    ORcp6_24 = -qd[17]*RLcp6_34
    ORcp6_34 = qd[17]*RLcp6_24
    VIcp6_24 = qd[15]+ORcp6_24
    VIcp6_34 = qd[16]+ORcp6_34
    OPcp6_14 = qdd[17]+qdd[20]
    ACcp6_24 = qdd[15]-qd[17]*ORcp6_34-qdd[17]*RLcp6_34
    ACcp6_34 = qdd[16]+qd[17]*ORcp6_24+qdd[17]*RLcp6_24
    RLcp6_25 = ROcp6_520*s.dpt[2,17]
    RLcp6_35 = ROcp6_620*s.dpt[2,17]
    POcp6_25 = POcp6_24+RLcp6_25
    POcp6_35 = POcp6_34+RLcp6_35
    OMcp6_15 = qd[21]+OMcp6_14
    ORcp6_25 = -OMcp6_14*RLcp6_35
    ORcp6_35 = OMcp6_14*RLcp6_25
    VIcp6_25 = ORcp6_25+VIcp6_24
    VIcp6_35 = ORcp6_35+VIcp6_34
    OPcp6_15 = qdd[21]+OPcp6_14
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
    PxF6[1] = 0
    PxF6[2] = POcp6_26
    PxF6[3] = POcp6_36
    RxF6[1,1] = (1.0)
    RxF6[2,2] = ROcp6_521
    RxF6[2,3] = ROcp6_621
    RxF6[3,2] = ROcp6_821
    RxF6[3,3] = ROcp6_921
    VxF6[1] = 0
    VxF6[2] = VIcp6_26
    VxF6[3] = VIcp6_36
    OMxF6[1] = OMcp6_15
    OMxF6[2] = 0
    OMxF6[3] = 0
    AxF6[1] = 0
    AxF6[2] = ACcp6_26
    AxF6[3] = ACcp6_36
    OMPxF6[1] = OPcp6_15
    OMPxF6[2] = 0
    OMPxF6[3] = 0
 
# Sensor Forces 

    SWr1 = s.user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1)
    SWr2 = s.user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2)
    SWr3 = s.user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3)
    SWr4 = s.user_ExtForces(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4)
    SWr5 = s.user_ExtForces(PxF5,RxF5,VxF5,OMxF5,AxF5,OMPxF5,s,tsim,5)
    SWr6 = s.user_ExtForces(PxF6,RxF6,VxF6,OMxF6,AxF6,OMPxF6,s,tsim,6)
    xfrc11 = RxF1[1,1]*SWr1[1]
    xfrc21 = RxF1[2,2]*SWr1[2]+RxF1[2,3]*SWr1[3]
    xfrc31 = RxF1[3,2]*SWr1[2]+RxF1[3,3]*SWr1[3]
    xtrq11 = RxF1[1,1]*SWr1[4]
    xtrq21 = RxF1[2,2]*SWr1[5]+RxF1[2,3]*SWr1[6]
    xtrq31 = RxF1[3,2]*SWr1[5]+RxF1[3,3]*SWr1[6]
    trqext_1_5_0 = xtrq11-xfrc21*SWr1[9]+xfrc31*(SWr1[8]-s.l[2,5])
    trqext_2_5_0 = xtrq21+xfrc11*SWr1[9]-xfrc31*SWr1[7]
    trqext_3_5_0 = xtrq31-xfrc11*(SWr1[8]-s.l[2,5])+xfrc21*SWr1[7]
    xfrc12 = RxF2[1,1]*SWr2[1]
    xfrc22 = RxF2[2,2]*SWr2[2]+RxF2[2,3]*SWr2[3]
    xfrc32 = RxF2[3,2]*SWr2[2]+RxF2[3,3]*SWr2[3]
    xtrq12 = RxF2[1,1]*SWr2[4]
    xtrq22 = RxF2[2,2]*SWr2[5]+RxF2[2,3]*SWr2[6]
    xtrq32 = RxF2[3,2]*SWr2[5]+RxF2[3,3]*SWr2[6]
    trqext_1_7_1 = xtrq12-xfrc22*SWr2[9]+xfrc32*(SWr2[8]-s.l[2,7])
    trqext_2_7_1 = xtrq22+xfrc12*SWr2[9]-xfrc32*SWr2[7]
    trqext_3_7_1 = xtrq32-xfrc12*(SWr2[8]-s.l[2,7])+xfrc22*SWr2[7]
    xfrc13 = RxF3[1,1]*SWr3[1]
    xfrc23 = RxF3[2,2]*SWr3[2]+RxF3[2,3]*SWr3[3]
    xfrc33 = RxF3[3,2]*SWr3[2]+RxF3[3,3]*SWr3[3]
    xtrq13 = RxF3[1,1]*SWr3[4]
    xtrq23 = RxF3[2,2]*SWr3[5]+RxF3[2,3]*SWr3[6]
    xtrq33 = RxF3[3,2]*SWr3[5]+RxF3[3,3]*SWr3[6]
    trqext_1_12_2 = xtrq13-xfrc23*SWr3[9]+xfrc33*(SWr3[8]-s.l[2,12])
    trqext_2_12_2 = xtrq23+xfrc13*SWr3[9]-xfrc33*SWr3[7]
    trqext_3_12_2 = xtrq33-xfrc13*(SWr3[8]-s.l[2,12])+xfrc23*SWr3[7]
    xfrc14 = RxF4[1,1]*SWr4[1]
    xfrc24 = RxF4[2,2]*SWr4[2]+RxF4[2,3]*SWr4[3]
    xfrc34 = RxF4[3,2]*SWr4[2]+RxF4[3,3]*SWr4[3]
    xtrq14 = RxF4[1,1]*SWr4[4]
    xtrq24 = RxF4[2,2]*SWr4[5]+RxF4[2,3]*SWr4[6]
    xtrq34 = RxF4[3,2]*SWr4[5]+RxF4[3,3]*SWr4[6]
    trqext_1_14_3 = xtrq14-xfrc24*SWr4[9]+xfrc34*(SWr4[8]-s.l[2,14])
    trqext_2_14_3 = xtrq24+xfrc14*SWr4[9]-xfrc34*SWr4[7]
    trqext_3_14_3 = xtrq34-xfrc14*(SWr4[8]-s.l[2,14])+xfrc24*SWr4[7]
    xfrc15 = RxF5[1,1]*SWr5[1]
    xfrc25 = RxF5[2,2]*SWr5[2]+RxF5[2,3]*SWr5[3]
    xfrc35 = RxF5[3,2]*SWr5[2]+RxF5[3,3]*SWr5[3]
    xtrq15 = RxF5[1,1]*SWr5[4]
    xtrq25 = RxF5[2,2]*SWr5[5]+RxF5[2,3]*SWr5[6]
    xtrq35 = RxF5[3,2]*SWr5[5]+RxF5[3,3]*SWr5[6]
    trqext_1_19_4 = xtrq15-xfrc25*SWr5[9]+xfrc35*(SWr5[8]-s.l[2,19])
    trqext_2_19_4 = xtrq25+xfrc15*SWr5[9]-xfrc35*SWr5[7]
    trqext_3_19_4 = xtrq35-xfrc15*(SWr5[8]-s.l[2,19])+xfrc25*SWr5[7]
    xfrc16 = RxF6[1,1]*SWr6[1]
    xfrc26 = RxF6[2,2]*SWr6[2]+RxF6[2,3]*SWr6[3]
    xfrc36 = RxF6[3,2]*SWr6[2]+RxF6[3,3]*SWr6[3]
    xtrq16 = RxF6[1,1]*SWr6[4]
    xtrq26 = RxF6[2,2]*SWr6[5]+RxF6[2,3]*SWr6[6]
    xtrq36 = RxF6[3,2]*SWr6[5]+RxF6[3,3]*SWr6[6]
    trqext_1_21_5 = xtrq16-xfrc26*SWr6[9]+xfrc36*(SWr6[8]-s.l[2,21])
    trqext_2_21_5 = xtrq26+xfrc16*SWr6[9]-xfrc36*SWr6[7]
    trqext_3_21_5 = xtrq36-xfrc16*(SWr6[8]-s.l[2,21])+xfrc26*SWr6[7]
 
# Symbolic model output

    frc[1,5] = s.frc[1,5]+xfrc11
    frc[2,5] = s.frc[2,5]+xfrc21
    frc[3,5] = s.frc[3,5]+xfrc31
    trq[1,5] = s.trq[1,5]+trqext_1_5_0
    trq[2,5] = s.trq[2,5]+trqext_2_5_0
    trq[3,5] = s.trq[3,5]+trqext_3_5_0
    frc[1,7] = s.frc[1,7]+xfrc12
    frc[2,7] = s.frc[2,7]+xfrc22
    frc[3,7] = s.frc[3,7]+xfrc32
    trq[1,7] = s.trq[1,7]+trqext_1_7_1
    trq[2,7] = s.trq[2,7]+trqext_2_7_1
    trq[3,7] = s.trq[3,7]+trqext_3_7_1
    frc[1,12] = s.frc[1,12]+xfrc13
    frc[2,12] = s.frc[2,12]+xfrc23
    frc[3,12] = s.frc[3,12]+xfrc33
    trq[1,12] = s.trq[1,12]+trqext_1_12_2
    trq[2,12] = s.trq[2,12]+trqext_2_12_2
    trq[3,12] = s.trq[3,12]+trqext_3_12_2
    frc[1,14] = s.frc[1,14]+xfrc14
    frc[2,14] = s.frc[2,14]+xfrc24
    frc[3,14] = s.frc[3,14]+xfrc34
    trq[1,14] = s.trq[1,14]+trqext_1_14_3
    trq[2,14] = s.trq[2,14]+trqext_2_14_3
    trq[3,14] = s.trq[3,14]+trqext_3_14_3
    frc[1,19] = s.frc[1,19]+xfrc15
    frc[2,19] = s.frc[2,19]+xfrc25
    frc[3,19] = s.frc[3,19]+xfrc35
    trq[1,19] = s.trq[1,19]+trqext_1_19_4
    trq[2,19] = s.trq[2,19]+trqext_2_19_4
    trq[3,19] = s.trq[3,19]+trqext_3_19_4
    frc[1,21] = s.frc[1,21]+xfrc16
    frc[2,21] = s.frc[2,21]+xfrc26
    frc[3,21] = s.frc[3,21]+xfrc36
    trq[1,21] = s.trq[1,21]+trqext_1_21_5
    trq[2,21] = s.trq[2,21]+trqext_2_21_5
    trq[3,21] = s.trq[3,21]+trqext_3_21_5

# Number of continuation lines = 0


