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
#	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def cons_jdqd(Jdqd, s):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
    S1 = sin(q[1])
    C1 = cos(q[1])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    ROjdqd1_72 = C4*S5+S4*C5
    ROjdqd1_92 = C4*C5-S4*S5
    RLjdqd1_12 = s.dpt[3,6]*S4
    RLjdqd1_32 = s.dpt[3,6]*C4
    OMjdqd1_22 = qd[4]+qd[5]
    ORjdqd1_12 = RLjdqd1_32*qd[4]
    ORjdqd1_32 = -RLjdqd1_12*qd[4]
    Apqpjdqd1_12 = ORjdqd1_32*qd[4]
    Apqpjdqd1_32 = -ORjdqd1_12*qd[4]
    RLjdqd1_13 = ROjdqd1_72*s.dpt[3,7]
    RLjdqd1_33 = ROjdqd1_92*s.dpt[3,7]
    ORjdqd1_13 = OMjdqd1_22*RLjdqd1_33
    ORjdqd1_33 = -OMjdqd1_22*RLjdqd1_13
    Apqpjdqd1_13 = Apqpjdqd1_12+OMjdqd1_22*ORjdqd1_33
    Apqpjdqd1_33 = Apqpjdqd1_32-OMjdqd1_22*ORjdqd1_13
    RLjdqd2_12 = s.dpt[3,4]*S1
    RLjdqd2_32 = s.dpt[3,4]*C1
    ORjdqd2_12 = RLjdqd2_32*qd[1]
    ORjdqd2_32 = -RLjdqd2_12*qd[1]
    Apqpjdqd2_12 = ORjdqd2_32*qd[1]
    Apqpjdqd2_32 = -ORjdqd2_12*qd[1]
    jdqd1 = Apqpjdqd1_13-Apqpjdqd2_12
    jdqd3 = Apqpjdqd1_33-Apqpjdqd2_32
    Jdqd[1] = jdqd1
    Jdqd[2] = jdqd3

# Number of continuation lines = 0


