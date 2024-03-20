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
#	==> Function: F8 - Constraints and Constraints Jacobian(h, J)
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def cons_hJ(h, Jac, s):
    q = s.q
 
# Trigonometric functions

    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
    S1 = sin(q[1])
    C1 = cos(q[1])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

    ROlp1_72 = C4*S5+S4*C5
    ROlp1_92 = C4*C5-S4*S5
    RLlp1_12 = s.dpt[3,6]*S4
    RLlp1_32 = s.dpt[3,6]*C4
    POlp1_12 = RLlp1_12+s.dpt[1,2]
    RLlp1_13 = ROlp1_72*s.dpt[3,7]
    RLlp1_33 = ROlp1_92*s.dpt[3,7]
    POlp1_13 = POlp1_12+RLlp1_13
    POlp1_33 = RLlp1_32+RLlp1_33
    JTlp1_13_1 = RLlp1_32+RLlp1_33
    JTlp1_33_1 = -RLlp1_12-RLlp1_13
    RLlp2_12 = s.dpt[3,4]*S1
    RLlp2_32 = s.dpt[3,4]*C1
    h_1 = POlp1_13-RLlp2_12
    h_3 = POlp1_33-RLlp2_32
    h[1] = h_1
    h[2] = h_3
    Jac[1,1] = -RLlp2_32
    Jac[1,2] = 0
    Jac[1,3] = 0
    Jac[1,4] = JTlp1_13_1
    Jac[1,5] = RLlp1_33
    Jac[2,1] = RLlp2_12
    Jac[2,2] = 0
    Jac[2,3] = 0
    Jac[2,4] = JTlp1_33_1
    Jac[2,5] = -RLlp1_13

# Number of continuation lines = 0


