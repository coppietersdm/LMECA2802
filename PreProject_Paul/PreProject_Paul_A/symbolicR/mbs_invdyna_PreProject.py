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
#	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def invdyna(phi,s,tsim):
    Qq = phi  # compatibility with symbolic generation
    q = s.q
    qd = s.qd
    qdd = s.qdd
 
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

 
# Forward Kinematics

    ALPHA32 = qdd[2]-s.g[3]
    BS53 = -qd[3]*qd[3]
    ALPHA23 = qdd[1]*C3+ALPHA32*S3
    ALPHA33 = -qdd[1]*S3+ALPHA32*C3
    OM14 = qd[3]+qd[4]
    OMp14 = qdd[3]+qdd[4]
    BS54 = -OM14*OM14
    ALPHA24 = C4*(ALPHA23+BS53*s.dpt[2,1])+S4*(ALPHA33+qdd[3]*s.dpt[2,1])
    ALPHA34 = C4*(ALPHA33+qdd[3]*s.dpt[2,1])-S4*(ALPHA23+BS53*s.dpt[2,1])
    OM15 = qd[3]+qd[5]
    OMp15 = qdd[3]+qdd[5]
    BS55 = -OM15*OM15
    ALPHA25 = C5*(ALPHA23+BS53*s.dpt[2,2])+S5*(ALPHA33+qdd[3]*s.dpt[2,2])
    ALPHA35 = C5*(ALPHA33+qdd[3]*s.dpt[2,2])-S5*(ALPHA23+BS53*s.dpt[2,2])
    ALPHA37 = qdd[7]-s.g[3]
    BS58 = -qd[8]*qd[8]
    ALPHA28 = qdd[6]*C8+ALPHA37*S8
    ALPHA38 = -qdd[6]*S8+ALPHA37*C8
    OM19 = qd[8]+qd[9]
    OMp19 = qdd[8]+qdd[9]
    BS59 = -OM19*OM19
    ALPHA29 = C9*(ALPHA28+BS58*s.dpt[2,5])+S9*(ALPHA38+qdd[8]*s.dpt[2,5])
    ALPHA39 = C9*(ALPHA38+qdd[8]*s.dpt[2,5])-S9*(ALPHA28+BS58*s.dpt[2,5])
    OM110 = qd[10]+OM19
    OMp110 = qdd[10]+OMp19
    BS510 = -OM110*OM110
    ALPHA210 = C10*(ALPHA29+BS59*s.dpt[2,7])+S10*(ALPHA39+OMp19*s.dpt[2,7])
    ALPHA310 = C10*(ALPHA39+OMp19*s.dpt[2,7])-S10*(ALPHA29+BS59*s.dpt[2,7])
    OM111 = qd[11]+qd[8]
    OMp111 = qdd[11]+qdd[8]
    BS511 = -OM111*OM111
    ALPHA211 = C11*(ALPHA28+BS58*s.dpt[2,6])+S11*(ALPHA38+qdd[8]*s.dpt[2,6])
    ALPHA311 = C11*(ALPHA38+qdd[8]*s.dpt[2,6])-S11*(ALPHA28+BS58*s.dpt[2,6])
    OM112 = qd[12]+OM111
    OMp112 = qdd[12]+OMp111
    BS512 = -OM112*OM112
    ALPHA212 = C12*(ALPHA211+BS511*s.dpt[2,9])+S12*(ALPHA311+OMp111*s.dpt[2,9])
    ALPHA312 = C12*(ALPHA311+OMp111*s.dpt[2,9])-S12*(ALPHA211+BS511*s.dpt[2,9])
    ALPHA314 = qdd[14]-s.g[3]
    BS515 = -qd[15]*qd[15]
    ALPHA215 = qdd[13]*C15+ALPHA314*S15
    ALPHA315 = -qdd[13]*S15+ALPHA314*C15
    OM116 = qd[15]+qd[16]
    OMp116 = qdd[15]+qdd[16]
    BS516 = -OM116*OM116
    ALPHA216 = C16*(ALPHA215+BS515*s.dpt[2,11])+S16*(ALPHA315+qdd[15]*s.dpt[2,11])
    ALPHA316 = C16*(ALPHA315+qdd[15]*s.dpt[2,11])-S16*(ALPHA215+BS515*s.dpt[2,11])
    OM117 = qd[17]+OM116
    OMp117 = qdd[17]+OMp116
    BS517 = -OM117*OM117
    ALPHA217 = C17*(ALPHA216+BS516*s.dpt[2,13])+S17*(ALPHA316+OMp116*s.dpt[2,13])
    ALPHA317 = C17*(ALPHA316+OMp116*s.dpt[2,13])-S17*(ALPHA216+BS516*s.dpt[2,13])
    OM118 = qd[15]+qd[18]
    OMp118 = qdd[15]+qdd[18]
    BS518 = -OM118*OM118
    ALPHA218 = C18*(ALPHA215+BS515*s.dpt[2,12])+S18*(ALPHA315+qdd[15]*s.dpt[2,12])
    ALPHA318 = C18*(ALPHA315+qdd[15]*s.dpt[2,12])-S18*(ALPHA215+BS515*s.dpt[2,12])
    OM119 = qd[19]+OM118
    OMp119 = qdd[19]+OMp118
    BS519 = -OM119*OM119
    ALPHA219 = C19*(ALPHA218+BS518*s.dpt[2,15])+S19*(ALPHA318+OMp118*s.dpt[2,15])
    ALPHA319 = C19*(ALPHA318+OMp118*s.dpt[2,15])-S19*(ALPHA218+BS518*s.dpt[2,15])
 
# Backward Dynamics

    Fs219 = -s.frc[2,19]+s.m[19]*(ALPHA219+BS519*s.l[2,19])
    Fs319 = -s.frc[3,19]+s.m[19]*(ALPHA319+OMp119*s.l[2,19])
    Cq119 = -s.trq[1,19]+s.In[1,19]*OMp119+Fs319*s.l[2,19]
    Fs218 = -s.frc[2,18]+s.m[18]*(ALPHA218+BS518*s.l[2,18])
    Fs318 = -s.frc[3,18]+s.m[18]*(ALPHA318+OMp118*s.l[2,18])
    Fq218 = Fs218+Fs219*C19-Fs319*S19
    Fq318 = Fs318+Fs219*S19+Fs319*C19
    Cq118 = -s.trq[1,18]+Cq119+s.In[1,18]*OMp118+Fs318*s.l[2,18]+s.dpt[2,15]*(Fs219*S19+Fs319*C19)
    Fs217 = -s.frc[2,17]+s.m[17]*(ALPHA217+BS517*s.l[2,17])
    Fs317 = -s.frc[3,17]+s.m[17]*(ALPHA317+OMp117*s.l[2,17])
    Cq117 = -s.trq[1,17]+s.In[1,17]*OMp117+Fs317*s.l[2,17]
    Fs216 = -s.frc[2,16]+s.m[16]*(ALPHA216+BS516*s.l[2,16])
    Fs316 = -s.frc[3,16]+s.m[16]*(ALPHA316+OMp116*s.l[2,16])
    Fq216 = Fs216+Fs217*C17-Fs317*S17
    Fq316 = Fs316+Fs217*S17+Fs317*C17
    Cq116 = -s.trq[1,16]+Cq117+s.In[1,16]*OMp116+Fs316*s.l[2,16]+s.dpt[2,13]*(Fs217*S17+Fs317*C17)
    Fs215 = -s.frc[2,15]+s.m[15]*ALPHA215
    Fs315 = -s.frc[3,15]+s.m[15]*ALPHA315
    Fq215 = Fs215+Fq216*C16+Fq218*C18-Fq316*S16-Fq318*S18
    Fq315 = Fs315+Fq216*S16+Fq218*S18+Fq316*C16+Fq318*C18
    Cq115 = -s.trq[1,15]+Cq116+Cq118+qdd[15]*s.In[1,15]+s.dpt[2,11]*(Fq216*S16+Fq316*C16)+s.dpt[2,12]*(Fq218*S18+Fq318*C18)
    Fq214 = Fq215*C15-Fq315*S15
    Fq314 = Fq215*S15+Fq315*C15
    Fs212 = -s.frc[2,12]+s.m[12]*(ALPHA212+BS512*s.l[2,12])
    Fs312 = -s.frc[3,12]+s.m[12]*(ALPHA312+OMp112*s.l[2,12])
    Cq112 = -s.trq[1,12]+s.In[1,12]*OMp112+Fs312*s.l[2,12]
    Fs211 = -s.frc[2,11]+s.m[11]*(ALPHA211+BS511*s.l[2,11])
    Fs311 = -s.frc[3,11]+s.m[11]*(ALPHA311+OMp111*s.l[2,11])
    Fq211 = Fs211+Fs212*C12-Fs312*S12
    Fq311 = Fs311+Fs212*S12+Fs312*C12
    Cq111 = -s.trq[1,11]+Cq112+s.In[1,11]*OMp111+Fs311*s.l[2,11]+s.dpt[2,9]*(Fs212*S12+Fs312*C12)
    Fs210 = -s.frc[2,10]+s.m[10]*(ALPHA210+BS510*s.l[2,10])
    Fs310 = -s.frc[3,10]+s.m[10]*(ALPHA310+OMp110*s.l[2,10])
    Cq110 = -s.trq[1,10]+s.In[1,10]*OMp110+Fs310*s.l[2,10]
    Fs29 = -s.frc[2,9]+s.m[9]*(ALPHA29+BS59*s.l[2,9])
    Fs39 = -s.frc[3,9]+s.m[9]*(ALPHA39+OMp19*s.l[2,9])
    Fq29 = Fs29+Fs210*C10-Fs310*S10
    Fq39 = Fs39+Fs210*S10+Fs310*C10
    Cq19 = -s.trq[1,9]+Cq110+s.In[1,9]*OMp19+Fs39*s.l[2,9]+s.dpt[2,7]*(Fs210*S10+Fs310*C10)
    Fs28 = -s.frc[2,8]+s.m[8]*ALPHA28
    Fs38 = -s.frc[3,8]+s.m[8]*ALPHA38
    Fq28 = Fs28+Fq211*C11+Fq29*C9-Fq311*S11-Fq39*S9
    Fq38 = Fs38+Fq211*S11+Fq29*S9+Fq311*C11+Fq39*C9
    Cq18 = -s.trq[1,8]+Cq111+Cq19+qdd[8]*s.In[1,8]+s.dpt[2,5]*(Fq29*S9+Fq39*C9)+s.dpt[2,6]*(Fq211*S11+Fq311*C11)
    Fq27 = Fq28*C8-Fq38*S8
    Fq37 = Fq28*S8+Fq38*C8
    Fs25 = -s.frc[2,5]+s.m[5]*(ALPHA25+BS55*s.l[2,5])
    Fs35 = -s.frc[3,5]+s.m[5]*(ALPHA35+OMp15*s.l[2,5])
    Cq15 = -s.trq[1,5]+s.In[1,5]*OMp15+Fs35*s.l[2,5]
    Fs24 = -s.frc[2,4]+s.m[4]*(ALPHA24+BS54*s.l[2,4])
    Fs34 = -s.frc[3,4]+s.m[4]*(ALPHA34+OMp14*s.l[2,4])
    Cq14 = -s.trq[1,4]+s.In[1,4]*OMp14+Fs34*s.l[2,4]
    Fs23 = -s.frc[2,3]+s.m[3]*ALPHA23
    Fs33 = -s.frc[3,3]+s.m[3]*ALPHA33
    Fq23 = Fs23+Fs24*C4+Fs25*C5-Fs34*S4-Fs35*S5
    Fq33 = Fs33+Fs24*S4+Fs25*S5+Fs34*C4+Fs35*C5
    Cq13 = -s.trq[1,3]+Cq14+Cq15+qdd[3]*s.In[1,3]+s.dpt[2,1]*(Fs24*S4+Fs34*C4)+s.dpt[2,2]*(Fs25*S5+Fs35*C5)
    Fq22 = Fq23*C3-Fq33*S3
    Fq32 = Fq23*S3+Fq33*C3
 
# Symbolic model output

    Qq[1] = Fq22
    Qq[2] = Fq32
    Qq[3] = Cq13
    Qq[4] = Cq14
    Qq[5] = Cq15
    Qq[6] = Fq27
    Qq[7] = Fq37
    Qq[8] = Cq18
    Qq[9] = Cq19
    Qq[10] = Cq110
    Qq[11] = Cq111
    Qq[12] = Cq112
    Qq[13] = Fq214
    Qq[14] = Fq314
    Qq[15] = Cq115
    Qq[16] = Cq116
    Qq[17] = Cq117
    Qq[18] = Cq118
    Qq[19] = Cq119

# Number of continuation lines = 0


