//
//	MBsysTran - Release 8.1
//
//	Copyright 
//	Universite catholique de Louvain (UCLouvain) 
//	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
//	2, Place du Levant
//	1348 Louvain-la-Neuve 
//	Belgium 
//
//	http://www.robotran.be 
//
//	==> Generation Date: Tue Mar 12 18:46:28 2024
//
//	==> Project name: PreProject
//
//	==> Number of joints: 21
//
//	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
//
//	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
//

#include <math.h> 

#include "mbs_data.h"

 
void mbs_invdyna(double *Qq,
MbsData *s, double tsim)
{
#include "mbs_invdyna_PreProject.h"

double *q, *qd, *qdd;
double *g, *m;
double **l, **In, **dpt, **frc, **trq;

q = s->q;
qd = s->qd;
qdd = s->qdd;

dpt = s->dpt;
l   = s->l;

m = s->m;
In  = s->In;

frc = s->frc;
trq = s->trq;
g = s->g;
 
// Trigonometric functions

S3 = sin(q[3]);
C3 = cos(q[3]);
S4 = sin(q[4]);
C4 = cos(q[4]);
S5 = sin(q[5]);
C5 = cos(q[5]);
S6 = sin(q[6]);
C6 = cos(q[6]);
S7 = sin(q[7]);
C7 = cos(q[7]);
S10 = sin(q[10]);
C10 = cos(q[10]);
S11 = sin(q[11]);
C11 = cos(q[11]);
S12 = sin(q[12]);
C12 = cos(q[12]);
S13 = sin(q[13]);
C13 = cos(q[13]);
S14 = sin(q[14]);
C14 = cos(q[14]);
S17 = sin(q[17]);
C17 = cos(q[17]);
S18 = sin(q[18]);
C18 = cos(q[18]);
S19 = sin(q[19]);
C19 = cos(q[19]);
S20 = sin(q[20]);
C20 = cos(q[20]);
S21 = sin(q[21]);
C21 = cos(q[21]);
 
// Augmented Joint Position Vectors

 
// Forward Kinematics

ALPHA32 = qdd[2]-g[3];
BS53 = -qd[3]*qd[3];
ALPHA23 = qdd[1]*C3+ALPHA32*S3;
ALPHA33 = -qdd[1]*S3+ALPHA32*C3;
OM14 = qd[3]+qd[4];
OMp14 = qdd[3]+qdd[4];
BS54 = -OM14*OM14;
ALPHA24 = C4*(ALPHA23+BS53*dpt[2][1])+S4*(ALPHA33+qdd[3]*dpt[2][1]);
ALPHA34 = C4*(ALPHA33+qdd[3]*dpt[2][1])-S4*(ALPHA23+BS53*dpt[2][1]);
OM15 = qd[5]+OM14;
OMp15 = qdd[5]+OMp14;
BS55 = -OM15*OM15;
ALPHA25 = C5*(ALPHA24+BS54*dpt[2][3])+S5*(ALPHA34+OMp14*dpt[2][3]);
ALPHA35 = C5*(ALPHA34+OMp14*dpt[2][3])-S5*(ALPHA24+BS54*dpt[2][3]);
OM16 = qd[3]+qd[6];
OMp16 = qdd[3]+qdd[6];
BS56 = -OM16*OM16;
ALPHA26 = C6*(ALPHA23+BS53*dpt[2][2])+S6*(ALPHA33+qdd[3]*dpt[2][2]);
ALPHA36 = C6*(ALPHA33+qdd[3]*dpt[2][2])-S6*(ALPHA23+BS53*dpt[2][2]);
OM17 = qd[7]+OM16;
OMp17 = qdd[7]+OMp16;
BS57 = -OM17*OM17;
ALPHA27 = C7*(ALPHA26+BS56*dpt[2][5])+S7*(ALPHA36+OMp16*dpt[2][5]);
ALPHA37 = C7*(ALPHA36+OMp16*dpt[2][5])-S7*(ALPHA26+BS56*dpt[2][5]);
ALPHA39 = qdd[9]-g[3];
BS510 = -qd[10]*qd[10];
ALPHA210 = qdd[8]*C10+ALPHA39*S10;
ALPHA310 = -qdd[8]*S10+ALPHA39*C10;
OM111 = qd[10]+qd[11];
OMp111 = qdd[10]+qdd[11];
BS511 = -OM111*OM111;
ALPHA211 = C11*(ALPHA210+BS510*dpt[2][7])+S11*(ALPHA310+qdd[10]*dpt[2][7]);
ALPHA311 = C11*(ALPHA310+qdd[10]*dpt[2][7])-S11*(ALPHA210+BS510*dpt[2][7]);
OM112 = qd[12]+OM111;
OMp112 = qdd[12]+OMp111;
BS512 = -OM112*OM112;
ALPHA212 = C12*(ALPHA211+BS511*dpt[2][9])+S12*(ALPHA311+OMp111*dpt[2][9]);
ALPHA312 = C12*(ALPHA311+OMp111*dpt[2][9])-S12*(ALPHA211+BS511*dpt[2][9]);
OM113 = qd[10]+qd[13];
OMp113 = qdd[10]+qdd[13];
BS513 = -OM113*OM113;
ALPHA213 = C13*(ALPHA210+BS510*dpt[2][8])+S13*(ALPHA310+qdd[10]*dpt[2][8]);
ALPHA313 = C13*(ALPHA310+qdd[10]*dpt[2][8])-S13*(ALPHA210+BS510*dpt[2][8]);
OM114 = qd[14]+OM113;
OMp114 = qdd[14]+OMp113;
BS514 = -OM114*OM114;
ALPHA214 = C14*(ALPHA213+BS513*dpt[2][11])+S14*(ALPHA313+OMp113*dpt[2][11]);
ALPHA314 = C14*(ALPHA313+OMp113*dpt[2][11])-S14*(ALPHA213+BS513*dpt[2][11]);
ALPHA316 = qdd[16]-g[3];
BS517 = -qd[17]*qd[17];
ALPHA217 = qdd[15]*C17+ALPHA316*S17;
ALPHA317 = -qdd[15]*S17+ALPHA316*C17;
OM118 = qd[17]+qd[18];
OMp118 = qdd[17]+qdd[18];
BS518 = -OM118*OM118;
ALPHA218 = C18*(ALPHA217+BS517*dpt[2][13])+S18*(ALPHA317+qdd[17]*dpt[2][13]);
ALPHA318 = C18*(ALPHA317+qdd[17]*dpt[2][13])-S18*(ALPHA217+BS517*dpt[2][13]);
OM119 = qd[19]+OM118;
OMp119 = qdd[19]+OMp118;
BS519 = -OM119*OM119;
ALPHA219 = C19*(ALPHA218+BS518*dpt[2][15])+S19*(ALPHA318+OMp118*dpt[2][15]);
ALPHA319 = C19*(ALPHA318+OMp118*dpt[2][15])-S19*(ALPHA218+BS518*dpt[2][15]);
OM120 = qd[17]+qd[20];
OMp120 = qdd[17]+qdd[20];
BS520 = -OM120*OM120;
ALPHA220 = C20*(ALPHA217+BS517*dpt[2][14])+S20*(ALPHA317+qdd[17]*dpt[2][14]);
ALPHA320 = C20*(ALPHA317+qdd[17]*dpt[2][14])-S20*(ALPHA217+BS517*dpt[2][14]);
OM121 = qd[21]+OM120;
OMp121 = qdd[21]+OMp120;
BS521 = -OM121*OM121;
ALPHA221 = C21*(ALPHA220+BS520*dpt[2][17])+S21*(ALPHA320+OMp120*dpt[2][17]);
ALPHA321 = C21*(ALPHA320+OMp120*dpt[2][17])-S21*(ALPHA220+BS520*dpt[2][17]);
 
// Backward Dynamics

Fs221 = -frc[2][21]+m[21]*(ALPHA221+BS521*l[2][21]);
Fs321 = -frc[3][21]+m[21]*(ALPHA321+OMp121*l[2][21]);
Cq121 = -trq[1][21]+In[1][21]*OMp121+Fs321*l[2][21];
Fs220 = -frc[2][20]+m[20]*(ALPHA220+BS520*l[2][20]);
Fs320 = -frc[3][20]+m[20]*(ALPHA320+OMp120*l[2][20]);
Fq220 = Fs220+Fs221*C21-Fs321*S21;
Fq320 = Fs320+Fs221*S21+Fs321*C21;
Cq120 = -trq[1][20]+Cq121+In[1][20]*OMp120+Fs320*l[2][20]+dpt[2][17]*(Fs221*S21+Fs321*C21);
Fs219 = -frc[2][19]+m[19]*(ALPHA219+BS519*l[2][19]);
Fs319 = -frc[3][19]+m[19]*(ALPHA319+OMp119*l[2][19]);
Cq119 = -trq[1][19]+In[1][19]*OMp119+Fs319*l[2][19];
Fs218 = -frc[2][18]+m[18]*(ALPHA218+BS518*l[2][18]);
Fs318 = -frc[3][18]+m[18]*(ALPHA318+OMp118*l[2][18]);
Fq218 = Fs218+Fs219*C19-Fs319*S19;
Fq318 = Fs318+Fs219*S19+Fs319*C19;
Cq118 = -trq[1][18]+Cq119+In[1][18]*OMp118+Fs318*l[2][18]+dpt[2][15]*(Fs219*S19+Fs319*C19);
Fs217 = -frc[2][17]+m[17]*ALPHA217;
Fs317 = -frc[3][17]+m[17]*ALPHA317;
Fq217 = Fs217+Fq218*C18+Fq220*C20-Fq318*S18-Fq320*S20;
Fq317 = Fs317+Fq218*S18+Fq220*S20+Fq318*C18+Fq320*C20;
Cq117 = -trq[1][17]+Cq118+Cq120+qdd[17]*In[1][17]+dpt[2][13]*(Fq218*S18+Fq318*C18)+dpt[2][14]*(Fq220*S20+Fq320*C20);
Fq216 = Fq217*C17-Fq317*S17;
Fq316 = Fq217*S17+Fq317*C17;
Fs214 = -frc[2][14]+m[14]*(ALPHA214+BS514*l[2][14]);
Fs314 = -frc[3][14]+m[14]*(ALPHA314+OMp114*l[2][14]);
Cq114 = -trq[1][14]+In[1][14]*OMp114+Fs314*l[2][14];
Fs213 = -frc[2][13]+m[13]*ALPHA213;
Fs313 = -frc[3][13]+m[13]*ALPHA313;
Fq213 = Fs213+Fs214*C14-Fs314*S14;
Fq313 = Fs313+Fs214*S14+Fs314*C14;
Cq113 = -trq[1][13]+Cq114+In[1][13]*OMp113+dpt[2][11]*(Fs214*S14+Fs314*C14);
Fs212 = -frc[2][12]+m[12]*(ALPHA212+BS512*l[2][12]);
Fs312 = -frc[3][12]+m[12]*(ALPHA312+OMp112*l[2][12]);
Cq112 = -trq[1][12]+In[1][12]*OMp112+Fs312*l[2][12];
Fs211 = -frc[2][11]+m[11]*(ALPHA211+BS511*l[2][11]);
Fs311 = -frc[3][11]+m[11]*(ALPHA311+OMp111*l[2][11]);
Fq211 = Fs211+Fs212*C12-Fs312*S12;
Fq311 = Fs311+Fs212*S12+Fs312*C12;
Cq111 = -trq[1][11]+Cq112+In[1][11]*OMp111+Fs311*l[2][11]+dpt[2][9]*(Fs212*S12+Fs312*C12);
Fs210 = -frc[2][10]+m[10]*ALPHA210;
Fs310 = -frc[3][10]+m[10]*ALPHA310;
Fq210 = Fs210+Fq211*C11+Fq213*C13-Fq311*S11-Fq313*S13;
Fq310 = Fs310+Fq211*S11+Fq213*S13+Fq311*C11+Fq313*C13;
Cq110 = -trq[1][10]+Cq111+Cq113+qdd[10]*In[1][10]+dpt[2][7]*(Fq211*S11+Fq311*C11)+dpt[2][8]*(Fq213*S13+Fq313*C13);
Fq29 = Fq210*C10-Fq310*S10;
Fq39 = Fq210*S10+Fq310*C10;
Fs27 = -frc[2][7]+m[7]*(ALPHA27+BS57*l[2][7]);
Fs37 = -frc[3][7]+m[7]*(ALPHA37+OMp17*l[2][7]);
Cq17 = -trq[1][7]+In[1][7]*OMp17+Fs37*l[2][7];
Fs26 = -frc[2][6]+m[6]*(ALPHA26+BS56*l[2][6]);
Fs36 = -frc[3][6]+m[6]*(ALPHA36+OMp16*l[2][6]);
Fq26 = Fs26+Fs27*C7-Fs37*S7;
Fq36 = Fs36+Fs27*S7+Fs37*C7;
Cq16 = -trq[1][6]+Cq17+In[1][6]*OMp16+Fs36*l[2][6]+dpt[2][5]*(Fs27*S7+Fs37*C7);
Fs25 = -frc[2][5]+m[5]*(ALPHA25+BS55*l[2][5]);
Fs35 = -frc[3][5]+m[5]*(ALPHA35+OMp15*l[2][5]);
Cq15 = -trq[1][5]+In[1][5]*OMp15+Fs35*l[2][5];
Fs24 = -frc[2][4]+m[4]*(ALPHA24+BS54*l[2][4]);
Fs34 = -frc[3][4]+m[4]*(ALPHA34+OMp14*l[2][4]);
Fq24 = Fs24+Fs25*C5-Fs35*S5;
Fq34 = Fs34+Fs25*S5+Fs35*C5;
Cq14 = -trq[1][4]+Cq15+In[1][4]*OMp14+Fs34*l[2][4]+dpt[2][3]*(Fs25*S5+Fs35*C5);
Fs23 = -frc[2][3]+m[3]*ALPHA23;
Fs33 = -frc[3][3]+m[3]*ALPHA33;
Fq23 = Fs23+Fq24*C4+Fq26*C6-Fq34*S4-Fq36*S6;
Fq33 = Fs33+Fq24*S4+Fq26*S6+Fq34*C4+Fq36*C6;
Cq13 = -trq[1][3]+Cq14+Cq16+qdd[3]*In[1][3]+dpt[2][1]*(Fq24*S4+Fq34*C4)+dpt[2][2]*(Fq26*S6+Fq36*C6);
Fq22 = Fq23*C3-Fq33*S3;
Fq32 = Fq23*S3+Fq33*C3;
 
// Symbolic model output

Qq[1] = Fq22;
Qq[2] = Fq32;
Qq[3] = Cq13;
Qq[4] = Cq14;
Qq[5] = Cq15;
Qq[6] = Cq16;
Qq[7] = Cq17;
Qq[8] = Fq29;
Qq[9] = Fq39;
Qq[10] = Cq110;
Qq[11] = Cq111;
Qq[12] = Cq112;
Qq[13] = Cq113;
Qq[14] = Cq114;
Qq[15] = Fq216;
Qq[16] = Fq316;
Qq[17] = Cq117;
Qq[18] = Cq118;
Qq[19] = Cq119;
Qq[20] = Cq120;
Qq[21] = Cq121;

// Number of continuation lines = 0

}