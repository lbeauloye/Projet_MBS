//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universite catholique de Louvain 
//	Departement de Mecanique 
//	Unite de Production Mecanique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Sun May 31 21:41:20 2020
//
//	==> Project name : Simple_manege
//	==> using XML input file 
//
//	==> Number of joints : 9
//
//	==> Function : F 2 : Inverse Dynamics : RNEA
//	==> Flops complexity : 415
//
//	==> Generation Time :  0.000 seconds
//	==> Post-Processing :  0.010 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_invdyna(double *Qq,
MbsData *s, double tsim)

// double Qq[9];
{ 
 
#include "mbs_invdyna_Simple_manege.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

// = = Block_0_0_0_0_0_2 = = 
 
// Augmented Joint Position Vectors   

  Dz21 = q[2]+s->dpt[1][1];
  Dz53 = q[5]+s->dpt[3][4];
 
// Trigonometric Variables  

  C3 = cos(q[3]);
  S3 = sin(q[3]);
  C4 = cos(q[4]);
  S4 = sin(q[4]);

// = = Block_0_0_0_0_0_3 = = 
 
// Augmented Joint Position Vectors   

  Dz61 = q[6]+s->dpt[1][2];
  Dz93 = q[9]+s->dpt[3][6];
 
// Trigonometric Variables  

  C7 = cos(q[7]);
  S7 = sin(q[7]);
  C8 = cos(q[8]);
  S8 = sin(q[8]);

// = = Block_0_1_0_0_0_0 = = 
 
// Forward Kinematics 

  BS11 = -qd[1]*qd[1];
  BS12 = -qd[1]*qd[1];
  ALPHA12 = qdd[2]+BS11*Dz21;
  ALPHA22 = (2.0)*qd[1]*qd[2]+qdd[1]*Dz21;
  OM33 = qd[1]*C3;
  OMp33 = -(qd[1]*qd[3]*S3-qdd[1]*C3);
  ALPHA13 = ALPHA12+BS12*s->dpt[1][3];
  ALPHA23 = -(s->g[3]*S3-C3*(ALPHA22+qdd[1]*s->dpt[1][3]));
  ALPHA33 = -(s->g[3]*C3+S3*(ALPHA22+qdd[1]*s->dpt[1][3]));
  OM14 = qd[3]*C4-OM33*S4;
  OM24 = qd[4]+qd[1]*S3;
  OM34 = qd[3]*S4+OM33*C4;
  OMp14 = C4*(qdd[3]-qd[4]*OM33)-S4*(OMp33+qd[3]*qd[4]);
  OMp24 = qdd[4]+qd[1]*qd[3]*C3+qdd[1]*S3;
  BS94 = -(OM14*OM14+OM24*OM24);
  BETA34 = OMp24+OM14*OM34;
  BETA64 = OM24*OM34-OMp14;
  ALPHA14 = ALPHA13*C4-ALPHA33*S4;
  ALPHA34 = ALPHA13*S4+ALPHA33*C4;
  BS16 = -qd[1]*qd[1];
  ALPHA16 = qdd[6]+BS11*Dz61;
  ALPHA26 = (2.0)*qd[1]*qd[6]+qdd[1]*Dz61;
  OM37 = qd[1]*C7;
  OMp37 = -(qd[1]*qd[7]*S7-qdd[1]*C7);
  ALPHA17 = ALPHA16+BS16*s->dpt[1][5];
  ALPHA27 = -(s->g[3]*S7-C7*(ALPHA26+qdd[1]*s->dpt[1][5]));
  ALPHA37 = -(s->g[3]*C7+S7*(ALPHA26+qdd[1]*s->dpt[1][5]));
  OM18 = qd[7]*C8-OM37*S8;
  OM28 = qd[8]+qd[1]*S7;
  OM38 = qd[7]*S8+OM37*C8;
  OMp18 = C8*(qdd[7]-qd[8]*OM37)-S8*(OMp37+qd[7]*qd[8]);
  OMp28 = qdd[8]+qd[1]*qd[7]*C7+qdd[1]*S7;
  BS98 = -(OM18*OM18+OM28*OM28);
  BETA38 = OMp28+OM18*OM38;
  BETA68 = OM28*OM38-OMp18;
  ALPHA18 = ALPHA17*C8-ALPHA37*S8;
  ALPHA38 = ALPHA17*S8+ALPHA37*C8;
 
// Backward Dynamics 

  Fs19 = -(s->frc[1][9]-s->m[9]*(ALPHA18+(2.0)*qd[9]*OM28+BETA38*Dz93));
  Fs29 = -(s->frc[2][9]-s->m[9]*(ALPHA27-(2.0)*qd[9]*OM18+BETA68*Dz93));
  Fs39 = -(s->frc[3][9]-s->m[9]*(qdd[9]+ALPHA38+BS98*Dz93));
  Fs18 = -(s->frc[1][8]-s->m[8]*(ALPHA18+BETA38*s->l[3][8]));
  Fs28 = -(s->frc[2][8]-s->m[8]*(ALPHA27+BETA68*s->l[3][8]));
  Fq18 = Fs18+Fs19;
  Fq28 = Fs28+Fs29;
  Fq38 = -(s->frc[3][8]-Fs39-s->m[8]*(ALPHA38+BS98*s->l[3][8]));
  Cq18 = -(s->trq[1][8]+s->trq[1][9]-s->In[1][8]*OMp18-s->In[1][9]*OMp18+s->In[5][8]*OM28*OM38+Dz93*Fs29+Fs28*s->l[3][8]
 +OM28*OM38*(s->In[5][9]-s->In[9][9]));
  Cq28 = -(s->trq[2][8]+s->trq[2][9]-s->In[1][8]*OM18*OM38-s->In[5][8]*OMp28-s->In[5][9]*OMp28-Dz93*Fs19-Fs18*s->l[3][8]
 -OM18*OM38*(s->In[1][9]-s->In[9][9]));
  Cq38 = -(s->trq[3][8]+s->trq[3][9]-s->In[9][9]*(C8*(OMp37+qd[7]*qd[8])+S8*(qdd[7]-qd[8]*OM37))+OM18*OM28*(s->In[1][8]-
 s->In[5][8])+OM18*OM28*(s->In[1][9]-s->In[5][9]));
  Fq37 = -(Fq18*S8-Fq38*C8);
  Cq17 = Cq18*C8+Cq38*S8;
  Fs26 = -(s->frc[2][6]-s->m[6]*(ALPHA26+qdd[1]*s->l[1][6]));
  Fq16 = -(s->frc[1][6]-s->m[6]*(ALPHA16+BS16*s->l[1][6])-Fq18*C8-Fq38*S8);
  Fs15 = -(s->frc[1][5]-s->m[5]*(ALPHA14+(2.0)*qd[5]*OM24+BETA34*Dz53));
  Fs25 = -(s->frc[2][5]-s->m[5]*(ALPHA23-(2.0)*qd[5]*OM14+BETA64*Dz53));
  Fs35 = -(s->frc[3][5]-s->m[5]*(qdd[5]+ALPHA34+BS94*Dz53));
  Fs14 = -(s->frc[1][4]-s->m[4]*(ALPHA14+BETA34*s->l[3][4]));
  Fs24 = -(s->frc[2][4]-s->m[4]*(ALPHA23+BETA64*s->l[3][4]));
  Fq14 = Fs14+Fs15;
  Fq24 = Fs24+Fs25;
  Fq34 = -(s->frc[3][4]-Fs35-s->m[4]*(ALPHA34+BS94*s->l[3][4]));
  Cq14 = -(s->trq[1][4]+s->trq[1][5]-s->In[1][4]*OMp14-s->In[1][5]*OMp14+s->In[5][4]*OM24*OM34+Dz53*Fs25+Fs24*s->l[3][4]
 +OM24*OM34*(s->In[5][5]-s->In[9][5]));
  Cq24 = -(s->trq[2][4]+s->trq[2][5]-s->In[1][4]*OM14*OM34-s->In[5][4]*OMp24-s->In[5][5]*OMp24-Dz53*Fs15-Fs14*s->l[3][4]
 -OM14*OM34*(s->In[1][5]-s->In[9][5]));
  Cq34 = -(s->trq[3][4]+s->trq[3][5]-s->In[9][5]*(C4*(OMp33+qd[3]*qd[4])+S4*(qdd[3]-qd[4]*OM33))+OM14*OM24*(s->In[1][4]-
 s->In[5][4])+OM14*OM24*(s->In[1][5]-s->In[5][5]));
  Fq33 = -(Fq14*S4-Fq34*C4);
  Cq13 = Cq14*C4+Cq34*S4;
  Fs22 = -(s->frc[2][2]-s->m[2]*(ALPHA22+qdd[1]*s->l[1][2]));
  Fq12 = -(s->frc[1][2]-s->m[2]*(ALPHA12+BS12*s->l[1][2])-Fq14*C4-Fq34*S4);
  Cq31 = -(s->trq[3][1]+s->trq[3][2]+s->trq[3][6]-qdd[1]*s->In[9][1]-qdd[1]*s->In[9][2]-qdd[1]*s->In[9][6]-Cq24*S3-Cq28*
 S7-Dz21*(Fs22+Fq24*C3-Fq33*S3)-Dz61*(Fs26+Fq28*C7-Fq37*S7)-Fs22*s->l[1][2]-Fs26*s->l[1][6]-s->dpt[1][3]*(Fq24*C3-Fq33*S3)-
 s->dpt[1][5]*(Fq28*C7-Fq37*S7)+C3*(Cq14*S4-Cq34*C4)+C7*(Cq18*S8-Cq38*C8));

// = = Block_0_2_0_0_0_0 = = 
 
// Symbolic Outputs  

  Qq[1] = Cq31;
  Qq[2] = Fq12;
  Qq[3] = Cq13;
  Qq[4] = Cq24;
  Qq[5] = Fs35;
  Qq[6] = Fq16;
  Qq[7] = Cq17;
  Qq[8] = Cq28;
  Qq[9] = Fs39;

// ====== END Task 0 ====== 


}
 

