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
//	==> Function : F 1 : Direct Dynamics (Semi-Explicit formulation) : RNEA
//	==> Flops complexity : 577
//
//	==> Generation Time :  0.000 seconds
//	==> Post-Processing :  0.010 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_dirdyna(double **M,double *c,
MbsData *s, double tsim)

// double M[9][9];
// double c[9];
{ 
 
#include "mbs_dirdyna_Simple_manege.h" 
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

// = = Block_0_1_0_0_0_1 = = 
 
// Forward Kinematics 

  BS11 = -qd[1]*qd[1];

// = = Block_0_1_0_1_0_2 = = 
 
// Forward Kinematics 

  BS12 = -qd[1]*qd[1];
  AlF12 = BS11*Dz21;
  AlF22 = (2.0)*qd[1]*qd[2];
  OM33 = qd[1]*C3;
  OpF23 = qd[1]*qd[3]*C3;
  OpF33 = -qd[1]*qd[3]*S3;
  AlF13 = AlF12+BS12*s->dpt[1][3];
  AlF23 = AlF22*C3-s->g[3]*S3;
  AlF33 = -(AlF22*S3+s->g[3]*C3);
  AlM23_1 = C3*(Dz21+s->dpt[1][3]);
  AlM33_1 = -S3*(Dz21+s->dpt[1][3]);
  OM14 = qd[3]*C4-OM33*S4;
  OM24 = qd[4]+qd[1]*S3;
  OM34 = qd[3]*S4+OM33*C4;
  OpF14 = -(qd[4]*OM33*C4+S4*(OpF33+qd[3]*qd[4]));
  BS94 = -(OM14*OM14+OM24*OM24);
  BeF34 = OpF23+OM14*OM34;
  BeF64 = OM24*OM34-OpF14;
  AlF14 = AlF13*C4-AlF33*S4;
  AlF34 = AlF13*S4+AlF33*C4;
  OpM14_1 = -C3*S4;
  AlM14_1 = -AlM33_1*S4;
  AlM34_1 = AlM33_1*C4;

// = = Block_0_1_0_1_0_3 = = 
 
// Forward Kinematics 

  BS16 = -qd[1]*qd[1];
  AlF16 = BS11*Dz61;
  AlF26 = (2.0)*qd[1]*qd[6];
  OM37 = qd[1]*C7;
  OpF27 = qd[1]*qd[7]*C7;
  OpF37 = -qd[1]*qd[7]*S7;
  AlF17 = AlF16+BS16*s->dpt[1][5];
  AlF27 = AlF26*C7-s->g[3]*S7;
  AlF37 = -(AlF26*S7+s->g[3]*C7);
  AlM27_1 = C7*(Dz61+s->dpt[1][5]);
  AlM37_1 = -S7*(Dz61+s->dpt[1][5]);
  OM18 = qd[7]*C8-OM37*S8;
  OM28 = qd[8]+qd[1]*S7;
  OM38 = qd[7]*S8+OM37*C8;
  OpF18 = -(qd[8]*OM37*C8+S8*(OpF37+qd[7]*qd[8]));
  BS98 = -(OM18*OM18+OM28*OM28);
  BeF38 = OpF27+OM18*OM38;
  BeF68 = OM28*OM38-OpF18;
  AlF18 = AlF17*C8-AlF37*S8;
  AlF38 = AlF17*S8+AlF37*C8;
  OpM18_1 = -C7*S8;
  AlM18_1 = -AlM37_1*S8;
  AlM38_1 = AlM37_1*C8;

// = = Block_0_2_0_1_0_2 = = 
 
// Backward Dynamics 

  FA15 = -(s->frc[1][5]-s->m[5]*(AlF14+(2.0)*qd[5]*OM24+BeF34*Dz53));
  FA25 = -(s->frc[2][5]-s->m[5]*(AlF23-(2.0)*qd[5]*OM14+BeF64*Dz53));
  FA35 = -(s->frc[3][5]-s->m[5]*(AlF34+BS94*Dz53));
  FB15_1 = s->m[5]*(AlM14_1+Dz53*S3);
  FB25_1 = s->m[5]*(AlM23_1-Dz53*OpM14_1);
  FB35_1 = s->m[5]*AlM34_1;
  CM35_1 = s->In[9][5]*C3*C4;
  FB35_2 = s->m[5]*S4;
  FA14 = -(s->frc[1][4]-s->m[4]*(AlF14+BeF34*s->l[3][4]));
  FA24 = -(s->frc[2][4]-s->m[4]*(AlF23+BeF64*s->l[3][4]));
  FF14 = FA14+FA15;
  FF24 = FA24+FA25;
  FF34 = -(s->frc[3][4]-FA35-s->m[4]*(AlF34+BS94*s->l[3][4]));
  CF14 = -(s->trq[1][4]+s->trq[1][5]-s->In[1][4]*OpF14-s->In[1][5]*OpF14+s->In[5][4]*OM24*OM34+Dz53*FA25+FA24*s->l[3][4]
 +OM24*OM34*(s->In[5][5]-s->In[9][5]));
  CF24 = -(s->trq[2][4]+s->trq[2][5]-s->In[1][4]*OM14*OM34-s->In[5][4]*OpF23-s->In[5][5]*OpF23-Dz53*FA15-FA14*s->l[3][4]
 -OM14*OM34*(s->In[1][5]-s->In[9][5]));
  CF34 = -(s->trq[3][4]+s->trq[3][5]+s->In[9][5]*(qd[4]*OM33*S4-C4*(OpF33+qd[3]*qd[4]))+OM14*OM24*(s->In[1][4]-
 s->In[5][4])+OM14*OM24*(s->In[1][5]-s->In[5][5]));
  FB14_1 = s->m[4]*(AlM14_1+s->l[3][4]*S3);
  FB24_1 = s->m[4]*(AlM23_1-OpM14_1*s->l[3][4]);
  FM14_1 = FB14_1+FB15_1;
  FM24_1 = FB24_1+FB25_1;
  FM34_1 = FB35_1+s->m[4]*AlM34_1;
  CM14_1 = -(Dz53*FB25_1+FB24_1*s->l[3][4]-OpM14_1*(s->In[1][4]+s->In[1][5]));
  CM24_1 = Dz53*FB15_1+FB14_1*s->l[3][4]+S3*(s->In[5][4]+s->In[5][5]);
  CM24_2 = C4*(s->m[4]*s->l[3][4]+s->m[5]*Dz53);
  CM24_4 = s->In[5][4]+s->In[5][5]+s->m[4]*s->l[3][4]*s->l[3][4]+s->m[5]*Dz53*Dz53;
  FF3_34 = -(FF14*S4-FF34*C4);
  CF3_14 = CF14*C4+CF34*S4;
  FM31_14 = FM14_1*C4+FM34_1*S4;
  FM31_34 = -(FM14_1*S4-FM34_1*C4);
  CM31_14 = CM14_1*C4+CM35_1*S4;
  CM33_14 = s->In[9][5]*S4*S4+C4*C4*(s->In[1][4]+s->In[1][5]+s->m[4]*s->l[3][4]*s->l[3][4]+s->m[5]*Dz53*Dz53);
  FA22 = -(s->frc[2][2]-s->m[2]*AlF22);
  FF12 = -(s->frc[1][2]-s->m[2]*(AlF12+BS12*s->l[1][2])-FF14*C4-FF34*S4);
  FB22_1 = s->m[2]*(Dz21+s->l[1][2]);
  FM12_2 = s->m[2]+s->m[4]+s->m[5];

// = = Block_0_2_0_1_0_3 = = 
 
// Backward Dynamics 

  FA19 = -(s->frc[1][9]-s->m[9]*(AlF18+(2.0)*qd[9]*OM28+BeF38*Dz93));
  FA29 = -(s->frc[2][9]-s->m[9]*(AlF27-(2.0)*qd[9]*OM18+BeF68*Dz93));
  FA39 = -(s->frc[3][9]-s->m[9]*(AlF38+BS98*Dz93));
  FB19_1 = s->m[9]*(AlM18_1+Dz93*S7);
  FB29_1 = s->m[9]*(AlM27_1-Dz93*OpM18_1);
  FB39_1 = s->m[9]*AlM38_1;
  CM39_1 = s->In[9][9]*C7*C8;
  FB39_6 = s->m[9]*S8;
  FA18 = -(s->frc[1][8]-s->m[8]*(AlF18+BeF38*s->l[3][8]));
  FA28 = -(s->frc[2][8]-s->m[8]*(AlF27+BeF68*s->l[3][8]));
  FF18 = FA18+FA19;
  FF28 = FA28+FA29;
  FF38 = -(s->frc[3][8]-FA39-s->m[8]*(AlF38+BS98*s->l[3][8]));
  CF18 = -(s->trq[1][8]+s->trq[1][9]-s->In[1][8]*OpF18-s->In[1][9]*OpF18+s->In[5][8]*OM28*OM38+Dz93*FA29+FA28*s->l[3][8]
 +OM28*OM38*(s->In[5][9]-s->In[9][9]));
  CF28 = -(s->trq[2][8]+s->trq[2][9]-s->In[1][8]*OM18*OM38-s->In[5][8]*OpF27-s->In[5][9]*OpF27-Dz93*FA19-FA18*s->l[3][8]
 -OM18*OM38*(s->In[1][9]-s->In[9][9]));
  CF38 = -(s->trq[3][8]+s->trq[3][9]+s->In[9][9]*(qd[8]*OM37*S8-C8*(OpF37+qd[7]*qd[8]))+OM18*OM28*(s->In[1][8]-
 s->In[5][8])+OM18*OM28*(s->In[1][9]-s->In[5][9]));
  FB18_1 = s->m[8]*(AlM18_1+s->l[3][8]*S7);
  FB28_1 = s->m[8]*(AlM27_1-OpM18_1*s->l[3][8]);
  FM18_1 = FB18_1+FB19_1;
  FM28_1 = FB28_1+FB29_1;
  FM38_1 = FB39_1+s->m[8]*AlM38_1;
  CM18_1 = -(Dz93*FB29_1+FB28_1*s->l[3][8]-OpM18_1*(s->In[1][8]+s->In[1][9]));
  CM28_1 = Dz93*FB19_1+FB18_1*s->l[3][8]+S7*(s->In[5][8]+s->In[5][9]);
  CM28_6 = C8*(s->m[8]*s->l[3][8]+s->m[9]*Dz93);
  CM28_8 = s->In[5][8]+s->In[5][9]+s->m[8]*s->l[3][8]*s->l[3][8]+s->m[9]*Dz93*Dz93;
  FF7_38 = -(FF18*S8-FF38*C8);
  CF7_18 = CF18*C8+CF38*S8;
  FM71_18 = FM18_1*C8+FM38_1*S8;
  FM71_38 = -(FM18_1*S8-FM38_1*C8);
  CM71_18 = CM18_1*C8+CM39_1*S8;
  CM77_18 = s->In[9][9]*S8*S8+C8*C8*(s->In[1][8]+s->In[1][9]+s->m[8]*s->l[3][8]*s->l[3][8]+s->m[9]*Dz93*Dz93);
  FA26 = -(s->frc[2][6]-s->m[6]*AlF26);
  FF16 = -(s->frc[1][6]-s->m[6]*(AlF16+BS16*s->l[1][6])-FF18*C8-FF38*S8);
  FB26_1 = s->m[6]*(Dz61+s->l[1][6]);
  FM16_6 = s->m[6]+s->m[8]+s->m[9];

// = = Block_0_2_0_2_0_1 = = 
 
// Backward Dynamics 

  CF31 = -(s->trq[3][1]+s->trq[3][2]+s->trq[3][6]-CF24*S3-CF28*S7-Dz21*(FA22+FF24*C3-FF3_34*S3)-Dz61*(FA26+FF28*C7-
 FF7_38*S7)-FA22*s->l[1][2]-FA26*s->l[1][6]-s->dpt[1][3]*(FF24*C3-FF3_34*S3)-s->dpt[1][5]*(FF28*C7-FF7_38*S7)+C3*(CF14*S4-
 CF34*C4)+C7*(CF18*S8-CF38*C8));
  CM31_1 = s->In[9][1]+s->In[9][2]+s->In[9][6]+CM24_1*S3+CM28_1*S7+Dz21*(FB22_1+FM24_1*C3-FM31_34*S3)+Dz61*(FB26_1+
 FM28_1*C7-FM71_38*S7)+FB22_1*s->l[1][2]+FB26_1*s->l[1][6]+s->dpt[1][3]*(FM24_1*C3-FM31_34*S3)+s->dpt[1][5]*(FM28_1*C7-
 FM71_38*S7)-C3*(CM14_1*S4-CM35_1*C4)-C7*(CM18_1*S8-CM39_1*C8);

// = = Block_0_3_0_0_0_0 = = 
 
// Symbolic Outputs  

  M[1][1] = CM31_1;
  M[1][2] = FM31_14;
  M[1][3] = CM31_14;
  M[1][4] = CM24_1;
  M[1][5] = FB35_1;
  M[1][6] = FM71_18;
  M[1][7] = CM71_18;
  M[1][8] = CM28_1;
  M[1][9] = FB39_1;
  M[2][1] = FM31_14;
  M[2][2] = FM12_2;
  M[2][4] = CM24_2;
  M[2][5] = FB35_2;
  M[3][1] = CM31_14;
  M[3][3] = CM33_14;
  M[4][1] = CM24_1;
  M[4][2] = CM24_2;
  M[4][4] = CM24_4;
  M[5][1] = FB35_1;
  M[5][2] = FB35_2;
  M[5][5] = s->m[5];
  M[6][1] = FM71_18;
  M[6][6] = FM16_6;
  M[6][8] = CM28_6;
  M[6][9] = FB39_6;
  M[7][1] = CM71_18;
  M[7][7] = CM77_18;
  M[8][1] = CM28_1;
  M[8][6] = CM28_6;
  M[8][8] = CM28_8;
  M[9][1] = FB39_1;
  M[9][6] = FB39_6;
  M[9][9] = s->m[9];
  c[1] = CF31;
  c[2] = FF12;
  c[3] = CF3_14;
  c[4] = CF24;
  c[5] = FA35;
  c[6] = FF16;
  c[7] = CF7_18;
  c[8] = CF28;
  c[9] = FA39;

// ====== END Task 0 ====== 


}
 

