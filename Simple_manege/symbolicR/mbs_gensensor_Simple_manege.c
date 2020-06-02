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
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 736
//
//	==> Generation Time :  0.000 seconds
//	==> Post-Processing :  0.020 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
#include "mbs_sensor.h"
 
void  mbs_gensensor(MbsSensor *sens, 
              MbsData *s,
              int isens)
{ 
 
#include "mbs_gensensor_Simple_manege.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 
 
// Sensor Kinematics 



// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

  C1 = cos(q[1]);
  S1 = sin(q[1]);

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

// ====== END Task 0 ====== 

// ===== BEGIN task 1 ===== 
 
switch(isens)
{
 
// 
break;
case 1:
 


// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->R[1][1] = C1;
    sens->R[1][2] = S1;
    sens->R[2][1] = -S1;
    sens->R[2][2] = C1;
    sens->R[3][3] = (1.0);
    sens->OM[3] = qd[1];
    sens->OMP[3] = qdd[1];
 
// 
break;
case 2:
 


// = = Block_1_0_0_2_0_2 = = 
 
// Sensor Kinematics 


    RLcp1_12 = Dz21*C1;
    RLcp1_22 = Dz21*S1;
    ORcp1_12 = -RLcp1_22*qd[1];
    ORcp1_22 = RLcp1_12*qd[1];
    VIcp1_12 = ORcp1_12+qd[2]*C1;
    VIcp1_22 = ORcp1_22+qd[2]*S1;
    ACcp1_12 = -(RLcp1_22*qdd[1]-qdd[2]*C1+qd[1]*(ORcp1_22+(2.0)*qd[2]*S1));
    ACcp1_22 = RLcp1_12*qdd[1]+qdd[2]*S1+qd[1]*(ORcp1_12+(2.0)*qd[2]*C1);

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = RLcp1_12;
    sens->P[2] = RLcp1_22;
    sens->P[3] = s->dpt[3][1];
    sens->R[1][1] = C1;
    sens->R[1][2] = S1;
    sens->R[2][1] = -S1;
    sens->R[2][2] = C1;
    sens->R[3][3] = (1.0);
    sens->V[1] = VIcp1_12;
    sens->V[2] = VIcp1_22;
    sens->OM[3] = qd[1];
    sens->A[1] = ACcp1_12;
    sens->A[2] = ACcp1_22;
    sens->OMP[3] = qdd[1];
 
// 
break;
case 3:
 


// = = Block_1_0_0_3_0_2 = = 
 
// Sensor Kinematics 


    ROcp2_43 = -S1*C3;
    ROcp2_53 = C1*C3;
    ROcp2_73 = S1*S3;
    ROcp2_83 = -C1*S3;
    RLcp2_12 = Dz21*C1;
    RLcp2_22 = Dz21*S1;
    ORcp2_12 = -RLcp2_22*qd[1];
    ORcp2_22 = RLcp2_12*qd[1];
    RLcp2_13 = s->dpt[1][3]*C1;
    RLcp2_23 = s->dpt[1][3]*S1;
    POcp2_13 = RLcp2_12+RLcp2_13;
    POcp2_23 = RLcp2_22+RLcp2_23;
    POcp2_33 = s->dpt[3][1]+s->dpt[3][3];
    OMcp2_13 = qd[3]*C1;
    OMcp2_23 = qd[3]*S1;
    ORcp2_13 = -RLcp2_23*qd[1];
    ORcp2_23 = RLcp2_13*qd[1];
    VIcp2_13 = ORcp2_12+ORcp2_13+qd[2]*C1;
    VIcp2_23 = ORcp2_22+ORcp2_23+qd[2]*S1;
    OPcp2_13 = qdd[3]*C1-qd[1]*qd[3]*S1;
    OPcp2_23 = qdd[3]*S1+qd[1]*qd[3]*C1;
    ACcp2_13 = -(ORcp2_23*qd[1]+RLcp2_22*qdd[1]+RLcp2_23*qdd[1]-qdd[2]*C1+qd[1]*(ORcp2_22+(2.0)*qd[2]*S1));
    ACcp2_23 = ORcp2_13*qd[1]+RLcp2_12*qdd[1]+RLcp2_13*qdd[1]+qdd[2]*S1+qd[1]*(ORcp2_12+(2.0)*qd[2]*C1);

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_13;
    sens->P[2] = POcp2_23;
    sens->P[3] = POcp2_33;
    sens->R[1][1] = C1;
    sens->R[1][2] = S1;
    sens->R[2][1] = ROcp2_43;
    sens->R[2][2] = ROcp2_53;
    sens->R[2][3] = S3;
    sens->R[3][1] = ROcp2_73;
    sens->R[3][2] = ROcp2_83;
    sens->R[3][3] = C3;
    sens->V[1] = VIcp2_13;
    sens->V[2] = VIcp2_23;
    sens->OM[1] = OMcp2_13;
    sens->OM[2] = OMcp2_23;
    sens->OM[3] = qd[1];
    sens->A[1] = ACcp2_13;
    sens->A[2] = ACcp2_23;
    sens->OMP[1] = OPcp2_13;
    sens->OMP[2] = OPcp2_23;
    sens->OMP[3] = qdd[1];
 
// 
break;
case 4:
 


// = = Block_1_0_0_4_0_2 = = 
 
// Sensor Kinematics 


    ROcp3_43 = -S1*C3;
    ROcp3_53 = C1*C3;
    ROcp3_73 = S1*S3;
    ROcp3_83 = -C1*S3;
    ROcp3_14 = -(ROcp3_73*S4-C1*C4);
    ROcp3_24 = -(ROcp3_83*S4-S1*C4);
    ROcp3_34 = -C3*S4;
    ROcp3_74 = ROcp3_73*C4+C1*S4;
    ROcp3_84 = ROcp3_83*C4+S1*S4;
    ROcp3_94 = C3*C4;
    RLcp3_12 = Dz21*C1;
    RLcp3_22 = Dz21*S1;
    ORcp3_12 = -RLcp3_22*qd[1];
    ORcp3_22 = RLcp3_12*qd[1];
    RLcp3_13 = s->dpt[1][3]*C1;
    RLcp3_23 = s->dpt[1][3]*S1;
    POcp3_13 = RLcp3_12+RLcp3_13;
    POcp3_23 = RLcp3_22+RLcp3_23;
    POcp3_33 = s->dpt[3][1]+s->dpt[3][3];
    OMcp3_13 = qd[3]*C1;
    OMcp3_23 = qd[3]*S1;
    ORcp3_13 = -RLcp3_23*qd[1];
    ORcp3_23 = RLcp3_13*qd[1];
    VIcp3_13 = ORcp3_12+ORcp3_13+qd[2]*C1;
    VIcp3_23 = ORcp3_22+ORcp3_23+qd[2]*S1;
    ACcp3_13 = -(ORcp3_23*qd[1]+RLcp3_22*qdd[1]+RLcp3_23*qdd[1]-qdd[2]*C1+qd[1]*(ORcp3_22+(2.0)*qd[2]*S1));
    ACcp3_23 = ORcp3_13*qd[1]+RLcp3_12*qdd[1]+RLcp3_13*qdd[1]+qdd[2]*S1+qd[1]*(ORcp3_12+(2.0)*qd[2]*C1);
    OMcp3_14 = OMcp3_13+ROcp3_43*qd[4];
    OMcp3_24 = OMcp3_23+ROcp3_53*qd[4];
    OMcp3_34 = qd[1]+qd[4]*S3;
    OPcp3_14 = ROcp3_43*qdd[4]+qdd[3]*C1-qd[1]*qd[3]*S1+qd[4]*(OMcp3_23*S3-ROcp3_53*qd[1]);
    OPcp3_24 = ROcp3_53*qdd[4]+qdd[3]*S1+qd[1]*qd[3]*C1-qd[4]*(OMcp3_13*S3-ROcp3_43*qd[1]);
    OPcp3_34 = qdd[1]+qdd[4]*S3+qd[3]*qd[4]*C3;

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_13;
    sens->P[2] = POcp3_23;
    sens->P[3] = POcp3_33;
    sens->R[1][1] = ROcp3_14;
    sens->R[1][2] = ROcp3_24;
    sens->R[1][3] = ROcp3_34;
    sens->R[2][1] = ROcp3_43;
    sens->R[2][2] = ROcp3_53;
    sens->R[2][3] = S3;
    sens->R[3][1] = ROcp3_74;
    sens->R[3][2] = ROcp3_84;
    sens->R[3][3] = ROcp3_94;
    sens->V[1] = VIcp3_13;
    sens->V[2] = VIcp3_23;
    sens->OM[1] = OMcp3_14;
    sens->OM[2] = OMcp3_24;
    sens->OM[3] = OMcp3_34;
    sens->A[1] = ACcp3_13;
    sens->A[2] = ACcp3_23;
    sens->OMP[1] = OPcp3_14;
    sens->OMP[2] = OPcp3_24;
    sens->OMP[3] = OPcp3_34;
 
// 
break;
case 5:
 


// = = Block_1_0_0_5_0_2 = = 
 
// Sensor Kinematics 


    ROcp4_43 = -S1*C3;
    ROcp4_53 = C1*C3;
    ROcp4_73 = S1*S3;
    ROcp4_83 = -C1*S3;
    ROcp4_14 = -(ROcp4_73*S4-C1*C4);
    ROcp4_24 = -(ROcp4_83*S4-S1*C4);
    ROcp4_34 = -C3*S4;
    ROcp4_74 = ROcp4_73*C4+C1*S4;
    ROcp4_84 = ROcp4_83*C4+S1*S4;
    ROcp4_94 = C3*C4;
    RLcp4_12 = Dz21*C1;
    RLcp4_22 = Dz21*S1;
    ORcp4_12 = -RLcp4_22*qd[1];
    ORcp4_22 = RLcp4_12*qd[1];
    RLcp4_13 = s->dpt[1][3]*C1;
    RLcp4_23 = s->dpt[1][3]*S1;
    OMcp4_13 = qd[3]*C1;
    OMcp4_23 = qd[3]*S1;
    ORcp4_13 = -RLcp4_23*qd[1];
    ORcp4_23 = RLcp4_13*qd[1];
    OMcp4_14 = OMcp4_13+ROcp4_43*qd[4];
    OMcp4_24 = OMcp4_23+ROcp4_53*qd[4];
    OMcp4_34 = qd[1]+qd[4]*S3;
    OPcp4_14 = ROcp4_43*qdd[4]+qdd[3]*C1-qd[1]*qd[3]*S1+qd[4]*(OMcp4_23*S3-ROcp4_53*qd[1]);
    OPcp4_24 = ROcp4_53*qdd[4]+qdd[3]*S1+qd[1]*qd[3]*C1-qd[4]*(OMcp4_13*S3-ROcp4_43*qd[1]);
    OPcp4_34 = qdd[1]+qdd[4]*S3+qd[3]*qd[4]*C3;
    RLcp4_15 = Dz53*ROcp4_74;
    RLcp4_25 = Dz53*ROcp4_84;
    RLcp4_35 = Dz53*ROcp4_94;
    POcp4_15 = RLcp4_12+RLcp4_13+RLcp4_15;
    POcp4_25 = RLcp4_22+RLcp4_23+RLcp4_25;
    POcp4_35 = RLcp4_35+s->dpt[3][1]+s->dpt[3][3];
    ORcp4_15 = OMcp4_24*RLcp4_35-OMcp4_34*RLcp4_25;
    ORcp4_25 = -(OMcp4_14*RLcp4_35-OMcp4_34*RLcp4_15);
    ORcp4_35 = OMcp4_14*RLcp4_25-OMcp4_24*RLcp4_15;
    VIcp4_15 = ORcp4_12+ORcp4_13+ORcp4_15+ROcp4_74*qd[5]+qd[2]*C1;
    VIcp4_25 = ORcp4_22+ORcp4_23+ORcp4_25+ROcp4_84*qd[5]+qd[2]*S1;
    VIcp4_35 = ORcp4_35+ROcp4_94*qd[5];
    ACcp4_15 = OMcp4_24*ORcp4_35-OMcp4_34*ORcp4_25+OPcp4_24*RLcp4_35-OPcp4_34*RLcp4_25-ORcp4_23*qd[1]-RLcp4_22*qdd[1]-
 RLcp4_23*qdd[1]+ROcp4_74*qdd[5]+qdd[2]*C1-qd[1]*(ORcp4_22+(2.0)*qd[2]*S1)+(2.0)*qd[5]*(OMcp4_24*ROcp4_94-OMcp4_34*ROcp4_84);
    ACcp4_25 = -(OMcp4_14*ORcp4_35-OMcp4_34*ORcp4_15+OPcp4_14*RLcp4_35-OPcp4_34*RLcp4_15-ORcp4_13*qd[1]-RLcp4_12*qdd[1]-
 RLcp4_13*qdd[1]-ROcp4_84*qdd[5]-qdd[2]*S1-qd[1]*(ORcp4_12+(2.0)*qd[2]*C1)+(2.0)*qd[5]*(OMcp4_14*ROcp4_94-OMcp4_34*ROcp4_74));
    ACcp4_35 = OMcp4_14*ORcp4_25-OMcp4_24*ORcp4_15+OPcp4_14*RLcp4_25-OPcp4_24*RLcp4_15+ROcp4_94*qdd[5]+(2.0)*qd[5]*(OMcp4_14*
 ROcp4_84-OMcp4_24*ROcp4_74);

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_15;
    sens->P[2] = POcp4_25;
    sens->P[3] = POcp4_35;
    sens->R[1][1] = ROcp4_14;
    sens->R[1][2] = ROcp4_24;
    sens->R[1][3] = ROcp4_34;
    sens->R[2][1] = ROcp4_43;
    sens->R[2][2] = ROcp4_53;
    sens->R[2][3] = S3;
    sens->R[3][1] = ROcp4_74;
    sens->R[3][2] = ROcp4_84;
    sens->R[3][3] = ROcp4_94;
    sens->V[1] = VIcp4_15;
    sens->V[2] = VIcp4_25;
    sens->V[3] = VIcp4_35;
    sens->OM[1] = OMcp4_14;
    sens->OM[2] = OMcp4_24;
    sens->OM[3] = OMcp4_34;
    sens->A[1] = ACcp4_15;
    sens->A[2] = ACcp4_25;
    sens->A[3] = ACcp4_35;
    sens->OMP[1] = OPcp4_14;
    sens->OMP[2] = OPcp4_24;
    sens->OMP[3] = OPcp4_34;
 
// 
break;
case 6:
 


// = = Block_1_0_0_6_0_3 = = 
 
// Sensor Kinematics 


    RLcp5_16 = Dz61*C1;
    RLcp5_26 = Dz61*S1;
    ORcp5_16 = -RLcp5_26*qd[1];
    ORcp5_26 = RLcp5_16*qd[1];
    VIcp5_16 = ORcp5_16+qd[6]*C1;
    VIcp5_26 = ORcp5_26+qd[6]*S1;
    ACcp5_16 = -(RLcp5_26*qdd[1]-qdd[6]*C1+qd[1]*(ORcp5_26+(2.0)*qd[6]*S1));
    ACcp5_26 = RLcp5_16*qdd[1]+qdd[6]*S1+qd[1]*(ORcp5_16+(2.0)*qd[6]*C1);

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = RLcp5_16;
    sens->P[2] = RLcp5_26;
    sens->P[3] = s->dpt[3][2];
    sens->R[1][1] = C1;
    sens->R[1][2] = S1;
    sens->R[2][1] = -S1;
    sens->R[2][2] = C1;
    sens->R[3][3] = (1.0);
    sens->V[1] = VIcp5_16;
    sens->V[2] = VIcp5_26;
    sens->OM[3] = qd[1];
    sens->A[1] = ACcp5_16;
    sens->A[2] = ACcp5_26;
    sens->OMP[3] = qdd[1];
 
// 
break;
case 7:
 


// = = Block_1_0_0_7_0_3 = = 
 
// Sensor Kinematics 


    ROcp6_47 = -S1*C7;
    ROcp6_57 = C1*C7;
    ROcp6_77 = S1*S7;
    ROcp6_87 = -C1*S7;
    RLcp6_16 = Dz61*C1;
    RLcp6_26 = Dz61*S1;
    ORcp6_16 = -RLcp6_26*qd[1];
    ORcp6_26 = RLcp6_16*qd[1];
    RLcp6_17 = s->dpt[1][5]*C1;
    RLcp6_27 = s->dpt[1][5]*S1;
    POcp6_17 = RLcp6_16+RLcp6_17;
    POcp6_27 = RLcp6_26+RLcp6_27;
    POcp6_37 = s->dpt[3][2]+s->dpt[3][5];
    OMcp6_17 = qd[7]*C1;
    OMcp6_27 = qd[7]*S1;
    ORcp6_17 = -RLcp6_27*qd[1];
    ORcp6_27 = RLcp6_17*qd[1];
    VIcp6_17 = ORcp6_16+ORcp6_17+qd[6]*C1;
    VIcp6_27 = ORcp6_26+ORcp6_27+qd[6]*S1;
    OPcp6_17 = qdd[7]*C1-qd[1]*qd[7]*S1;
    OPcp6_27 = qdd[7]*S1+qd[1]*qd[7]*C1;
    ACcp6_17 = -(ORcp6_27*qd[1]+RLcp6_26*qdd[1]+RLcp6_27*qdd[1]-qdd[6]*C1+qd[1]*(ORcp6_26+(2.0)*qd[6]*S1));
    ACcp6_27 = ORcp6_17*qd[1]+RLcp6_16*qdd[1]+RLcp6_17*qdd[1]+qdd[6]*S1+qd[1]*(ORcp6_16+(2.0)*qd[6]*C1);

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_17;
    sens->P[2] = POcp6_27;
    sens->P[3] = POcp6_37;
    sens->R[1][1] = C1;
    sens->R[1][2] = S1;
    sens->R[2][1] = ROcp6_47;
    sens->R[2][2] = ROcp6_57;
    sens->R[2][3] = S7;
    sens->R[3][1] = ROcp6_77;
    sens->R[3][2] = ROcp6_87;
    sens->R[3][3] = C7;
    sens->V[1] = VIcp6_17;
    sens->V[2] = VIcp6_27;
    sens->OM[1] = OMcp6_17;
    sens->OM[2] = OMcp6_27;
    sens->OM[3] = qd[1];
    sens->A[1] = ACcp6_17;
    sens->A[2] = ACcp6_27;
    sens->OMP[1] = OPcp6_17;
    sens->OMP[2] = OPcp6_27;
    sens->OMP[3] = qdd[1];
 
// 
break;
case 8:
 


// = = Block_1_0_0_8_0_3 = = 
 
// Sensor Kinematics 


    ROcp7_47 = -S1*C7;
    ROcp7_57 = C1*C7;
    ROcp7_77 = S1*S7;
    ROcp7_87 = -C1*S7;
    ROcp7_18 = -(ROcp7_77*S8-C1*C8);
    ROcp7_28 = -(ROcp7_87*S8-S1*C8);
    ROcp7_38 = -C7*S8;
    ROcp7_78 = ROcp7_77*C8+C1*S8;
    ROcp7_88 = ROcp7_87*C8+S1*S8;
    ROcp7_98 = C7*C8;
    RLcp7_16 = Dz61*C1;
    RLcp7_26 = Dz61*S1;
    ORcp7_16 = -RLcp7_26*qd[1];
    ORcp7_26 = RLcp7_16*qd[1];
    RLcp7_17 = s->dpt[1][5]*C1;
    RLcp7_27 = s->dpt[1][5]*S1;
    POcp7_17 = RLcp7_16+RLcp7_17;
    POcp7_27 = RLcp7_26+RLcp7_27;
    POcp7_37 = s->dpt[3][2]+s->dpt[3][5];
    OMcp7_17 = qd[7]*C1;
    OMcp7_27 = qd[7]*S1;
    ORcp7_17 = -RLcp7_27*qd[1];
    ORcp7_27 = RLcp7_17*qd[1];
    VIcp7_17 = ORcp7_16+ORcp7_17+qd[6]*C1;
    VIcp7_27 = ORcp7_26+ORcp7_27+qd[6]*S1;
    ACcp7_17 = -(ORcp7_27*qd[1]+RLcp7_26*qdd[1]+RLcp7_27*qdd[1]-qdd[6]*C1+qd[1]*(ORcp7_26+(2.0)*qd[6]*S1));
    ACcp7_27 = ORcp7_17*qd[1]+RLcp7_16*qdd[1]+RLcp7_17*qdd[1]+qdd[6]*S1+qd[1]*(ORcp7_16+(2.0)*qd[6]*C1);
    OMcp7_18 = OMcp7_17+ROcp7_47*qd[8];
    OMcp7_28 = OMcp7_27+ROcp7_57*qd[8];
    OMcp7_38 = qd[1]+qd[8]*S7;
    OPcp7_18 = ROcp7_47*qdd[8]+qdd[7]*C1-qd[1]*qd[7]*S1+qd[8]*(OMcp7_27*S7-ROcp7_57*qd[1]);
    OPcp7_28 = ROcp7_57*qdd[8]+qdd[7]*S1+qd[1]*qd[7]*C1-qd[8]*(OMcp7_17*S7-ROcp7_47*qd[1]);
    OPcp7_38 = qdd[1]+qdd[8]*S7+qd[7]*qd[8]*C7;

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_17;
    sens->P[2] = POcp7_27;
    sens->P[3] = POcp7_37;
    sens->R[1][1] = ROcp7_18;
    sens->R[1][2] = ROcp7_28;
    sens->R[1][3] = ROcp7_38;
    sens->R[2][1] = ROcp7_47;
    sens->R[2][2] = ROcp7_57;
    sens->R[2][3] = S7;
    sens->R[3][1] = ROcp7_78;
    sens->R[3][2] = ROcp7_88;
    sens->R[3][3] = ROcp7_98;
    sens->V[1] = VIcp7_17;
    sens->V[2] = VIcp7_27;
    sens->OM[1] = OMcp7_18;
    sens->OM[2] = OMcp7_28;
    sens->OM[3] = OMcp7_38;
    sens->A[1] = ACcp7_17;
    sens->A[2] = ACcp7_27;
    sens->OMP[1] = OPcp7_18;
    sens->OMP[2] = OPcp7_28;
    sens->OMP[3] = OPcp7_38;
 
// 
break;
case 9:
 


// = = Block_1_0_0_9_0_3 = = 
 
// Sensor Kinematics 


    ROcp8_47 = -S1*C7;
    ROcp8_57 = C1*C7;
    ROcp8_77 = S1*S7;
    ROcp8_87 = -C1*S7;
    ROcp8_18 = -(ROcp8_77*S8-C1*C8);
    ROcp8_28 = -(ROcp8_87*S8-S1*C8);
    ROcp8_38 = -C7*S8;
    ROcp8_78 = ROcp8_77*C8+C1*S8;
    ROcp8_88 = ROcp8_87*C8+S1*S8;
    ROcp8_98 = C7*C8;
    RLcp8_16 = Dz61*C1;
    RLcp8_26 = Dz61*S1;
    ORcp8_16 = -RLcp8_26*qd[1];
    ORcp8_26 = RLcp8_16*qd[1];
    RLcp8_17 = s->dpt[1][5]*C1;
    RLcp8_27 = s->dpt[1][5]*S1;
    OMcp8_17 = qd[7]*C1;
    OMcp8_27 = qd[7]*S1;
    ORcp8_17 = -RLcp8_27*qd[1];
    ORcp8_27 = RLcp8_17*qd[1];
    OMcp8_18 = OMcp8_17+ROcp8_47*qd[8];
    OMcp8_28 = OMcp8_27+ROcp8_57*qd[8];
    OMcp8_38 = qd[1]+qd[8]*S7;
    OPcp8_18 = ROcp8_47*qdd[8]+qdd[7]*C1-qd[1]*qd[7]*S1+qd[8]*(OMcp8_27*S7-ROcp8_57*qd[1]);
    OPcp8_28 = ROcp8_57*qdd[8]+qdd[7]*S1+qd[1]*qd[7]*C1-qd[8]*(OMcp8_17*S7-ROcp8_47*qd[1]);
    OPcp8_38 = qdd[1]+qdd[8]*S7+qd[7]*qd[8]*C7;
    RLcp8_19 = Dz93*ROcp8_78;
    RLcp8_29 = Dz93*ROcp8_88;
    RLcp8_39 = Dz93*ROcp8_98;
    POcp8_19 = RLcp8_16+RLcp8_17+RLcp8_19;
    POcp8_29 = RLcp8_26+RLcp8_27+RLcp8_29;
    POcp8_39 = RLcp8_39+s->dpt[3][2]+s->dpt[3][5];
    ORcp8_19 = OMcp8_28*RLcp8_39-OMcp8_38*RLcp8_29;
    ORcp8_29 = -(OMcp8_18*RLcp8_39-OMcp8_38*RLcp8_19);
    ORcp8_39 = OMcp8_18*RLcp8_29-OMcp8_28*RLcp8_19;
    VIcp8_19 = ORcp8_16+ORcp8_17+ORcp8_19+ROcp8_78*qd[9]+qd[6]*C1;
    VIcp8_29 = ORcp8_26+ORcp8_27+ORcp8_29+ROcp8_88*qd[9]+qd[6]*S1;
    VIcp8_39 = ORcp8_39+ROcp8_98*qd[9];
    ACcp8_19 = OMcp8_28*ORcp8_39-OMcp8_38*ORcp8_29+OPcp8_28*RLcp8_39-OPcp8_38*RLcp8_29-ORcp8_27*qd[1]-RLcp8_26*qdd[1]-
 RLcp8_27*qdd[1]+ROcp8_78*qdd[9]+qdd[6]*C1-qd[1]*(ORcp8_26+(2.0)*qd[6]*S1)+(2.0)*qd[9]*(OMcp8_28*ROcp8_98-OMcp8_38*ROcp8_88);
    ACcp8_29 = -(OMcp8_18*ORcp8_39-OMcp8_38*ORcp8_19+OPcp8_18*RLcp8_39-OPcp8_38*RLcp8_19-ORcp8_17*qd[1]-RLcp8_16*qdd[1]-
 RLcp8_17*qdd[1]-ROcp8_88*qdd[9]-qdd[6]*S1-qd[1]*(ORcp8_16+(2.0)*qd[6]*C1)+(2.0)*qd[9]*(OMcp8_18*ROcp8_98-OMcp8_38*ROcp8_78));
    ACcp8_39 = OMcp8_18*ORcp8_29-OMcp8_28*ORcp8_19+OPcp8_18*RLcp8_29-OPcp8_28*RLcp8_19+ROcp8_98*qdd[9]+(2.0)*qd[9]*(OMcp8_18*
 ROcp8_88-OMcp8_28*ROcp8_78);

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_19;
    sens->P[2] = POcp8_29;
    sens->P[3] = POcp8_39;
    sens->R[1][1] = ROcp8_18;
    sens->R[1][2] = ROcp8_28;
    sens->R[1][3] = ROcp8_38;
    sens->R[2][1] = ROcp8_47;
    sens->R[2][2] = ROcp8_57;
    sens->R[2][3] = S7;
    sens->R[3][1] = ROcp8_78;
    sens->R[3][2] = ROcp8_88;
    sens->R[3][3] = ROcp8_98;
    sens->V[1] = VIcp8_19;
    sens->V[2] = VIcp8_29;
    sens->V[3] = VIcp8_39;
    sens->OM[1] = OMcp8_18;
    sens->OM[2] = OMcp8_28;
    sens->OM[3] = OMcp8_38;
    sens->A[1] = ACcp8_19;
    sens->A[2] = ACcp8_29;
    sens->A[3] = ACcp8_39;
    sens->OMP[1] = OPcp8_18;
    sens->OMP[2] = OPcp8_28;
    sens->OMP[3] = OPcp8_38;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 

