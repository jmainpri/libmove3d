/***************************************************************************
 *   Copyright (C) 2007 by Xavier Broquere                                 *
 *   xavier.broquere@laas.fr                                               *
 *   Undergraduate from SUPMECA Toulon                                     *
 *   June 2007                                                             *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "softMotion.h"
#include "softMotionStruct.h"

//int xarmJerkProfile(TP_COND IC, TP_COND FC, TP_LIMITS limitsGoto, TP_TIMES *T_Jerk, int *TrajectoryType, );
//int xarmJerkProfileLocal(TP_COND IC, TP_COND FC, TP_LIMITS limitsGoto, TP_TIMES *T_Jerk, int *TrajectoryType, double* lc, int* Zone);
int radomMain();
int calculGraphToPlot(double ICV0, double ICVF);
int calculGraphToPlot2(double ICV0, double ICVF);
int calculGraphToPlotJerk(double ICV0, double ICVF);
int findTransitionTime();

int main2();
int main2()
{
  SM_COND IC; // initial condition
  SM_COND FC; // final condition
  SM_LIMITS limitsGoto;
  SM_TIMES T_Jerk; 
  int TrajectoryType;
  int zone;
  double lc;

  /*Set Limits*/
  limitsGoto.maxVel = 0.1;
  limitsGoto.maxAcc = 0.2;
  limitsGoto.maxJerk = 0.6;



   IC.a = 0.172622647299249;
   IC.v = -0.0256238518498243;
   IC.x =   0;
   FC.a = 0.0464741401789894;
   FC.v = 0.0192032888878636;
   FC.x = -0.00031891099863472;
    
   sm_ComputeSoftMotionLocal(IC, FC, limitsGoto, &T_Jerk, &TrajectoryType, &lc, &zone);
   radomMain(); 
   return 0;
}

//int main()
//{
//  double ICV0, ICVF;
//  ICV0 = 0.05;
//  ICVF = 0.05;
//  calculGraphToPlotJerk(ICV0, ICVF);
//  return 0;
//}

int main()
{
  findTransitionTime();
  return 0;
}

int radomMain()
{
  SM_COND IC;
  SM_COND FC;
  SM_LIMITS limitsGoto;
  int i, j, k;
  FILE * fichier;
  SM_TIMES T_Jerk; 
  int TrajectoryType;
  int zone;
  double lc;

  fichier=  fopen("toto","w+");

  /*Set Limits*/
  limitsGoto.maxVel = 0.1;
  limitsGoto.maxAcc = 0.2;
  limitsGoto.maxJerk = 0.6;

  srand48(1234); // init random generator
  j = 0;
  k = 0;

  for (i=0; i<10000; i++) {
      //  printf("\n********  Iteration: %d ********\n",i);
    if (j == 1000000) {
      j = 0;
      printf("\n********  Iteration: %d ********\n",i);
    }
    j ++;

    IC.a = (drand48() -0.5) * 2. * limitsGoto.maxAcc; 
    IC.v = (drand48() -0.5) * 2. * limitsGoto.maxVel; 
    IC.x = 0;
    FC.a = (drand48() -0.5) * 2. * limitsGoto.maxAcc; 
    FC.v = (drand48() -0.5) * 2. * limitsGoto.maxVel; 
    FC.x = (drand48() -0.5) * 2. * 0.015;



    if (sm_ComputeSoftMotionLocal(IC, FC, limitsGoto, &T_Jerk, &TrajectoryType, &lc, &zone) != 0) {
      k++;
      printf("\n********  Iteration: %d ********\n",i);
      fprintf(fichier,"\n*****Erreur no %d a Iteration: %d ********\n",k,i);
      fprintf(fichier, "\n---------ERROR jerk profile---------\n");
      fprintf(fichier, "limitsGoto.maxVel = %g\n", limitsGoto.maxVel);
      fprintf(fichier, "limitsGoto.maxAcc = %g\n", limitsGoto.maxAcc);
      fprintf(fichier, "limitsGoto.maxJerk = %g\n", limitsGoto.maxJerk);
      fprintf(fichier, "IC.a = %3.20g;\n", IC.a);
      fprintf(fichier, "IC.v = %3.20g;\n", IC.v);
      fprintf(fichier, "IC.x = %3.20g;\n", IC.x);
      fprintf(fichier, "FC.a = %3.20g;\n", FC.a);
      fprintf(fichier, "FC.v = %3.20g;\n", FC.v);
      fprintf(fichier, "FC.x = %3.20g;\n\n", FC.x);

      printf("\n********  Iteration: %d ********\n",i);
      fprintf(stderr,"\n*****Erreur no %d a Iteration: %d ********\n",k,i);
      fprintf(stderr, "\n---------ERROR jerk profile---------\n");
      fprintf(stderr, "limitsGoto.maxVel = %g\n", limitsGoto.maxVel);
      fprintf(stderr, "limitsGoto.maxAcc = %g\n", limitsGoto.maxAcc);
      fprintf(stderr, "limitsGoto.maxJerk = %g\n", limitsGoto.maxJerk);
      fprintf(stderr, "IC.a = %3.20g;\n", IC.a);
      fprintf(stderr, "IC.v = %3.20g;\n", IC.v);
      fprintf(stderr, "IC.x = %3.20g;\n", IC.x);
      fprintf(stderr, "FC.a = %3.20g;\n", FC.a);
      fprintf(stderr, "FC.v = %3.20g;\n", FC.v);
      fprintf(stderr, "FC.x = %3.20g;\n\n", FC.x);


      fflush(fichier);
     return 0;
    }
  }
  fclose(fichier);
  return 0;
}

int calculGraphToPlot(double ICV0, double ICVF) {
  SM_COND IC; // initial condition
  SM_COND FC; // final condition
  SM_LIMITS limitsGoto, newLimits, limitsGotoVel;
  SM_TIMES T_Jerk; 
  double Jerk;
  int TrajectoryType;
  int zone;
  double lc;
  double optimalTime;
  double imposedTime;
  double stopTime;
  int typeOfFunction;
  FILE * fichier;
  SM_TIMES_GLOBAL T;
  int dir_a, dir_b;
  double GoalDist, V0, Vf;

  fichier =  fopen("data.dat","w+");

 /*Set Limits*/
  limitsGoto.maxVel = 0.1;
  limitsGoto.maxAcc = 0.2;
  limitsGoto.maxJerk = 0.6;

  limitsGotoVel.maxVel = 0.1;
  limitsGotoVel.maxAcc = 0.2;
  limitsGotoVel.maxJerk = 0.6;
  IC.a = 0; 
  IC.v = ICV0; 
  IC.x = 0;
  FC.a = 0; 
  FC.v = ICVF; 
 

  /* set initial and final conditions */ 
  V0 = IC.v;
  Vf = FC.v;

  sm_CalculTimeProfileWithVcFixed( V0, Vf, 0.0, limitsGoto, &T, &dir_a, &dir_b);
  stopTime = T.T1 + T.T2 + T.T3 + T.T5 + T.T6 + T.T7;

  sm_CalculOfDistance( V0, limitsGoto.maxJerk, &T, &dir_a, &dir_b, &GoalDist);

  FC.x = GoalDist;

  if (sm_ComputeSoftMotionLocal(IC, FC, limitsGoto, &T_Jerk, &TrajectoryType, &lc, &zone) != 0) {
    printf("ERROR fonction de base\n");
    return 1;
  }

  optimalTime = T_Jerk.Tjpa + T_Jerk.Taca + T_Jerk.Tjna + T_Jerk.Tvc + T_Jerk.Tjnb + T_Jerk.Tacb + T_Jerk.Tjpb;
  
  imposedTime = optimalTime ;


  for(imposedTime = optimalTime; imposedTime < stopTime ;  imposedTime = imposedTime + 0.01) {
     /* set initial and final conditions */ 
  
    typeOfFunction = 0;
    if(sm_AdjustTimeSlowingVelocity(V0, Vf, GoalDist, limitsGotoVel, imposedTime, &T_Jerk, &dir_a, &dir_b)!=0) {
      typeOfFunction = 0;
    } else {
      typeOfFunction = 1;
    }

    if(sm_AdjustTimeSlowingJerk(IC, FC, imposedTime, limitsGoto, &T_Jerk, &Jerk, &newLimits, &dir_a)!=0) {
   
    } else {
      typeOfFunction = typeOfFunction + 2;
    }

    fprintf(fichier,"%f %f %d\n",GoalDist, imposedTime,typeOfFunction);
    fflush(fichier);
  }
  fclose(fichier);
  return 0;
}

int calculGraphToPlot2(double ICV0, double ICVF) {
  SM_COND IC; // initial condition
  SM_COND FC; // final condition
  SM_LIMITS limitsGoto, newLimits, limitsGotoTrans;
  SM_TIMES T_Jerk; 
  double Jerk;
  int TrajectoryType;
  int zone;
  double lc;
  double optimalTime;
  double imposedTime;
  double stopTime;
  int typeOfFunction;
  FILE * fichier;
  SM_TIMES_GLOBAL T;
  int dir_a, dir_b;
  double GoalDist, V0, Vf;

  int nbfail=0;
  int nbtest=0;

  fichier =  fopen("data2.dat","w+");

 /*Set Limits*/
  limitsGoto.maxVel = 0.1;
  limitsGoto.maxAcc = 0.2;
  limitsGoto.maxJerk = 0.6;

  limitsGotoTrans.maxVel = 0.1;
  limitsGotoTrans.maxAcc = 0.2;
  limitsGotoTrans.maxJerk = 1.2;
  IC.a = 0; 
  IC.v = ICV0; 
  IC.x = 0;
  FC.a = 0; 
  FC.v = ICVF; 

  fprintf(fichier,"V0        Vf    Goaldist opt time  stopTime imptime\n");
 
  for(V0=-0.1;V0<=0.1;V0=V0+0.005) {

     for(Vf=-0.1;Vf<=0.1;Vf=Vf+0.005) { 

        /* set initial and final conditions */ 
      IC.v = V0;
      FC.v = Vf;

      sm_CalculTimeProfileWithVcFixed( V0, Vf, 0.0, limitsGoto, &T, &dir_a, &dir_b);
      stopTime = T.T1 + T.T2 + T.T3 + T.T5 + T.T6 + T.T7;
      sm_CalculOfDistance( V0, limitsGoto.maxJerk, &T, &dir_a, &dir_b, &GoalDist);
      FC.x = GoalDist;

      if (sm_ComputeSoftMotionLocal(IC, FC, limitsGoto, &T_Jerk, &TrajectoryType, &lc, &zone) != 0) {
	fprintf(fichier,"ERROR fonction de base\n");
	return 1;
      }
      
      optimalTime = T_Jerk.Tjpa + T_Jerk.Taca + T_Jerk.Tjna + T_Jerk.Tvc + T_Jerk.Tjnb + T_Jerk.Tacb + T_Jerk.Tjpb;
      
      imposedTime = optimalTime ;

      if (optimalTime > stopTime) {
	  fprintf(fichier,"......optimalTime > stopTime\n");
      } else {
	for(imposedTime = optimalTime; imposedTime < stopTime ;  imposedTime = imposedTime + 0.01) {
	  /* set initial and final conditions */ 
	  
	  typeOfFunction = 0;
	  if(sm_AdjustTimeSlowingVelocity(V0, Vf, GoalDist, limitsGotoTrans, imposedTime, &T_Jerk, &dir_a, &dir_b)!=0) {
	    typeOfFunction = 0;
	  } else {
	    typeOfFunction = 1;
	  }
	  
	  if(sm_AdjustTimeSlowingJerk(IC, FC, imposedTime, limitsGoto, &T_Jerk, &Jerk, &newLimits, &dir_a)!=0) {
	    
	  } else {
	    typeOfFunction = typeOfFunction + 2;
	  }
	  
	  if (typeOfFunction ==0) {
	    nbfail ++;
	    fprintf(fichier,"%f %f %f %f %f %f %d\n",V0, Vf, GoalDist,optimalTime,stopTime,  imposedTime,typeOfFunction);
	  }
	  nbtest ++;
	  //	fprintf(fichier,"%f %f %f %f %f %f %d\n",V0, Vf, GoalDist,optimalTime,stopTime,  imposedTime,typeOfFunction);
	  //	fprintf(fichier,"%f %f %d\n",V0, Vf, typeOfFunction);
	
	  //	fprintf(fichier,"%d\n",typeOfFunction);
	  fflush(fichier);
	}
	// fprintf(fichier,"\nNew Cond V0 %f Vf %f\n",V0,Vf);
	fflush(fichier);
      }
    }
  }
  fprintf(fichier,"nbfail %d\n",nbfail);
  fprintf(fichier,"nbtest %d\n",nbtest);
  fprintf(fichier,"error_per_cent %f\n",(nbfail*100)/((double)(nbtest)));
  fflush(fichier);
  fclose(fichier);
  return 0;
}



int calculGraphToPlotJerk(double ICV0, double ICVF) {
  SM_COND IC, FC; // initial final conditions
  SM_LIMITS limitsGoto, auxLimits;
  SM_TIMES T_Jerk; 
  SM_TIMES_GLOBAL T;

  int TrajectoryType, zone, dir_a, dir_b;
  double lc, J, A , stopTime, TotalTime, GoalDist, V0, Vf;

  FILE * fichier;
  FILE * fichier2;

  /* Set file'names */
  fichier =  fopen("data.dat","w+");
  fichier2 =  fopen("dataError.dat","w+");

  /*Set Limits*/
  limitsGoto.maxVel = 0.1;
  limitsGoto.maxAcc = 0.2;
  limitsGoto.maxJerk = 0.6;

  /* Set IC and FC*/
  IC.a = 0; 
  IC.v = ICV0; 
  IC.x = 0;
  FC.a = 0; 
  FC.v = ICVF; 
  V0 = IC.v;
  Vf = FC.v; 

  
  sm_CalculTimeProfileWithVcFixed( V0, Vf, 0.0, limitsGoto, &T, &dir_a, &dir_b);
  stopTime = T.T1 + T.T2 + T.T3 + T.T5 + T.T6 + T.T7;
  T.T4 = 0;
  sm_CalculOfDistance( V0, limitsGoto.maxJerk, &T, &dir_a, &dir_b, &GoalDist);
  fprintf(fichier,"stopTime %f\n",stopTime);  
  fprintf(fichier,"GoalDist %f\n",GoalDist);
 
  FC.x = GoalDist;
  fprintf(fichier,"   V0       Vf        J        A    GoalDist   dc    TotalTime TrajectoryType \n");
  // for (J = limitsGoto.maxJerk; J>0.001; J = J-0.001) {
    for (A = 0.2; A>0.000001; A = A-0.000001) {
      J=0.6;
      auxLimits.maxAcc = A;
      auxLimits.maxJerk = J;
      auxLimits.maxVel = limitsGoto.maxVel;
      FC.x = GoalDist;
 
      if (sm_ComputeSoftMotionLocal(IC, FC, auxLimits, &T_Jerk, &TrajectoryType, &lc, &zone) != 0) {
	fprintf(fichier2,"%f %f %f %f\n",V0, Vf, J, A);
      }
      
      TotalTime = (T_Jerk.Tjpa + T_Jerk.Taca + T_Jerk.Tjna + T_Jerk.Tvc + T_Jerk.Tjnb + T_Jerk.Tacb + T_Jerk.Tjpb);
      fprintf(fichier,"%f %f %f %f %f %f %f %d \n",V0, Vf, J, A, GoalDist, lc, TotalTime, TrajectoryType);
    }
    fprintf(fichier,"\n");
    // }
    
  fflush(fichier);
  fflush(fichier2); 
  
  fclose(fichier2);
  fclose(fichier);
  return 0;
}


int findTransitionTime() {

  FILE* fileptr = NULL;
  FILE* fileptr2 = NULL;
  int k = 0, l=0;
  int i=0, j=0;
  int axis =0;
  int numberOfTransition=0;
  SM_COND ICn[240][7], FCn[240][7]; // initial final conditions
  float a, b, c, d, e, f;
  SM_COND IC; // initial condition
  SM_COND FC; // final condition
  SM_LIMITS limitsGoto;
  SM_TIMES T_Jerk; 
  //double Jerk;
  int TrajectoryType;
  int zone;
  double lc;
  double optimalTime;
  double imposedTime;
  double stopTime[7];
  //int typeOfFunction;
  //FILE * fichier;
  SM_TIMES_GLOBAL T;
  int dir_a, dir_b;
  double V0, Vf;

  int interval[7][300];
  int intervalResult[300];

  int optimalTimeM, imposedTimeM, stopTimeM;
 /* Init */
 for (i=0; i<7; i++) {
   for (k=0; k<300;k++) {
      interval[i][k] = 0;
      intervalResult[k] = 0;
   }
 }

 k=0;
 
 if ((fileptr = fopen("RefTransition1.dat","r"))==NULL) {
   printf("Open File RefTransition1.dat ");
 }
  if ((fileptr2 = fopen("TimeVector.dat","wb"))==NULL) {
   printf("Open File TimeVector.dat ");
 }
 //  fichier =  fopen("data.dat","w+");

 while(!feof(fileptr)) { 
   for(i=0;i<7;i++) { 
     fscanf(fileptr, "%d %f %f %f %f %f %f\n",&axis, &a, &b, &c, &d, &e, &f);
     ICn[k][i].a = a;
     ICn[k][i].v = b;
     ICn[k][i].x = c;
     FCn[k][i].a = d;
     FCn[k][i].v = e;
     FCn[k][i].x = f;
   }
   k ++;
 }
 numberOfTransition = k;
 
 printf("Number of transition read: %d\n", numberOfTransition);
 printf("Transition %d\n",k);
 k=0;

 //  for(l=0;l<numberOfTransition;l++) {
   
 for(l=0;l<1;l++) {
 

   for(i=0;i<7;i++) { 
     printf("%f %f %f %f %f %f\n", ICn[l][i].a, ICn[l][i].v,  ICn[l][i].x, FCn[l][i].a, FCn[l][i].v, FCn[l][i].x);
     if (i < 3) {
       /*Set Limits*/
       limitsGoto.maxVel = 0.15;
       limitsGoto.maxAcc = 0.3;
       limitsGoto.maxJerk = 0.9;
     } else {
       limitsGoto.maxVel = 0.1;
       limitsGoto.maxAcc = 0.2;
       limitsGoto.maxJerk = 0.6;
     }
     
     IC.a = ICn[l][i].a; 
     IC.v = ICn[l][i].v; 
     IC.x = 0;
     FC.a = FCn[l][i].a; 
     FC.v = FCn[l][i].v; 
     FC.x = FCn[l][i].x;
     

     /* set initial and final conditions */ 
     V0 = IC.v;
     Vf = FC.v;

     sm_CalculTimeProfileWithVcFixed( V0, Vf, 0.0, limitsGoto, &T, &dir_a, &dir_b);
     stopTime[i] = T.T1 + T.T2 + T.T3 + T.T5 + T.T6 + T.T7;
     stopTimeM = (int)(stopTime[i]*100);
     // printf("stopTime[%d]=%f\n",i,stopTime[i]);
     for(j=stopTimeM;j<300;j++) {
       interval[i][j] = 1;
     }

     if (sm_ComputeSoftMotionLocal(IC, FC, limitsGoto, &T_Jerk, &TrajectoryType, &lc, &zone) != 0) {
       printf("ERROR fonction de base\n");
       return 1;
     }
     optimalTime = T_Jerk.Tjpa + T_Jerk.Taca + T_Jerk.Tjna + T_Jerk.Tvc + T_Jerk.Tjnb + T_Jerk.Tacb + T_Jerk.Tjpb;
     optimalTimeM = (int)(optimalTime*100);     
     imposedTimeM = optimalTimeM ;
     for(imposedTimeM = optimalTimeM; imposedTimeM < stopTimeM ;  imposedTimeM = imposedTimeM + 1) {
       /* set initial and final conditions */ 
       imposedTime = imposedTimeM /100.0;
       if(sm_AdjustTimeSlowingVelocity(V0, Vf, FC.x, limitsGoto, imposedTime, &T_Jerk, &dir_a, &dir_b)!=0) {
	 interval[i][imposedTimeM] = 0;
       } else {
	  interval[i][imposedTimeM] = 1;
	  if (i==0) {
	    // printf("imposedTime %f\n",imposedTime);
	  }
       }
     }
   }
   /* Find time to impose */ 
   k=0;
   for(k=0;k<300;k++) {
     if (interval[0][k]==1 && interval[1][k]==1 && interval[2][k]==1 && interval[3][k]==1 && interval[4][k]==1 && interval[5][k]==1 && interval[6][k]==1){
       break;
     }
   }   
   imposedTimeM = k;
   imposedTime = k / 100.0;
   printf("imposedTime %f \n",imposedTime);
 }

 k=0;
 for(k=0;k<300;k++) {
   fprintf(fileptr2,"%d %d %d %d\n",k,interval[0][k],interval[1][k],interval[2][k]);

 }
 /* Verify if imposedTime works */
   for(i=0;i<7;i++) { 
     if(imposedTime < stopTime[i]) {
       // printf("%f %f %f %f %f %f\n", ICn[l][i].a, ICn[l][i].v,  ICn[l][i].x, FCn[l][i].a, FCn[l][i].v, FCn[l][i].x);
       if (i < 3) {
	 /*Set Limits*/
	 limitsGoto.maxVel = 0.15;
	 limitsGoto.maxAcc = 0.3;
	 limitsGoto.maxJerk = 0.9;
       } else {
	 limitsGoto.maxVel = 0.1;
	 limitsGoto.maxAcc = 0.2;
	 limitsGoto.maxJerk = 0.6;
       }
       
       IC.a = ICn[l][i].a; 
       IC.v = ICn[l][i].v; 
       IC.x = 0;
       FC.a = FCn[l][i].a; 
       FC.v = FCn[l][i].v; 
       FC.x = FCn[l][i].x;
       
       
       /* set initial and final conditions */ 
       V0 = IC.v;
       Vf = FC.v;
       
       if(sm_AdjustTimeSlowingVelocity(V0, Vf, FC.x, limitsGoto, imposedTime, &T_Jerk, &dir_a, &dir_b)!=0) {
	 printf("ERROR Axis %d with imposedTime %f\n",i,imposedTime);
       } else {
	 printf("Axis %d works with imposedTime %f\n",i,imposedTime);
       }
     } else {
       printf("Axis %d works: imposedTime %f > %f stopTime\n",i,imposedTime,stopTime[i]);
       
     }
   }
   
   fclose(fileptr);
   fclose(fileptr2);
   return 1;
}


