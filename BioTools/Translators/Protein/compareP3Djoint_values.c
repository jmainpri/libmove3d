/**********************************************************************/
// INCLUDES

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//#include "mathfcts.h"

/**********************************************************************/
// EXTERN FUNCTIONS

/**********************************************************************/
// GLOBAL VARIABLES

// call arguments
//  - files
#define ALL_DIHEDS 1
#define ONLY_BKB_DIHEDS 2

static char str_which_diheds[10];
static double print_from_ang_diff;
static char p3dfile1name[255],p3dfile2name[255],outputfilename[255];
static FILE *p3dfile1,*p3dfile2,*outputfile;
//  - options
//     NONE

/**********************************************************************/
// GENERAL FUNCTIONS

static int read_call_arguments(int argc, char **argv)
{
  if(argc < 5) {
    printf("Not enought arguments\n");
    return -1;
  }
   
  if((!strcpy(str_which_diheds,argv[1])) ||
     (!strcpy(p3dfile1name,argv[2])) ||
     (!strcpy(p3dfile2name,argv[3])) ||
     (!strcpy(outputfilename,argv[4])) ||
     (!sscanf(argv[5],"%lf",&print_from_ang_diff))) {
    return -1;
  }
  return 1;
}

/**********************************************************************/
/**********************************************************************/

static int read_desc_name(FILE *fd, char *name) {
  if(fscanf(fd,"%s", name) != 1) return(0);
  return(1);
}

static int read_desc_double(FILE *fd, int n, double *f) {
  int i;
  
  for(i=0;i<n;i++) {
    if(fscanf(fd,"%lf", &f[i]) != 1)return(0);
  }
  return(1);
}


static char *givemeword(char string[], char c, int *index){
  
  static char cadena[256];
  int i=0;
  //printf("cad=%s , c=", string);putchar(c);printf("\n");
  while ((string[*index]!= c) && (string[*index]!= '\0')){
    cadena[i++]=string[(*index)++];
  }
  if (i > (256 - 1)){
    printf("ERROR en givemeword\n");//QUITAR
    printf("cadena=%s , caracter=", string);putchar(c);printf("\n");
  }
  cadena[i]='\0';
  (*index)++;
  return cadena;
}
 


/**********************************************************************/
/**********************************************************************/
// MAIN
/**********************************************************************/
/**********************************************************************/

// operations with only CA
int main(int argc, char **argv)
{
  int which_joints;
  char fct1[256],fct2[256],namejnt[256],gc;
  char *jnt_type_name1,*jnt_type_name2;
  int isFF1,isFF2;
  int readval1=0,readval2=0;
  int cancompare1=0,cancompare2=0;
  int cj1=0,cj2=0;
  double dtab1[6],dtab2[6],diff[6];
  int indexletter=0;
  int i;

  if(read_call_arguments(argc, argv) < 0) {
    printf("Usage: compareP3Djoint_values <-all OR -bkb> <p3dfile1> <p3dfile2> <outputfile> <print_from_ang_diff>\n");
    return -1;
  }

  // open files
  p3dfile1 = fopen(p3dfile1name, "r");
  if (p3dfile1 == NULL) {
    printf("p3d file 1 cannot be open\n");
    return -1;
  }
  p3dfile2 = fopen(p3dfile2name, "r");
  if (p3dfile2 == NULL) {
    printf("p3d file 2 cannot be open\n");
    fclose(p3dfile1);
    return -1;
  }
  outputfile = fopen(outputfilename, "w");
  if (outputfile == NULL) {
    printf("output file cannot be open\n");
    fclose(p3dfile1);
    fclose(p3dfile2);
    return -1;
  }

  // which joints are selected for operations
  if(strcmp(str_which_diheds,"-all") == 0) {
    which_joints = ALL_DIHEDS;
    printf("\nComputing values for all joints\n");
  }
  else if (strcmp(str_which_diheds,"-bkb") == 0) {
    which_joints = ONLY_BKB_DIHEDS;
    printf("\nComputing values for backbone joints\n");
  }
  else {
    printf("ERROR : wrong joint selection : argument must be -all, or -bkb\n");
    fclose(p3dfile1);
    fclose(p3dfile2);
    fclose(outputfile);
    return -1;
  }
    
  // read the 2 input files in parallel, compare joint by joint, and write output file 
  while(fscanf(p3dfile1,"%s",fct1) != EOF) {
    if(fscanf(p3dfile2,"%s",fct2) != EOF) {
      if(fct1[0]=='#') {
	/* comment in file: ignore the line ! */
	do{
	  gc=getc(p3dfile1);
	}
	while(gc!='\n');
	continue;
      }
      
      if(fct2[0]=='#') {
	/* comment in file: ignore the line ! */
	do{
	  gc=getc(p3dfile2);
	}
	while(gc!='\n');
	continue;
      }
      
      if(strcmp(fct1,"p3d_set_name")== 0) {
	if(!read_desc_name(p3dfile1,namejnt)) {
	  printf("ERROR : while reading input file 1\n");    
	  fclose(p3dfile1);
	  fclose(p3dfile2);
	  fclose(outputfile);
	  return -1;
	}
	indexletter = 0;
	jnt_type_name1 = givemeword(namejnt, '.', &indexletter);      
	if(strcmp(jnt_type_name1,"") == 0) {
	  jnt_type_name1 = givemeword(namejnt, '.', &indexletter);
	  isFF1 = 1;
	}
	else{
	  isFF1 = 0;
	}
	if(which_joints == ALL_DIHEDS) {
	  readval1 = 1;
	}
	else {
	  if((strcmp(jnt_type_name1,"omega") == 0) ||
	     (strcmp(jnt_type_name1,"phi") == 0) ||
	     (strcmp(jnt_type_name1,"psi") == 0) ||
	     (strcmp(jnt_type_name1,"chain_base") == 0)) {
	    readval1 = 1;
	  }
	  else {
	    readval1 = 0;
	  }
	}
      }
      
      if(strcmp(fct2,"p3d_set_name")== 0) {
	if(!read_desc_name(p3dfile2,namejnt)) {
	  printf("ERROR : while reading input file 2\n");    
	  fclose(p3dfile1);
	  fclose(p3dfile2);
	  fclose(outputfile);
	  return -1;
	}
	indexletter = 0;
	jnt_type_name2 = givemeword(namejnt, '.', &indexletter);      
	if(strcmp(jnt_type_name2,"") == 0) {
	  jnt_type_name2 = givemeword(namejnt, '.', &indexletter);
	  isFF2 = 1;
	}
	else{
	  isFF2 = 0;
	}
	if(which_joints == ALL_DIHEDS) {
	  readval2 = 1;
	}
	else {
	  if((strcmp(jnt_type_name2,"omega") == 0) ||
	     (strcmp(jnt_type_name2,"phi") == 0) ||
	     (strcmp(jnt_type_name2,"psi") == 0) ||
	     (strcmp(jnt_type_name2,"chain_base") == 0)) {
	    readval2 = 1;
	  }
	  else {
	    readval2 = 0;
	  }
	}
      }
      
      if((strcmp(fct1,"p3d_set_pos_axe")== 0) && (readval1 == 1) && (isFF1 == 1)) {
	if(!read_desc_double(p3dfile1,6,dtab1)) {
	  printf("ERROR : while reading input file 1\n");    
	  fclose(p3dfile1);
	  fclose(p3dfile2);
	  fclose(outputfile);
	  return -1;
	}
	cj1++;
	cancompare1 = 1;
      }
      
      if((strcmp(fct2,"p3d_set_pos_axe")== 0) && (readval2 == 1) && (isFF2 == 1)) {
	if(!read_desc_double(p3dfile2,6,dtab2)) {
	  printf("ERROR : while reading input file 2\n");    
	  fclose(p3dfile1);
	  fclose(p3dfile2);
	  fclose(outputfile);
	  return -1;
	}
	cj2++;
	cancompare2 = 1;
      }
      
      if((strcmp(fct1,"p3d_set_dof")== 0) && (readval1 == 1) && (isFF1 == 0)) {
	if(!read_desc_double(p3dfile1,1,dtab1)) {
	  printf("ERROR : while reading input file 1\n");    
	  fclose(p3dfile1);
	  fclose(p3dfile2);
	  fclose(outputfile);
	  return -1;
	}
	cj1++;
	cancompare1 = 1;
      }
      
      if((strcmp(fct2,"p3d_set_dof")== 0) && (readval2 == 1) && (isFF2 == 0)) {
	if(!read_desc_double(p3dfile2,1,dtab2)) {
	  printf("ERROR : while reading input file 2\n");    
	  fclose(p3dfile1);
	  fclose(p3dfile2);
	  fclose(outputfile);
	  return -1;
	}
	cj2++;
	cancompare2 = 1;
      }
      
      if((cancompare1 == 1) && (cancompare2 == 1) && (cj1 == cj2)) {
	if(strcmp(jnt_type_name1,jnt_type_name2) != 0) {
	  printf("ERROR : joint miscorrespondence between input files\n");    
	  fclose(p3dfile1);
	  fclose(p3dfile2);
	  fclose(outputfile);
	  return -1;
	}
	else {
	  if(isFF1) {
	    for(i=0; i<6; i++) {
	      diff[i] = fabs(dtab1[i] - dtab2[i]);
	    }	
	    fprintf(outputfile,"%s : %f %f %f %f %f %f\n",namejnt,
		    diff[0],diff[1],diff[2],diff[3]*(180.0/3.1416),diff[4]*(180.0/3.1416),diff[5]*(180.0/3.1416));  
	  } 
	  else {
	    diff[0] = fabs(dtab1[0] - dtab2[0]);
	    if(diff[0] > 180.0) 
	      diff[0] = 360.0 - diff[0];
	    if(diff[0] >= print_from_ang_diff) {
	      fprintf(outputfile,"%s : %f\n",namejnt,diff[0]);  
	    }
	  }
	}
	cancompare1 = 0;
	cancompare2 = 0;
      }    
      
    }    
    
  }
  // close files
  fclose(p3dfile1);
  fclose(p3dfile2);
  fclose(outputfile);

  return 1;
}

