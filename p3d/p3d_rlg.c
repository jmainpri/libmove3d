/* (jcortes) */
/* WARNING :
   This file is already in testing phase.
   Some functions work well only in particular cases.
*/
   
#include "P3d-pkg.h"
#include "Util-pkg.h"

#ifdef P3D_PLANNER
#include "Planner-pkg.h"
#endif

#ifdef P3D_COLLISION_CHECKING
#include "Collision-pkg.h"
#endif

#include "Bio-pkg.h"


#define IN3DCC_EPS 0.001

#define PRINTPROC_RLG 0

// NOTA : This parameter is used in several places.
//        Different parameters could be tried. 
#define TRIES_BEFORE_RESHOOT 5


static int RLG_OK = 0;
static int RLG = 1;



static int p3d_fct_set_random_loop_generator(char *rlg_type, int ctnum, int faj, int laj, double rmax, double rmin);
static int p3d_fct_set_rlg_normal_chain(int ctnum, int faj, int laj, double rmax, double rmin);
static int p3d_fct_set_rlg_holonom_base(int ctnum, int ibj);

static int  p3d_compute_passive_approxWS(p3d_cntrt *ct, double *rmax, double *rmin);

static int p3d_generate_ct_conf(p3d_rob *robotPt, p3d_cntrt *ct, configPt q);
static int p3d_generate_ct_conf_parallel(p3d_rob *robotPt, p3d_cntrt *ct, configPt q);
static int p3d_generate_ct_conf_basesandchains(p3d_rob *robotPt, p3d_cntrt *ct, configPt q);

static int p3d_generate_cc_active_part_normal(p3d_cntrt *ct, configPt q);

static int p3d_generate_holonom_base_conf(p3d_cntrt *ct, configPt q);
static int p3d_generate_holonom_plane_basejnt_conf(p3d_cntrt *ct, configPt q);
static int p3d_generate_holonom_freeflyer_basejnt_conf(p3d_cntrt *ct, configPt q);

static double p3d_ref_in_jnt_frame(p3d_rlg_chain *rlgchPt, int irlgd, p3d_vector3 vref);

static void random_point_on_disc(double rmax, double rmin, double *x, double *y);
static void random_point_on_halfdisc(double rmax, double rmin, p3d_matrix4 Tgrip, double *x, double *y);

/* static double p3d_random_in_several_intervals(int iv1, int iv2, p3d_matrix2 interv1, p3d_matrix2 interv2, double refval); */

/* static int p3d_inter_ang_regions(double ar1m, double ar1M, double ar2m, double ar2M, p3d_matrix2 sol); */

static int p3d_jnt_frames_at_same_point(p3d_jnt *Ja, p3d_jnt *Jb);

static int p3d_transf_jnt_frame(p3d_matrix4 Tref, p3d_vector3 v, p3d_matrix4 Tt);

static void p3d_put_dof_into_range(p3d_jnt *jntPt, int idof, double *thedof);

/* static double p3d_random_in_two_intervals(double minlim1, double maxlim1, double minlim2, double maxlim2); */
/* static double p3d_random_in_three_intervals(double minlim1, double maxlim1, double minlim2, double maxlim2, double minlim3, double maxlim3); */




/*****************************************************************************/
/*****************************************************************************/
/*                   IMPLEMENTATION RESTRICTIONS                             */
/*                                                                           */
/*  - each frame is on one of the axis of the preceding one                  */
/*    ( -> partially extended limitation )                                   */
/*  - translations :                                                         */
/*    - max. 1 translation between two rotations                             */
/*    - translation along the line between the two rotations                 */ 
/*                                                                           */
/*****************************************************************************/
/*****************************************************************************/



/*****************************************************************************/

void p3d_generate_rlg(p3d_cntrt *ctPt)
{
  ctPt->rlgPt = MY_ALLOC(p3d_rlg,1);
  ctPt->rlgPt->rlgchPt = NULL;
  ctPt->rlgPt->rlgbsPt = NULL;
}

/*****************************************************************************/


static void compute_prismatic_dof_minmax_length(p3d_jnt *J, double *datmin, double *datmax, p3d_vector3 X1, p3d_vector3 X1b)
{  
  p3d_vector3 X2;
  double val,vmin,vmax;

  p3d_jnt_get_cur_vect_point(J->prev_jnt, X2);
      
  val = p3d_jnt_get_dof(J,0);
  p3d_jnt_set_dof(J,0, val);
  p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);

  p3d_jnt_set_dof(J,0,vmax);
  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
  p3d_jnt_get_cur_vect_point(J, X1);
  *datmax = sqrt(SQR(X2[0]-X1[0])+SQR(X2[1]-X1[1])+SQR(X2[2]-X1[2]));

  p3d_jnt_set_dof(J,0,vmin);
  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
  p3d_jnt_get_cur_vect_point(J, X1b);
  *datmin = sqrt(SQR(X2[0]-X1b[0])+SQR(X2[1]-X1b[1])+SQR(X2[2]-X1b[2]));

  p3d_jnt_set_dof(J,0,val);
  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
}


/* this function computes the radii of the sphere used to approximate
   the WS of the passive part of the loop
   these radii correspond to an approximation of the maximum and minimum 
   lengths between the base-joint and the end-joint

   NOTE : the passive part must be a kinematic chain (consecutive joints) 
   defined in the constraint 

   BRUTE IMPLEMENTATION : not try to accurate result by considering perpendicular links ... 
*/
static int  p3d_compute_passive_approxWS(p3d_cntrt *ct, double *rmax, double *rmin)
{
  int i;
  p3d_jnt *Je,*J,*Jb;
  p3d_vector3 X1,X1b,X2;  
  double datmax, datmin;
  double acM_len = 0.0, acm_len = 0.0; 

  /* WARNING : suppose that the base of the chain is the first passive joint 
     and the end the last one */
  Je = ct->pasjnts[ct->npasjnts - 1];
  Jb = ct->pasjnts[0];
  J = Je;
  
  for(i = 0; i < ct->npasjnts; i ++) {
    /* case of translational joint */
    /* NOTES :
       - the values of translations are ALWAYS considered POSITIVE 
       - the jnt must be defined at the end-frame of the link
       - if the first joint of the chain is translational the goodness 
         of the result is not guaranteed
    */  

    if(J->type == P3D_TRANSLATE) {

      compute_prismatic_dof_minmax_length(J,&datmin,&datmax,X1,X1b);

      acM_len += datmax;
      acm_len += datmin;
    }

    /* case of rotational joint */
    else if(J->type == P3D_ROTATE) {
      if(J != Jb) {  /* nothing to do with first joint if rotational */
	if(J->prev_jnt->type != P3D_TRANSLATE) { /* computed in that case */

	  p3d_jnt_get_cur_vect_point(J, X1);
	  p3d_jnt_get_cur_vect_point(J->prev_jnt, X2);

	  datmax = sqrt(SQR(X2[0]-X1[0])+SQR(X2[1]-X1[1])+SQR(X2[2]-X1[2]));
	  
	  acM_len += datmax;
	  acm_len += datmax;
	}
      }
    }
    else {
      PrintInfo(("ERROR: p3d_compute_passive_approxWS : only P3D_TRANSLATE and P3D_ROTATE are already trleated\n"));
    }
    J = J->prev_jnt;
  }
  *rmax = acM_len;
  *rmin = 0.0;
  
  /* case of rmin>0 not already treated !!!!!!!!! */

  return(TRUE);
}


/*****************************************************************************/

int p3d_set_random_loop_generator(char *rlg_type, int ctnum, int faj, int laj, double rmax, double rmin)
{ p3d_rob *r;
  p3d_cntrt *ct;
  int setok;
  
  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  ct = r->cntrt_manager->cntrts[ctnum];

  setok = p3d_fct_set_random_loop_generator(rlg_type, ctnum, faj, laj, rmax, rmin);
  if(ct->rlgPt != NULL) {
    if(!RLG_OK) {
      p3d_active_RLG_flags();
    }
  }
  return (setok);
}

int p3d_random_loop_generator_ok(void)
{
  return RLG_OK;
}

void p3d_active_RLG_flags(void)
{
  RLG_OK = 1;
  RLG = 1;
}

int p3d_get_RLG(void)
{
  return(RLG && RLG_OK);
}

void p3d_set_RLG(int val)
{
  RLG = val;
}


/*****************************************************************************/

/* function generatind data structures for RLG */

static int p3d_fct_set_random_loop_generator(char *rlg_type, int ctnum, int faj, int laj, double rmax, double rmin)
{
  if(strcmp(rlg_type,"normal_chain")==0) 
    return(p3d_fct_set_rlg_normal_chain(ctnum,faj,laj,rmax,rmin));
  if(strcmp(rlg_type,"holonom_base")==0) 
    return(p3d_fct_set_rlg_holonom_base(ctnum,faj));

  return(FALSE);
}

/*****************************************************************************/

/* function generatind data structures for RLG normal-chain */

static int p3d_fct_set_rlg_normal_chain(int ctnum, int faj, int laj, double rmax, double rmin)
{ p3d_rob *r;
  p3d_cntrt *ct;
  pp3d_rlg_chain_data rlg_data; 
  pp3d_rlg_chain rlgchPt;
  p3d_vector3 v3a,v3b;
  p3d_vector3 X1,X1b,Xij;
  p3d_vector3 vij1,vaxe1,vaxe1_lr,vaxeij;
  p3d_vector3 v3aux;
  double cosang;
  double val,datmax,datmin;
  double refval;
  int i, j;
  int nj,ndof;
  p3d_matrix4 T1,T2,pos,prev_pos;
  double acM_len = 0.0, acm_len = 0.0; 
  double rm_len;
  double l_last;
  p3d_vector3 dir_last;
  int Jmax_lmin = -1;
  p3d_jnt *Jlaj, *Jfaj, *J;

  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  ct = r->cntrt_manager->cntrts[ctnum];
  /* consecutively defined active joints */
  Jfaj = r->joints[faj];
  Jlaj = r->joints[laj];
  J = Jlaj;
  nj = ndof = 0;
  do {
    nj++;
    ndof += J->dof_equiv_nbr;
    J = J->prev_jnt;
  } while (J != Jfaj->prev_jnt);

  /*memory allocation */
  if(ct->rlgPt == NULL)
    p3d_generate_rlg(ct);
  else {
    p3d_destroy_rlg(ct->rlgPt);
    PrintError(("RLG setting : RLG-chain must be defined before RLG-base !\n"));
    return (FALSE);
  }

  rlgchPt = MY_ALLOC(p3d_rlg_chain,1);
  rlgchPt->nlinksrlgch = ndof;
  rlgchPt->rlg_data = MY_ALLOC(pp3d_rlg_chain_data,nj);

  /* set generation function */
  rlgchPt->rlg_chain_fct = p3d_generate_cc_active_part_normal;

  /* if rmax and rmin are not given by the user they are auto. computed */
  if(rmax < 0.0)  /* is a flag */
    if(!p3d_compute_passive_approxWS(ct,&rmax,&rmin)) {
      p3d_destroy_rlg_chain(rlgchPt);
      return(FALSE);
    }
  
  rm_len = rmax;        /* <- length of passive part */
  /* case of rmin>0 not already treated !!!!!!!!! */

  /*** Parameters from laj to faj ***/

  /* for laj, information about attachment point is needed */
  /*  obtained from the ct structure */

  dir_last[0] = ct->Tatt[0][3];
  dir_last[1] = ct->Tatt[1][3];
  dir_last[2] = ct->Tatt[2][3];
  l_last = sqrt(SQR(dir_last[0])+SQR(dir_last[1])+SQR(dir_last[2]));

  /* iterative process beginning at laj */ 
  J = Jlaj;
  if (J!=NULL) {
    p3d_matInvertXform(J->pos0, T1);
    p3d_matMultXform(J->abs_pos, T1, pos);
    if (J->prev_jnt!=NULL) {
      p3d_matInvertXform(J->prev_jnt->pos0, T1);
      p3d_matMultXform(J->prev_jnt->abs_pos, T1, prev_pos);
    }
  }
  for(i=0; i < ndof; i++) {
    for(j=0; j < J->dof_equiv_nbr; j++) {
      rlg_data = MY_ALLOC(p3d_rlg_chain_data,1);
      rlgchPt->rlg_data[ndof-i-1] = rlg_data;
      rlgchPt->rlg_data[ndof-i-1]->jnt = J;
      rlgchPt->rlg_data[ndof-i-1]->num_dof_jnt = j;

      /* case of translational dof */
      if(p3d_jnt_is_dof_linear(J,j)) {
	if(((j == 0)&&(p3d_jnt_is_dof_linear(J->prev_jnt,J->prev_jnt->dof_equiv_nbr - 1)))||
	   ((j > 0)&&(p3d_jnt_is_dof_linear(J,j - 1)))) {
	  p3d_destroy_rlg_chain(rlgchPt);
#ifdef P3D_CONSTRAINTS
	  p3d_destroy_cntrt_manager(r->cntrt_manager);  /* delete all cntrts !!! */
#endif
	  r->cntrt_manager = NULL;
	  PrintInfo(("ERROR: p3d_set_random_loop_generator: two consecutive translational joints\n"));
	  return(FALSE);	  
	}

	if(rlgchPt->rlg_data[ndof-i]->jtype == RJD1) {
	  p3d_destroy_rlg_chain(rlgchPt);
#ifdef P3D_CONSTRAINTS
	  p3d_destroy_cntrt_manager(r->cntrt_manager);  /* delete all cntrts !!! */
#endif
	  r->cntrt_manager = NULL;
	  PrintInfo(("ERROR: p3d_set_random_loop_generator: mechanism can't be treated \n"));
	  return(FALSE);	  
	}      

	compute_prismatic_dof_minmax_length(J,&datmin,&datmax,X1,X1b);

	/* store joint type */
	rlgchPt->rlg_data[ndof-i-1]->jtype = TJ;
	/* store length between two rot.joints when trans.joint is at max and at min */
	if(i == 0) {
	  datmax += l_last;
	  datmin += l_last;
	}
	if(datmax >=  datmin) {
	  rlgchPt->rlg_data[nj-i-1]->dat[0] = datmax;
	  rlgchPt->rlg_data[nj-i-1]->dat[1] = datmin;
	}
	else {
	  rlgchPt->rlg_data[nj-i-1]->dat[0] = datmin;
	  rlgchPt->rlg_data[nj-i-1]->dat[1] = - datmax;  /* "-" is a flag to indicate the negative direction of the translation (still not used) */
	  /* vector used for "next" rotational joint */
	  p3d_vectCopy(X1b, X1);
	}	  
	/* -> length will be used for "next" rotational joint */
	acM_len += rlgchPt->rlg_data[ndof-i-1]->dat[0];     
	acm_len += fabs(rlgchPt->rlg_data[ndof-i-1]->dat[1]);
	if(i == 0) {
	  rlgchPt->rlg_data[ndof-i-1]->shell.rext = rm_len;
	}
	else {
	  rlgchPt->rlg_data[ndof-i-1]->shell.rext = rlgchPt->rlg_data[ndof-i]->shell.rext;
	}
      
	/* NOTE : the values of translations are ALWAYS considered POSITIVE */
      }
    
      /* case of rotational dof */
      else if(p3d_jnt_is_dof_angular(J,j)) {

	p3d_jnt_get_cur_vect_point(J, Xij);
	p3d_jnt_get_dof_cur_axis(J, 0, vaxeij);
      
	if(i == 0) {
	  p3d_xformVect(pos,dir_last,vij1);
	}
	else {
	  p3d_vectSub(X1, Xij, vij1);
	}

	/* analyze the joint type */
	/* NOTE : we reset the global variables  acM_len, acm_len and rm_len at this moment */  
      
	if((i == 0)||(rlgchPt->rlg_data[ndof-i]->jtype == TJ)) {
	  cosang = p3d_vectDotProd(vij1,vaxeij)/(p3d_vectNorm(vij1)*p3d_vectNorm(vaxeij));	
	  if((p3d_vectNorm(vij1)==0.0)||((fabs(cosang) <= 1 + IN3DCC_EPS)&&(fabs(cosang) >= 1 - IN3DCC_EPS))) {
	    rlgchPt->rlg_data[ndof-i-1]->jtype = RJC0;
	    /* NOTA : AQUI PUEDO HACER UNA CONSIDERACION :
	       SI i>2 Y EL EJE DE LA ANETRIOR ROTACION ES PERPENDICULAR A ESTE
	       SE PODRIA CONSIDERAR COMO RJC1 (PERO CON GENERACION MAS COMPLICADA) */
	  }
	  else if((cosang <= IN3DCC_EPS)&&(cosang >=  - IN3DCC_EPS)){
	    /* 	  p3d_vectSub(X1, Xij, vij1); */  /* QUITAR ESTO !!!! */
	    if((i > 2)&&(rlgchPt->rlg_data[ndof-i-1+2]->jtype == RJA1)&&(p3d_rel_pos_lines(vaxeij,vaxe1,vij1) == 2)) {
	      rlgchPt->rlg_data[ndof-i-1]->jtype = RJA2;
	    }
	    else {
	      rlgchPt->rlg_data[ndof-i-1]->jtype = RJA1;	   
	    }
	  }
	  else {
	    rlgchPt->rlg_data[ndof-i-1]->jtype = RJD1;	
	  }

	}
	/* NOTA : EL ANALISIS PUEDE MEJORARSE
	   EL QUE HAYA TRANSLACIONES ENTRE ROTACIONES NO IMPIDE PODER CONSIDERAR
	   ARTICULACIONES DE TIPO RJB
	   PERO ESTO COMPLICA LA GENERACION 
	   (-> ESTUDIAR ESTA MEJORA )  */
	else {
	  if((j > 0)||(p3d_jnt_frames_at_same_point(J,J->next_jnt[0]))) {
	    if((i == 0)||(p3d_rel_pos_lines(vaxeij,vaxe1,vij1) < 2)) {
	      rlgchPt->rlg_data[ndof-i-1]->jtype = RJC0; /* can rotate in its whole range */
	    }
	    else {
	      if((((rlgchPt->rlg_data[ndof-i]->jtype == RJC1)||(rlgchPt->rlg_data[ndof-i]->jtype == RJC0))&&(rlgchPt->rlg_data[ndof-i]->lmax > IN3DCC_EPS))||
		 (rlgchPt->rlg_data[ndof-i]->jtype == RJD1)||(rlgchPt->rlg_data[ndof-i]->jtype == RJD2)) {
		rlgchPt->rlg_data[ndof-i-1]->jtype = RJA1;
	      }
	      else {
		rlgchPt->rlg_data[ndof-i-1]->jtype = RJC1;
	      }
	    }
	    /* 	  rm_len += acM_len; */
	    /* 	  acM_len = 0.0; */
	    /* 	  acm_len = 0.0; */
	  }
	  else {
	    switch(p3d_rel_pos_lines(vaxeij,vaxe1,vij1)) {
	    case 0: 
	      /* there are two consecutive joints with the same axis   */
	      /* the first one rotates in its whole range and the next */
	      /* will be controlled                                    */
	      /* ( other heuristic is possible )                       */
	      rlgchPt->rlg_data[ndof-i-1]->jtype = RJC0;	    
	      break;
	    case 1:
	      /* there are two consecutive joints with parallel axes */
	      cosang = p3d_vectDotProd(vij1,vaxeij)/(p3d_vectNorm(vij1)*p3d_vectNorm(vaxeij));
	      if((cosang <= IN3DCC_EPS)&&(cosang >= - IN3DCC_EPS)) {
		if(!((rlgchPt->rlg_data[ndof-i]->jtype == RJC0)||(rlgchPt->rlg_data[ndof-i]->jtype == RJC1))) {
		  rlgchPt->rlg_data[ndof-i-1]->jtype = RJA1;
		}		
		else {
		  if(rlgchPt->rlg_data[ndof-i]->lmax == 0.0) {
		    rlgchPt->rlg_data[ndof-i-1]->jtype = RJA1;
		  }
		  else {
		    rlgchPt->rlg_data[ndof-i-1]->jtype = RJB1;
		  }
		}
	      }
	      else {
		rlgchPt->rlg_data[ndof-i-1]->jtype = RJD1;
	      }
	      rm_len += acM_len;
	      acM_len = 0.0;
	      acm_len = 0.0;	      
	      break;
	    case 2:
	      /* there are two consecutive joints with cutting axes */
	      /* NOTE : we suppose that cutting axes are perpendicular !!! */
	      if(p3d_rel_pos_lines(vaxeij,vij1,vij1) < 2) {
		if(!((rlgchPt->rlg_data[ndof-i]->jtype == RJC0)||(rlgchPt->rlg_data[ndof-i]->jtype == RJC1))) {
		  rlgchPt->rlg_data[ndof-i-1]->jtype = RJC1;
		}		
		else {
		  if(rlgchPt->rlg_data[ndof-i]->lmax == 0.0) {
		    rlgchPt->rlg_data[ndof-i-1]->jtype = RJC0;
		  }
		  else {
		    rlgchPt->rlg_data[ndof-i-1]->jtype = RJB3;
		  }
		}
	      }
	      else {
		if(rlgchPt->rlg_data[ndof-i]->jtype == RJA1) {
		  rlgchPt->rlg_data[ndof-i-1]->jtype = RJA2;
		}
		else {
		  rlgchPt->rlg_data[ndof-i-1]->jtype = RJA1;	   
		}
	      }
	      rm_len += acM_len;
	      acM_len = 0.0;
	      acm_len = 0.0;
	      break;
	    case 3:
	      /* there are two consecutive joints with crossing axes   */
	      cosang = p3d_vectDotProd(vij1,vaxeij)/(p3d_vectNorm(vij1)*p3d_vectNorm(vaxeij));
	      if((cosang <= IN3DCC_EPS)&&(cosang >= - IN3DCC_EPS)) {
		if(!((rlgchPt->rlg_data[ndof-i]->jtype == RJC0)||(rlgchPt->rlg_data[ndof-i]->jtype == RJC1))) {
		  rlgchPt->rlg_data[ndof-i-1]->jtype = RJA1;
		}		
		else {
		  if(rlgchPt->rlg_data[ndof-i]->lmax == 0.0) {
		    rlgchPt->rlg_data[ndof-i-1]->jtype = RJA1;
		  }
		  else {
		    rlgchPt->rlg_data[ndof-i-1]->jtype = RJB2;
		  }
		}	 
	      }
	      else {
		rlgchPt->rlg_data[ndof-i-1]->jtype = RJD1;
	      }     
	      rm_len += acM_len;
	      acM_len = 0.0;
	      acm_len = 0.0;
	      break;	     
	    }
	  }
	}
            
	/* ATENCION !!! : EL TIPO DE TRATAMIENTO DE DE rm_len Y acM_len 
	   HACE QUE SE PIERDA INFORMACION PARA CALCULAR shell.rint */
      
	/* treatment of different cases */
	switch(rlgchPt->rlg_data[ndof-i-1]->jtype) {
	  /* ----------------------------------------------- */
	case RJC0: 
	  /* -> length will be used for "next" rotational joint */
	  if(i == 0) {
	    acM_len += l_last;
	    acm_len += l_last;
	  }
	  else if(rlgchPt->rlg_data[ndof-i]->jtype != TJ) {
	    acM_len += sqrt(SQR(vij1[0])+SQR(vij1[1])+SQR(vij1[2]));
	    acm_len += sqrt(SQR(vij1[0])+SQR(vij1[1])+SQR(vij1[2]));
	  }
	  /* the sign indicates de direction of the rotation axix */
	  cosang = p3d_vectDotProd(vij1,vaxeij)/(p3d_vectNorm(vij1)*p3d_vectNorm(vaxeij));	
	  if(cosang > 0.0) {
	    rlgchPt->rlg_data[ndof-i-1]->lmax = acM_len;
	    rlgchPt->rlg_data[ndof-i-1]->lmin = acm_len;	
	  }
	  else {
	    rlgchPt->rlg_data[ndof-i-1]->lmax = - acM_len;
	    rlgchPt->rlg_data[ndof-i-1]->lmin = - acm_len;	
	  }
	  
	  rlgchPt->rlg_data[ndof-i-1]->shell.rext = rm_len;
	
	  /* references for angles not necessery */
	  break;
	
	  /* ----------------------------------------------- */
	case RJA1: 
	case RJA2: 
	  /* if previous joint is tranlational the length has been computed yet */
	  if(i == 0) {
	    acM_len += l_last;
	    acm_len += l_last;
	  }
	  else if(rlgchPt->rlg_data[ndof-i]->jtype != TJ) {
	    acM_len += sqrt(SQR(vij1[0])+SQR(vij1[1])+SQR(vij1[2]));
	    acm_len += sqrt(SQR(vij1[0])+SQR(vij1[1])+SQR(vij1[2]));
	  }
	  rlgchPt->rlg_data[ndof-i-1]->lmax = acM_len;
	  rlgchPt->rlg_data[ndof-i-1]->lmin = acm_len;
	  
	  rlgchPt->rlg_data[ndof-i-1]->shell.rext = rm_len;
	  
	  /* treatment for shell.rint */
	  if(Jmax_lmin > 0) 
	    val = rm_len - rlgchPt->rlg_data[ndof-Jmax_lmin]->lmax - rlgchPt->rlg_data[ndof-Jmax_lmin]->lmin;
	  else 
	    val = 0.0;
	  
	  if(val < 0.0) {
	    rlgchPt->rlg_data[ndof-i-1]->shell.rint = - val;
	  }
	  else {
	    rlgchPt->rlg_data[ndof-i-1]->shell.rint = 0.0;
	  }
	  
	  if(rm_len - rlgchPt->rlg_data[ndof-i-1]->lmin < val) {
	    Jmax_lmin = i;
	  }
	  
	  /* differentiate cases of RJD */
	  /* QUE PASA SI HAY TRANSLACIONES ???!!! : caso no considerado por el momento */
	  if(rlgchPt->rlg_data[ndof-i]->jtype == RJD1) {
	    if(p3d_rel_pos_lines(vaxeij,vaxe1,vij1) == 2) {
	      rlgchPt->rlg_data[ndof-i]->jtype = RJD2;
	    }
	  }

	  if(rlgchPt->rlg_data[ndof-i-1]->jtype == RJA2) {
	    if(rlgchPt->rlg_data[ndof-i]->jtype == TJ)
	      j = 2;
	    else
	      j = 1;
	    rm_len = rlgchPt->rlg_data[ndof-i-1+j]->shell.rext + sqrt(SQR(rlgchPt->rlg_data[ndof-i-1]->lmax)+SQR(rlgchPt->rlg_data[ndof-i-1+j]->lmax));
	    acM_len = 0.0;
	    acm_len = 0.0;
	  }
	  
	  /* references for angles */
	  /* 1- calculate homogeneous transformation of frame corresponding to preceding joint */
	  /*    in order to make its z-axis parallel to this rot. axis */
	  if(!p3d_transf_jnt_frame(prev_pos, vaxeij, T1)) {
	    p3d_destroy_rlg_chain(rlgchPt);
#ifdef P3D_CONSTRAINTS
	    p3d_destroy_cntrt_manager(r->cntrt_manager);  /* delete all cntrts !!! */
#endif
	    r->cntrt_manager = NULL;
	    PrintInfo(("ERROR: p3d_set_random_loop_generator: not orthogonal axes\n"));
	    return(FALSE);	  	    
	  }
	  p3d_mat4Copy(T1, rlgchPt->rlg_data[ndof-i-1]->Tref);
	  /* 2- calculate current value of vij1 in this reference frame */
	  p3d_mat4Mult(prev_pos, T1, T2);
	  p3d_matInvertXform(T2, T1);
	  if(p3d_vectNorm(vij1) > IN3DCC_EPS) { 
	    p3d_vectCopy(vij1,v3a);
	  }
	  else {
	    p3d_jnt_get_dof_cur_axis_before(J->next_jnt[0], 0, v3a);
	  }
	  p3d_xformVect(T1,v3a,v3b);
	  refval = atan2(v3b[1],v3b[0]) - p3d_jnt_get_dof(J,j);
	  if(refval < 0.0)
	    refval = (2.0*M_PI) + refval;     /* range 0 - 360 */
	  rlgchPt->rlg_data[ndof-i-1]->refval = refval;
	
	  break;

	  /* ----------------------------------------------- */
	case RJB1: 
	  /* previously analyzed joint is RJC0 or RJC1 (parallel axes) */
	  rlgchPt->rlg_data[ndof-i-1]->lmax = rlgchPt->rlg_data[ndof-i-1]->lmin = sqrt(SQR(vij1[0])+SQR(vij1[1])+SQR(vij1[2]));
	
	  rlgchPt->rlg_data[ndof-i-1]->shell.rext = rm_len - rlgchPt->rlg_data[ndof-i]->lmax;
	
	  /* treatment for shell.rint */
	  if(Jmax_lmin > 0) 
	    val = rm_len - rlgchPt->rlg_data[ndof-Jmax_lmin]->lmax - rlgchPt->rlg_data[ndof-Jmax_lmin]->lmin;
	  else 
	    val = 0.0;
	
	  if(val < 0.0) {
	    rlgchPt->rlg_data[ndof-i-1]->shell.rint = - val;
	  }
	  else {
	    rlgchPt->rlg_data[ndof-i-1]->shell.rint = 0.0;
	  }

	  if(rm_len - rlgchPt->rlg_data[ndof-i-1]->lmin < val) {
	    Jmax_lmin = i;
	  }

	  /* recompute rm_len */
	  rm_len = rlgchPt->rlg_data[ndof-i-1]->shell.rext + sqrt(SQR(rlgchPt->rlg_data[ndof-i-1]->lmax)+SQR(rlgchPt->rlg_data[ndof-i]->lmax));
	
	  /* shift for generation */
	  cosang = p3d_vectDotProd(vaxe1,vaxeij)/(p3d_vectNorm(vaxe1)*p3d_vectNorm(vaxeij));	
	  if(cosang > 0.0) {
	    rlgchPt->rlg_data[ndof-i-1]->dat[0] = rlgchPt->rlg_data[ndof-i]->lmax;
	    rlgchPt->rlg_data[ndof-i-1]->dat[1] = rlgchPt->rlg_data[ndof-i]->lmin;
	  }
	  else {
	    rlgchPt->rlg_data[ndof-i-1]->dat[0] = - rlgchPt->rlg_data[ndof-i]->lmax;
	    rlgchPt->rlg_data[ndof-i-1]->dat[1] = - rlgchPt->rlg_data[ndof-i]->lmin;
	  }
	
	  /* references for angles */
	  /* 1- calculate homogeneous transformation of frame corresponding to preceding joint */
	  /*    in order to make its z-axis parallel to this rot. axis */
	  if(!p3d_transf_jnt_frame(prev_pos, vaxeij, T1)) {
	    p3d_destroy_rlg_chain(rlgchPt);
#ifdef P3D_CONSTRAINTS
	    p3d_destroy_cntrt_manager(r->cntrt_manager);  /* delete all cntrts !!! */
#endif
	    r->cntrt_manager = NULL;
	    PrintInfo(("ERROR: p3d_set_random_loop_generator: not orthogonal axes\n"));
	    return(FALSE);	  	    
	  }
	  p3d_mat4Copy(T1, rlgchPt->rlg_data[ndof-i-1]->Tref);
	  /* 2- calculate current value of vij1 in this reference frame */
	  p3d_mat4Mult(prev_pos, T1, T2);
	  p3d_matInvertXform(T2, T1);
	  p3d_vectCopy(vij1,v3a);
	  p3d_matvec4Mult(T1,v3a,v3b);
	  refval = atan2(v3b[1],v3b[0]) - p3d_jnt_get_dof(J,j);
	  if(refval < 0.0)
	    refval = (2.0*M_PI) + refval;     /* range 0 - 360 */
	  rlgchPt->rlg_data[ndof-i-1]->refval = refval;

	  break;
	  
	  /* ----------------------------------------------- */
	case RJB2: 
	  /* previously analyzed joint is RJC0 or RJC1 (crossing axes) */
	  rlgchPt->rlg_data[ndof-i-1]->lmax = rlgchPt->rlg_data[ndof-i-1]->lmin = sqrt(SQR(vij1[0])+SQR(vij1[1])+SQR(vij1[2]));
	
	  rlgchPt->rlg_data[ndof-i-1]->shell.rext = rm_len - rlgchPt->rlg_data[ndof-i]->lmax;
	  
	  /* treatment for shell.rint */
	  if(Jmax_lmin > 0) 
	    val = rm_len - rlgchPt->rlg_data[ndof-Jmax_lmin]->lmax - rlgchPt->rlg_data[ndof-Jmax_lmin]->lmin;
	  else 
	    val = 0.0;
	  
	  if(val < 0.0) {
	    rlgchPt->rlg_data[ndof-i-1]->shell.rint = - val;
	  }
	  else {
	    rlgchPt->rlg_data[ndof-i-1]->shell.rint = 0.0;
	  }
	  
	  if(rm_len - rlgchPt->rlg_data[ndof-i-1]->lmin < val) {
	    Jmax_lmin = i;
	  }
	
	  /* recompute rm_len */
	  rm_len = rlgchPt->rlg_data[ndof-i-1]->shell.rext + sqrt(SQR(rlgchPt->rlg_data[ndof-i-1]->lmax)+SQR(rlgchPt->rlg_data[ndof-i]->lmax));
	
	  rlgchPt->rlg_data[ndof-i-1]->dat[0] = sqrt(SQR(rlgchPt->rlg_data[ndof-i-1]->lmax)+SQR(rlgchPt->rlg_data[ndof-i]->lmax));
	  rlgchPt->rlg_data[ndof-i-1]->dat[1] = sqrt(SQR(rlgchPt->rlg_data[ndof-i-1]->lmax)+SQR(rlgchPt->rlg_data[ndof-i]->lmin));
	  
	  /* angle to add for generation */   
	  p3d_vectXprod(vaxe1,vaxeij,v3aux);
	  cosang = p3d_vectDotProd(v3aux,vij1)/(p3d_vectNorm(v3aux)*p3d_vectNorm(vij1));	
	  if(cosang > 0.0) {
	    rlgchPt->rlg_data[ndof-i-1]->dat[2] = atan(rlgchPt->rlg_data[ndof-i]->lmax/rlgchPt->rlg_data[ndof-i-1]->lmax);
	    rlgchPt->rlg_data[ndof-i-1]->dat[3] = atan(rlgchPt->rlg_data[ndof-i]->lmin/rlgchPt->rlg_data[ndof-i-1]->lmax);
	  }
	  else {
	    rlgchPt->rlg_data[ndof-i-1]->dat[2] = - atan(rlgchPt->rlg_data[ndof-i]->lmax/rlgchPt->rlg_data[ndof-i-1]->lmax);
	    rlgchPt->rlg_data[ndof-i-1]->dat[3] = - atan(rlgchPt->rlg_data[ndof-i]->lmin/rlgchPt->rlg_data[ndof-i-1]->lmax);
	  }
	
	  /* references for angles */
	  /* 1- calculate homogeneous transformation of frame corresponding to preceding joint */
	  /*    in order to make its z-axis parallel to this rot. axis */
	  if(!p3d_transf_jnt_frame(prev_pos, vaxeij, T1)) {
	    p3d_destroy_rlg_chain(rlgchPt);
#ifdef P3D_CONSTRAINTS
	    p3d_destroy_cntrt_manager(r->cntrt_manager);  /* delete all cntrts !!! */
#endif
	    r->cntrt_manager = NULL;
	    PrintInfo(("ERROR: p3d_set_random_loop_generator: not orthogonal axes\n"));
	    return(FALSE);	  	    
	  }
	  p3d_mat4Copy(T1, rlgchPt->rlg_data[ndof-i-1]->Tref);
	  /* 2- calculate current value of vij1 in this reference frame */
	  p3d_mat4Mult(prev_pos, T1, T2);
	  p3d_matInvertXform(T2, T1);
	  p3d_vectCopy(vij1,v3a);
	  p3d_matvec4Mult(T1,v3a,v3b);
	  refval = atan2(v3b[1],v3b[0]) - p3d_jnt_get_dof(J,j);
	  if(refval < 0.0)
	    refval = (2.0*M_PI) + refval;     /* range 0 - 360 */
	  rlgchPt->rlg_data[ndof-i-1]->refval = refval;
	  
	  break;
	  
	  /* ----------------------------------------------- */
	case RJB3: 
	  /* previously analyzed joint is RJC0 or RJC1 (cutting axes) */
	  /* lmax will be used to deplace the generation plane -> the sign is necessary */
	  cosang = p3d_vectDotProd(vaxeij,vij1)/(p3d_vectNorm(vaxeij)*p3d_vectNorm(vij1));	
	  if(cosang > 0.0) {
	    rlgchPt->rlg_data[ndof-i-1]->lmax = rlgchPt->rlg_data[ndof-i-1]->lmin = sqrt(SQR(vij1[0])+SQR(vij1[1])+SQR(vij1[2]));
	  }
	  else {
	    rlgchPt->rlg_data[ndof-i-1]->lmax = rlgchPt->rlg_data[ndof-i-1]->lmin = - sqrt(SQR(vij1[0])+SQR(vij1[1])+SQR(vij1[2]));
	  }
	  
	  rlgchPt->rlg_data[ndof-i-1]->shell.rext = rlgchPt->rlg_data[ndof-i]->shell.rext;

	  /* treatment for shell.rint */
	  if(Jmax_lmin > 0) 
	    val = rm_len - rlgchPt->rlg_data[ndof-Jmax_lmin]->lmax - rlgchPt->rlg_data[ndof-Jmax_lmin]->lmin;
	  else 
	    val = 0.0;
	  
	  if(val < 0.0) {
	    rlgchPt->rlg_data[ndof-i-1]->shell.rint = - val;
	  }
	  else {
	    rlgchPt->rlg_data[ndof-i-1]->shell.rint = 0.0;
	  }
	  
	  if(rm_len - fabs(rlgchPt->rlg_data[ndof-i-1]->lmin) < val) {
	    Jmax_lmin = i;
	  }
	  
	  /* recompute rm_len */
	  rm_len = rm_len - rlgchPt->rlg_data[ndof-i]->lmax + sqrt(SQR(rlgchPt->rlg_data[ndof-i-1]->lmax)+SQR(rlgchPt->rlg_data[ndof-i]->lmax));
	
	  rlgchPt->rlg_data[ndof-i-1]->dat[0] = rlgchPt->rlg_data[ndof-i]->lmax;
	  rlgchPt->rlg_data[ndof-i-1]->dat[1] = rlgchPt->rlg_data[ndof-i]->lmin;
	  
	  /* references for angles */
	  /* 1- calculate homogeneous transformation of frame corresponding to preceding joint */
	  /*    in order to make its z-axis parallel to this rot. axis */
	  if(!p3d_transf_jnt_frame(prev_pos, vaxeij, T1)) {
	    p3d_destroy_rlg_chain(rlgchPt);
#ifdef P3D_CONSTRAINTS
	    p3d_destroy_cntrt_manager(r->cntrt_manager);  /* delete all cntrts !!! */
#endif
	    r->cntrt_manager = NULL;
	    PrintInfo(("ERROR: p3d_set_random_loop_generator: not orthogonal axes\n"));
	    return(FALSE);	  	    
	  }
	  p3d_mat4Copy(T1, rlgchPt->rlg_data[ndof-i-1]->Tref);
	  /* 2- calculate current value of vij1 in this reference frame */
	  p3d_mat4Mult(prev_pos, T1, T2);
	  p3d_matInvertXform(T2, T1);
	  p3d_vectCopy(vij1,v3a);
	  p3d_matvec4Mult(T1,v3a,v3b);
	  refval = atan2(v3b[1],v3b[0]) - p3d_jnt_get_dof(J,j);
	  if(refval < 0.0)
	    refval = (2.0*M_PI) + refval;     /* range 0 - 360 */
	  rlgchPt->rlg_data[ndof-i-1]->refval = refval;

	  break;

	  /* ----------------------------------------------- */
	case RJC1:
	  /* previous joint can't be a translation */
	  /* acM_len must be 0 at this moment !!! */
	  /* we compute the length and keep it in acM_len */
	  /* sign must be known for generation */
	  cosang = p3d_vectDotProd(vaxeij,vij1)/(p3d_vectNorm(vaxeij)*p3d_vectNorm(vij1));	
	  if(cosang > 0.0) {
	    rlgchPt->rlg_data[ndof-i-1]->lmax = rlgchPt->rlg_data[ndof-i-1]->lmin = sqrt(SQR(vij1[0])+SQR(vij1[1])+SQR(vij1[2]));
	  }
	  else {
	    rlgchPt->rlg_data[ndof-i-1]->lmax = rlgchPt->rlg_data[ndof-i-1]->lmin = - sqrt(SQR(vij1[0])+SQR(vij1[1])+SQR(vij1[2]));
	  }	

	  acM_len = acm_len = fabs(rlgchPt->rlg_data[ndof-i-1]->lmax);
	  
	  rlgchPt->rlg_data[ndof-i-1]->shell.rext = rlgchPt->rlg_data[ndof-i]->shell.rext;

	  /* we don't consider shell.rint for generation in this case */
	  /* PODRIA RECALCULARLA, PERO NO SERVIRIA DE MUCHO ?? */
	  rlgchPt->rlg_data[nj-i-1]->shell.rint = 0.0;
	  
	  /* updates */
	  /* rm_len will be increased later */
	  /* 	rm_len += rlgchPt->rlg_data[ndof-i-1]->lmax; */
	  /* 	acM_len = 0.0; */
	  /* 	acm_len = 0.0; */
	  
	  rlgchPt->rlg_data[ndof-i-1]->dat[0] = rlgchPt->rlg_data[ndof-i]->lmax;
	  rlgchPt->rlg_data[ndof-i-1]->dat[1] = rlgchPt->rlg_data[ndof-i]->lmin;

	  /* recompute rm_len */
	  rm_len = rlgchPt->rlg_data[ndof-i-1]->shell.rext + rlgchPt->rlg_data[ndof-i-1]->dat[0];

	  /* references for angles */
	  /* 1- calculate homogeneous transformation of frame corresponding to preceding joint */
	  /*    in order to make its z-axis parallel to this rot. axis */
	  if(!p3d_transf_jnt_frame(prev_pos, vaxeij, T1)) {
	    p3d_destroy_rlg_chain(rlgchPt);
#ifdef P3D_CONSTRAINTS
	    p3d_destroy_cntrt_manager(r->cntrt_manager);  /* delete all cntrts !!! */
#endif
	    r->cntrt_manager = NULL;
	    PrintInfo(("ERROR: p3d_set_random_loop_generator: not orthogonal axes\n"));
	    return(FALSE);	  	    
	  }
	  p3d_mat4Copy(T1, rlgchPt->rlg_data[ndof-i-1]->Tref);
	  /* 2- calculate current value of vij1 in this reference frame */
	  p3d_mat4Mult(prev_pos, T1, T2);
	  p3d_matInvertXform(T2, T1);
	  p3d_vectCopy(vaxe1,v3a); /* ESTO ES LO QUE CAMBIA CON RESPECTO A LAS OTRAS : podria meter esto en una funcion   !!!*/
	  p3d_matvec4Mult(T1,v3a,v3b);
	  refval = atan2(v3b[1],v3b[0]) - p3d_jnt_get_dof(J,j);
	  if(refval < 0.0)
	    refval = (2.0*M_PI) + refval;     /* range 0 - 360 */
	  rlgchPt->rlg_data[ndof-i-1]->refval = refval;
	  
	  break;

	  /* ----------------------------------------------- */
	case RJD1:
	case RJD2:
	  /* case of vij1 not perpendicular to vaxeij */
	  /* WARNING !! : it only works well in the case of no prismatic intermediate joint */
	  if(i == 0) {
	    acM_len += l_last;
	    acm_len += l_last;
	  }
	  else {
	    if(rlgchPt->rlg_data[ndof-i]->jtype == TJ) {
	      p3d_destroy_rlg_chain(rlgchPt);
#ifdef P3D_CONSTRAINTS
	      p3d_destroy_cntrt_manager(r->cntrt_manager);  /* delete all cntrts !!! */
#endif
	      r->cntrt_manager = NULL;
	      PrintInfo(("ERROR: p3d_set_random_loop_generator: not allowed prismatic joint\n"));
	      return(FALSE);	  
	    }

	    acM_len += sqrt(SQR(vij1[0])+SQR(vij1[1])+SQR(vij1[2]));
	    acm_len += sqrt(SQR(vij1[0])+SQR(vij1[1])+SQR(vij1[2]));	
	  }

	  rlgchPt->rlg_data[ndof-i-1]->lmax = acM_len;
	  rlgchPt->rlg_data[ndof-i-1]->lmin = acm_len;
	
	  rlgchPt->rlg_data[ndof-i-1]->shell.rext = rm_len;
	
	  /* treatment for shell.rint */
	  if(Jmax_lmin > 0) 
	    val = rm_len - rlgchPt->rlg_data[ndof-Jmax_lmin]->lmax - rlgchPt->rlg_data[ndof-Jmax_lmin]->lmin;
	  else 
	    val = 0.0;
	  
	  if(val < 0.0) {
	    rlgchPt->rlg_data[ndof-i-1]->shell.rint = - val;
	  }
	  else {
	    rlgchPt->rlg_data[ndof-i-1]->shell.rint = 0.0;
	  }
	  
	  if(rm_len - rlgchPt->rlg_data[ndof-i-1]->lmin < val) {
	    Jmax_lmin = i;
	  }
	  
	  /* HACEMOS AQUI EN ESTE CASO : A VERIFICAR !!! */
	  rm_len += acM_len;
	  acM_len = 0.0;
	  acm_len = 0.0;

	  cosang = p3d_vectDotProd(vaxeij,vij1)/(p3d_vectNorm(vij1)*p3d_vectNorm(vaxeij));	
	  rlgchPt->rlg_data[ndof-i-1]->dat[0] = rlgchPt->rlg_data[ndof-i-1]->lmax * cosang;
	  rlgchPt->rlg_data[ndof-i-1]->dat[1] = rlgchPt->rlg_data[ndof-i-1]->lmax * sin(acos(cosang));

	  /* references for angles */
	  /* 1- calculate homogeneous transformation of frame corresponding to preceding joint */
	  /*    in order to make its z-axis parallel to this rot. axis */
	  if(!p3d_transf_jnt_frame(prev_pos, vaxeij, T1)) {
	    p3d_destroy_rlg_chain(rlgchPt);
#ifdef P3D_CONSTRAINTS
	    p3d_destroy_cntrt_manager(r->cntrt_manager);  /* delete all cntrts !!! */
#endif
	    r->cntrt_manager = NULL;
	    PrintInfo(("ERROR: p3d_set_random_loop_generator: not orthogonal axes\n"));
	    return(FALSE);	  	    
	  }
	  p3d_mat4Copy(T1, rlgchPt->rlg_data[ndof-i-1]->Tref);
	  /* 2- calculate current value of vij1 in this reference frame */
	  p3d_mat4Mult(prev_pos, T1, T2);
	  p3d_matInvertXform(T2, T1);
	  if(p3d_vectNorm(vij1) > IN3DCC_EPS) { 
	    p3d_vectCopy(vij1,v3a);
	  }
	  else {
	    p3d_jnt_get_dof_cur_axis_before(J->next_jnt[0], 0, v3b);
	  }
	  p3d_xformVect(T1,v3a,v3b);
	  refval = atan2(v3b[1],v3b[0]) - p3d_jnt_get_dof(J,j);
	  if(refval < 0.0)
	    refval = (2.0*M_PI) + refval;     /* range 0 - 360 */
	  rlgchPt->rlg_data[ndof-i-1]->refval = refval;

	  break;
	} 

	/* update variables */
	p3d_vectCopy(Xij, X1);
	p3d_vectCopy(vaxeij, vaxe1);
	p3d_vectCopy(vaxeij, vaxe1_lr);
      }
      
      else {
	p3d_destroy_rlg_chain(rlgchPt);
	PrintInfo(("ERROR: p3d_set_random_loop_generator: wrong dof type\n"));
	return(FALSE);	 
      }
    }
    J = J->prev_jnt;
    if (J!=NULL) {
      p3d_matInvertXform(J->pos0, T1);
      p3d_matMultXform(J->abs_pos, T1, pos);
      if (J->prev_jnt!=NULL) {
	p3d_matInvertXform(J->prev_jnt->pos0, T1);
	p3d_matMultXform(J->prev_jnt->abs_pos, T1, prev_pos);
      }
    }
  }
  rlgchPt->rlg_data[0]->totrml = rm_len + fabs(rlgchPt->rlg_data[0]->lmax);
    
  /* flag for the generation of configurations */
  /* this flag is not currently used, but it will be used in the shoot */
  for(i=0; i<ndof; i++) {
    r->cntrt_manager->in_cntrt[Jfaj->index_dof  + i] = -1;
  }
  
  ct->rlgPt->rlgchPt = rlgchPt;
  return(TRUE);
}


/*****************************************************************************/

/* function generatind data structures for RLG holonom-base */

static int p3d_fct_set_rlg_holonom_base(int ctnum, int ibj)
{ p3d_rob *r;
  p3d_cntrt *ct;
  pp3d_rlg_base rlgbsPt;
  p3d_jnt *manipbasejntPt;
  p3d_matrix4 Tbm;

  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  ct = r->cntrt_manager->cntrts[ctnum];

  /* memory allocation */
  if(ct->rlgPt == NULL)
    p3d_generate_rlg(ct);
  /* WARNING !!! : if rlg-chain it must be defined first ! */

  rlgbsPt = MY_ALLOC(p3d_rlg_base,1);

  /* base joint */
  rlgbsPt->basejntPt = r->joints[ibj];
  if((rlgbsPt->basejntPt->type != P3D_PLAN) && (rlgbsPt->basejntPt->type != P3D_FREEFLYER)) {
    p3d_destroy_rlg_base(rlgbsPt);
#ifdef P3D_CONSTRAINTS
    p3d_destroy_cntrt_manager(r->cntrt_manager);  /* delete all cntrts !!! */
#endif
    r->cntrt_manager = NULL;
    PrintInfo(("ERROR: p3d_set_random_loop_generator: robot base must be P3D_PLAN or P3D_FREEFLYER\n"));
    return(FALSE);	      
  }

  /* set generation function */
  rlgbsPt->rlg_base_fct = p3d_generate_holonom_base_conf;

  /* data */
  if(ct->rlgPt->rlgchPt != NULL) {
    manipbasejntPt = ct->rlgPt->rlgchPt->rlg_data[0]->jnt;
  }
  else{
    manipbasejntPt = ct->pasjnts[0];
  }
  p3d_compute_T_base_robot_manip(ct,rlgbsPt->basejntPt,manipbasejntPt,Tbm);
  p3d_mat4Copy(Tbm, rlgbsPt->Tbm);
  rlgbsPt->lbm = sqrt(SQR(Tbm[0][3]) + SQR(Tbm[1][3]) + SQR(Tbm[2][3]));

  // MIRAR COMO GUARDAR Y USAR DATOS DE Tbm
  // GUARDAR DATOS DE VALORES INICIALES Y REFERNCIAS PARA LA FCT DE CALCULO !!!???

  rlgbsPt->rmax = p3d_get_max_extension_manipulator(ct);
  /*   rlgbsPt->rmin = p3d_get_min_extension_manipulator(ct); */   // TO DO !!!
  rlgbsPt->rmin = 0.0;  // PROVISIONAL

  ct->rlgPt->rlgbsPt = rlgbsPt;
  return (TRUE);
}


/*****************************************************************************/
/*****************************************************************************/

int p3d_random_loop_generator(p3d_rob *robotPt, configPt q)
{
  p3d_cntrt *ct;

  if(robotPt->cntrt_manager->cntrts != NULL) {
    p3d_set_robot_config(robotPt,q);
    p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt); 
    for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list);
	 dbl_list_more(robotPt->cntrt_manager->cntrt_call_list); 
	 dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
      ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
      if(!p3d_generate_ct_conf(robotPt,ct,q)) {
	return(FALSE);
      }
    }
  }
  return(TRUE);
}

/*****************************************************************************/

int p3d_random_loop_generator_without_parallel(p3d_rob *robotPt, configPt q)
{
  p3d_cntrt *ct;

  if(robotPt->cntrt_manager->cntrts != NULL) {
    p3d_set_robot_config(robotPt,q);
    p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt); 
    for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list);
	 dbl_list_more(robotPt->cntrt_manager->cntrt_call_list); 
	 dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
      ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
      if(!p3d_generate_ct_conf_basesandchains(robotPt,ct,q)) {
	return(FALSE);
      }
    }
  }
  return(TRUE);
}

/*****************************************************************************/

int p3d_reshoot_and_set_config_from_ct(p3d_rob *robotPt, p3d_cntrt *ct)
{
  configPt q;
  int tries,go_on,ict;
  
  q = p3d_get_robot_config(robotPt);
 
  tries = 0;

  while(tries < TRIES_BEFORE_RESHOOT) {
    /* increase nb_q, even if it is just a partial shoot !!! <- NO */
/*     if(robotPt->GRAPH != NULL) */
/*       robotPt->GRAPH->nb_q ++; */
    if(!p3d_generate_ct_conf(robotPt,ct,q)) {
      return (FALSE);
    }
    ict = 0;
    go_on = 1;
    while((go_on)&&(ict < ct->nctud)) {
      if(ct->ct_to_update[ict]->active) {
	if(!(*ct->ct_to_update[ict]->fct_cntrt)(ct->ct_to_update[ict],-1,NULL,0.0)) {
	  go_on = 0;
	}
      }
      ict++;
    }
    if(go_on) {
      return (TRUE);
    }
    tries++;
  }  
  return (FALSE);
}


/*****************************************************************************/

static int p3d_generate_ct_conf(p3d_rob *robotPt, p3d_cntrt *ct, configPt q)
{
  // modif Juan (for BioMove3d)
  // Warning : another type of flag should be necessary
#ifdef P3D_COLLISION_CHECKING
  if(p3d_col_get_mode() == p3d_col_mode_bio) {
    if(!bio_generate_ct_conf(robotPt,ct,q))
      return(FALSE);
  }
  else {
    
    if(!p3d_generate_ct_conf_parallel(robotPt,ct,q))
      return(FALSE);

    if(!p3d_generate_ct_conf_basesandchains(robotPt,ct,q))
      return(FALSE);
  }
#endif

  return(TRUE);
  // fmodif Juan
}

/*****************************************************************************/

static int p3d_generate_ct_conf_parallel(p3d_rob *robotPt, p3d_cntrt *ct, configPt q)
{
  int tries, go_on;
  
  if((ct->parallel_sys_data != NULL) && 
     (ct->parallel_sys_data->fct_parplatf_shoot != NULL)) {
    /* generation of the parallel platform */
    /* WARNING : sequence of parallel platforms :
       cntrts ordered from the base to the top */
    // NOTA : ITERAR ESTO !!!???
    go_on = 1;
    tries = 0;
    while(go_on && (tries < TRIES_BEFORE_RESHOOT)) {
      tries++;
      if((ct->parallel_sys_data->fct_parplatf_shoot)(ct->parallel_sys_data,q)) {
	p3d_set_robot_config(robotPt,q);
	p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt); 	  
	go_on = 0;
      }
    }
    if(go_on) {
      return(FALSE);
    }
    
    if(PRINTPROC_RLG) {
      printf("Conf. for cntrt num = %d generated at ntry = %d\n",ct->num,tries+1);
      printf("Platform conf.:\n");
      if(ct->parallel_sys_data->platform_jntPt->type == P3D_KNEE) {
	printf("rx =%f, ry =%f, rz =%f\n\n",
	       q[ct->parallel_sys_data->platform_jntPt->index_dof] * (180.0/M_PI),
	       q[ct->parallel_sys_data->platform_jntPt->index_dof+1] * (180.0/M_PI),
	       q[ct->parallel_sys_data->platform_jntPt->index_dof+2] * (180.0/M_PI));
      }
      else {  // P3D_FREEFLYER
	printf("x =%f, y =%f, z =%f\n",
	       q[ct->parallel_sys_data->platform_jntPt->index_dof],
	       q[ct->parallel_sys_data->platform_jntPt->index_dof+1],
	       q[ct->parallel_sys_data->platform_jntPt->index_dof+2]);
	printf("rx =%f, ry =%f, rz =%f\n\n",
	       q[ct->parallel_sys_data->platform_jntPt->index_dof+3] * (180.0/M_PI),
	       q[ct->parallel_sys_data->platform_jntPt->index_dof+4] * (180.0/M_PI),
	       q[ct->parallel_sys_data->platform_jntPt->index_dof+5] * (180.0/M_PI));
      }
    }
  }
  
  return(TRUE); 
}

/*****************************************************************************/

static int p3d_generate_ct_conf_basesandchains(p3d_rob *robotPt, p3d_cntrt *ct, configPt q)
{
  int tries, go_on;

  /* generation of the active chain (including mobile base) of each closed chain */
  if((ct->active)&&(ct->rlgPt != NULL)) {
    /* first : generate base */
    if(ct->rlgPt->rlgbsPt != NULL) {
      if((*ct->rlgPt->rlgbsPt->rlg_base_fct)(ct,q)) {
	p3d_set_robot_config(robotPt,q);
	p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt); 	  
      }
      else {
	return(FALSE);
      }
    }
    /* then : generate active chain */
    if(ct->rlgPt->rlgchPt != NULL) {
      go_on = 1;
      tries = 0;
      /* call rlg several times before global reshoot */
      while((go_on)&&(tries < TRIES_BEFORE_RESHOOT)) {
	if((*ct->rlgPt->rlgchPt->rlg_chain_fct)(ct,q))
	  go_on = 0;
/* 	else */
/* 	  if(robotPt->GRAPH != NULL) */
/* 	    robotPt->GRAPH->nb_q ++; */
	tries++;
      }
      if(go_on) {
	return(FALSE);
      }
      else {
	p3d_set_robot_config(robotPt,q);
	p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt); 	 
      }
    }
  }

  return(TRUE); 
}

 
/*****************************************************************************/

static int p3d_generate_cc_active_part_normal(p3d_cntrt *ct, configPt q)
{ 
  int j, irlgd; 
  p3d_jnt *Jfaj, *Jlaj, *Jpbj, *J;
  p3d_vector3 v3a,v3b;
  p3d_vector3 v1,v1b,v2,vref1,vref2;
  p3d_vector3 X1,X1b,X2;
  p3d_matrix4 Ttot, pos, prev_pos, T1;
  static p3d_matrix4 Tshift = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  double distance,distance_b = 0.;
  double dist_p,dist_pp,rml_pr,mMdif_pr;
  double lij,lrecto;
  double max_lim,min_lim;
  double ang,alpha,theta;
  int tries,hayb,invertb = 0;
  double whole_rml,whole_mMdif;
  double refofsym,refofsym2;
  double increfmin,increfmax;
  double vmin,vmax;
  p3d_matrix2 interv1,interv2;
  int iv1=0,iv2=0;
  pp3d_rlg_chain rlgchPt;

  rlgchPt = ct->rlgPt->rlgchPt;

  whole_rml = rlgchPt->rlg_data[0]->totrml;
  
  whole_mMdif = rlgchPt->rlg_data[0]->shell.rint - rlgchPt->rlg_data[0]->lmax;  /* VERIFICAR !!!!!!!!! */
  
  Jfaj = rlgchPt->rlg_data[0]->jnt;
  Jlaj = rlgchPt->rlg_data[rlgchPt->nlinksrlgch - 1]->jnt;
  
  if(strcmp(ct->namecntrt,"p3d_in_sphere")==0) {
    X2[0] = ct->argu_d[0];
    X2[1] = ct->argu_d[1];
    X2[2] = ct->argu_d[2];
  }    
  else {
    Jpbj = ct->pasjnts[0];  /* WARNING !!! : we consider that fist passive joint is the base */
    p3d_jnt_get_cur_vect_point_before(Jpbj, X2);
  }

  /* generation for faj to laj */
     
  /* 1- verify distance between faj and pbj */  

  p3d_jnt_get_cur_vect_point_before(Jfaj, X1);
  p3d_vectSub(X2, X1, v1);	
  distance = sqrt(SQR(v1[0])+SQR(v1[1])+SQR(v1[2]));
  
  /* faj is in the ring ? */
  if((distance > whole_rml)||(distance < whole_mMdif)) {
    return(FALSE);
  }
  
  if(PRINTPROC_RLG)
    printf("test 0 OK\n"); 

  /* 2- generate for faj -> laj */ 

  J = Jfaj;
  if (J!=NULL) {
    p3d_matInvertXform(J->pos0, T1);
    p3d_matMultXform(J->abs_pos, T1, pos);
    if (J->prev_jnt!=NULL) {
      p3d_matInvertXform(J->prev_jnt->pos0, T1);
      p3d_matMultXform(J->prev_jnt->abs_pos, T1, prev_pos);
    }
  }
  irlgd = 0;
  while((J != NULL)&&(J->prev_jnt != Jlaj)) {
    for(j=0; j < J->dof_equiv_nbr; j++) {

      /* case of translational dof */
      if(p3d_jnt_is_dof_linear(J,j)) {
	tries = 0;
	p3d_jnt_get_dof_bounds(J,j,&vmin,&vmax);
	do {
#ifdef P3D_PLANNER
	  q[J->index_dof + j] = p3d_random(vmin,vmax); 
#endif
	  p3d_jnt_set_dof(J,j,q[J->index_dof + j]);
	  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
	  /* just ONE consecutive translation ! */
	  p3d_jnt_get_cur_vect_point(J->next_jnt[0], X1);
	  p3d_vectSub(X2, X1, v1);	
	  distance = sqrt(SQR(v1[0])+SQR(v1[1])+SQR(v1[2]));
	  /* if(distance >= rm_l) { */
	  /* NOTE: we would try to generate in the valid portion of the range */
	  /*       by using the tendency of the distance and the translation  */
	  /* } */
	  tries++;

	} while(((distance > rlgchPt->rlg_data[irlgd+1]->shell.rext)||(distance < rlgchPt->rlg_data[irlgd+1]->shell.rint))&&(tries < 5));

	if(tries >= 5) {
	  return(FALSE);
	}
      }
    
      /* case of rotational dof */
      /* NOTE : no other case is already possible */
      else {
	switch(rlgchPt->rlg_data[irlgd]->jtype) {
	  /* ----------------------------------------------- */
	case RJC0:
	  p3d_jnt_get_dof_bounds(J,j,&vmin,&vmax);
#ifdef P3D_PLANNER
	  q[J->index_dof + j] = p3d_random(vmin,vmax); 
#endif
	  p3d_jnt_set_dof(J,j,q[J->index_dof + j]);
	  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 

	  break;
	
	  /* ----------------------------------------------- */
	case RJA1: 
	case RJA2: 
	  /* placement of joint J and distance to pbj */
	  /* yet calculated for the case of faj */
	  if(J != Jfaj) {
	    p3d_jnt_get_cur_vect_point_before(J, X1);
	    p3d_vectSub(X2, X1, v1);	
	    distance = sqrt(SQR(v1[0])+SQR(v1[1])+SQR(v1[2]));       
	  }
			
	  /* analysis for shell.rext */
	
	  /* dof axis */
	  p3d_jnt_get_dof_cur_axis_before(J, j, v2);
	
	  /* distances to plane of displacement of next joint */
	  ang = acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
	  dist_pp = fabs(distance * sin(ang));     /* pp -> perpendicular to axis */  
	  dist_p = fabs(distance * cos(ang));      /* p  -> parallel to axis      */
	
	  /* if dist_p > shell.rext -> can't close */
	  if(dist_p > rlgchPt->rlg_data[irlgd]->shell.rext) {
	    return(FALSE);
	  }
	
	  /* plane proyections */
	  rml_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rext)-SQR(dist_p));
	
	  if(rlgchPt->rlg_data[irlgd]->shell.rint > dist_p) 
	    mMdif_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rint)-SQR(dist_p));
	  else
	    mMdif_pr = 0.0;
	
	  /* SOLO PARA PRIMERA PRUEBA !!! */
	  if(J != Jlaj) {
	    if(rlgchPt->rlg_data[irlgd+1]->jtype == RJD2) {
	      rlgchPt->rlg_data[irlgd]->lmin = rlgchPt->rlg_data[irlgd]->lmax = rlgchPt->rlg_data[irlgd+1]->dat[0];
	    }
	  }
	
	  /* if dist_pp > shell.rml_pr + lmax   -> can't close */
	  /* if lmin > rml_pr + dist_pp   -> can't close */
	  /* if dist_pp + lmax < mMdif_pr -> can't close */
	  if((dist_pp > rml_pr + rlgchPt->rlg_data[irlgd]->lmax)||
	     (rlgchPt->rlg_data[irlgd]->lmin > rml_pr + dist_pp)||
	     (dist_pp + rlgchPt->rlg_data[irlgd]->lmax < mMdif_pr)) {
	    return(FALSE);
	  }
	
	  /* NOTE: we calculate an interval in relation to the line Jij-pbj */
	  /* this interval is symmetric, so we can calculate the range 0-180 */
	  
	  /* calculate max_lim of interval */
	  if(dist_pp + rlgchPt->rlg_data[irlgd]->lmin <= rml_pr) {
	    max_lim = M_PI;
	  }
	  else {
	    if(dist_pp <= rml_pr) {
	      lij = rlgchPt->rlg_data[irlgd]->lmin;
	    }
	    else {
	      lrecto = sqrt(SQR(dist_pp)-SQR(rml_pr));
	      if((lrecto <= rlgchPt->rlg_data[irlgd]->lmax)&&(lrecto >= rlgchPt->rlg_data[irlgd]->lmin))
		lij = lrecto;
	      else {
		if(fabs(rlgchPt->rlg_data[irlgd]->lmax-lrecto) < fabs(rlgchPt->rlg_data[irlgd]->lmin-lrecto))
		  lij = rlgchPt->rlg_data[irlgd]->lmax;
		else
		  lij = rlgchPt->rlg_data[irlgd]->lmin;
	      }
	    }
	    max_lim = acos((SQR(dist_pp)+SQR(lij)-SQR(rml_pr))/(2*dist_pp*lij));
	  }
	
	  /* calculate min_lim of interval */
	  if(mMdif_pr > 0.0) {
	    if((dist_pp >= mMdif_pr + rlgchPt->rlg_data[irlgd]->lmin)||
	       (dist_pp + mMdif_pr <= rlgchPt->rlg_data[irlgd]->lmax)) {
	      min_lim = 0.0;
	    }
	    else {
	      if(dist_pp <= mMdif_pr) {
		lij = rlgchPt->rlg_data[irlgd]->lmax;
	      }
	      else {
		lrecto = sqrt(SQR(dist_pp)-SQR(mMdif_pr));
		if(fabs(rlgchPt->rlg_data[irlgd]->lmax-lrecto) >= fabs(rlgchPt->rlg_data[irlgd]->lmin-lrecto))
		  lij = rlgchPt->rlg_data[irlgd]->lmax;
		else
		  lij = rlgchPt->rlg_data[irlgd]->lmin;
	      }
	      min_lim = acos((SQR(dist_pp)+SQR(lij)-SQR(mMdif_pr))/(2*dist_pp*lij));
	    }
	  }
	  else 
	    min_lim = 0.0;
	
	  /* calculate intervals */
	  p3d_jnt_get_dof_bounds(J,j,&vmin,&vmax);
	  if((min_lim <= 0.0)&&(max_lim >= M_PI)) {
#ifdef P3D_PLANNER
	    q[J->index_dof + j] = p3d_random(vmin,vmax); 
#endif
	  }
	  else {
	    refofsym = p3d_ref_in_jnt_frame(rlgchPt,irlgd,v1); 
	    if(min_lim <= 0.0) {
	      iv1 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
					  refofsym-max_lim,refofsym+max_lim,interv1);
	      if(!iv1)
		return(FALSE);
	      iv2 = 0;
	    }
	    else {
	      iv1 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
					  refofsym+min_lim,refofsym+max_lim,interv1);
	      iv2 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
					  refofsym-max_lim,refofsym-min_lim,interv2);
	      if((!iv1)&&(!iv2))
		return(FALSE); 
	    }
	    /* random shoot */
	    q[J->index_dof + j] = p3d_random_in_several_intervals(iv1,iv2,interv1,interv2,rlgchPt->rlg_data[irlgd]->refval);
	  }	  
	  p3d_put_dof_into_range(J, j, &q[J->index_dof + j]);
	  p3d_jnt_set_dof(J,j,q[J->index_dof + j]);
	  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
	
	break;
	
	/* ----------------------------------------------- */
	case RJB1: 
	  /* intersection of planes for generation with dof axis */
	  /* and distance to pbj */
	  p3d_jnt_get_vect_point(J, v3a);
	  p3d_jnt_get_dof_axis(J, j, v3b);
	  /* we suppose that the axis is NORMALIZED */
	  /* farthest point of the axis */
	  Tshift[0][3] = v3b[0] * rlgchPt->rlg_data[irlgd]->dat[0];
	  Tshift[1][3] = v3b[1] * rlgchPt->rlg_data[irlgd]->dat[0];
	  Tshift[2][3] = v3b[2] * rlgchPt->rlg_data[irlgd]->dat[0];
	  p3d_mat4Mult(prev_pos,Tshift,Ttot);
	  p3d_xformPoint(Ttot,v3a,X1); 
	  p3d_vectSub(X2, X1, v1);	
	  distance = sqrt(SQR(v1[0])+SQR(v1[1])+SQR(v1[2]));
	
	  /* nearest point (if it exist)*/
	  if(rlgchPt->rlg_data[irlgd]->dat[1] != rlgchPt->rlg_data[irlgd]->dat[0]) {
	    hayb = 1;
	    Tshift[0][3] = v3b[0] * rlgchPt->rlg_data[irlgd]->dat[1];
	    Tshift[1][3] = v3b[1] * rlgchPt->rlg_data[irlgd]->dat[1];
	    Tshift[2][3] = v3b[2] * rlgchPt->rlg_data[irlgd]->dat[1];
	    p3d_mat4Mult(prev_pos,Tshift,Ttot);
	    p3d_xformPoint(Ttot,v3a,X1b); 
	    p3d_vectSub(X2, X1b, v1b);	
	    distance_b = sqrt(SQR(v1b[0])+SQR(v1b[1])+SQR(v1b[2]));
	    if(distance > distance_b) 
	      invertb = 1;
	    else
	      invertb = 0;	   
	  }
	  else 
	    hayb = 0;
	
	  /* analysis for shell.rext (we use nearest point) */
	
	  /* dof axis */
	  p3d_xformVect(prev_pos,v3b,v2);
	
	  /* distances to plane of displacement of next joint */
	  if((hayb)&&(invertb)) {
	    ang = acos(p3d_vectDotProd(v1b,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
	    dist_pp = fabs(distance_b * sin(ang));     /* pp -> perpendicular to axis */  
	    dist_p = fabs(distance_b * cos(ang));      /* p  -> parallel to axis      */
	    p3d_vectCopy(v1b, vref1);
	  }
	  else {
	    ang = acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
	    dist_pp = fabs(distance * sin(ang));
	    dist_p = fabs(distance * cos(ang));
	    p3d_vectCopy(v1, vref1);
	  }
	  
	  /* if dist_p > shell.rext -> can't close */
	  if(dist_p > rlgchPt->rlg_data[irlgd]->shell.rext) {
	    return(FALSE);
	  }
	
	  /* plane proyections */
	  rml_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rext)-SQR(dist_p));
	  
	  if(rlgchPt->rlg_data[irlgd]->shell.rint > dist_p) 
	    mMdif_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rint)-SQR(dist_p)); /* <- JUST FOR TEST */
	  else
	    mMdif_pr = 0.0;
	  
	  /* if dist_pp > rml_pr + lmax   -> can't close */
	  /* if lmin > rml_pr + dist_pp   -> can't close */
	  /* if dist_pp + lmax < mMdif_pr -> can't close */
	  if((dist_pp > rml_pr + rlgchPt->rlg_data[irlgd]->lmax)||
	     (rlgchPt->rlg_data[irlgd]->lmin > rml_pr + dist_pp)||
	     (dist_pp + rlgchPt->rlg_data[irlgd]->lmax < mMdif_pr)) {
	    return(FALSE);
	  }
	
	  /* NOTE: we calculate an interval in relation to the line Jij-pbj */
	  /* this interval is symmetric, so we can calculate the range 0-180 */
	  
	  /* calculate max_lim of interval */
	  if(dist_pp + rlgchPt->rlg_data[irlgd]->lmin <= rml_pr) {
	    max_lim = M_PI;
	  }
	  else {
	    if(dist_pp <= rml_pr) {
	      lij = rlgchPt->rlg_data[irlgd]->lmin;
	    }
	    else {
	      lrecto = sqrt(SQR(dist_pp)-SQR(rml_pr));
	      if((lrecto <= rlgchPt->rlg_data[irlgd]->lmax)&&(lrecto >= rlgchPt->rlg_data[irlgd]->lmin))
		lij = lrecto;
	      else {
		if(fabs(rlgchPt->rlg_data[irlgd]->lmax-lrecto) < fabs(rlgchPt->rlg_data[irlgd]->lmin-lrecto))
		  lij = rlgchPt->rlg_data[irlgd]->lmax;
		else
		  lij = rlgchPt->rlg_data[irlgd]->lmin;
	      }
	    }
	    max_lim = acos((SQR(dist_pp)+SQR(lij)-SQR(rml_pr))/(2*dist_pp*lij));
	  }
	
	  /* analysis of shell.rint (we use farthest point) */
	  /* REVISAR ESTO !!!!!!!!! */
	  /* calculate min_lim of interval */
	  if(mMdif_pr > 0.0) {
	    if((hayb)&&(!invertb)) {
	      ang = acos(p3d_vectDotProd(v1b,v2)/(p3d_vectNorm(v1b)*p3d_vectNorm(v2)));
	      dist_pp = fabs(distance_b * sin(ang));
	      dist_p = fabs(distance_b * cos(ang));
	      p3d_vectCopy(v1b, vref2);
	    }
	    else {
	      ang = acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
	      dist_pp = fabs(distance * sin(ang));
	      dist_p = fabs(distance * cos(ang));
	      p3d_vectCopy(v1b, vref2);
	    }
	    
	    if((dist_pp >= mMdif_pr + rlgchPt->rlg_data[irlgd]->lmin)||
	       (dist_pp + mMdif_pr <= rlgchPt->rlg_data[irlgd]->lmax)) {
	      min_lim = 0.0;
	    }
	    else {
	      if(dist_pp <= mMdif_pr) {
		lij = rlgchPt->rlg_data[irlgd]->lmax;
	      }
	      else {
		lrecto = sqrt(SQR(dist_pp)-SQR(mMdif_pr));
		if(fabs(rlgchPt->rlg_data[irlgd]->lmax-lrecto) >= fabs(rlgchPt->rlg_data[irlgd]->lmin-lrecto))
		  lij = rlgchPt->rlg_data[irlgd]->lmax;
		else
		  lij = rlgchPt->rlg_data[irlgd]->lmin;
	      }
	      min_lim = acos((SQR(dist_pp)+SQR(lij)-SQR(mMdif_pr))/(2*dist_pp*lij));
	    }
	  }
	  else 
	    min_lim = 0.0;
	  
	  /* calculate intervals */
	  p3d_jnt_get_dof_bounds(J,j,&vmin,&vmax);
	  if((min_lim <= 0.0)&&(max_lim >= M_PI)) {
#ifdef P3D_PLANNER
	    q[J->index_dof + j] = p3d_random(vmin,vmax); 
#endif
	  }
	  else {
	    refofsym = p3d_ref_in_jnt_frame(rlgchPt,irlgd,v1); 
	    if(min_lim <= 0.0) {
	      iv1 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
					  refofsym-max_lim,refofsym+max_lim,interv1);
	      if(!iv1)
		return(FALSE);
	      iv2 = 0;
	    }
	    else {
	      if(hayb)
		refofsym2 = p3d_ref_in_jnt_frame(rlgchPt,irlgd,vref2); 
	      else
		refofsym2 = refofsym;
	      iv1 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
					  refofsym2+min_lim,refofsym+max_lim,interv1);
	      iv2 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
					  refofsym-max_lim,refofsym2-min_lim,interv2);
	      if((!iv1)&&(!iv2))
		return(FALSE); 
	    }
	    /* random shoot */
	    q[J->index_dof + j] = p3d_random_in_several_intervals(iv1,iv2,interv1,interv2,rlgchPt->rlg_data[irlgd]->refval);
	  }
	  p3d_put_dof_into_range(J, j, &q[J->index_dof + j]);
	  p3d_jnt_set_dof(J,j,q[J->index_dof + j]);
	  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
	
	  break;
	
	  /* ----------------------------------------------- */
	case RJB2: 
	  /* placement of joint J and distance to pbj */
	  /* yet calculated for the case of faj */
	  if(J != Jfaj) {
	    p3d_jnt_get_cur_vect_point_before(J, X1);
	    p3d_vectSub(X2, X1, v1);	
	    distance = sqrt(SQR(v1[0])+SQR(v1[1])+SQR(v1[2]));       
	  }
			
	  /* analysis for shell.rext */
	
	  /* dof axis */
	  p3d_jnt_get_dof_cur_axis_before(J, j, v2);
	
	  /* distances to plane of displacement of next joint */
	  ang = acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
	  dist_pp = fabs(distance * sin(ang));     /* pp -> perpendicular to axis */  
	  dist_p = fabs(distance * cos(ang));      /* p  -> parallel to axis      */
	
	  /* if dist_p > shell.rext -> can't close */
	  if(dist_p > rlgchPt->rlg_data[irlgd]->shell.rext) {
	    return(FALSE);
	  }
	  
	  /* plane proyections */
	  rml_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rext)-SQR(dist_p));
	  
	  if(rlgchPt->rlg_data[irlgd]->shell.rint > dist_p) 
	    mMdif_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rint)-SQR(dist_p));
	  else
	    mMdif_pr = 0.0;
	  
	  /* in this case lmax and lmin are the stoked in dat */
	  /* if dist_pp > rml_pr + lmax   -> can't close */
	  /* if lmin > rml_pr + dist_pp   -> can't close */
	  /* if dist_pp + lmax < mMdif_pr -> can't close */
	  if((dist_pp > rml_pr + rlgchPt->rlg_data[irlgd]->dat[0])||
	     (rlgchPt->rlg_data[irlgd]->dat[1] > rml_pr + dist_pp)||
	     (dist_pp + rlgchPt->rlg_data[irlgd]->dat[0] < mMdif_pr)) {
	    return(FALSE);
	  }
	  
	  /* NOTE: we calculate an interval in relation to the line Jij-pbj */
	  /* this interval is symmetric, so we can calculate the range 0-180 */
	
	  /* calculate max_lim of interval */
	  if(dist_pp + rlgchPt->rlg_data[irlgd]->dat[1] <= rml_pr) {
	    max_lim = M_PI;
	    increfmax = 0.0;
	  }
	  else {
	    if(dist_pp <= rml_pr) {
	      lij = rlgchPt->rlg_data[irlgd]->dat[1];
	      increfmax = rlgchPt->rlg_data[irlgd]->dat[3];
	    }
	    else {
	      lrecto = sqrt(SQR(dist_pp)-SQR(rml_pr));
	      if((lrecto <= rlgchPt->rlg_data[irlgd]->dat[0])&&(lrecto >= rlgchPt->rlg_data[irlgd]->dat[1])) {
		lij = lrecto;
		/* calculate increfmax corresponding to this length */
		if(rlgchPt->rlg_data[irlgd]->dat[3] > 0.0) {
		  increfmax = acos(rlgchPt->rlg_data[irlgd]->lmax/lrecto);
		}
		else {
		  increfmax = - acos(rlgchPt->rlg_data[irlgd]->lmax/lrecto);
		}
	      }
	      else {
		if(fabs(rlgchPt->rlg_data[irlgd]->dat[0]-lrecto) < fabs(rlgchPt->rlg_data[irlgd]->dat[1]-lrecto)) {
		  lij = rlgchPt->rlg_data[irlgd]->dat[0];
		  increfmax = rlgchPt->rlg_data[irlgd]->dat[2];
		}
		else {
		  lij = rlgchPt->rlg_data[irlgd]->dat[1];
		  increfmax = rlgchPt->rlg_data[irlgd]->dat[3];
		}
	      }
	    }
	    max_lim = acos((SQR(dist_pp)+SQR(lij)-SQR(rml_pr))/(2*dist_pp*lij));
	  }
	
	  /* calculate min_lim of interval */
	  if(mMdif_pr > 0.0) {
	    if((dist_pp >= mMdif_pr + rlgchPt->rlg_data[irlgd]->dat[1])||
	       (dist_pp + mMdif_pr <= rlgchPt->rlg_data[irlgd]->dat[0])) {
	      min_lim = 0.0;
	      increfmin = 0.0;	   
	    }
	    else {
	      if(dist_pp <= mMdif_pr) {
		lij = rlgchPt->rlg_data[irlgd]->dat[0];
		increfmin = rlgchPt->rlg_data[irlgd]->dat[2];
	      }
	      else {
		lrecto = sqrt(SQR(dist_pp)-SQR(mMdif_pr));
		if(fabs(rlgchPt->rlg_data[irlgd]->dat[0]-lrecto) >= fabs(rlgchPt->rlg_data[irlgd]->dat[1]-lrecto)) {
		  lij = rlgchPt->rlg_data[irlgd]->dat[0];
		  increfmin = rlgchPt->rlg_data[irlgd]->dat[2];
		}
		else {
		  lij = rlgchPt->rlg_data[irlgd]->dat[1];
		  increfmin = rlgchPt->rlg_data[irlgd]->dat[3];
		}
	      }
	      min_lim = acos((SQR(dist_pp)+SQR(lij)-SQR(mMdif_pr))/(2*dist_pp*lij));
	    }
	  }
	  else {
	    min_lim = 0.0;
	    increfmin = 0.0;	   
	  }     
	
	  /* calculate intervals */
	  p3d_jnt_get_dof_bounds(J,j,&vmin,&vmax);
	  if((min_lim <= 0.0)&&(max_lim >= M_PI)) {
#ifdef P3D_PLANNER
	    q[J->index_dof + j] = p3d_random(vmin,vmax); 
#endif
	  }
	  else {
	    refofsym = p3d_ref_in_jnt_frame(rlgchPt,irlgd,v1); 
	    /* POR EL MOMENTO NO CONSIDERO min_lim PORQUE COMPLICA LOS CALCULOS */
	    /* HABRIA QUE REPROGRAMAR LA FUNCION p3d_inter_ang_regions */
	    /* 	 if(min_lim <= 0.0) { */
	    iv1 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+increfmax+vmin,rlgchPt->rlg_data[irlgd]->refval+increfmax+vmax,
					refofsym-max_lim,refofsym+max_lim,interv1);
	    if(!iv1)
	      return(FALSE);
	    iv2 = 0;
	    /*          } */
	    /* 	 else { */
	    
	    /* 	   iv1 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
		   refofsym+min_lim,refofsym+max_lim,interv1); */
	    /* 	   iv2 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
		   refofsym-max_lim,refofsym-min_lim,interv2); */
	    /* 	   if((!iv1)&&(!iv2)) */
	    /* 	     return(FALSE);  */
	    /* 	 } */
	    /* random shoot */
	    q[J->index_dof + j] = p3d_random_in_several_intervals(iv1,iv2,interv1,interv2,rlgchPt->rlg_data[irlgd]->refval);
	  }
	  p3d_put_dof_into_range(J, j, &q[J->index_dof + j]);
	  p3d_jnt_set_dof(J,j,q[J->index_dof + j]);
	  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
	
	  break;
	  
	  /* ----------------------------------------------- */
	case RJB3: 
	  /* point used in generation */
	  p3d_jnt_get_vect_point(J, v3a);
	  p3d_jnt_get_dof_axis(J, j, v3b);
	  /* we suppose that the axis is NORMALIZED */
	  Tshift[0][3] = v3b[0] * rlgchPt->rlg_data[irlgd]->lmax;
	  Tshift[1][3] = v3b[1] * rlgchPt->rlg_data[irlgd]->lmax;
	  Tshift[2][3] = v3b[2] * rlgchPt->rlg_data[irlgd]->lmax;
	  p3d_mat4Mult(prev_pos,Tshift,Ttot);
	  p3d_xformPoint(Ttot,v3a,X1); 
	  p3d_vectSub(X2, X1, v1);	
	  distance = sqrt(SQR(v1[0])+SQR(v1[1])+SQR(v1[2]));
	  
	  /* analysis for shell.rext */
	
	  /* dof axis */
	  p3d_xformVect(prev_pos,v3b,v2);
	
	  /* distances to plane of displacement of next joint */
	  ang = acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
	  dist_pp = fabs(distance * sin(ang));     /* pp -> perpendicular to axis */  
	  dist_p = fabs(distance * cos(ang));      /* p  -> parallel to axis      */
	  
	  /* if dist_p > shell.rext -> can't close */
	  if(dist_p > rlgchPt->rlg_data[irlgd]->shell.rext) {
	    return(FALSE);
	  }
	  
	  /* plane proyections */
	  rml_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rext)-SQR(dist_p));
	  
	  if(rlgchPt->rlg_data[irlgd]->shell.rint > dist_p) 
	    mMdif_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rint)-SQR(dist_p));
	  else
	    mMdif_pr = 0.0;
	  
	  /* if dist_pp > rml_pr + lmax   -> can't close */
	  /* if lmin > rml_pr + dist_pp   -> can't close */
	  /* if dist_pp + lmax < mMdif_pr -> can't close */
	  if((dist_pp > rml_pr + rlgchPt->rlg_data[irlgd]->lmax)||
	     (rlgchPt->rlg_data[irlgd]->lmin > rml_pr + dist_pp)||
	     (dist_pp + rlgchPt->rlg_data[irlgd]->lmax < mMdif_pr)) {
	    return(FALSE);
	  }
	   
	  /* NOTE: we calculate an interval in relation to the line Jij-pbj */
	  /* this interval is symmetric, so we can calculate the range 0-180 */
	  
	  /* calculate max_lim of interval */
	  if(dist_pp + rlgchPt->rlg_data[irlgd]->lmin <= rml_pr) {
	    max_lim = M_PI;
	  }
	  else {
	    if(dist_pp <= rml_pr) {
	      lij = rlgchPt->rlg_data[irlgd]->lmin;
	    }
	    else {
	      lrecto = sqrt(SQR(dist_pp)-SQR(rml_pr));
	      if((lrecto <= rlgchPt->rlg_data[irlgd]->lmax)&&(lrecto >= rlgchPt->rlg_data[irlgd]->lmin))
		lij = lrecto;
	      else {
		if(fabs(rlgchPt->rlg_data[irlgd]->lmax-lrecto) < fabs(rlgchPt->rlg_data[irlgd]->lmin-lrecto))
		  lij = rlgchPt->rlg_data[irlgd]->lmax;
		else
		  lij = rlgchPt->rlg_data[irlgd]->lmin;
	      }
	    }
	    max_lim = acos((SQR(dist_pp)+SQR(lij)-SQR(rml_pr))/(2*dist_pp*lij));
	  }
	  
	  /* calculate min_lim of interval */
	  if(mMdif_pr > 0.0) {
	    if((dist_pp >= mMdif_pr + rlgchPt->rlg_data[irlgd]->lmin)||
	       (dist_pp + mMdif_pr <= rlgchPt->rlg_data[irlgd]->lmax)) {
	      min_lim = 0.0;
	    }
	    else {
	      if(dist_pp <= mMdif_pr) {
		lij = rlgchPt->rlg_data[irlgd]->lmax;
	      }
	      else {
		lrecto = sqrt(SQR(dist_pp)-SQR(mMdif_pr));
		if(fabs(rlgchPt->rlg_data[irlgd]->lmax-lrecto) >= fabs(rlgchPt->rlg_data[irlgd]->lmin-lrecto))
		  lij = rlgchPt->rlg_data[irlgd]->lmax;
		else
		  lij = rlgchPt->rlg_data[irlgd]->lmin;
	      }
	      min_lim = acos((SQR(dist_pp)+SQR(lij)-SQR(mMdif_pr))/(2*dist_pp*lij));
	    }
	  }
	  else 
	    min_lim = 0.0;
	  
	  /* calculate intervals */
	  p3d_jnt_get_dof_bounds(J,j,&vmin,&vmax);
	  if((min_lim <= 0.0)&&(max_lim >= M_PI)) {
#ifdef P3D_PLANNER
	    q[J->index_dof + j] = p3d_random(vmin,vmax); 
#endif
	  }
	  else {
	    refofsym = p3d_ref_in_jnt_frame(rlgchPt,irlgd,v1); 
	    if(min_lim <= 0.0) {
	      iv1 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
					  refofsym-max_lim,refofsym+max_lim,interv1);
	      if(!iv1)
		return(FALSE); 
	      iv2 = 0;
	    }
	    else {
	      iv1 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
					  refofsym+min_lim,refofsym+max_lim,interv1);
	      iv2 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
					  refofsym-max_lim,refofsym-min_lim,interv2);
	      if((!iv1)&&(!iv2))
		return(FALSE); 
	    }
	  /* random shoot */
	    q[J->index_dof + j] = p3d_random_in_several_intervals(iv1,iv2,interv1,interv2,rlgchPt->rlg_data[irlgd]->refval);
	  }
	  p3d_put_dof_into_range(J, j, &q[J->index_dof + j]);
	  p3d_jnt_set_dof(J,j,q[J->index_dof + j]);
	  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
	
	  break;
	
	  /* ----------------------------------------------- */
	case RJC1: 
	  /* point used in generation */
	  p3d_jnt_get_vect_point(J, v3a);
	  p3d_jnt_get_dof_axis(J, j, v3b);
	  /* we suppose that the axis is NORMALIZED */
	  Tshift[0][3] = v3b[0] * rlgchPt->rlg_data[irlgd]->lmax;
	  Tshift[1][3] = v3b[1] * rlgchPt->rlg_data[irlgd]->lmax;
	  Tshift[2][3] = v3b[2] * rlgchPt->rlg_data[irlgd]->lmax;
	  p3d_mat4Mult(prev_pos,Tshift,Ttot);
	  p3d_xformPoint(Ttot,v3a,X1); 
	  p3d_vectSub(X2, X1, v1);	
	  distance = sqrt(SQR(v1[0])+SQR(v1[1])+SQR(v1[2]));
	
	  /* we just treat for rml */
	
	  if(distance > rlgchPt->rlg_data[irlgd]->shell.rext + rlgchPt->rlg_data[irlgd]->dat[0]){
	    return(FALSE);
	  }
	
	  if(rlgchPt->rlg_data[irlgd]->shell.rext >= sqrt(SQR(rlgchPt->rlg_data[irlgd]->dat[1])+SQR(distance))) {       
	    theta = 10;  /* > M_PI/2 */
	  }
	  else {
	    /* OJO : CONSIDERO QUE lmin - distance !!!!!!! */
	    lrecto = sqrt(fabs(SQR(rlgchPt->rlg_data[irlgd]->shell.rext)-SQR(distance)));
	    if((lrecto <= rlgchPt->rlg_data[irlgd]->dat[0])&&(lrecto >= rlgchPt->rlg_data[irlgd]->dat[1]))
	      lij = lrecto;
	    else {
	      if((rlgchPt->rlg_data[irlgd]->dat[0] < lrecto)||(rlgchPt->rlg_data[irlgd]->dat[1] < lrecto))
		lij = rlgchPt->rlg_data[irlgd]->dat[0];
	      else
		lij = rlgchPt->rlg_data[irlgd]->dat[1];
	    }	 
	    theta = acos((SQR(distance)+SQR(lij)-SQR(rlgchPt->rlg_data[irlgd]->shell.rext))/(2*distance*lij)); 
	  }
	
	
	  /* REVISAR CALCULOS DE ANGULOS */
	  /* OJO : NO ES >M_PI/2 , ES >M_PI !!!! */
	  /* REFERENCIA PARA LA GENERACION ???!!!! */
	
	  if(theta >= M_PI/2) {
	    max_lim = 10;  /* > M_PI/2 */
	  }
	  else {
	    /* dof axis */
	    p3d_xformVect(prev_pos,v3b,v2);
	  
	    /* alpha = angle line X2-X1 to axis */
	    ang = acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
	    alpha = fabs(ang - M_PI/2);
	    
	    if((alpha + theta) >= M_PI/2) {
	      max_lim = 10;
	    }
	    else {	   
	      max_lim =  acos(cos(alpha)*cos(theta));
	    }
	  }	   

	  /* calculate intervals */
	  p3d_jnt_get_dof_bounds(J,j,&vmin,&vmax);
	  if(max_lim >= M_PI/2) {
#ifdef P3D_PLANNER
	    q[J->index_dof + j] = p3d_random(vmin,vmax); 
#endif
	  }
	  else {
	    refofsym = p3d_ref_in_jnt_frame(rlgchPt,irlgd,v1); 
	    if(refofsym > M_PI)
	      refofsym -= M_PI;     /* range 0 - 360 */
	    ang = rlgchPt->rlg_data[irlgd]->refval;
	    /* two symetric intervals */
	    /* reference is a perpendicular axis -> + - M_PI/2 */
	    iv1 = p3d_inter_ang_regions(ang+vmin,ang+vmax,refofsym+M_PI/2-max_lim,refofsym+M_PI/2+max_lim,interv1);
	    iv2 = p3d_inter_ang_regions(ang+vmin,ang+vmax,refofsym-M_PI/2-max_lim,refofsym-M_PI/2+max_lim,interv2);
	    if((!iv1)&&(!iv2)) {
	      return(FALSE); 
	    }
	    /* random shoot */
	    q[J->index_dof + j] = p3d_random_in_several_intervals(iv1,iv2,interv1,interv2,rlgchPt->rlg_data[irlgd]->refval);
	  }
	  p3d_put_dof_into_range(J, j, &q[J->index_dof + j]);
	  p3d_jnt_set_dof(J,j,q[J->index_dof + j]);
	  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
	
	  break;
	
	  /* ----------------------------------------------- */
	case RJD1: 
	case RJD2: 
	  /* placement of joint J and distance to pbj */
	  p3d_jnt_get_vect_point(J, v3a);
	  p3d_jnt_get_dof_axis(J, j, v3b);	
	  /* we suppose that the axis is NORMALIZED */
	  Tshift[0][3] = v3b[0] * rlgchPt->rlg_data[irlgd]->dat[0];
	  Tshift[1][3] = v3b[1] * rlgchPt->rlg_data[irlgd]->dat[0];
	  Tshift[2][3] = v3b[2] * rlgchPt->rlg_data[irlgd]->dat[0];
	  p3d_mat4Mult(prev_pos,Tshift,Ttot);
	  p3d_xformPoint(Ttot,v3a,X1); 
	  p3d_vectSub(X2, X1, v1);	
	  distance = sqrt(SQR(v1[0])+SQR(v1[1])+SQR(v1[2]));       

	  /* analysis for shell.rext */
	  
	  /* dof axis */
	  p3d_xformVect(prev_pos,v3b,v2);
	
	  /* distances to plane of displacement of next joint */
	  ang = acos(p3d_vectDotProd(v1,v2)/(p3d_vectNorm(v1)*p3d_vectNorm(v2)));
	  dist_pp = fabs(distance * sin(ang));     /* pp -> perpendicular to axis */  
	  dist_p = fabs(distance * cos(ang));      /* p  -> parallel to axis      */
	
	  /* if dist_p > shell.rext -> can't close */
	  if(dist_p > rlgchPt->rlg_data[irlgd]->shell.rext) {
	    return(FALSE);
	  }
	  
	  /* plane proyections */
	  rml_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rext)-SQR(dist_p));
	
	  if(rlgchPt->rlg_data[irlgd]->shell.rint > dist_p) 
	    mMdif_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rint)-SQR(dist_p));
	  else
	    mMdif_pr = 0.0;
	
	  /* if dist_pp > rml_pr + lmax   -> can't close */
	  /* if lmin > rml_pr + dist_pp   -> can't close */
	  /* if dist_pp + lmax < mMdif_pr -> can't close */
	  /* NOTE : in this case lmax = lmin = dat[1] */
	  if((dist_pp > rml_pr + rlgchPt->rlg_data[irlgd]->dat[1])||
	     (rlgchPt->rlg_data[irlgd]->dat[1] > rml_pr + dist_pp)||
	     (dist_pp + rlgchPt->rlg_data[irlgd]->dat[1] < mMdif_pr)) {
	    return(FALSE);
	  }
	
	  /* NOTE: we calculate an interval in relation to the line Jij-pbj */
	  /* this interval is symmetric, so we can calculate the range 0-180 */
	  
	  /* calculate max_lim of interval */
	  if(dist_pp + rlgchPt->rlg_data[irlgd]->dat[1] <= rml_pr) {
	    max_lim = M_PI;
	  }
	  else {
	    lij = rlgchPt->rlg_data[irlgd]->dat[1];
	     
	    max_lim = acos((SQR(dist_pp)+SQR(lij)-SQR(rml_pr))/(2*dist_pp*lij));
	  }
	  
	  /* calculate min_lim of interval */
	  if(mMdif_pr > 0.0) {
	    if((dist_pp >= mMdif_pr + rlgchPt->rlg_data[irlgd]->dat[1])||
	       (dist_pp + mMdif_pr <= rlgchPt->rlg_data[irlgd]->dat[1])) {
	      min_lim = 0.0;
	    }
	    else {
	      lij = rlgchPt->rlg_data[irlgd]->dat[1];
	      
	      min_lim = acos((SQR(dist_pp)+SQR(lij)-SQR(mMdif_pr))/(2*dist_pp*lij));
	    }
	  }
	  else 
	    min_lim = 0.0;
	  
	  /* calculate intervals */
	  p3d_jnt_get_dof_bounds(J,j,&vmin,&vmax);
	  if((min_lim <= 0.0)&&(max_lim >= M_PI)) {
#ifdef P3D_PLANNER
	    q[J->index_dof + j] = p3d_random(vmin,vmax); 
#endif
	  }
	  else {
	    refofsym = p3d_ref_in_jnt_frame(rlgchPt,irlgd,v1); 
	    if(min_lim <= 0.0) {
	      iv1 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
					  refofsym-max_lim,refofsym+max_lim,interv1);
	      if(!iv1)
		return(FALSE);
	      iv2 = 0;
	    }
	    else {
	      iv1 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
					  refofsym+min_lim,refofsym+max_lim,interv1);
	      iv2 = p3d_inter_ang_regions(rlgchPt->rlg_data[irlgd]->refval+vmin,rlgchPt->rlg_data[irlgd]->refval+vmax,
					  refofsym-max_lim,refofsym-min_lim,interv2);
	      if((!iv1)&&(!iv2))
		return(FALSE); 
	    }
	    /* random shoot */
	    q[J->index_dof + j] = p3d_random_in_several_intervals(iv1,iv2,interv1,interv2,rlgchPt->rlg_data[irlgd]->refval);
	  }
	  p3d_put_dof_into_range(J, j, &q[J->index_dof + j]);
	  p3d_jnt_set_dof(J,j,q[J->index_dof + j]);
	  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
	
	  break;
	
	}
      }

      irlgd++;
    
      if(PRINTPROC_RLG)
	printf("test %d OK\n",irlgd); 
    }
    
    if(J->next_jnt != NULL) {
      J = J->next_jnt[0];
      if (J!=NULL) {
	p3d_matInvertXform(J->pos0, T1);
	p3d_matMultXform(J->abs_pos, T1, pos);
	if (J->prev_jnt!=NULL) {
	  p3d_matInvertXform(J->prev_jnt->pos0, T1);
	  p3d_matMultXform(J->prev_jnt->abs_pos, T1, prev_pos);
	}
      }
    } else
      J = NULL;
    
  } /* end of main loop */
  
  return(TRUE);
}


/************************************************************/


static int p3d_generate_holonom_base_conf(p3d_cntrt *ct, configPt q)
{ 
  p3d_jnt *Jbase;

  Jbase = ct->rlgPt->rlgbsPt->basejntPt;

  switch(Jbase->type) {
  case P3D_PLAN:
    return (p3d_generate_holonom_plane_basejnt_conf(ct,q));
    break;
  case P3D_FREEFLYER:
    return (p3d_generate_holonom_freeflyer_basejnt_conf(ct,q));
    break;
  default:
    return FALSE;
  }
}


static int p3d_generate_holonom_plane_basejnt_conf(p3d_cntrt *ct, configPt q)
{
  /* NOTA : considero que todo esta bien en la inicializacion 
            (todo a 0 para las referencias ...)
	    Habria que usar datos el p3d_jnt para hacer algo mas general -> COMO ???
	    (porsiciones y orintaciones en el modelo ...)
  */

  p3d_jnt *Jbase,*Jend;
  p3d_matrix4 rotmat;
  p3d_vector3 zaxis;
  p3d_rlg_base *rlgbsPt;
  double maxdist,mindist,maxdist_p,mindist_p;
  double x_base,y_base,z_base,rz_base;
  double x_manip,y_manip,z_manip;
  double z_end,dz;
  double xmin,xmax,ymin,ymax,rzmin,rzmax;
  int go_on, tries;
  p3d_matrix4 Tgrip;
  
  rlgbsPt = ct->rlgPt->rlgbsPt;
  Jbase = rlgbsPt->basejntPt;
  Jend = ct->actjnts[0];

  zaxis[0] = 0.0; zaxis[1] = 0.0; zaxis[2] = 1.0;

  maxdist = rlgbsPt->rmax;
  mindist = rlgbsPt->rmin;

  z_base = rlgbsPt->basejntPt->pos0[2][3];
  z_manip = z_base + rlgbsPt->Tbm[2][3];
  z_end = Jend->abs_pos[2][3];

  dz = fabs(z_end - z_manip);
  if(dz > maxdist)
    return(FALSE);
  maxdist_p = sqrt(SQR(maxdist) - SQR(dz));
  if(dz > mindist)  
    mindist_p = 0.0;
  else 
    mindist_p = sqrt(SQR(mindist) - SQR(dz));
  
  p3d_mat4Mult(Jend->abs_pos,ct->Tatt,Tgrip);
 
  p3d_jnt_get_dof_rand_bounds(Jbase,0,&xmin,&xmax);
  p3d_jnt_get_dof_rand_bounds(Jbase,1,&ymin,&ymax);
  p3d_jnt_get_dof_rand_bounds(Jbase,2,&rzmin,&rzmax);
  go_on = 1;
  tries = 0;
  do {
    /* gripper orientation :
       if the z axis of Tgrip is (anti)parallel to the z axis of the base_jnt (== absolute z axis)
         then use the full sphere
       else use the vertical half-sphere perpendicular to the plane of this axis
       
       WARNING !!! : this method is valid for 6R arm
                     what happens with other cases ?
    */    
    if((SQR(Tgrip[0][2])+SQR(Tgrip[1][2])) <= EPS6)
      random_point_on_disc(maxdist_p,mindist_p,&x_manip,&y_manip);
    else
      random_point_on_halfdisc(maxdist_p,mindist_p,Tgrip,&x_manip,&y_manip);      
    x_manip += Jend->abs_pos[0][3];
    y_manip += Jend->abs_pos[1][3];

#ifdef P3D_PLANNER
    /* random base rotation */
    rz_base = p3d_random(rzmin,rzmax);
#endif

    /* compute base position */
    // HERE WE SHOULD USE DATA FROM INITIALIZATION !!!
    p3d_mat4Rot(rotmat,zaxis,rz_base);
    x_base = x_manip - (rotmat[0][0]*rlgbsPt->Tbm[0][3] + rotmat[0][1]*rlgbsPt->Tbm[1][3] + rotmat[0][2]*rlgbsPt->Tbm[2][3]);
    y_base = y_manip - (rotmat[1][0]*rlgbsPt->Tbm[0][3] + rotmat[1][1]*rlgbsPt->Tbm[1][3] + rotmat[1][2]*rlgbsPt->Tbm[2][3]);

    /* the base motion limits are tested */ 
    if((x_base >= xmin)&&(x_base <= xmax)&&(y_base >= ymin)&&(y_base <= ymax))
      go_on = 0;
    tries ++;
  } while(go_on && (tries < TRIES_BEFORE_RESHOOT));

  if(go_on)
    return(FALSE);

  /* set base jnt config */
  q[Jbase->index_dof] = x_base;
  q[Jbase->index_dof + 1] = y_base;
  q[Jbase->index_dof + 2] = rz_base;

  return(TRUE);
}


static int p3d_generate_holonom_freeflyer_basejnt_conf(p3d_cntrt *ct, configPt q)
{

  // HACER !!!
  /* usar p3d_mat4ExtractPosReverseOrder ... ??? */

  return TRUE;     
}



/************************************************************/


static void random_point_on_disc(double rmax, double rmin, double *x, double *y)
{
  double r,theta;
#ifdef P3D_PLANNER
  r = p3d_random(rmin,rmax);
#endif
#ifdef P3D_PLANNER
  theta = p3d_random(0.0,(2.0*M_PI));
#endif

  *x = r * cos(theta);
  *y = r * sin(theta);
}


static void random_point_on_halfdisc(double rmax, double rmin, p3d_matrix4 Tgrip, double *x, double *y)
{
  double z_x,z_y;
  double r,theta,theta_z;

  z_x = Tgrip[0][2];
  z_y = Tgrip[1][2];
#ifdef P3D_PLANNER
  r = p3d_random(rmin,rmax);
#endif
#ifdef P3D_PLANNER
  theta = p3d_random(-M_PI/2,M_PI/2);
#endif
  theta_z = atan2(z_y,z_x) - M_PI;
  theta += theta_z;

  *x = r * cos(theta);
  *y = r * sin(theta);
}


/************************************************************/
/************************************************************/


/* Returns angle of reference of the interval in relation to joint frame */
static double p3d_ref_in_jnt_frame(p3d_rlg_chain *rlgchPt, int irlgd, p3d_vector3 vref)
{
 p3d_vector4 v3a,v3b;
 p3d_matrix4 Ttot,Tinv,Minv,pos;
 double refofsym;

 /* reference line Jij-pbj */
 p3d_matInvertXform(rlgchPt->rlg_data[irlgd]->jnt->prev_jnt->pos0, Minv);
 p3d_matMultXform(rlgchPt->rlg_data[irlgd]->jnt->prev_jnt->abs_pos, Minv, pos);
 p3d_mat4Mult(pos,rlgchPt->rlg_data[irlgd]->Tref,Ttot);
 p3d_matInvertXform(Ttot, Tinv);
 v3a[0] = vref[0];
 v3a[1] = vref[1];
 v3a[2] = vref[2];
 v3a[3] = 0.0;
 p3d_matvec4Mult(Tinv,v3a,v3b);
 refofsym = atan2(v3b[1],v3b[0]);
 if(refofsym < 0.0)
     refofsym = (2.0*M_PI) + refofsym;     /* range 0 - 360 */

 return(refofsym);
}


/* Function calculating the intersection of two anguar regions */
/* Returns 0 if null intersection */
int p3d_inter_ang_regions(double ar1m, double ar1M, double ar2m, double ar2M, p3d_matrix2 sol)
{
 double ar1m_l, ar1M_l, ar2m_l, ar2M_l;
 double offset; 
 double auxang; 

 /* flag to indicate just one interval */
 sol[1][1] = 23.0;

 /* compare in 0-360 -> local variables */
 if((ar1m < 0.0)||(ar2m < 0.0)) {
   if(ar1m < ar2m)
     offset = - ar1m;
   else
     offset = - ar2m;     
   ar1m_l = ar1m + offset;
   ar1M_l = ar1M + offset;
   ar2m_l = ar2m + offset;
   ar2M_l = ar2M + offset;
 }
 else {
   offset = 0.0;
   ar1m_l = ar1m;
   ar1M_l = ar1M;
   ar2m_l = ar2m;
   ar2M_l = ar2M;
 }   

 if(PRINTPROC_RLG) {
   printf("ar1m = %f, ar1M = %f\n",ar1m,ar1M);
   printf("ar1m_l = %f, ar1M_l = %f\n",ar1m_l,ar1M_l);
   printf("ar2m = %f, ar2M = %f\n",ar2m,ar2M);
   printf("ar2m_l = %f, ar2M_l = %f\n",ar2m_l,ar2M_l);
 } 
 
 auxang = 0.0;

 if(ar1M_l >= ar2M_l) {
   if(ar1M_l > (2.0*M_PI)) {
     auxang = ar1M_l - (2.0*M_PI);
     ar1M_l -= auxang;
     ar2M_l -= auxang;
     ar1m_l -= auxang;
     ar2m_l -= auxang;
   }
   
   if(ar2m_l >= 0.0) {
     if(ar1m_l > ar2M_l)
       return(0);  /* out of range */
     sol[0][1] = ar2M_l - offset + auxang;
     if(ar1m_l <= ar2m_l)
       sol[0][0] = ar2m_l - offset + auxang;
     else
       sol[0][0] = ar1m_l - offset + auxang;
   }
   else {
     ar2m_l += (2.0*M_PI);    
     if(ar2m_l >= ar1M_l) {
       if(ar1m_l > ar2M_l)
	 return(0);  /* out of range */
       sol[0][1] = ar2M_l - offset + auxang;
       sol[0][0] = ar1m_l - offset + auxang;
     }	 
     else {
       sol[0][1] = ar1M_l - offset + auxang;
       sol[0][0] = ar2m_l - offset + auxang;
       if(ar2M_l > ar1m_l) {
	 sol[1][1] = ar2M_l - offset + auxang;
	 sol[1][0] = ar1m_l - offset + auxang;
       }
     }
   }    
 }
 else {  /* ar2M_l > ar1M_l */
   if(ar2M_l > (2.0*M_PI)) {
     auxang = ar2M_l - (2.0*M_PI);  
     ar1M_l -= auxang;
     ar2M_l -= auxang;
     ar1m_l -= auxang;
     ar2m_l -= auxang;
   }
   
   if(ar1m_l >= 0.0) {
     if(ar2m_l > ar1M_l)
       return(0);  /* out of range */
     sol[0][1] = ar1M_l - offset + auxang;
     if(ar1m_l <= ar2m_l)
       sol[0][0] = ar2m_l - offset + auxang;
     else
       sol[0][0] = ar1m_l - offset + auxang;
   }
   else {
     ar1m_l += (2.0*M_PI);
     if(ar1m_l >= ar2M_l) {
       if(ar2m_l > ar1M_l)
	 return(0);  /* out of range */
       sol[0][1] = ar1M_l - offset + auxang;
       sol[0][0] = ar2m_l - offset + auxang;
     }	 
     else {
       sol[0][1] = ar2M_l - offset + auxang;
       sol[0][0] = ar1m_l - offset + auxang;
       if(ar1M_l > ar2m_l) {
	 sol[1][1] = ar1M_l - offset + auxang;
	 sol[1][0] = ar2m_l - offset + auxang;
       }
     }
   }
 }

 /* order intervals */
 if((sol[1][1] != 23.0) && (sol[1][0] < sol[0][0])) {
   auxang = sol[0][0];
   sol[0][0] = sol[1][0];
   sol[1][0] = auxang;
   auxang = sol[0][1];
   sol[0][1] = sol[1][1];
   sol[1][1] = auxang;
 }

 /* put into (-2pi,2pi) */
 if(sol[0][0] < - (2.0*M_PI)) {
   sol[0][0] += (2.0*M_PI);
   sol[0][1] += (2.0*M_PI);
   if(sol[1][1] != 23.0) {
     sol[1][0] += (2.0*M_PI);
     sol[1][1] += (2.0*M_PI);
   }
 }
 else if((sol[1][1] != 23.0) && (sol[1][1] > (2.0*M_PI))) {
   sol[0][0] -= (2.0*M_PI);
   sol[0][1] -= (2.0*M_PI);
   sol[1][0] -= (2.0*M_PI);
   sol[1][1] -= (2.0*M_PI);
 }
 else if(sol[0][1] > (2.0*M_PI)) {
   sol[0][0] -= (2.0*M_PI);
   sol[0][1] -= (2.0*M_PI);
 }

 /* eliminate if consecutive */
 if(sol[1][1] != 23.0) {
   if(fabs(sol[0][1] - sol[1][0]) < 2*EPS6) {
     sol[0][1] = sol[1][1];
     sol[1][1] = 23.0;
   } 
   else if((sol[1][0] - sol[0][1]) > ((2.0*M_PI) - 2*EPS6)) {
     if(sol[0][0] < 0.0) {
       sol[0][0] += (2.0*M_PI);   
       sol[0][1] = sol[1][1];
     }
     else {
       sol[0][1] = sol[1][1] - (2.0*M_PI);
     }
     sol[1][1] = 23.0;
   }
   else if((fabs(sol[1][1] - sol[0][0]) > ((2.0*M_PI) - EPS6)) &&
	   (fabs(sol[1][1] - sol[0][0]) < ((2.0*M_PI) + EPS6))) {
     if(sol[0][0] < 0.0) {
       sol[0][0] = sol[1][0];
       sol[0][1] += (2.0*M_PI);   
     }
     else  {
       sol[0][0] = sol[1][0] - (2.0*M_PI);
     }
     sol[1][1] = 23.0;
   }
 }

 if(PRINTPROC_RLG) {
   printf("S00 = %f, S01 = %f\n",sol[0][0],sol[0][1]);
   printf("S10 = %f, S11 = %f\n",sol[1][0],sol[1][1]);
 }

 return(1);
}


/* Function calculating the intersection of two anguar regions - version II */
/* Returns 0 if null intersection */
int p3d_inter_ang_regions_II(double ar1m, double ar1M, double ar2m, double ar2M, p3d_matrix2 sol)
{
  double auxd;

  /* flag to indicate just one interval */
  sol[1][1] = 23.0; 

  /* case one interval is full 2pi*/
  if((ar1M - ar1m) >= ((2.0*M_PI) - EPS6)) {
    sol[0][1] = ar2M;
    sol[0][0] = ar2m;
    return TRUE;
  }
  if((ar2M - ar2m) >= ((2.0*M_PI) - EPS6)) {
    sol[0][1] = ar1M;
    sol[0][0] = ar1m;
    return TRUE;
  }

  /* compare in + */
  if(ar1m < 0.0) {
    ar1m += (2.0*M_PI);
    ar1M += (2.0*M_PI);
  }
  if(ar2m < 0.0) {
    ar2m += (2.0*M_PI);
    ar2M += (2.0*M_PI);
  }

  /* order */
  if(ar2m < ar1m) {
    auxd = ar1m;
    ar1m = ar2m;
    ar2m = auxd;
    auxd = ar1M;
    ar1M = ar2M;
    ar2M = auxd;
  }
  
  /* min at 0 (minval = ar1m) */
  ar1M -= ar1m;
  ar2m -= ar1m;
  ar2M -= ar1m;

  /* intersect */
  if(ar2M <= (2.0*M_PI)) {
    if(ar2m > ar1M)
      return FALSE;
    sol[0][0] = ar2m + ar1m;
    if(ar2M < ar1M)
      sol[0][1] = ar2M + ar1m;
    else
      sol[0][1] = ar1M + ar1m;
    return TRUE;
  }
  else {  // ar2M > (2.0*M_PI)
    ar2M -= (2.0*M_PI);    
    sol[0][0] = ar1m;
    if(ar2M >= ar1M)
      sol[0][1] = ar1M + ar1m;
    else {
      sol[0][1] = ar2M + ar1m;
      if(ar2m <= ar1M) {
	sol[1][0] = ar2m + ar1m;
	sol[1][1] = ar1M + ar1m;
      }
    }
    return TRUE;
  }
}


/* computes de intersection of two sets of angular intervals */
/* returns the  number of result intervals */
/* the result is copied in the firts interval set */
int p3d_merge_ang_intervals(int nint1, int nint2, double *intmin1, double *intmax1, double *intmin2, double *intmax2)
{
  int nint;
  double intmin[4],intmax[4];
  int i,j;
  p3d_matrix2 solint;
  
  nint = 0;
  for(i=0; i<nint1; i++) {
    for(j=0; j<nint2; j++) {
      if(p3d_inter_ang_regions_II(intmin1[i],intmax1[i],intmin2[j],intmax2[j],solint)) {
	intmin[nint] = solint[0][0];
	intmax[nint] = solint[0][1];
	nint++;
	if(solint[1][1] != 23.0) {
	  intmin[nint] = solint[1][0];
	  intmax[nint] = solint[1][1];
	  nint++;
	}
      }
    }
  }
  
  if(nint > 4) {
    printf("ERROR : ANGULAR INTERSECTION GIVES MORE THAN 4 INTERVALS !!!\n");
  }

  for(i=0; i<nint; i++) {
    intmin1[i] = intmin[i];
    intmax1[i] = intmax[i];
  }
  return nint;
}


/* Returs random configuration of joint in some intervals */
double p3d_random_in_several_intervals(int iv1, int iv2, p3d_matrix2 interv1, p3d_matrix2 interv2, double refval)
{
 double valor; 

 if(iv1&&(!iv2)) {
   if(interv1[1][1] == 23.0) {
#ifdef P3D_PLANNER
     valor = (double) p3d_random(interv1[0][0],interv1[0][1]); 
#endif
   }
   else {
     valor = p3d_random_in_two_intervals(interv1[0][0],interv1[0][1],interv1[1][0],interv1[1][1]);
   }
   valor -= refval;
   return(valor);
 }
 else if((!iv1)&&iv2) {
   if(interv2[1][1] == 23.0) {
#ifdef P3D_PLANNER
     valor = p3d_random(interv2[0][0],interv2[0][1]); 
#endif
   }
   else {
     valor = p3d_random_in_two_intervals(interv2[0][0],interv2[0][1],interv2[1][0],interv2[1][1]);
   }
   valor -= refval;
   return(valor);   
 }
 
 if((interv1[1][1] == 23.0)&&(interv2[1][1] == 23.0)) {
   valor = p3d_random_in_two_intervals(interv1[0][0],interv1[0][1],interv2[0][0],interv2[0][1]);
 }
 else {
   if(interv1[1][1] == 23.0) {
     valor = p3d_random_in_three_intervals(interv1[0][0],interv1[0][1],interv2[0][0],interv2[0][1],interv2[1][0],interv2[1][1]);
   }
   else { /* interv2[1][1] == 23.0 */
     valor = p3d_random_in_three_intervals(interv1[0][0],interv1[0][1],interv2[0][0],interv2[0][1],interv1[1][0],interv1[1][1]);
   }
   /* NOTE : !(interv1[1][1] == 23.0)&&!(interv2[1][1] == 23.0) is not possible */
   /* the maximum number of intervals is 3 (VERIFICAR ???) */
 }
 valor -= refval;
 return(valor);

}


/* random sampling in the set of two intervals */
double p3d_random_in_two_intervals(double minlim1, double maxlim1, double minlim2, double maxlim2)
{
 double valor;
 double d1,nv;
 /* double d1,d2,c1,c2,nv; */

 /* POSSIBILITY 1 : */
 /* shoot in normalized interval */
 /*      d1 = minlim1; */
 /*      c1 = (maxlim1 - minlim1)/M_PI; */
 /*      d2 = minlim2; */
 /*      c2 = (maxlim2 - minlim2)/M_PI; */
 /*      nv = p3d_random(0,(2.0*M_PI));  */
 /*      if(nv <= M_PI)  */
 /*        valor = d1 + nv * c1; */
 /*      else */
 /*        valor = d2 + (nv - M_PI) * c2; */
 /*      return(valor); */  
 
 /* POSSIBILITY 2 : */
 /* annex intervals */
 if(minlim1 >= minlim2) {
   d1 = minlim1 - maxlim2;
#ifdef P3D_PLANNER
   nv = p3d_random(minlim2,maxlim1-d1);
#endif
   if(nv > maxlim2)
     valor = d1 + nv;
   else
     valor = nv;
 }
 else {
   d1 = minlim2 - maxlim1;
#ifdef P3D_PLANNER
   nv = p3d_random(minlim1,maxlim2-d1);
#endif
   if(nv > maxlim1)
     valor = d1 + nv;
   else
     valor = nv;
 }
 return(valor);      
}

/* random sampling in the set of three intervals */
double p3d_random_in_three_intervals(double minlim1, double maxlim1, double minlim2, double maxlim2, double minlim3, double maxlim3)
{
 double valor;
 double d1,d2,nv;
 double intervs[3][2];
 int first,second,third;

 /* annex intervals */

 intervs[0][0] = minlim1;
 intervs[0][1] = maxlim1;
 intervs[1][0] = minlim2;
 intervs[1][1] = maxlim2;
 intervs[2][0] = minlim3;
 intervs[2][1] = maxlim3;

 if((minlim1 <= minlim2)&&(minlim1 <= minlim3)) {
   first = 0;
   if(minlim2 <= minlim3) {
     second = 1;
     third = 2;
   }
   else { 
     second = 2;
     third = 1;
   }
 }
 else if((minlim2 <= minlim1)&&(minlim2 <= minlim3)) {
   first = 1;
   if(minlim1 <= minlim3) {
     second = 0;
     third = 2;
  }
   else { 
     second = 2;
     third = 0;
   }
 }
 else {
   first = 2;
   if(minlim1 <= minlim2) {
     second = 0;
     third = 1;
   }
   else { 
     second = 1;
     third = 0;
   }
 }

 d1 = intervs[second][0]-intervs[first][1];
 d2 = intervs[third][0]-intervs[second][1];
#ifdef P3D_PLANNER
 nv = p3d_random(intervs[first][0],intervs[third][1]-d1-d2);
#endif
 if(nv <= intervs[first][1])
   valor = nv;
 else {
   if(nv <= intervs[second][1] - d1)
     valor = d1 + nv;
   else
     valor = d1 + d2 + nv;
 }
 return(valor);      
}


/*****************************************************************************/
/*****************************************************************************/

static int p3d_jnt_frames_at_same_point(p3d_jnt *Ja, p3d_jnt *Jb)
{
  if((Ja->p0.x == Jb->p0.x)&&
     (Ja->p0.y == Jb->p0.y)&&
     (Ja->p0.z == Jb->p0.z))
    return(1);
  else
    return(0);
}

/*****************************************************************************/


/* relative position of two axes :   */
/* rpra == 0 -> coincident axes      */	
/* rpra == 1 -> parallel axes        */
/* rpra == 2 -> cutting axes         */
/* rpra == 3 -> crossing axes        */
int p3d_rel_pos_lines(p3d_vector3 u1, p3d_vector3 u2, p3d_vector3 P1P2)
{ p3d_matrix3 mat;
 
  mat[0][0] = u1[0];
  mat[0][1] = u1[1];
  mat[0][2] = u1[2];
  mat[1][0] = u2[0];
  mat[1][1] = u2[1];
  mat[1][2] = u2[2];
  mat[2][0] = P1P2[0];
  mat[2][1] = P1P2[1];
  mat[2][2] = P1P2[2];

  switch(p3d_mat3Rank(mat,IN3DCC_EPS)) {
  case 0:
    return 0;
    break;
  case 1:
    return 0;
    break;
  case 2:
    mat[2][0] = 0.0;
    mat[2][1] = 0.0;
    mat[2][2] = 0.0;    
    if(p3d_mat3Rank(mat,IN3DCC_EPS) == 1)
      return 1;
    else 
      return 2;
    break;
  case 3:
    return 3;       
    break;
  } 
  return 0;
}


/********************************************************************************/
/* calculates Tt that reorientates T to made its z-asis parallel to v           */ 
/* returns 0 if v is not parellel to one of the axes of frame represented by T  */
static int p3d_transf_jnt_frame(p3d_matrix4 Tref, p3d_vector3 v, p3d_matrix4 Tt)
{ p3d_vector3 vi;
  double cosang; 
  int i, goon; 
  p3d_matrix4 T_rot,T_aux;
  p3d_matrix4 I4 = {{1.0,0.0,0.0,0.0},{0.0,1.0,0.0,0.0},{0.0,0.0,1.0,0.0},{0.0,0.0,0.0,1.0}};
  p3d_matrix4 r90x = {{1.0,0.0,0.0,0.0},{0.0,0.0,-1.0,0.0},{0.0,1.0,0.0,0.0},{0.0,0.0,0.0,1.0}};
  p3d_matrix4 r90y = {{0.0,0.0,1.0,0.0},{0.0,1.0,0.0,0.0},{-1.0,0.0,0.0,0.0},{0.0,0.0,0.0,1.0}};
  p3d_matrix4 r180x = {{1.0,0.0,0.0,0.0},{0.0,-1.0,0.0,0.0},{0.0,0.0,-1.0,0.0},{0.0,0.0,0.0,1.0}};
  
  i = 0;
  goon = 1;
  while(goon && (i<3)) {
    vi[0] = Tref[0][i];
    vi[1] = Tref[1][i];
    vi[2] = Tref[2][i];
    cosang = p3d_vectDotProd(vi,v)/(p3d_vectNorm(vi)*p3d_vectNorm(v));
    goon = !((fabs(cosang) >= 1.0-IN3DCC_EPS)&&(fabs(cosang) <= 1.0+IN3DCC_EPS));
    i++;
  }
  if(goon)
    return FALSE;
  else {
    switch(i - 1) {
    case 0: 
      p3d_mat4Copy(r90y,T_rot);
      break;
    case 1: 
      p3d_mat4Copy(r90x,T_rot);
      break;      
    case 2: 
      p3d_mat4Copy(I4,T_rot);
      break;    
    }
    
    p3d_mat4Mult(Tref,T_rot,T_aux);
    vi[0] = T_aux[0][2];
    vi[1] = T_aux[1][2];
    vi[2] = T_aux[2][2];
    cosang = p3d_vectDotProd(vi,v)/(p3d_vectNorm(vi)*p3d_vectNorm(v));

    if(cosang < 0.0) {
      p3d_mat4Mult(T_rot,r180x,Tt);
    }
    else {
      p3d_mat4Copy(T_rot,Tt);
    }
  }
  return TRUE;
}



/*****************************************************************************/

void p3d_solve_eqsys_2(p3d_matrix2 A, p3d_vector2 B,  p3d_vector2 unk)
{
  p3d_matrix2 iA;

  /* inverse of 2x2 matrix */
  iA[0][0] =  A[1][1]/(A[0][0]*A[1][1] - A[0][1]*A[1][0]);
  iA[0][1] =  -A[0][1]/(A[0][0]*A[1][1] - A[0][1]*A[1][0]);
  iA[1][0] =  -A[1][0]/(A[0][0]*A[1][1] - A[0][1]*A[1][0]);
  iA[1][1] =  A[0][0]/(A[0][0]*A[1][1] - A[0][1]*A[1][0]);

  unk[0] = iA[0][0]* B[0] + iA[0][1]* B[1];
  unk[1] = iA[1][0]* B[0] + iA[1][1]* B[1];

}

/*****************************************************************************/

/* next functions could be in  p3d_matrix.c */

/*****************************************************************************\
 @ p3d_mat3Rank()
 -----------------------------------------------------------------------------
 description : rank of matrix 3x3
 input       : the matrix, the tolerance
 output      : the rank
 notes       : 
\*****************************************************************************/
int p3d_mat3Rank(p3d_matrix3 mat, double tolerance)
{  int i,j;
   double det2; 
 
   if(fabs(p3d_mat3Det(mat)) > tolerance)
     return 3;

   for(i=0;i<3;i++) {
     if(i<2) j = i+1;
     else    j = 0;
     det2 = mat[i][0]*mat[j][1]-mat[j][0]*mat[i][1];
     if(fabs(det2) > tolerance)
       return 2;
     det2 = mat[i][1]*mat[j][2]-mat[j][1]*mat[i][2];
     if(fabs(det2) > tolerance)
       return 2;
     det2 = mat[i][0]*mat[j][2]-mat[j][0]*mat[i][2];
     if(fabs(det2) > tolerance)
       return 2;
   }

   for(i=0;i<3;i++) {
     for(j=0;j<3;j++) {
       if(fabs(mat[i][j]) > tolerance)
	 return 1;
     }
   }

   return 0;
}



/*****************************************************************************\
 @ p3d_lineDist()
 -----------------------------------------------------------------------------
 description : line distance

               Compute the distance from a point to a line.  The line is
               specified by a point and a vector

 input       : the line (point + vector), the point 
 output      : the distance
 notes       :
\*****************************************************************************/
double p3d_lineDist(p3d_vector3 P0, p3d_vector3 v, p3d_vector3 point)
{ p3d_vector3 P0p,vprod;

  p3d_vectSub(point, P0, P0p);	
  p3d_vectXprod(v,P0p,vprod);
  return(p3d_vectNorm(vprod)/p3d_vectNorm(v));
} /** End of p3d_lineDist() **/


/*****************************************************************************\
 @ p3d_lineInt()
 -----------------------------------------------------------------------------
 description : intersection between two cutting lines

 input       : the lines (point + vector)
 output      : the point
 notes       :
\*****************************************************************************/
int p3d_lineInt(p3d_vector3 P1, p3d_vector3 v1, p3d_vector3 P2, p3d_vector3 v2, p3d_vector3 res)
{ double A[2][2];
  double iA[2][2];
  double B[2];
  int i,i1,i2;
  double unk1;
    
  /* solving a system wich 3 eqs. and 2 unks. */
  /* solve for 1 unk. from two of them */
  /* check if null eq. */
  for(i=0,i1=-1,i2=-1;(i<3)&&(i2<0);i++) {
    if((v1[i] != 0.0)||(v2[i] != 0.0)) {
      if(i1<0)
	i1 = i;
      else
	i2 = i;
    }
  }
  A[0][0] = v1[i1];
  A[0][1] = -v2[i1];
  A[1][0] = v1[i2];
  A[1][1] = -v2[i2];
  B[0] = P2[i1] - P1[i1];
  B[1] = P2[i2] - P1[i2];

  /* inverse of 2x2 matrix */
  iA[0][0] =  A[1][1]/(A[0][0]*A[1][1] - A[0][1]*A[1][0]);
  iA[0][1] =  -A[0][1]/(A[0][0]*A[1][1] - A[0][1]*A[1][0]);
  iA[1][0] =  -A[1][0]/(A[0][0]*A[1][1] - A[0][1]*A[1][0]);
  iA[1][1] =  A[0][0]/(A[0][0]*A[1][1] - A[0][1]*A[1][0]);

  unk1 = iA[0][0]* B[0] + iA[0][1]* B[1];

  /* obtaining the point in line 1 */
  res[0] = P1[0] + unk1 * v1[0];
  res[1] = P1[1] + unk1 * v1[1];
  res[2] = P1[2] + unk1 * v1[2];

  return(TRUE);
} /** End of p3d_lineInt() **/


/*****************************************************************************\
 @ p3d_line_planeInt()
 -----------------------------------------------------------------------------
 description : intersection between line and plane

 input       : the line (2 points), the plane (point + normal vector)
 output      : the point
 notes       :
\*****************************************************************************/
int p3d_line_planeInt(p3d_vector3 Pl1, p3d_vector3 Pl2, p3d_vector3 Pp, p3d_vector3 vp, p3d_vector3 res)
{
  double u;
  p3d_vector3 Pl2subPl1,PpsubPl1;
  double num,den;

  p3d_vectSub(Pl2,Pl1,Pl2subPl1);
  p3d_vectSub(Pp,Pl1,PpsubPl1);

  num = p3d_vectDotProd(vp,PpsubPl1);
  den = p3d_vectDotProd(vp,Pl2subPl1);
  u = num/den;
/*   u = p3d_vectDotProd(vp,PpsubPl1)/p3d_vectDotProd(vp,Pl2subPl1); */

  /* obtaining the point in line 1 */
  res[0] = Pl1[0] + u * Pl2subPl1[0];
  res[1] = Pl1[1] + u * Pl2subPl1[1];
  res[2] = Pl1[2] + u * Pl2subPl1[2];

  return(TRUE);
} /** End of p3d_line_planeInt() **/

int p3d_line_planeInt_II(p3d_vector3 Pl1, p3d_vector3 Pl2, p3d_vector3 Pp1, p3d_vector3 Pp2, p3d_vector3 Pp3, p3d_vector3 res)
{
  double t;
  p3d_vector3 Pl1subPl2;
  p3d_matrix4 Mn,Md;
  double num,den;

  p3d_vectSub(Pl1,Pl2,Pl1subPl2);

  Mn[0][0] = Md[0][0] = 1.0;
  Mn[0][1] = Md[0][1] = 1.0;
  Mn[0][2] = Md[0][2] = 1.0;
  Mn[0][3] = 1.0;
  Md[0][3] = 0.0;
  Mn[1][0] = Md[1][0] = Pp1[0];
  Mn[1][1] = Md[1][1] = Pp2[0];
  Mn[1][2] = Md[1][2] = Pp3[0];
  Mn[1][3] = Pl1[0];
  Md[1][3] = - Pl1subPl2[0];
  Mn[2][0] = Md[2][0] = Pp1[1];
  Mn[2][1] = Md[2][1] = Pp2[1];
  Mn[2][2] = Md[2][2] = Pp3[1];
  Mn[2][3] = Pl1[1];
  Md[2][3] = - Pl1subPl2[1];
  Mn[3][0] = Md[3][0] = Pp1[2];
  Mn[3][1] = Md[3][1] = Pp2[2];
  Mn[3][2] = Md[3][2] = Pp3[2];
  Mn[3][3] = Pl1[2];
  Md[3][3] = - Pl1subPl2[2];

  num = p3d_mat4Det(Mn);
  den = p3d_mat4Det(Md);
  t = (p3d_mat4Det(Mn))/(p3d_mat4Det(Md));  
  t = num/den;

  /* obtaining the point in line */
  res[0] = Pl1[0] + t * Pl1subPl2[0];
  res[1] = Pl1[1] + t * Pl1subPl2[1];
  res[2] = Pl1[2] + t * Pl1subPl2[2];

  return(TRUE);
} /** End of p3d_line_planeInt() **/

/*****************************************************************************\
 @ p3d_same_sign_vect()
 -----------------------------------------------------------------------------
 description : determines if two parallel vectors have the same sign or not

 input       : the two vectors
 returns     : TRUE if same sign
 notes       :
\*****************************************************************************/
int p3d_same_sign_vect(p3d_vector3 v1, p3d_vector3 v2)
{
  p3d_vector3 v1n,v2n;

  p3d_vectNormalize(v1,v1n);
  p3d_vectNormalize(v2,v2n);
  if(p3d_vectDotProd(v1n,v2n) > 0.0)
    return 1;
  else
    return 0;

} /** End of p3d_same_sign_vect() **/




/**********************************************************/

/**********************************************************/

void p3d_destroy_rlg_chain(p3d_rlg_chain *rlgchPt)
{
  int i;

  for(i=0;i<rlgchPt->nlinksrlgch;i++) 
      { MY_FREE(rlgchPt->rlg_data[i],p3d_rlg_chain_data,1); }
  MY_FREE(rlgchPt->rlg_data, pp3d_rlg_chain_data, rlgchPt->nlinksrlgch);
  MY_FREE(rlgchPt, p3d_rlg_chain, 1);
}


void p3d_destroy_rlg_base(p3d_rlg_base *rlgbsPt)
{
  MY_FREE(rlgbsPt, p3d_rlg_base, 1);
}


void p3d_destroy_rlg(p3d_rlg *rlgPt)
{
  if(rlgPt->rlgchPt != NULL) {
    p3d_destroy_rlg_chain(rlgPt->rlgchPt);
    rlgPt->rlgchPt = NULL;
  }
  if(rlgPt->rlgbsPt != NULL) {
    p3d_destroy_rlg_base(rlgPt->rlgbsPt);
    rlgPt->rlgbsPt = NULL;
  }
  MY_FREE(rlgPt, p3d_rlg, 1);
}




/*****************************************************************************/
/*****************************************************************************/

double p3d_get_max_extension_manipulator(p3d_cntrt *manip_cntrt) 
{
  double maxext;

  /* get max extension of manipulator */
  /* if redundant (using RLG) */
  if(manip_cntrt->rlgPt->rlgchPt != NULL) {
    maxext = manip_cntrt->rlgPt->rlgchPt->rlg_data[0]->totrml;  // <- WARNING : modif. made in p3d_rlg.c
  }
  /* if directly using IK */
  else {
    /* NOTE :
       the max. extension of the manipulator is stored in the last element of vector ct->argu_d
       the computation of this value must be made in  p3d_set_"name_cntrt" 
    */
    maxext = manip_cntrt->argu_d[MAX_ARGU_CNTRT - 1];
  }

  return (maxext);
}


int p3d_compute_T_base_robot_manip(p3d_cntrt *ctPt, p3d_jnt *robotbasejntPt, p3d_jnt *manipbasejntPt, p3d_matrix4 Tbm)
{
   p3d_matrix4 invTm;
 
  /* transformation between baserobot-base-frame and manipulator-base-frame
     considered constant (no joints between them) !!!
  */
  p3d_matInvertXform(robotbasejntPt->pos0,invTm);
  p3d_mat4Mult(invTm,manipbasejntPt->pos0,Tbm);

  return TRUE;
}


double p3d_get_zmin_manipulator(p3d_cntrt *manip_cntrt)
{
  p3d_jnt *robotbasejntPt, *manipbasejntPt;
  p3d_matrix4 Tbm;
  double zmin_baserobot,zmin_basemanip;
  double vmin,vmax;

  /* VERIFICAR !!! */
  if(manip_cntrt->rlgPt->rlgchPt != NULL) {
    manipbasejntPt = manip_cntrt->rlgPt->rlgchPt->rlg_data[0]->jnt;
  }
  else{
    manipbasejntPt = manip_cntrt->pasjnts[0];
  }
  if(manip_cntrt->rlgPt->rlgbsPt != NULL) {
    robotbasejntPt = manip_cntrt->rlgPt->rlgbsPt->basejntPt;
  }
  else{
    robotbasejntPt = manipbasejntPt;
  }

  p3d_compute_T_base_robot_manip(manip_cntrt,robotbasejntPt,manipbasejntPt,Tbm);

  /* case FREEFLYER */
  if(robotbasejntPt->type == P3D_FREEFLYER) {
    p3d_jnt_get_dof_bounds(robotbasejntPt,2,&vmin,&vmax);
    zmin_baserobot = robotbasejntPt->pos0[2][3] + vmin;
    /* we assume completely free rotation */
    zmin_basemanip = zmin_baserobot - sqrt(SQR(Tbm[0][3]) + SQR(Tbm[1][3]) + SQR(Tbm[2][3]));
  }
  /* case FIXED or PLANE */    // WARNING : other cases !!!
  else {
    zmin_baserobot = robotbasejntPt->pos0[2][3];
    zmin_basemanip = zmin_baserobot + Tbm[2][3];
  }

  return (zmin_basemanip - p3d_get_max_extension_manipulator(manip_cntrt));
}


double p3d_get_zmax_manipulator(p3d_cntrt *manip_cntrt) 
{
  p3d_jnt *robotbasejntPt, *manipbasejntPt;
  p3d_matrix4 Tbm;
  double zmax_baserobot,zmax_basemanip;
  double vmin,vmax;

  /* VERIFICAR !!! */
  if(manip_cntrt->rlgPt->rlgchPt != NULL) {
    manipbasejntPt = manip_cntrt->rlgPt->rlgchPt->rlg_data[0]->jnt;
  }
  else{
    manipbasejntPt = manip_cntrt->pasjnts[0];
  }
  if(manip_cntrt->rlgPt->rlgbsPt != NULL) {
    robotbasejntPt = manip_cntrt->rlgPt->rlgbsPt->basejntPt;
  }
  else{
    robotbasejntPt = manipbasejntPt;
  }

  p3d_compute_T_base_robot_manip(manip_cntrt,robotbasejntPt,manipbasejntPt,Tbm);

  /* case FREEFLYER */
  if(robotbasejntPt->type == P3D_FREEFLYER) {
    p3d_jnt_get_dof_bounds(robotbasejntPt,2,&vmin,&vmax);
    zmax_baserobot = robotbasejntPt->pos0[2][3] + vmax;
    /* we assume completely free rotation */
    zmax_basemanip = zmax_baserobot + sqrt(SQR(Tbm[0][3]) + SQR(Tbm[1][3]) + SQR(Tbm[2][3]));
  }
  /* case FIXED or PLANE */    // WARNING : other cases !!!
  else {
    zmax_baserobot = robotbasejntPt->pos0[2][3];
    zmax_basemanip = zmax_baserobot + Tbm[2][3];
  }

  return (zmax_basemanip + p3d_get_max_extension_manipulator(manip_cntrt));
}


static void p3d_put_dof_into_range(p3d_jnt *jntPt, int idof, double *thedof)
{
  double vmin, vmax;
  
  if(p3d_jnt_is_dof_angular(jntPt, idof)){
    p3d_jnt_get_dof_bounds(jntPt, idof, &vmin, &vmax);
    if(*thedof < vmin) {
      *thedof += 2 * M_PI;
    }
    else if(*thedof > vmax) {
      *thedof -= 2 * M_PI;
    }
  }
}
