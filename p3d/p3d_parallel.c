/****************************************************************************/
/*!
 *  \file p3d_parallel.c
 *
 *     Parallel system configuration sampling.
 *
 */
/****************************************************************************/


#include "Util-pkg.h"
#include "P3d-pkg.h"

#ifdef P3D_PLANNER
#include "Planner-pkg.h"
#endif


/*--------------------------------------------------------------------------*/

#define PRINTPROC_PS 0
#define NUM_VALIDCONF_TEST 1000

#define MAXNITV 20

/*--------------------------------------------------------------------------*/

static int p3d_set_parallel_sys_mobmanip(p3d_parallel *psdataPt);
static int p3d_set_parallel_sys_fixmanip(p3d_parallel *psdataPt);

static int p3d_my_parallel_boundary_method_mobmanip(p3d_parallel *psdataPt);
static int p3d_random_position_boundary_method_fixmanip(p3d_parallel *psdataPt);
static int p3d_random_position_boundary_method_fixmanip_PRU(p3d_parallel *psdataPt);  // PARA PRUEBA
static int p3d_xyz_platfm_boundary_method_fixmanip(p3d_parallel *psdataPt);

static int s_p3d_mpbm_mobmanip_bound_x_y(p3d_parallel *psdataPt);
static int s_p3d_mpbm_mobmanip_bound_z(p3d_parallel *psdataPt);
static int s_p3d_mpbm_mobmanip_bound_rx_ry(p3d_parallel *psdataPt);
static int s_p3d_mpbm_mobmanip_bound_rz(p3d_parallel *psdataPt);

static int p3d_set_for_shoot_platform_orientation_fixmanip(p3d_parallel *psdataPt);
static int p3d_shoot_platform_fixmanip(p3d_parallel *psdataPt, configPt q);
static int p3d_shoot_platform_position_fixmanip(p3d_parallel *psdataPt, configPt q);
static int p3d_shoot_z_platform_fixmanip(p3d_parallel *psdataPt, double xplatf, double yplatf, double *zplatf);
static int p3d_shoot_platform_orientation_fixmanip(p3d_parallel *psdataPt, configPt q);
static int p3d_shoot_1dof_platform_orientation_fixmanip(p3d_parallel *psdataPt, 
							p3d_matrix4 Tbp, int ind_rot, double *value);
static int p3d_compute_1manip_plat_1dof_rot_bounds_fixmanip(p3d_parallel *psdataPt, p3d_matrix4 Tbp, 
							    int imanip, int idof, double angrefb, p3d_matrix2 interv);

static int p3d_shoot_platform_in_precomputed_bounds(p3d_parallel *psdataPt, configPt q);
static int p3d_shoot_platform_position_in_precomputed_bounds(p3d_parallel *psdataPt, configPt q);
static void p3d_update_Tbp(p3d_matrix4 Tbp, int ind_rot, double value);

static double p3d_platform_zmin_in_tri_att(p3d_parallel *psdataPt, p3d_tri_attach *tri_att);
static double p3d_platform_zmax_in_tri_att(p3d_parallel *psdataPt, p3d_tri_attach *tri_att);
static double p3d_platform_thetamax_in_tri_att(p3d_parallel *psdataPt, p3d_tri_attach *tri_att);

static void p3d_compute_triangles(p3d_parallel *psdataPt);
static void p3d_compute_one_triangle(p3d_parallel *psdataPt, int i, int j, int k, p3d_tri_attach *tri_att);

static int p3d_associate_frame_to_triangle(p3d_vector3 X1, p3d_vector3 v2, p3d_vector3 v3, p3d_matrix4 Tframe);

static void p3d_modify_cntrt_call_list_for_parallel(p3d_cntrt_management *cntrt_manager, p3d_cntrt *ct);

static int p3d_set_passive_parallel_platform(p3d_cntrt *ct, p3d_parallel *psdataPt); 
static int p3d_fct_passive_parallel_platform(p3d_cntrt *ct, int placeholder_i, configPt placeholder_c, double placeholder_d);


/*--------------------------------------------------------------------------------------------------------*/

static int (*p3d_bound_parallel_platform_dofs_mobmanip)(p3d_parallel *psdataPt) = p3d_my_parallel_boundary_method_mobmanip;
/* static int (*p3d_bound_parallel_platform_position_dofs_fixmanip)() = p3d_random_position_boundary_method_fixmanip; */
/* static int (*p3d_bound_parallel_platform_position_dofs_fixmanip)() = p3d_random_position_boundary_method_fixmanip_PRU; */
static int (*p3d_bound_parallel_platform_position_dofs_fixmanip)(p3d_parallel *psdataPt) = p3d_xyz_platfm_boundary_method_fixmanip;

/*--------------------------------------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*! \ingroup parallel
 *
 * \brief  Generates the structure of the  parallel system
 *
 * \param  type of system (fixed or mobile manipulator bases)
 * \param  base_jnt_index :  
 * \param  platform_jnt_index : 
 * \param  nmanip : 
 * \param  cntrt_index_vect : 
 *
 * \return TRUE if success, FALSE if it fails.
 */

int p3d_set_parallel_sys(const char *type_bases, int base_jnt_index, int platform_jnt_index,
			 int nmanip, int *cntrt_index_vect)
{
  p3d_parallel *psdataPt;
  p3d_rob *robotPt;
  p3d_cntrt_management *cntrt_manager;
  p3d_cntrt *ct;
  int i,ilink;
  int go_on;
  p3d_vector3 posTatt;
  
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  cntrt_manager = robotPt->cntrt_manager;
  
  psdataPt = MY_ALLOC(p3d_parallel,1);
  
  /* base */
  psdataPt->base_jntPt = robotPt->joints[base_jnt_index];
  
  /* platform */
  psdataPt->platform_jntPt = robotPt->joints[platform_jnt_index];
  if((psdataPt->platform_jntPt->type != P3D_KNEE)&&(psdataPt->platform_jntPt->type != P3D_FREEFLYER)) {
    MY_FREE(psdataPt,p3d_parallel,1);
    PrintError(("Wrong parallel system structure !!!\n"));
    return FALSE;
  }
  
  /* attachmets & manipulator cntrts */
  psdataPt->n_manipulators = nmanip;
  
  for(i=0; i<nmanip; i++) {
    psdataPt->manip_cntrt[i] = cntrt_manager->cntrts[cntrt_index_vect[i]];
    /* case of a parellel manipulator */
    if(p3d_cntrt_is_parallel_sys(psdataPt->manip_cntrt[i])) {
      if(!p3d_set_passive_parallel_platform(psdataPt->manip_cntrt[i],psdataPt)) {
	MY_FREE(psdataPt,p3d_parallel,1);
	PrintError(("Error in setting of passive parallel platform !!!\n"));
	return FALSE;
      }
    }
    /* WARNING !!! : verify that this asignation is always right */    
    psdataPt->att_jnt[i] = psdataPt->manip_cntrt[i]->actjnts[0];
    psdataPt->dpa[i] = sqrt(SQR(psdataPt->att_jnt[0]->abs_pos[0][3] - psdataPt->platform_jntPt->abs_pos[0][3]) +
			    SQR(psdataPt->att_jnt[0]->abs_pos[1][3] - psdataPt->platform_jntPt->abs_pos[1][3]) +
			    SQR(psdataPt->att_jnt[0]->abs_pos[2][3] - psdataPt->platform_jntPt->abs_pos[2][3]));
   /* Index for link_between_joint platform-attach */ 
    go_on = 1;
    ilink = 0;
    while(go_on && (ilink < psdataPt->platform_jntPt->n_link_jnt)) {
      if(psdataPt->platform_jntPt == psdataPt->platform_jntPt->link_jnt_arr[ilink]->prev_jnt) {
	if(psdataPt->manip_cntrt[i]->actjnts[0] == psdataPt->platform_jntPt->link_jnt_arr[ilink]->next_jnt) {
	  psdataPt->index_link_platf[i] = ilink;
	  go_on = 0;
	}
      }
      ilink++;
    }
    if(go_on) {
      MY_FREE(psdataPt,p3d_parallel,1);
      PrintError(("Wrong attachment manipulator - parallel platform !!!\n"));
      return FALSE;
    }

    /* ct->Tatt must be just a rotation matrix ! */
    posTatt[0] = psdataPt->manip_cntrt[i]->Tatt[0][3];
    posTatt[1] = psdataPt->manip_cntrt[i]->Tatt[1][3];
    posTatt[2] = psdataPt->manip_cntrt[i]->Tatt[2][3];
    if(p3d_vectNorm(posTatt) > EPS6) {
      MY_FREE(psdataPt,p3d_parallel,1);
      PrintError(("Wrong attachment manipulator - parallel platform !!!\n"));
      return FALSE;
    }
  }

  if(strcmp(type_bases, "fixmanip")==0) {
    if(p3d_set_parallel_sys_fixmanip(psdataPt)) {
#ifdef P3D_CONSTRAINTS
      ct = p3d_create_generic_cntrts(cntrt_manager,"parallel_sys_fixmanip",0,NULL,NULL,NULL,0,NULL,NULL,NULL);
#endif
    }
    else {
      MY_FREE(psdataPt,p3d_parallel,1);
      PrintError(("Wrong parallel system structure !!!\n"));
      return FALSE;
    }
  }
  else if(strcmp(type_bases, "mobmanip")==0) {
    if(p3d_set_parallel_sys_mobmanip(psdataPt)) {
#ifdef P3D_CONSTRAINTS
      ct = p3d_create_generic_cntrts(cntrt_manager,"parallel_sys_mobmanip",0,NULL,NULL,NULL,0,NULL,NULL,NULL);
#endif
    }
    else {
      MY_FREE(psdataPt,p3d_parallel,1);
      PrintError(("Wrong parallel system structure !!!\n"));
      return FALSE;
    }
  }
  else {
    PrintError(("Wrong manipulator bases type definition !!!\n"));
    return FALSE;
  }

  /* deactivate new cntrt */
  ct->active = FALSE;
  
  /* set flags in_cntrt for platform dofs -> active */
  for(i=0; i < psdataPt->platform_jntPt->dof_equiv_nbr; i++) {
    ct->cntrt_manager->in_cntrt[psdataPt->platform_jntPt->index_dof + i] = 1;
  }

  ct->parallel_sys_data = psdataPt; 
  
  /* change cntrt_manager->cntrt_call_list */
  p3d_modify_cntrt_call_list_for_parallel(cntrt_manager,ct);

  /* set pointers for reshoot */
  ct->nctud = nmanip;
  ct->ct_to_update = MY_ALLOC(pp3d_cntrt, nmanip);
  for(i=0; i<nmanip; i++) {
    ct->ct_to_update[i] = psdataPt->manip_cntrt[i];
    /* NO FUNCIONA EN EL CASO DE MANIP_PARALELO !!!??? */
    /* habria que eliminar el reshoot_ct de los manipuladores del maniulador paralelo que pasa a ser pasivo ??? */
    if(!p3d_cntrt_is_parallel_sys(psdataPt->manip_cntrt[i])) {
      ct->ct_to_update[i]->reshoot_ct = ct;
    }
/*     ct->ct_to_update[i]->reshoot_ct = ct; */
 }

  /* set RLG flags */
  p3d_active_RLG_flags();

  return TRUE;
}


/*--------------------------------------------------------------------------*/


static int p3d_set_parallel_sys_mobmanip(p3d_parallel *psdataPt)
{

  /* triangles of attach. */
  p3d_compute_triangles(psdataPt);  
  /* compute new platform bounds for random shoot */
  if(!(*p3d_bound_parallel_platform_dofs_mobmanip)(psdataPt)) {
    return FALSE;
  }
  /* function to shoot platform configuration */
  psdataPt->fct_parplatf_shoot = p3d_shoot_platform_in_precomputed_bounds;

  return TRUE;
}


/*--------------------------------------------------------------------------*/


static int p3d_set_parallel_sys_fixmanip(p3d_parallel *psdataPt)
{

  if(psdataPt->platform_jntPt->type == P3D_FREEFLYER) {
    /* compute new platform position bounds for random shoot */
    if(!(*p3d_bound_parallel_platform_position_dofs_fixmanip)(psdataPt)) {
      return FALSE;
    }
  }

  if(!p3d_set_for_shoot_platform_orientation_fixmanip(psdataPt)) {
    return FALSE;
  } 

  /* function to shoot platform configuration */
  psdataPt->fct_parplatf_shoot = p3d_shoot_platform_fixmanip;
 /*  psdataPt->fct_parplatf_shoot = p3d_shoot_platform_in_precomputed_bounds;  */  // PRUEBA !!!

  return TRUE;
}


/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! \ingroup parallel
 *
 * \brief  Destroy the structure of the parallel system
 *
 * \param  the pointer to the structure 
 *
 */
void p3d_destroy_parallel_sys_data(p3d_parallel *psdataPt)
{
  p3d_tri_attach *triatt,*dtriatt;
  
  triatt = psdataPt->triangles;
  do {
    dtriatt = triatt;
    triatt = triatt->next;
    MY_FREE(dtriatt,p3d_tri_attach,1);
  } while(triatt != NULL);

  MY_FREE(psdataPt,p3d_parallel,1);
}


/*****************************************************************************/
/*****************************************************************************/

/* SETTING FIXMANIP */

// FUNCION PARA PRUEBAS (NO USA BOUNDARY METHOD)
static int p3d_random_position_boundary_method_fixmanip_PRU(p3d_parallel *psdataPt)
{
  double vmin,vmax;
  int i;


  /* store bounds */
  for(i=0; i<3; i++) {
    // DEJO LOS LIMITES ORIGINALES DE MOMENTO !!!
    p3d_jnt_get_dof_bounds(psdataPt->platform_jntPt,i,&vmin,&vmax);   
    psdataPt->platform_dof_max_bounds[i] = vmax;
    psdataPt->platform_dof_min_bounds[i] = vmin;   
  } 

  return TRUE;
}

static int p3d_random_position_boundary_method_fixmanip(p3d_parallel *psdataPt)
{
  double dbpbm,dppem,dist,maxdist;
  double fbM[6],fbm[6];
  double bM[3],bm[3];
  double val,vmin,vmax;
  int only_pos_z;  
  int n_iter,i;
  int go_on;
  p3d_jnt *jntPt;

  /* first bounding box */
  /* -> compute a box containing the WS of the platform in relation to the base */
  /* NOTE :
     - we consider that the frame of the base is the reference system of the platform
     - the z-axis is perpendicular to the base
  */

  /* max. possible distance between base and platform 
     (considering just one manipulator) */
  maxdist = 0.0;
  only_pos_z = 1;
  for(i=0; i < psdataPt->n_manipulators; i++) {
    dbpbm = sqrt(SQR(psdataPt->manip_cntrt[i]->pasjnts[0]->abs_pos[0][3] - psdataPt->base_jntPt->abs_pos[0][3]) +
		 SQR(psdataPt->manip_cntrt[i]->pasjnts[0]->abs_pos[1][3] - psdataPt->base_jntPt->abs_pos[1][3]) +
		 SQR(psdataPt->manip_cntrt[i]->pasjnts[0]->abs_pos[2][3] - psdataPt->base_jntPt->abs_pos[2][3]));
    //VERIFICAR QUE REL_POS ES LA TRANSFORMACION ENTRE EL FRAME DE LA PLATF Y EL ATT !!!
    dppem = sqrt(SQR(psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[i]]->rel_pos[0][3]) +
		 SQR(psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[i]]->rel_pos[1][3]) +
		 SQR(psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[i]]->rel_pos[2][3])); 
    dist = dbpbm + dppem + psdataPt->manip_cntrt[i]->argu_d[MAX_ARGU_CNTRT - 1];
    if(dist > maxdist) {
      maxdist = dist;
    }
    /* for Z : the platform motion can be limited to the "top" */
    /* WARNING !!! : this is not a general test !!! */
    p3d_jnt_get_dof_bounds(psdataPt->manip_cntrt[i]->pasjnts[0],0,&vmin,&vmax);
    if(((psdataPt->manip_cntrt[i]->pasjnts[0]->abs_pos[2][3] - psdataPt->base_jntPt->abs_pos[2][3]) < 0.0) ||
       ((vmax - vmin) > (M_PI + EPS6))) {
      only_pos_z = 0;
    }
  }  
  /* box edge = 2 * maxdist */
  fbM[0] = maxdist;
  fbm[0] = -maxdist;
  fbM[1] = maxdist;
  fbm[1] = -maxdist;
  fbM[2] = maxdist;
  if(only_pos_z)
    fbm[2] = 0.0;
  else
    fbm[2] = -maxdist;

  /* rotation model bounds */
  p3d_jnt_get_dof_bounds(psdataPt->platform_jntPt,3,&fbm[3],&fbM[3]);   
  p3d_jnt_get_dof_bounds(psdataPt->platform_jntPt,4,&fbm[4],&fbM[4]);   
  p3d_jnt_get_dof_bounds(psdataPt->platform_jntPt,5,&fbm[5],&fbM[5]);   

  /* bounding box */  
  /* iterate : - random shoot of the configuration of the platform (in first bounding box) 
               - verify cntrts
  */
  bM[0] = bM[1] = bM[2] = -P3D_HUGE;
  bm[0] = bm[1] = bm[2] = P3D_HUGE;
  n_iter = 0;
#ifdef P3D_CONSTRAINTS
  p3d_set_TEST_PHASE(TRUE);
#endif
  while(n_iter < NUM_VALIDCONF_TEST) {
    for(i=0; i<6; i++) {
#ifdef P3D_PLANNER
      val = p3d_random(fbm[i],fbM[i]);
#endif
      p3d_jnt_set_dof(psdataPt->platform_jntPt,i,val);
    }
    // ????????????????????
    if(psdataPt->base_jntPt->prev_jnt != NULL)
      psdataPt->base_jntPt->prev_jnt->pos_updated = TRUE;
    p3d_update_jnt_pos(psdataPt->base_jntPt);
    psdataPt->base_jntPt->rob->j_modif = NULL;
    for(i=0; i <= psdataPt->base_jntPt->rob->njoints; i++) {
      jntPt =  psdataPt->base_jntPt->rob->joints[i];
      p3d_jnt_clean_update(jntPt);
    }
    // ????????????????????
    i = 0;
    go_on = 1;
    while(go_on && (i < psdataPt->n_manipulators)) {
      // multiple iksol is not compatible (yet) with parallel !
      go_on = (*psdataPt->manip_cntrt[i]->fct_cntrt)(psdataPt->manip_cntrt[i],-1,NULL,0.0);
      i++;
    }
    if(go_on) {
      n_iter++;
      for(i=0; i<3; i++) {
	if(bM[i] < p3d_jnt_get_dof(psdataPt->platform_jntPt,i)) {
	  bM[i] = p3d_jnt_get_dof(psdataPt->platform_jntPt,i);
	}
      }
      for(i=0; i<3; i++) {
	if(bm[i] > p3d_jnt_get_dof(psdataPt->platform_jntPt,i)) {
	  bm[i] = p3d_jnt_get_dof(psdataPt->platform_jntPt,i);
	}
      }
    }
  }
#ifdef P3D_CONSTRAINTS
  p3d_set_TEST_PHASE(FALSE);
#endif
  /* store bounds */
  for(i=0; i<3; i++) {
    // NO FUNCIONA ESTA FUNCION !!!!
    // DEJO LOS LIMITES ORIGINALES DE MOMENTO !!!
    p3d_jnt_get_dof_bounds(psdataPt->platform_jntPt,i,&vmin,&vmax);   
    psdataPt->platform_dof_max_bounds[i] = vmax;
    psdataPt->platform_dof_min_bounds[i] = vmin;   
    /*     if(bM[i] > vmax) */
    /*       bM[i] = vmax; */
    /*     if(bm[i] < vmin) */
    /*       bm[i] = vmin; */
    /*    psdataPt->platform_dof_max_bounds[i] = bM[i]; */
    /*    psdataPt->platform_dof_min_bounds[i] = bm[i]; */
  } 

  return TRUE;
}


static int p3d_xyz_platfm_boundary_method_fixmanip(p3d_parallel *psdataPt)
{
  double xmin,xmax,ymin,ymax,zmin,zmax;
  double xmin_i,xmax_i,ymin_i,ymax_i,zmin_i,zmax_i;
  double dbpmax;
  int i;
  double vmin,vmax;

  xmax = ymax = zmin = P3D_HUGE;
  xmin = ymin = zmax = -P3D_HUGE;

  /* WARNING FOR Z !!! :
     the methot to bound z is not right !
     it is required only to be used in some examples of Stewart platforms
  */

  for(i=0; i<psdataPt->n_manipulators; i++) {
    /* x-y */ 
    dbpmax = psdataPt->manip_cntrt[i]->argu_d[MAX_ARGU_CNTRT - 1] + psdataPt->dpa[i];
    xmin_i = psdataPt->manip_cntrt[i]->pasjnts[0]->abs_pos[0][3] - dbpmax;
    xmax_i = psdataPt->manip_cntrt[i]->pasjnts[0]->abs_pos[0][3] + dbpmax;
    ymin_i = psdataPt->manip_cntrt[i]->pasjnts[0]->abs_pos[1][3] - dbpmax;
    ymax_i = psdataPt->manip_cntrt[i]->pasjnts[0]->abs_pos[1][3] + dbpmax;
    if(PRINTPROC_PS) {
      printf("max_ext = %f, dpa = %f, dbpmax= %f\n",
	     psdataPt->manip_cntrt[i]->argu_d[MAX_ARGU_CNTRT - 1],psdataPt->dpa[i],dbpmax);
      printf("x_bm = %f, y_bm = %f\n",
	     psdataPt->manip_cntrt[i]->pasjnts[0]->abs_pos[0][3],psdataPt->manip_cntrt[i]->pasjnts[0]->abs_pos[1][3]);
      printf("xmin_i = %f, xmax_i = %f, ymin_i = %f, ymax_i = %f\n",xmin_i,xmax_i,ymin_i,ymax_i);
    } 
    if(xmin_i > xmin)
      xmin = xmin_i;
    if(xmax_i < xmax)
      xmax = xmax_i;
    if(ymin_i > ymin)
      ymin = ymin_i;
    if(ymax_i < ymax)
      ymax = ymax_i;
    /* z */ 
    zmax_i =  psdataPt->manip_cntrt[i]->Tbase[2][3] + psdataPt->manip_cntrt[i]->argu_d[MAX_ARGU_CNTRT - 1];
    p3d_jnt_get_dof_bounds(psdataPt->manip_cntrt[i]->pasjnts[0],0,&vmin,&vmax);
    if((vmax - vmin) > (M_PI + EPS6))
      zmin_i =  psdataPt->manip_cntrt[i]->Tbase[2][3] - psdataPt->manip_cntrt[i]->argu_d[MAX_ARGU_CNTRT - 1];
    else
      zmin_i =  psdataPt->manip_cntrt[i]->Tbase[2][3];  
    if(zmin_i < zmin)
      zmin = zmin_i;
    if(zmax_i > zmax)
      zmax = zmax_i;   
  }
  
  // make relative to base_jnt
  xmin -= psdataPt->base_jntPt->abs_pos[0][3];
  xmax -= psdataPt->base_jntPt->abs_pos[0][3];
  ymin -= psdataPt->base_jntPt->abs_pos[1][3];
  ymax -= psdataPt->base_jntPt->abs_pos[1][3];

  if(PRINTPROC_PS) {
    printf("x_bp = %f, y_bp = %f\n",psdataPt->base_jntPt->abs_pos[0][3],psdataPt->base_jntPt->abs_pos[1][3]);
    printf("xmin = %f, xmax = %f, ymin = %f, ymax = %f\n",xmin,xmax,ymin,ymax);
  }

  psdataPt->platform_dof_min_bounds[0] = xmin;   
  psdataPt->platform_dof_max_bounds[0] = xmax;
  psdataPt->platform_dof_min_bounds[1] = ymin;   
  psdataPt->platform_dof_max_bounds[1] = ymax;
  psdataPt->platform_dof_min_bounds[2] = zmin;   
  psdataPt->platform_dof_max_bounds[2] = zmax;

  return TRUE;
}

static int p3d_set_for_shoot_platform_orientation_fixmanip(p3d_parallel *psdataPt)
{

  /* WARNING !!! :
     preliminary version (as for p3d_set_prismatic_actuator_II_ik) :
     considerations about the model made the computation 
     of initial values are not necessary  
  */

  // PARA PRUEBA !!!
  int irot0,i;
  double vmin,vmax;

  if(psdataPt->platform_jntPt->type == P3D_FREEFLYER) 
    irot0 = 3;
  else
    irot0 = 0;

  for(i=0; i<3; i++) {
    p3d_jnt_get_dof_bounds(psdataPt->platform_jntPt,irot0+i,&vmin,&vmax);   
    psdataPt->platform_dof_max_bounds[irot0+i] = vmax;
    psdataPt->platform_dof_min_bounds[irot0+i] = vmin; 
  }

  return TRUE;
}



/*****************************************************************************/
/*****************************************************************************/

/* SETTING MOBMANIP */

/*--------------------------------------------------------------------------*/
/*! \ingroup parallel
 *
 *  \brief  Juan Cortes's method to bound the parallel platform dofs
 *          in the case of manipulators with mobile bases defined from
 *          the absolute reference frame 
 *
 *  \param  The parallel system structure data pointer
 *
 *  \return TRUE if success
 *
 *  \internal  
 */
static int p3d_my_parallel_boundary_method_mobmanip(p3d_parallel *psdataPt)
{
  if(!s_p3d_mpbm_mobmanip_bound_x_y(psdataPt)||
     !s_p3d_mpbm_mobmanip_bound_z(psdataPt)||
     !s_p3d_mpbm_mobmanip_bound_rx_ry(psdataPt)||
     !s_p3d_mpbm_mobmanip_bound_rz(psdataPt)) {
    return FALSE;
  }
  else {
    return TRUE;
  }
}


static int s_p3d_mpbm_mobmanip_bound_x_y(p3d_parallel *psdataPt)
{
  double vmin,vmax;

  /* no case if P3D_KNEE !!! */
  if(psdataPt->platform_jntPt->type == P3D_KNEE) {
    return TRUE;
  }

  /* in the firts version I consider that all the bases can
     move all over the plane, so x and y are not bounded
  */
  p3d_jnt_get_dof_rand_bounds(psdataPt->platform_jntPt,0,&vmin,&vmax);
  psdataPt->platform_dof_min_bounds[0] = vmin;
  psdataPt->platform_dof_max_bounds[0] = vmax;
  p3d_jnt_get_dof_rand_bounds(psdataPt->platform_jntPt,1,&vmin,&vmax);
  psdataPt->platform_dof_min_bounds[1] = vmin;
  psdataPt->platform_dof_max_bounds[1] = vmax;
 
  return TRUE;
}


static int s_p3d_mpbm_mobmanip_bound_z(p3d_parallel *psdataPt)
{
  p3d_tri_attach *tri_att;
  double mz_min,mz_max,z_min,z_max,p_zmin,p_zmax;

  z_min = - P3D_HUGE;
  z_max = P3D_HUGE;   

  /* no case if P3D_KNEE !!! */
  if(psdataPt->platform_jntPt->type == P3D_KNEE) {
    return TRUE;
  }

  if (psdataPt->n_manipulators == 2) {
    /* the computation is inmediate :
       - obtain line defining tangent at z_max (min)
       - the value is determined by the object frame in relation to this line 
       HACER !!! 
    */
  }
  else {
    /* try all combination of three manipulators */
    tri_att = psdataPt->triangles;
    while(tri_att != NULL) {
      p_zmin = p3d_platform_zmin_in_tri_att(psdataPt,tri_att);
      p_zmax = p3d_platform_zmax_in_tri_att(psdataPt,tri_att);
      /* keep max(z_min) and  min(z_max) */
      if(p_zmin > z_min)
	z_min = p_zmin;
      if(p_zmax < z_max)
	z_max = p_zmax;
      tri_att = tri_att->next;
    }
  }
  /* store new platform z rand-bounds */
  p3d_jnt_get_dof_rand_bounds(psdataPt->platform_jntPt,2,&mz_min,&mz_max);
  if(z_min < mz_min)
    z_min = mz_min;
  if(z_max > mz_max)
    z_max = mz_max;
  
  psdataPt->platform_dof_min_bounds[2] = z_min;
  psdataPt->platform_dof_max_bounds[2] = z_max;
    
  return TRUE;
}


static int s_p3d_mpbm_mobmanip_bound_rx_ry(p3d_parallel *psdataPt)
{
  p3d_tri_attach *tri_att;
  double mt_min,mt_max,rx_min,rx_max,ry_min,ry_max,theta_max,p_tmax;
  int indtx,indty;

  /* NOTE :
     simplified and very conservative method that computes
     the maximum (and minimum) inclination of the z axis
     of the parallel platform:
        rx_max = ry_max = theta_max
        rx_min = ry_min = - theta_max

     * this z axis is considered parallel to the z_abs axis
       in the definition

     This approximation does not consider possible bounds in
     the relative motion of manipulators, so distances 
     corresponding to max. inclinations are reachable.
  */

  theta_max = 0.0;

  if (psdataPt->n_manipulators == 2) {
    /* the computation is inmediate :
       - obtain line defining tangent at z_max (min)
       - the value is determined by the object frame in relation to this line 
       HACER !!! 
    */
  }
  else {  
    /* try all combination of three manipulators */
    tri_att = psdataPt->triangles;
    while(tri_att != NULL) {
      p_tmax = p3d_platform_thetamax_in_tri_att(psdataPt,tri_att);
      /* keep max(theta_max) */
      if(p_tmax > theta_max)
	theta_max = p_tmax;    
      tri_att = tri_att->next;
    }
  }

  rx_min = -theta_max;
  rx_max = theta_max;
  ry_min = -theta_max;
  ry_max = theta_max;

  /* store new tx and ty rand-bounds */
  if(psdataPt->platform_jntPt->type == P3D_KNEE) {
    indtx = 0;
    indty = 1;
  }  
  else {
    indtx = 3;
    indty = 4;
  }

  p3d_jnt_get_dof_rand_bounds(psdataPt->platform_jntPt,indtx,&mt_min,&mt_max);
  if(rx_min < mt_min)
    rx_min = mt_min;
  if(rx_max > mt_max)
    rx_max = mt_max;
  psdataPt->platform_dof_min_bounds[indtx] = rx_min;
  psdataPt->platform_dof_max_bounds[indtx] = rx_max;

  p3d_jnt_get_dof_rand_bounds(psdataPt->platform_jntPt,indty,&mt_min,&mt_max);
  if(ry_min < mt_min)
    ry_min = mt_min;
  if(ry_max > mt_max)
    ry_max = mt_max;
  psdataPt->platform_dof_min_bounds[indty] = ry_min;
  psdataPt->platform_dof_max_bounds[indty] = ry_max;

  return TRUE;  
}


static int s_p3d_mpbm_mobmanip_bound_rz(p3d_parallel *psdataPt)
{
  double vmin,vmax;
  int indrz;

  /* in the firts version I consider that all the bases can
     move all over the plane, so tz is not bounded
  */
  if(psdataPt->platform_jntPt->type == P3D_KNEE) {
    indrz = 2;
  }  
  else {
    indrz = 5;
  }  
  p3d_jnt_get_dof_rand_bounds(psdataPt->platform_jntPt,indrz,&vmin,&vmax);
  psdataPt->platform_dof_min_bounds[indrz] = vmin;
  psdataPt->platform_dof_max_bounds[indrz] = vmax;

  return TRUE;
}



/*****************************************************************************/
/*****************************************************************************/

/* FUNCTIONS USED TO GENERATE THE PLANFORM CONFIGURATION  */

static int p3d_shoot_platform_fixmanip(p3d_parallel *psdataPt, configPt q)
{

  if(psdataPt->platform_jntPt->type == P3D_FREEFLYER) {
    /* shoot position */
/*     if(!p3d_shoot_platform_position_in_precomputed_bounds(psdataPt,q)) { */
    if(!p3d_shoot_platform_position_fixmanip(psdataPt,q)) {
     return FALSE;
    }
  }   

  /* shoot orientation */ 
  if(!p3d_shoot_platform_orientation_fixmanip(psdataPt,q)) {
    return FALSE;
  }  

  return TRUE;
}


static int p3d_shoot_platform_position_fixmanip(p3d_parallel *psdataPt, configPt q)
{
  int i;
  double val;
  
  /* generate for x y in the precomputed region */
  for(i=0; i < 2; i++) {
#ifdef P3D_PLANNER
    val = p3d_random(psdataPt->platform_dof_min_bounds[i],psdataPt->platform_dof_max_bounds[i]);
#endif
    q[psdataPt->platform_jntPt->index_dof + i] = val;
  }
  
  /* generate z using RLG notions */
  if(!p3d_shoot_z_platform_fixmanip(psdataPt,q[psdataPt->platform_jntPt->index_dof],
				    q[psdataPt->platform_jntPt->index_dof + 1],&val))
    return FALSE;
  q[psdataPt->platform_jntPt->index_dof + 2] = val;

  return TRUE;
}


static int p3d_shoot_z_platform_fixmanip(p3d_parallel *psdataPt, double xplatf, double yplatf, double *zplatf)
{
  double zmin,zmax;
  double zmin_i,zmax_i;
  double dbpmax;
  double dxy;
  int i;
  double vmin,vmax;

  zmax = P3D_HUGE;
  zmin = -P3D_HUGE;

  for(i=0; i<psdataPt->n_manipulators; i++) {
    dbpmax = psdataPt->manip_cntrt[i]->argu_d[MAX_ARGU_CNTRT - 1] + psdataPt->dpa[i];
    dxy = sqrt(SQR(psdataPt->manip_cntrt[i]->pasjnts[0]->abs_pos[0][3] - xplatf) + 
	       SQR(psdataPt->manip_cntrt[i]->pasjnts[0]->abs_pos[1][3] - yplatf));
    if(PRINTPROC_PS) {
      printf("x = %f, y = %f, dxy = %f, dbpmax = %f\n",xplatf,yplatf,dxy,dbpmax);
    }
    if(dxy > dbpmax)
      return FALSE;
    zmax_i = sqrt(SQR(dbpmax) - SQR(dxy));
    /* the platform motion is limited to the "top" ? */
    /* WARNING !!! : this is not a general test !!! */
    p3d_jnt_get_dof_bounds(psdataPt->manip_cntrt[i]->pasjnts[0],0,&vmin,&vmax);
    if((vmax - vmin) > (M_PI + EPS6))
      zmin_i = -zmax_i;
    else
      zmin_i = 0.0;      
    // make relative to base_jnt
    zmax_i += psdataPt->manip_cntrt[i]->Tbase[2][3];
    zmin_i += psdataPt->manip_cntrt[i]->Tbase[2][3];
    if(PRINTPROC_PS) {
      printf("zpbm = %f, zmin_i = %f, zmax_i = %f\n",psdataPt->manip_cntrt[i]->Tbase[2][3],zmin_i,zmax_i);
    } 
    if(zmin_i > zmin)
      zmin = zmin_i;
    if(zmax_i < zmax)
      zmax = zmax_i;
  }
#ifdef P3D_PLANNER
  *zplatf = p3d_random(zmin,zmax);
#endif
  if(PRINTPROC_PS) {
    printf("zmin = %f, zmax = %f, z = %f\n",zmin,zmax,*zplatf);
  }   
  return TRUE;
}


static int p3d_shoot_platform_orientation_fixmanip(p3d_parallel *psdataPt, configPt q)
{
  p3d_matrix4 Tbp;
  int i;
  int idof;
  double val;

  p3d_mat4Copy(p3d_mat4IDENTITY,Tbp);
   
  /* get platform position (in base reference) */
  if(psdataPt->platform_jntPt->type == P3D_FREEFLYER) {
    /* the reference for the platform_jnt is the base_jnt frame !!! */
    for(i=0; i<3; i++) {
      Tbp[i][3] = q[psdataPt->platform_jntPt->index_dof + i];
    }
    idof = 3;
  }
  else {   // psdataPt->platform_jntPt->type == P3D_KNEE
    // PUEDE HACERSE A PARTIR DE UN VALOR FIJO QUE ESTA ALMACENADO EN ALGUNA PARTE !!!
    for(i=0; i<3; i++) {
      Tbp[i][3] = psdataPt->platform_jntPt->abs_pos[i][3] - psdataPt->base_jntPt->abs_pos[i][3];      
    }
    idof = 0;
  }

  if(PRINTPROC_PS) {
    printf("POSITION : x = %f, y = %f, z = %f\n",Tbp[0][3],Tbp[1][3],Tbp[2][3]);
  } 

  /* progressively generate (orientation) configuration */
  /*  ---  similar to RLG  ---  */
  for(i=0; i<3; i++) {
    if(p3d_shoot_1dof_platform_orientation_fixmanip(psdataPt,Tbp,i,&val)) {
      if(PRINTPROC_PS)
	printf("val = %f\n",val*180.0/M_PI);
      p3d_update_Tbp(Tbp,i,val);
      q[psdataPt->platform_jntPt->index_dof + idof] = val;
      idof++;
    }
    else {
      return FALSE;  
    }
  }
  
  return TRUE;  
}



static int p3d_shoot_1dof_platform_orientation_fixmanip(p3d_parallel *psdataPt, p3d_matrix4 Tbp,
							int ind_rot, double *value)
{
  double rel_ang_ref;
  int imanip;
  p3d_matrix2 interv,intv_i;
  double relmin[MAXNITV],relmax[MAXNITV];
  double p_relmin[MAXNITV],p_relmax[MAXNITV];
  int i,j,imin,iit,irot;
  int iintv=0,iintve;
  double vmin,vmax;
  double intmin;
  p3d_vector4 vpb_a,vpb_r;
  p3d_vector3 vpb_r_p,rotref,rotax;
  p3d_vector3 vprod,vprod_n;
  p3d_matrix4 Tpb;

  /* relative reference for angle */
  for(i=0; i<3; i++) {
    vpb_a[i] = - Tbp[i][3];
  }
  vpb_a[3] = 0.0;

  p3d_matInvertXform(Tbp,Tpb); 
  p3d_matvec4Mult(Tpb,vpb_a,vpb_r);

  for(i=0; i<3; i++) {
    vpb_r_p[i] = vpb_r[i];
    rotref[i] = 0.0;
    rotax[i] = 0.0;
  }
  vpb_r_p[ind_rot] = 0.0;
  rotax[ind_rot] = 1.0;
  if(ind_rot == 0) {
    rotref[1] = -1.0;
  }
  else if (ind_rot == 1) {
    rotref[0] = 1.0;
  }
  else {
    rotref[0] = 1.0;
  }
  rel_ang_ref = acos(p3d_vectDotProd(rotref,vpb_r_p)/p3d_vectNorm(vpb_r_p));  // p3d_vectNorm(rotref) = 1.0 !!!
  p3d_vectXprod(rotref,vpb_r_p,vprod);
  p3d_vectNormalize(vprod,vprod_n);
  p3d_vectSub(rotax,vprod_n,vprod); 
  if((fabs(vprod[0])>0.0001)||(fabs(vprod[1])>0.0001)||(fabs(vprod[2])>0.0001))
    rel_ang_ref = -rel_ang_ref;    
 
  for(i=0; i<MAXNITV; i++) {
    relmax[i] = 23.0;     // falg
    p_relmax[i] = 23.0;   // falg
  }

  for(imanip=0; imanip < psdataPt->n_manipulators; imanip++) {
    if(p3d_compute_1manip_plat_1dof_rot_bounds_fixmanip(psdataPt,Tbp,imanip,ind_rot,rel_ang_ref,interv)) {
      if(imanip == 0) {
	relmin[0] = interv[0][0];
	relmax[0] = interv[0][1];
 	relmin[1] = interv[1][0];
	relmax[1] = interv[1][1];
	if(interv[1][1] == 23.0)
	  iintv = 1;
	else
	  iintv = 2;
      }
      else {
	iit = 0;
	iintv = 0;
	while((iit < MAXNITV) && (relmax[iit] != 23.0)) {
	  if(p3d_inter_ang_regions_II(interv[0][0],interv[0][1],relmin[iit],relmax[iit],intv_i)) {
	    if(iintv == MAXNITV) {
	      printf("MAXNITV SOBREPASADO !!!!!!!!!!!\n");
	      return FALSE;
	    }	   
	    p_relmin[iintv] = intv_i[0][0];
	    p_relmax[iintv] = intv_i[0][1];
	    iintv++;
	    if(intv_i[1][1] != 23.0) {
	      if(iintv == MAXNITV) {
		printf("MAXNITV SOBREPASADO !!!!!!!!!!!\n");
		return FALSE;
	      }
	      p_relmin[iintv] = intv_i[1][0];
	      p_relmax[iintv] = intv_i[1][1];
	      iintv++;
	    }
	  }
	  if((interv[1][1] != 23.0) && 
	     p3d_inter_ang_regions_II(interv[1][0],interv[1][1],relmin[iit],relmax[iit],intv_i)) {
	    if(iintv == MAXNITV) {
	      printf("MAXNITV SOBREPASADO !!!!!!!!!!!\n");
	      return FALSE;
	    }	
	    p_relmin[iintv] = intv_i[0][0];
	    p_relmax[iintv] = intv_i[0][1];
	    iintv++;
	    if(intv_i[1][1] != 23.0) {
	      if(iintv == MAXNITV) {
		printf("MAXNITV SOBREPASADO !!!!!!!!!!!\n");
		return FALSE;
	      }
	      p_relmin[iintv] = intv_i[1][0];
	      p_relmax[iintv] = intv_i[1][1];
	      iintv++;
	    }
	  }
	  iit++;
	}
	if(iintv == 0) {
	  /* no solution */
	  return FALSE; 
	}
	for(i=0; i<iintv; i++) {
	  relmin[i] = p_relmin[i];
	  relmax[i] = p_relmax[i];
	}	
	relmax[iintv] = 23.0;
      }
    }
    else {
      return FALSE;  
    }
  }

  for(i=0; i<iintv; i++) {
/*     if(rel_ang_ref <= (M_PI/2.0)) { */
/*       relmin[i] += ((M_PI/2.0) - rel_ang_ref); */
/*       relmax[i] += ((M_PI/2.0) - rel_ang_ref); */
/*     } */
/*     else { */
      relmin[i] += rel_ang_ref;
      relmax[i] += rel_ang_ref;
/*     } */
    if((relmax[i] - relmin[i]) >= ((2.0*M_PI) - EPS6)) {
      relmin[i] = - M_PI;
      relmax[i] = M_PI;
    }
    if(relmax[i] > (2.0*M_PI)) {
      relmin[i] -= (2.0*M_PI);
      relmax[i] -= (2.0*M_PI);
    }
    if(relmin[i] < -(2.0*M_PI)) {
      relmin[i] += (2.0*M_PI);
      relmax[i] += (2.0*M_PI);
    }
  }

  if(psdataPt->platform_jntPt->type == P3D_FREEFLYER) 
    irot = ind_rot + 3;
  else
    irot = ind_rot;
  
  /* order intervals */
  j = 0;
  while(j < (iintv-1)) {
    imin = j;
    intmin = relmin[j];
    for(i=j+1; i<iintv; i++) {
      if(relmin[i] < intmin) {
	imin = i;
	intmin = relmin[i];
      }
    }
    if(imin != j) {
      relmin[imin] = relmin[j];
      relmin[j] = intmin;
      intmin = relmax[imin];
      relmax[imin] = relmax[j];
      relmax[j] = intmin;
    }
    j++;
  }

  /* consider joint limits and update intervals */
  p3d_jnt_get_dof_bounds(psdataPt->platform_jntPt,irot,&vmin,&vmax);   
  if((vmin > relmax[iintv - 1]) || (vmax < relmin[0])) {
    /* no solution */
    return FALSE; 
  }
  iintve = 0;
  for(i=0; i<iintv; i++) {
    if(vmin > relmax[i])
      iintve++;
    else {
      if(relmin[i] < vmin)
	relmin[i] = vmin;
      break;
    }
  }
  if(iintve > 0) {
    iintv -= iintve;
    for(i=0; i<iintv; i++) { 
      relmin[i] = relmin[iintve+i];
      relmax[i] = relmax[iintve+i];
    }
  }
  iintve = 0;
  for(i = iintv - 1; i >= 0; i--) {
    if(relmin[i] > vmax)
      iintve++;
    else {
      if(relmax[i] > vmax)
	relmax[i] = vmax;    
      break;
    }
  }
  iintv -= iintve;
  
  *value = p3d_random_in_several_ordered_intervals(iintv,relmin,relmax);

  return TRUE;  
}

double p3d_random_in_several_ordered_intervals(int nint, double *relmin, double *relmax)
{
  int i;
  double nv,valor=0.0,maxso;
  double diff[MAXNITV],offset[MAXNITV];

  maxso = relmax[0];
  for(i=0; i<(nint-1); i++) {
    offset[i] = relmin[i+1] - relmax[i];
    if(i>0)
      offset[i] += offset[i-1]; 
    diff[i] = relmax[i+1] - relmin[i+1];
    maxso += diff[i];
  }
#ifdef P3D_PLANNER
  nv = p3d_random(relmin[0],maxso);
#endif
  if(nv <= relmax[0]) {
    valor = nv;
  }
  else {
    maxso = relmax[0];
    for(i=0; i<(nint-1); i++) {
      maxso += diff[i];
      if(nv <= maxso) {
	valor = nv + offset[i];
	break;
      }
    }
  }
  return(valor);
}


static int p3d_compute_1manip_plat_1dof_rot_bounds_fixmanip(p3d_parallel *psdataPt, p3d_matrix4 Tbp, 
							    int imanip, int idof, double angrefb, p3d_matrix2 interv)
{
  p3d_matrix4 Taxis,Tbp_p,Tpb_p;
  int i;
  double angrefm,difangrefs;
  double dist_axis,dist_att,ang_att;
  double distance,ang,dist_pp,dist_p;
  double max_lim,min_lim;
  double minext_p,maxext_p;
  p3d_vector4 vpbm_a,vpbm_r;
  p3d_vector3 vpbm_r_p,rotref,rotax;
  p3d_vector3 vprod,vprod_n;


  /* TODO ES RELATIVO AL FRAME DE LA BASE !!!!!!!!!!!!!! */

  if(idof == 0) {
    dist_att = sqrt(SQR(psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[1][3]) +
		    SQR(psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[2][3]));
    ang_att = atan2(psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[2][3],
		    - psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[1][3]);
  }
  else if (idof == 1) {
    dist_att = sqrt(SQR(psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[0][3]) +
		    SQR(psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[2][3]));
    ang_att = atan2(psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[2][3],
		    psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[0][3]);
   }
  else {
    dist_att = sqrt(SQR(psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[0][3]) +
		    SQR(psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[1][3]));
    ang_att = atan2(psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[1][3],
		    psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[0][3]);
   }

  dist_axis = psdataPt->platform_jntPt->link_jnt_arr[psdataPt->index_link_platf[imanip]]->rel_pos[idof][3];
  p3d_mat4Copy(p3d_mat4IDENTITY,Taxis);
  Taxis[idof][3] = dist_axis;
  p3d_mat4Mult(Tbp,Taxis,Tbp_p);

  for(i=0; i<3; i++) {  
    vpbm_a[i] = psdataPt->manip_cntrt[imanip]->Tbase[i][3] - Tbp_p[i][3];
  }
  vpbm_a[3] = 0.0;

  // NOTA : HAY CALCULOS QUE SE PUEDEN SUPRIMIR PARA IROT=0 !!!!!!

  p3d_matInvertXform(Tbp_p,Tpb_p); 
  p3d_matvec4Mult(Tpb_p,vpbm_a,vpbm_r);

  for(i=0; i<3; i++) {
    vpbm_r_p[i] = vpbm_r[i];
    rotref[i] = 0.0;
    rotax[i] = 0.0;
  }
  rotax[idof] = 1.0;

  distance = sqrt(SQR(vpbm_r[0])+SQR(vpbm_r[1])+SQR(vpbm_r[2]));   

  ang = acos(p3d_vectDotProd(rotax,vpbm_r_p)/p3d_vectNorm(vpbm_r_p));   // p3d_vectNorm(rotax) = 1.0 !!!  
  dist_pp = fabs(distance * sin(ang));     /* pp -> perpendicular to axis */  
  dist_p = fabs(distance * cos(ang));      /* p  -> parallel to axis      */

  if(PRINTPROC_PS) {
    printf("distance = %f, dist_pp = %f, dist_p = %f\n",distance,dist_pp,dist_p);
  } 
  
  if(dist_p > psdataPt->manip_cntrt[imanip]->argu_d[MAX_ARGU_CNTRT - 1]) {
    return(FALSE);
  }

  maxext_p = sqrt(SQR(psdataPt->manip_cntrt[imanip]->argu_d[MAX_ARGU_CNTRT - 1]) - SQR(dist_p));
  if(psdataPt->manip_cntrt[imanip]->argu_d[MAX_ARGU_CNTRT - 2] > dist_p) 
    minext_p = sqrt(SQR(psdataPt->manip_cntrt[imanip]->argu_d[MAX_ARGU_CNTRT - 2]) - SQR(dist_p));
  else
    minext_p = 0.0;

  if(PRINTPROC_PS) {
    printf("maxext_p = %f, minext_p = %f\n",maxext_p,minext_p);
  } 

  if((dist_pp > maxext_p + dist_att) ||
     (dist_pp + dist_att < minext_p)) {
    return(FALSE);
  }
  
  if(dist_pp + dist_att <= maxext_p) {
    max_lim = M_PI;
  }
  else {
    max_lim = acos((SQR(dist_pp)+SQR(dist_att)-SQR(maxext_p))/(2*dist_pp*dist_att));
  }
  
  /* calculate min_lim of interval */
  if(minext_p > 0.0) {
    if(minext_p <= dist_pp - dist_att) {
      min_lim = 0.0;
    }
    else {
      min_lim = acos((SQR(dist_pp)+SQR(dist_att)-SQR(minext_p))/(2*dist_pp*dist_att));
    }
  }
  else { 
    min_lim = 0.0;
  }

  if(PRINTPROC_PS) {
    printf("max_lim = %f, min_lim = %f\n",max_lim,min_lim);
  } 

  /* angle ref - manip */
  vpbm_r_p[idof] = 0.0;
  if(idof == 0) {
    rotref[1] = -1.0;
  }
  else if (idof == 1) {
    rotref[0] = 1.0;
  }
  else {
    rotref[0] = 1.0;
  }

  angrefm = acos(p3d_vectDotProd(rotref,vpbm_r_p)/p3d_vectNorm(vpbm_r_p));  // p3d_vectNorm(rotref) = 1.0 !!!
  p3d_vectXprod(rotref,vpbm_r_p,vprod);
  p3d_vectNormalize(vprod,vprod_n);
  p3d_vectSub(rotax,vprod_n,vprod); 
  if((fabs(vprod[0])>0.0001)||(fabs(vprod[1])>0.0001)||(fabs(vprod[2])>0.0001))
    angrefm = -angrefm;    
 
  difangrefs = angrefm - ang_att - angrefb;

  if(PRINTPROC_PS) {
    printf("ref angles : manip = %f, ang_att = %f, base = %f, diff = %f\n",angrefm,ang_att,angrefb,difangrefs);
  } 

  /* intervals */
  if(min_lim == 0.0) {
    interv[0][0] = - max_lim + difangrefs;
    interv[0][1] = max_lim + difangrefs;
    interv[1][1] = 23.0;   // <- flag
  } 
  else {
    interv[0][0] = - max_lim + difangrefs;
    interv[0][1] = - min_lim + difangrefs;
    interv[1][0] = min_lim + difangrefs;
    interv[1][1] = max_lim + difangrefs;
  }
  
  /* correct case 2pi */
  if((interv[0][1] - interv[0][0]) >= ((2.0*M_PI) - EPS6)) {
    interv[0][0] = - M_PI;
    interv[0][1] = M_PI;
  }
  if(interv[0][0] < - (2.0*M_PI)) {
    interv[0][0] += (2.0*M_PI);
    interv[0][1] += (2.0*M_PI);
    if(interv[1][1] != 23.0) {
      interv[1][0] += (2.0*M_PI);
      interv[1][1] += (2.0*M_PI);
    }
  }
  else if((interv[1][1] != 23.0) && (interv[1][1] > (2.0*M_PI))) {
    interv[0][0] -= (2.0*M_PI);
    interv[0][1] -= (2.0*M_PI);
    interv[1][0] -= (2.0*M_PI);
    interv[1][1] -= (2.0*M_PI);
  }
  else if(interv[0][1] > (2.0*M_PI)) {
    interv[0][0] -= (2.0*M_PI);
    interv[0][1] -= (2.0*M_PI);
  }
  
  if(PRINTPROC_PS) {
    printf("interv1 = (%f, %f)\ninterv2 = (%f, %f)\n",interv[0][0],interv[0][1],interv[1][0],interv[1][1]);
  } 
  
  return TRUE;
}


static void p3d_update_Tbp(p3d_matrix4 Tbp, int ind_rot, double val)
{
  p3d_matrix4 Trot,Taux;
  p3d_vector3 axe;

  axe[0] = 0.0;  axe[1] = 0.0;  axe[2] = 0.0;
  axe[ind_rot] = 1.0;
    
  p3d_mat4Rot(Trot,axe,val);
  p3d_mat4Mult(Tbp,Trot,Taux);
  p3d_mat4Copy(Taux,Tbp);
}




/*--------------------------------------------------------------------------*/
/*! \ingroup parallel
 *
 *  \brief  Function that uses precomputed bounds to make the shoot
 *          of the configuration of the joint
 *
 *  \param  The parallel system structure data pointer
 *  \param  The configuration
 *
 *  \return TRUE if success
 *
 *  \internal  
 */
static int p3d_shoot_platform_in_precomputed_bounds(p3d_parallel *psdataPt, configPt q)
{
  int i;
  double val;
  
  for(i=0; i < psdataPt->platform_jntPt->dof_equiv_nbr; i++) {
#ifdef P3D_PLANNER
    val = p3d_random(psdataPt->platform_dof_min_bounds[i],psdataPt->platform_dof_max_bounds[i]);
#endif
    q[psdataPt->platform_jntPt->index_dof + i] = val;
  }

  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*! \ingroup parallel
 *
 *  \brief  Function that uses precomputed bounds to make the shoot
 *          of the position of a FREEFLYER joint
 *
 *  \param  The parallel system structure data pointer
 *
 *  \return TRUE if success
 *
 *  \internal  
 */
static int p3d_shoot_platform_position_in_precomputed_bounds(p3d_parallel *psdataPt, configPt q)
{
  int i;
  double val;
  
  for(i=0; i < 3; i++) {
#ifdef P3D_PLANNER
    val = p3d_random(psdataPt->platform_dof_min_bounds[i],psdataPt->platform_dof_max_bounds[i]);
#endif
    q[psdataPt->platform_jntPt->index_dof + i] = val;
  }

  return TRUE;
}


/*****************************************************************************/
/*****************************************************************************/

/* MATHEMATICAL FUNCTIONS */

static int factorial (int n)
{
  if (n == 0)
    return 1;
  else
    return (n * factorial(n-1));
}


static int combination_n_r (int n, int r)
{
  return (factorial(n) / (factorial(r) * factorial(n-r)));
}


static double maximum (double a, double b)
{
  if (a > b)
    return a;
  else
    return b;
}


/*****************************************************************************/
/*****************************************************************************/

/* GEOMETRIC FUNCTIONS */

static int p3d_associate_frame_to_triangle(p3d_vector3 X1, p3d_vector3 v12, p3d_vector3 v13, p3d_matrix4 Tframe)
{
  p3d_vector3 v12n,vz,vzn,vy,vyn;

  p3d_vectXprod(v12,v13,vz);
  p3d_vectXprod(vz,v12,vy);
  p3d_vectNormalize(v12,v12n);
  p3d_vectNormalize(vz,vzn);    
  p3d_vectNormalize(vy,vyn);    
  
  Tframe[0][0] = v12n[0];  Tframe[0][1] = vyn[0];  Tframe[0][2] = vzn[0];  Tframe[0][3] = X1[0]; 
  Tframe[1][0] = v12n[1];  Tframe[1][1] = vyn[1];  Tframe[1][2] = vzn[1];  Tframe[1][3] = X1[1]; 
  Tframe[2][0] = v12n[2];  Tframe[2][1] = vyn[2];  Tframe[2][2] = vzn[2];  Tframe[2][3] = X1[2]; 
  Tframe[3][0] =    0.0;  Tframe[3][1] =   0.0;  Tframe[3][2] =   0.0;  Tframe[3][3] =   1.0; 

  return TRUE;
}


/* see: http://mathworld.wolfram.com/TriangleInterior.html  for general equations */
/* returns :
   0 -> inside
   1 -> inside ang 1 (ouside 2,3)
   2 -> inside ang 2 (ouside 1,3)
   3 -> inside ang 3 (ouside 1,2)
   4 -> inside neg ang 1 
   5 -> inside neg ang 2 
   6 -> inside neg ang 3
  -1 -> error
*/ 
static int p3d_frame_proyection_outside_triangle(p3d_tri_attach *tri_att)
{
  double p1[2],p2[2],ppframe[2];
  double v0[2],v1[2],v2[2];
  double a,b,d;
  int in0=0,in1=0,in2=0,in0n=0,in1n=0,in2n=0;

  /* p0 = {0,0} */
  p1[0] = tri_att->l12;
  p1[1] = 0.0;
  p2[0] = tri_att->l13 * cos(tri_att->angle12);
  p2[1] = tri_att->l13 * sin(tri_att->angle12);  
  ppframe[0] = tri_att->Ttri2pp[0][3];
  ppframe[1] = tri_att->Ttri2pp[1][3];

  /* inside angle v0 ? */
  /* v0 = p0 = {0,0} */
  v1[0] = p1[0];
  v1[1] = p1[1];
  v2[0] = p2[0];
  v2[1] = p2[1];
  d = v1[0]*v2[1]-v1[1]*v2[0];
  a = (ppframe[0]*v2[1]-ppframe[1]*v2[0])/d;
  b = -(ppframe[0]*v1[1]-ppframe[1]*v1[0])/d;
  in0 = ((a >= 0.0)&&(b >= 0.0));
  if(!in0) {
    v1[0] = -v1[0];
    v1[1] = -v1[1];
    v2[0] = -v2[0];
    v2[1] = -v2[1];
    a = (ppframe[0]*v2[1]-ppframe[1]*v2[0])/d;
    b = -(ppframe[0]*v1[1]-ppframe[1]*v1[0])/d;
    in0n = ((a > 0.0)&&(b > 0.0));
  }
  /* inside angle v1 ? */
  v0[0] = p1[0];
  v0[1] = p1[1];
  v1[0] = p2[0] - p1[0];
  v1[1] = p2[1] - p1[1];
  v2[0] = -p1[0];
  v2[1] = -p1[1];
  d = v1[0]*v2[1]-v1[1]*v2[0];
  a = ((ppframe[0]*v2[1]-ppframe[1]*v2[0])-(v0[0]*v2[1]-v0[1]*v2[0]))/d;
  b = -((ppframe[0]*v1[1]-ppframe[1]*v1[0])-(v0[0]*v1[1]-v0[1]*v1[0]))/d;
  in1 = ((a >= 0.0)&&(b >= 0.0)); 
  if(!in1) {
    v1[0] = -v1[0];
    v1[1] = -v1[1];
    v2[0] = -v2[0];
    v2[1] = -v2[1];
    a = ((ppframe[0]*v2[1]-ppframe[1]*v2[0])-(v0[0]*v2[1]-v0[1]*v2[0]))/d;
    b = -((ppframe[0]*v1[1]-ppframe[1]*v1[0])-(v0[0]*v1[1]-v0[1]*v1[0]))/d;
    in1n = ((a > 0.0)&&(b > 0.0));
  }
  /* inside angle v2 ? */
  v0[0] = p2[0];
  v0[1] = p2[1];
  v1[0] = -p2[0];
  v1[1] = -p2[1];
  v2[0] = p1[0] - p2[0];
  v2[1] = p1[1] - p2[1];
  d = v1[0]*v2[1]-v1[1]*v2[0];
  a = ((ppframe[0]*v2[1]-ppframe[1]*v2[0])-(v0[0]*v2[1]-v0[1]*v2[0]))/d;
  b = -((ppframe[0]*v1[1]-ppframe[1]*v1[0])-(v0[0]*v1[1]-v0[1]*v1[0]))/d;
  in2 = ((a >= 0.0)&&(b >= 0.0)); 
  if(!in2) {
    v1[0] = -v1[0];
    v1[1] = -v1[1];
    v2[0] = -v2[0];
    v2[1] = -v2[1];
    a = ((ppframe[0]*v2[1]-ppframe[1]*v2[0])-(v0[0]*v2[1]-v0[1]*v2[0]))/d;
    b = -((ppframe[0]*v1[1]-ppframe[1]*v1[0])-(v0[0]*v1[1]-v0[1]*v1[0]))/d;
    in2n = ((a >  0.0)&&(b > 0.0));
  }

  /* cases */
  /* NOTE: if inside two of the angles -> inside the triangle ??? */
  if(in0 && in1 && in2) 
    return 0;
  else if (in0) 
    return 1;
  else if (in1) 
    return 2;
  else if (in2) 
    return 3;
  else if (in0n) 
    return 4;
  else if (in1n) 
    return 5;
  else if (in2n) 
    return 6;

  return -1;
}


/*****************************************************************************/
/*****************************************************************************/

/* INTERMEDIATE FUNCTIONS */

static void p3d_compute_triangles(p3d_parallel *psdataPt)
{
  int nm;
/*   int nt; */
  int i,j,k;
  p3d_tri_attach *tri_att,*last_tri=NULL;

  nm = psdataPt->n_manipulators;
/*   nt = combination_n_r(nm,3); */
  
  i = 0;
  while(i < nm-2) {
    j = i + 1;
    while(j < nm-1) {
      k = j + 1;
      while(k < nm) {
	tri_att = MY_ALLOC(p3d_tri_attach, 1);
	if ((i == 0)&&(j == 1)&&(k == 2)) {
	  psdataPt->triangles = tri_att;	
	}
	else {
	  last_tri->next = tri_att;
	}
	p3d_compute_one_triangle(psdataPt,i,j,k,tri_att);
	last_tri = tri_att;
	k++;
      }    
      j++;
    }
    i++;
  }
}


/* NOTE :
   we consider that the end-frame of the manipulator and the att_jnt have the SAME POSITION !!!
   -> Tatt can represent a rotation but NOT A TRANSLATION !!!
*/
static void p3d_compute_one_triangle(p3d_parallel *psdataPt, int i, int j, int k, p3d_tri_attach *tri_att)
{
  p3d_matrix4 Tabstri,invTabstri;
  p3d_vector3 X1,X2,X3;
  p3d_vector3 v12,v13,v23,v;
  p3d_jnt *att1,*att2,*att3;
  int ind;

  att1 = psdataPt->att_jnt[i];
  att2 = psdataPt->att_jnt[j];
  att3 = psdataPt->att_jnt[k];

  p3d_jnt_get_vect_point(att1,X1);
  p3d_jnt_get_vect_point(att2,X2);
  p3d_jnt_get_vect_point(att3,X3);
  
  p3d_vectSub(X2,X1,v12);
  p3d_vectSub(X3,X1,v13);
  p3d_vectSub(X3,X2,v23);

  /* make triangle normal pointing up */
  /* ES NECESARIO ? */
  p3d_vectXprod(v12,v13,v);
  /* OJO CON ESTO ! : 
     problema en el caso de que el triangulo este vertical
  */
  if(v[2] < 0.0) {
    /* invert order */
    ind = j;
    j = k;
    k = ind;
    p3d_vectCopy(v12,v);
    p3d_vectCopy(v13,v12);
    p3d_vectCopy(v,v13);
  } 

  tri_att->index_att[0] = i;
  tri_att->index_att[1] = j;
  tri_att->index_att[2] = k;

  tri_att->l12 = p3d_vectNorm(v12);
  tri_att->l13 = p3d_vectNorm(v13);
  tri_att->l23 = p3d_vectNorm(v23);
  tri_att->angle12 = acos(p3d_vectDotProd(v12,v13)/(p3d_vectNorm(v12)*p3d_vectNorm(v13)));

  p3d_associate_frame_to_triangle(X1,v12,v13,Tabstri);
  p3d_matInvertXform(Tabstri,invTabstri);
  p3d_mat4Mult(invTabstri,psdataPt->platform_jntPt->abs_pos,tri_att->Ttri2pp);

  tri_att->next = NULL;
}


static double p3d_compute_z_tri_frame(p3d_tri_attach *tri_att, double z_att1, double z_att2, double z_att3) 
{
  p3d_vector3 X1,v12,v13;  
  double theta1;
  p3d_matrix4 Tabspp,Tabstri;

  /* compute the frame of the triangle for those z's */
  X1[0] = 0.0;
  X1[1] = 0.0;
  X1[2] = z_att1;
  theta1 = asin((z_att2 - z_att1)/tri_att->l12);
  v12[0] = cos(theta1);
  v12[1] = 0.0;
  v12[2] = sin(theta1);
  /* the coordinates of the third vertex of the triangle are computed 
     by solving a system of two equations (see in my hand notes) */
  // VERIFICAR !!!
  v13[2] = z_att3 - z_att1;
  if(v12[0] != 0.0)
    v13[0] = ((cos(tri_att->angle12) * tri_att->l13) - (v12[2] * v13[2])) / v12[0];
  else
    v13[0] = 0.0;
  v13[1] = sqrt(SQR(tri_att->l13) - SQR(v13[0]) - SQR(v13[2]));

  p3d_associate_frame_to_triangle(X1,v12,v13,Tabstri);  

  /* obtain the frame of the parallel platform */
  p3d_mat4Mult(Tabstri,tri_att->Ttri2pp,Tabspp);
  
  /* return abs z coordinate */
  return (Tabspp[2][3]);
}


static double p3d_platform_zmin_in_tri_att(p3d_parallel *psdataPt, p3d_tri_attach *tri_att)
{
  int typetri;
  double zmin_att1,zmin_att2,zmin_att3;
  double zmax_att1,zmax_att2,zmax_att3;
  double zminpp=0.0;
  
  zmin_att1 = p3d_get_zmin_manipulator(psdataPt->manip_cntrt[tri_att->index_att[0]]);
  zmin_att2 = p3d_get_zmin_manipulator(psdataPt->manip_cntrt[tri_att->index_att[1]]);
  zmin_att3 = p3d_get_zmin_manipulator(psdataPt->manip_cntrt[tri_att->index_att[2]]);
  zmax_att1 = p3d_get_zmax_manipulator(psdataPt->manip_cntrt[tri_att->index_att[0]]);
  zmax_att2 = p3d_get_zmax_manipulator(psdataPt->manip_cntrt[tri_att->index_att[1]]);
  zmax_att3 = p3d_get_zmax_manipulator(psdataPt->manip_cntrt[tri_att->index_att[2]]);
    
  typetri = p3d_frame_proyection_outside_triangle(tri_att);
  switch (typetri) {
  case 0:
    zminpp = p3d_compute_z_tri_frame(tri_att,zmin_att1,zmin_att2,zmin_att3);
    break;
  case 1:
    zminpp = p3d_compute_z_tri_frame(tri_att,zmax_att1,zmin_att2,zmin_att3);
    break;
  case 2:
    zminpp = p3d_compute_z_tri_frame(tri_att,zmin_att1,zmax_att2,zmin_att3);
    break;
  case 3:
    zminpp = p3d_compute_z_tri_frame(tri_att,zmin_att1,zmin_att2,zmax_att3);
    break;
  case 4:
    zminpp = p3d_compute_z_tri_frame(tri_att,zmin_att1,zmax_att2,zmax_att3);
    break;
  case 5:
    zminpp = p3d_compute_z_tri_frame(tri_att,zmax_att1,zmin_att2,zmax_att3);
    break;
  case 6:
    zminpp = p3d_compute_z_tri_frame(tri_att,zmax_att1,zmax_att2,zmin_att3);
    break;
  }

  return zminpp;
}


static double p3d_platform_zmax_in_tri_att(p3d_parallel *psdataPt, p3d_tri_attach *tri_att)
{
  int typetri;
  double zmin_att1,zmin_att2,zmin_att3;
  double zmax_att1,zmax_att2,zmax_att3;
  double zmaxpp=0.0;
  
  zmin_att1 = p3d_get_zmin_manipulator(psdataPt->manip_cntrt[tri_att->index_att[0]]);
  zmin_att2 = p3d_get_zmin_manipulator(psdataPt->manip_cntrt[tri_att->index_att[1]]);
  zmin_att3 = p3d_get_zmin_manipulator(psdataPt->manip_cntrt[tri_att->index_att[2]]);
  zmax_att1 = p3d_get_zmax_manipulator(psdataPt->manip_cntrt[tri_att->index_att[0]]);
  zmax_att2 = p3d_get_zmax_manipulator(psdataPt->manip_cntrt[tri_att->index_att[1]]);
  zmax_att3 = p3d_get_zmax_manipulator(psdataPt->manip_cntrt[tri_att->index_att[2]]);
    
  typetri = p3d_frame_proyection_outside_triangle(tri_att);
  switch (typetri) {
  case 0:
    zmaxpp = p3d_compute_z_tri_frame(tri_att,zmax_att1,zmax_att2,zmax_att3);
    break;
  case 1:
    zmaxpp = p3d_compute_z_tri_frame(tri_att,zmin_att1,zmax_att2,zmax_att3);
    break;
  case 2:
    zmaxpp = p3d_compute_z_tri_frame(tri_att,zmax_att1,zmin_att2,zmax_att3);
    break;
  case 3:
    zmaxpp = p3d_compute_z_tri_frame(tri_att,zmax_att1,zmax_att2,zmin_att3);
    break;
  case 4:
    zmaxpp = p3d_compute_z_tri_frame(tri_att,zmax_att1,zmin_att2,zmin_att3);
    break;
  case 5:
    zmaxpp = p3d_compute_z_tri_frame(tri_att,zmin_att1,zmax_att2,zmin_att3);
    break;
  case 6:
    zmaxpp = p3d_compute_z_tri_frame(tri_att,zmin_att1,zmin_att2,zmax_att3);
    break;
  }
  
  return zmaxpp;
}


static double p3d_platform_thetamax_in_tri_att(p3d_parallel *psdataPt, p3d_tri_attach *tri_att)
{
  double zmin_att1,zmin_att2,zmin_att3;
  double zmax_att1,zmax_att2,zmax_att3;
  double dz,p_tabstri,tabstri,ttripp,tabspp;

  zmin_att1 = p3d_get_zmin_manipulator(psdataPt->manip_cntrt[tri_att->index_att[0]]);
  zmin_att2 = p3d_get_zmin_manipulator(psdataPt->manip_cntrt[tri_att->index_att[1]]);
  zmin_att3 = p3d_get_zmin_manipulator(psdataPt->manip_cntrt[tri_att->index_att[2]]);
  zmax_att1 = p3d_get_zmax_manipulator(psdataPt->manip_cntrt[tri_att->index_att[0]]);
  zmax_att2 = p3d_get_zmax_manipulator(psdataPt->manip_cntrt[tri_att->index_att[1]]);
  zmax_att3 = p3d_get_zmax_manipulator(psdataPt->manip_cntrt[tri_att->index_att[2]]);
 
  /* compute tabstri for the three edges */
  dz = maximum(fabs(zmax_att1-zmin_att2),fabs(zmax_att2-zmin_att1));
  if(dz > tri_att->l12)
    tabstri = M_PI / 2;
  else
    tabstri = asin(dz/tri_att->l12);
  
  dz = maximum(fabs(zmax_att1-zmin_att3),fabs(zmax_att3-zmin_att1));
  if(dz > tri_att->l13)
    p_tabstri = M_PI / 2;
  else
    p_tabstri = asin(dz/tri_att->l13);
  
  if(p_tabstri > tabstri)
    tabstri = p_tabstri;

  dz = maximum(fabs(zmax_att2-zmin_att3),fabs(zmax_att3-zmin_att2));
  if(dz > tri_att->l23)
    p_tabstri = M_PI / 2;
  else
    p_tabstri = asin(dz/tri_att->l23);
  
  if(p_tabstri > tabstri)
    tabstri = p_tabstri;

  /* add triangle-frame_pp inclination (+) */
  /* NOTES : - we consider frame_pp "horizontal" in the definition !!!
             - a further analysis of the orientation of frames will
	       provide a more accurate approximation
  */
  ttripp = acos(tri_att->Ttri2pp[2][2]);

  tabspp = tabstri + ttripp;

  return (tabspp);
}



/*****************************************************************************/
/*****************************************************************************/

/* FUNCTIONS FOR THE CNTRT AFFECTING THE PARALLEL PLATFORM (PASSIVE PLATFORM) */


/*--------------------------------------------------------------------------*/
/*! \ingroup parallel
 *
 *  \brief  Function setting the cntrt of the parallel platform when it is passive 
 *
 *  \param  the cntrt
 *  \param  the data of the currently setting parallel system
 *
 *  \return TRUE if success
 *
 *  \internal  
 */
static int p3d_set_passive_parallel_platform(p3d_cntrt *ct, p3d_parallel *psdataPt) 
{
  p3d_jnt *Jatt;
  int i,ilink;
  double dplatfs[MAX_N_MANIPULATORS];
  double mindist;
  p3d_matrix4 invT;

  /* only valid if freeflyer !!! */
  if((ct->parallel_sys_data->platform_jntPt->type != P3D_FREEFLYER) ||
     (psdataPt->platform_jntPt->type != P3D_FREEFLYER)) {
    return FALSE;
  }

  /* function generating the configuration */
  ct->fct_cntrt = p3d_fct_passive_parallel_platform;

  /* NOTES :
     - Attachment joint :
       in order to identify it, we suppose that the relative position to be kept
       between the platforms is "similar" to the initial position in the model
     - We consider same references (position and orientation) for both platforms !!! 
  */
  for(ilink=0; ilink < psdataPt->platform_jntPt->n_link_jnt; ilink++) {
    if(psdataPt->platform_jntPt == psdataPt->platform_jntPt->link_jnt_arr[ilink]->prev_jnt) {
      Jatt = psdataPt->platform_jntPt->link_jnt_arr[ilink]->next_jnt;
      dplatfs[ilink] = sqrt(SQR(Jatt->abs_pos[0][3] - ct->parallel_sys_data->platform_jntPt->abs_pos[0][3]) +
			    SQR(Jatt->abs_pos[1][3] - ct->parallel_sys_data->platform_jntPt->abs_pos[1][3]) +
			    SQR(Jatt->abs_pos[2][3] - ct->parallel_sys_data->platform_jntPt->abs_pos[2][3]));
    }
    else {
      dplatfs[ilink] = P3D_HUGE;
    }
  }  
  mindist = dplatfs[0];
  Jatt = psdataPt->platform_jntPt->link_jnt_arr[0]->next_jnt;
  for(ilink=1; ilink < psdataPt->platform_jntPt->n_link_jnt; ilink++) {  
    if(dplatfs[ilink] < mindist) {
      mindist = dplatfs[ilink];
      Jatt = psdataPt->platform_jntPt->link_jnt_arr[ilink]->next_jnt;
    }
  }

  /* active and passive jnts */
  ct->nactjnts = 2;
  ct->npasjnts = 6;  
  ct->actjnts[0] = Jatt;
  ct->actjnts[1] = psdataPt->platform_jntPt;
  for(i=0; i<6; i++) {
    ct->pasjnts[i] = ct->parallel_sys_data->platform_jntPt;
    ct->pas_jnt_dof[i] = i;
    ct->pas_rob_dof[i] = ct->pasjnts[0]->index_dof + i;
  }

  /* references and other data */
  ct->ndval = 0;
  ct->nival = 0;

  /* max. (min.) extension (distance between base and platform) */
  /* NOTES : 
     - We consider that bounds are computed in relation to the reference joint frame !!!
     - We consider that the z-axis is the perpendicular to both platforms !!!  
  */
  ct->argu_d[MAX_ARGU_CNTRT - 1] = ct->parallel_sys_data->platform_dof_max_bounds[2];
  ct->argu_d[MAX_ARGU_CNTRT - 2] = ct->parallel_sys_data->platform_dof_min_bounds[2];

  /* transformation between the preceding joint and the base platform */
  p3d_matInvertXform(ct->parallel_sys_data->base_jntPt->prev_jnt->abs_pos,invT); 
  p3d_mat4Mult(invT,ct->parallel_sys_data->base_jntPt->abs_pos,ct->Tbase);    

  /* deactivate (make NULL) the function for the platform shoot 
     and activate the cntrt */
  ct->parallel_sys_data->fct_parplatf_shoot = NULL;
  ct->active = TRUE;
  /* WARNING : last_cntrt_set is not updated 
       -> possible problems using FORMconstraits 
  */

  /* set flags in_cntrt for platform dofs -> passive */
  for(i=0; i <ct->parallel_sys_data->platform_jntPt->dof_equiv_nbr; i++) {
    ct->cntrt_manager->in_cntrt[ct->parallel_sys_data->platform_jntPt->index_dof + i] = 1;
  }

#ifdef P3D_CONSTRAINTS
  if (!p3d_update_jnts_state(ct->cntrt_manager, ct, 1))
    { return FALSE; }
#endif

  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*! \ingroup parallel
 *
 *  \brief  Function generating the configuration of the parallel platform
 *          when it is passive 
 *
 *  \param  the cntrt
 *
 *  \return TRUE if success
 *
 *  \internal  
 */
static int p3d_fct_passive_parallel_platform(p3d_cntrt *ct, int placeholder_i, configPt placeholder_c, double placeholder_d)
{
  double val,vmin,vmax;
  int i;

  /* NOTE : we should use att_jnts + transformations + ...
     and then extract valuers of dofs by p3d_mat4ExtractPosReverseOrder
  */

  p3d_update_this_robot_pos_without_cntrt_and_obj(ct->actjnts[1]->rob);
 
  /* translation */
  for(i=0; i<3; i++) {
    p3d_jnt_get_dof_bounds(ct->pasjnts[0],i,&vmin,&vmax);   
    val = (ct->actjnts[0]->abs_pos[i][3] - ct->pasjnts[0]->pos0[i][3]);  // OJO !!! : esto no es general !, hay qye evitar pos0 !!!
    if((val >= vmin) && (val <= vmax))
      p3d_jnt_set_dof(ct->pasjnts[0],i,val);
    else
      return FALSE;
  }
  /* rotation */
  /* same values !!! */
  for(i=3; i<6; i++) {
    p3d_jnt_get_dof_bounds(ct->pasjnts[0],i,&vmin,&vmax);   
    val = p3d_jnt_get_dof(ct->actjnts[1],i);
    if((val >= vmin) && (val <= vmax))
      p3d_jnt_set_dof(ct->pasjnts[0],i,val);
    else
      return FALSE;
  }
  return TRUE;
}



/*****************************************************************************/
/*****************************************************************************/

/* OTHER FUNCTIONS RELATED WITH CNTRTS */


/*--------------------------------------------------------------------------*/
/*! \ingroup parallel
 *
 *  \brief  Checks if the cntrt corresponds to a parallel system
 *
 *  \param  the cntrt
 *
 *  \return TRUE if success
 *
 */
int p3d_cntrt_is_parallel_sys(p3d_cntrt *ct)
{
  if((strcmp(ct->namecntrt,"parallel_sys_fixmanip")== 0) || 
     (strcmp(ct->namecntrt,"parallel_sys_mobmanip")== 0)) 
    return TRUE;
  else
    return FALSE;
}


/*--------------------------------------------------------------------------*/
/*! \ingroup parallel
 *
 *  \brief  Places the cntrt concerning the platform before the cntrts of the manipulators
 *
 *  \param  the cntrt_manager
 *  \param  the cntrt of the platform
 *
 *  \internal  
 */

static void p3d_modify_cntrt_call_list_for_parallel(p3d_cntrt_management *cntrt_manager, p3d_cntrt *ct)
{
  int i,ind,find;

  find = dbl_list_get_data_indice(cntrt_manager->cntrt_call_list,ct);
  dbl_list_remove_data(cntrt_manager->cntrt_call_list,ct);
  for(i=0; i<ct->parallel_sys_data->n_manipulators; i++) {
    ind = dbl_list_get_data_indice(cntrt_manager->cntrt_call_list,ct->parallel_sys_data->manip_cntrt[i]);
    if(ind < find)
      find = ind;
  }
  dbl_list_goto_n(cntrt_manager->cntrt_call_list,find);
  dbl_list_insert_link_before(cntrt_manager->cntrt_call_list,ct);
}
