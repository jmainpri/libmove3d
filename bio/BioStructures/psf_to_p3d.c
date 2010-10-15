

#include <string.h>

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Bio-pkg.h"


#define PI 3.14159265358979323846

#define PSF_P3D_PROT_MAX_NAME_LENGTH PSF_MAX_NAME_LENGTH
#define PSF_P3D_LIG_MAX_NAME_LENGTH PSF_MAX_NAME_LENGTH
#define PSF_P3D_MOL_MAX_NAME_LENGTH PSF_MAX_NAME_LENGTH
#define PSF_P3D_JNT_MAX_NAME_LENGTH JNT_MAX_SIZE_NAME
#define PSF_P3D_BODY_MAX_NAME_LENGTH PSF_MAX_NAME_LENGTH
#define PSF_P3D_SPHERE_MAX_NAME_LENGTH PSF_MAX_NAME_LENGTH

#define COLLISION_DETECTION_CHAR 'V'
#define NO_COLLISION_DETECTION_CHAR 'X'

/**********************************************************************/
// BOX DIMENSIONS FOR LIGANDS
#define XBOX_HALF 50.0
#define YBOX_HALF 50.0
#define ZBOX_HALF 50.0

/////////// PRIVATE VARIABLES //////////////////////////////////////////////////

// joints
static int global_indJ = 0;
static int bkb_ref_indJ = 0;

static int global_lig_indR = 0;

// AAA Residu Index
static int aaa_ind = 0;
// current AAA list
static AAA_protein_data_list* AAA_protein_List = NULL;


/////////// PROTOTYPE OF PRIVATE FUNCTIONS /////////////////////////////////////

static int get_p3d_atom_color(atomTypes atomType, int* colorType, double* colorVect);

static p3d_read_jnt_data* psf_beg_jnt_data(p3d_type_joint typeJ, double scale);

static void psf_set_jnt_name(p3d_read_jnt_data* data, const char* name);
static void psf_set_jnt_IndRef(p3d_read_jnt_data* data, int indRef);
static void psf_set_jnt_pos(p3d_read_jnt_data* data, double* pos);
static void psf_set_jnt_vDof(p3d_read_jnt_data* data, double* v);
static void psf_set_jnt_vminDof(p3d_read_jnt_data* data, double* vmin);
static void psf_set_jnt_vmaxDof(p3d_read_jnt_data* data, double* vmax);
static void psf_set_jnt_v0Dof(p3d_read_jnt_data* data, double* v0);
static int psf_check_jnt_data(p3d_read_jnt_data * data);
static int psf_build_jnt_data(p3d_read_jnt_data * data);
static int psf_create_p3d_joint( p3d_type_joint jointType, char *jointName, int indref, double* posAtom,
				 double* vDof, double* v0Dof, double* vDofmin, double* vDofmax);

static void psf_create_p3d_omega_jnt(psf_residue *resPt, int indrefbkb, int art_bkb);
static void psf_create_p3d_phi_jnt(psf_residue *resPt, int indrefbkb, int art_bkb);
static void  psf_create_p3d_psi_jnt(psf_residue *resPt, int indrefbkb, int art_bkb);
static void  psf_create_p3d_gamma_jnt(psf_residue *resPt,int indref,char *name, double *a1pos,
			    double *a2pos, double *a3pos, double *a4pos);

static void psf_set_p3d_protein_env_box(psf_protein *proteinPt);
static int psf_create_p3d_protein_base_joint(psf_protein* protPt, int numprot);
static int psf_create_p3d_peptide_chain(psf_AAchain* chain, int numprot);
static int psf_create_p3d_residue(psf_residue *resPt, int indRefbkb);
static void psf_create_p3d_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_atom(psf_atom *aPt);

static void psf_create_p3d_ALAH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_ARG_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_ARGH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_ASN_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_ASNH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_ASP_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_ASPH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_CYS_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_CYSH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_GLN_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_GLNH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_GLU_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_GLUH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_HIS_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_HISH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_ILE_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_ILEH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_LEU_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_LEUH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_LYS_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_LYSH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_MET_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_METH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_PHE_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_PHEH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_SER_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_SERH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_THR_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_THRH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_TRP_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_TRPH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_TYR_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_TYRH_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_VAL_sidechain(psf_residue *resPt, int art_sch, int wb);
static void psf_create_p3d_VALH_sidechain(psf_residue *resPt, int art_sch, int wb);

static void psf_create_p3d_body_with_all_atoms(psf_residue *resPt);
static void psf_create_p3d_body_with_first_bkb_rigid(psf_residue *resPt);
static void psf_create_p3d_body_with_second_bkb_rigid(psf_residue *resPt);
static void psf_create_p3d_body_with_third_bkb_rigid(psf_residue *resPt);


static psf_rigid *get_rigid_connected_to_rigid_by_joint(psf_rigid *rigidPt, psf_joint *jointPt);
static void insert_next_rigids_in_list(psf_rigid *rigidPt, psf_rigid ***rigidList, int *nrigids);

static void psf_set_p3d_ligand_env_box(psf_ligand *ligPt);
static int psf_create_p3d_ligand_base_joint(psf_ligand *ligPt, int numlig);
static int psf_create_p3d_ligand_kinematic_tree_from_root(psf_ligand *ligPt, int numlig, psf_rigid *rootrPt);
static int psf_create_p3d_joint_with_updates(psf_joint *jointPt, int indRef);
static void psf_create_p3d_rigid_with_updates(psf_rigid* rigidPt, int numlig);

static int psf_make_p3d_from_psf(psf_molecule* molecule);

/////////// GENERAL USEFUL FUNCTIONS ///////////////////////////////////////////

static int get_p3d_atom_color(atomTypes atomType, int* colorType, double* colorVect) {

  // default
  colorVect[0] = 255;
  colorVect[1] = 255;
  colorVect[2] = 255;
  *colorType = Any;
 
  switch(atomType) {
  case psf_CARBON:
    *colorType = Green;
    return TRUE;

  case psf_NITROGEN:
  case psf_NITROGEN_H:
  case psf_NITROGEN_FULL:
   *colorType = Blue;
     return TRUE;
 
  case psf_OXYGEN:
  case psf_OXYGEN_H:
    *colorType = Red;
    return TRUE;
  
  case psf_SULPHUR:
  case psf_SULPHUR_H:
    *colorType = Yellow;
     return TRUE;
 
  case psf_HYDROGEN:
    *colorType = White;
    return TRUE;
  
  case psf_FLUORINE: 
    *colorType = Blue2;
    return TRUE;
 
  case psf_BROMINE:
    *colorType = Any;
    colorVect[0] = 153;
    colorVect[1] = 83;
    colorVect[2] = 4;
    
    return TRUE;
  
  case psf_CHLORINE:
    *colorType = Any;   
    colorVect[0] = 166;
    colorVect[1] = 247;
    colorVect[2] = 116;
    return TRUE;
  
  case psf_PHOSPHORUS:
    *colorType = Any;   
    colorVect[0] = 174;
    colorVect[1] = 80;
    colorVect[2] = 224;
   return TRUE;
  
  case psf_IODINE:
    *colorType = Any;   
    colorVect[0] = 70;
    colorVect[1] = 12;
    colorVect[2] = 102;
    return TRUE;

  default:
    return FALSE;
  }  
}

/////////// WRITE P3D FROM PSF /////////////////////////////////////////////////////


// PROTEIN JOINT DESCRIPTION

static p3d_read_jnt_data* psf_beg_jnt_data(p3d_type_joint typeJ, double scale) {

  p3d_read_jnt_data* data;

  data = p3d_create_read_jnt_data(typeJ);
  if (data != NULL) {
    data->scale = scale;
  }

  return data;
}

/**********************************************************************/

static void psf_set_jnt_name(p3d_read_jnt_data* data, const char* name) {

  if (data->flag_name) {
    PrintWarning(("!!! WARNING : redefine the name of the joint !!!\n"));
  }
  
  strncpy(data->name, name, JNT_MAX_SIZE_NAME-1);
  data->flag_name = TRUE;

}

/**********************************************************************/

static void psf_set_jnt_IndRef(p3d_read_jnt_data* data, int indRef) {

  if (data->flag_prev_jnt) {
    PrintWarning(("!!! WARNING : redefine the previous joint !!!\n"));
  }
  
  data->prev_jnt = indRef;
  data->flag_prev_jnt = TRUE;
  
}

/**********************************************************************/

static void psf_set_jnt_pos(p3d_read_jnt_data* data, double* pos) {

  if (data->flag_pos) {
    PrintWarning(("!!! WARNING : redefine the joint position !!!\n"));
  }
  
  p3d_convert_axe_to_mat(data->pos, pos);
  data->flag_pos = TRUE;
  
}

/**********************************************************************/

static void psf_set_jnt_vDof(p3d_read_jnt_data* data, double* v) {

  int i;

  if (data->flag_v) {
    PrintWarning(("!!! WARNING : redefine the degree of freedom values !!!\n"));
  }
  
  for (i=0; i<data->nb_dof; i++) {
    data->v[i] = v[i];
  }

  data->flag_v = TRUE;
  
}

/**********************************************************************/

static void psf_set_jnt_vminDof(p3d_read_jnt_data* data, double* vmin) {

  int i;

  if (data->flag_vmin) {
    PrintWarning(("!!! WARNING : redefine the degree of freedom mininimal values !!!\n"));
  }
  
  for (i=0; i<data->nb_dof; i++) {
    data->vmin[i] = vmin[i];
  }

  data->flag_vmin = TRUE;
  
}

/**********************************************************************/

static void psf_set_jnt_vmaxDof(p3d_read_jnt_data* data, double* vmax) {

  int i;

  if (data->flag_vmax) {
    PrintWarning(("!!! WARNING : redefine the degree of freedom maximal values !!!\n"));
  }
  
  for (i=0; i<data->nb_dof; i++) {
    data->vmax[i] = vmax[i];
  }

  data->flag_vmax = TRUE;
  
}

/**********************************************************************/

static void psf_set_jnt_v0Dof(p3d_read_jnt_data* data, double* v0) {

  int i;

  if (data->flag_v_pos0) {
    PrintWarning(("!!! WARNING : redefine the degree of freedom values for the initial position !!!\n"));
  }
  
  for (i=0; i<data->nb_dof; i++) {
    data->v_pos0[i] = v0[i];
  }

  data->flag_v_pos0 = TRUE;
  
}

/**********************************************************************/

static int psf_check_jnt_data(p3d_read_jnt_data * data)
{
  int i;

  if (!(data->flag_pos)) {
    PrintWarning(("!!! ERROR position of the joint not given !!!\n"));
    return FALSE;
  }
  if (data->nb_dof>0) {
    if (!(data->flag_vmin)) {
      PrintWarning(("!!! ERROR joint minimum values not given !!!\n"));
      return FALSE;
    }
    if (!(data->flag_vmax)) {
      PrintWarning(("!!! ERROR joint maximum values not given !!!\n"));
      return FALSE;
    }
  }
  if (!((data->nb_param==0) || (data->flag_param))) {
    PrintWarning(("!!! ERROR joint parameters not given !!!\n"));
    return FALSE;
  }
  for (i=0; i<data->nb_dof; i++) {
    if (data->vmin[i]>data->vmax[i]) {
      PrintWarning(("!!! ERROR vmin[%i] > vmax[%i] !!!\n", i, i));
      return FALSE;
    }
  }
  if (!(data->flag_vmax_rand)) {
    for (i=0; i<data->nb_dof; i++)
      { data->vmax_rand[i] = data->vmax[i]; }
  }
  if (!(data->flag_vmin_rand)) {
    for (i=0; i<data->nb_dof; i++)
      { data->vmin_rand[i] = data->vmin[i]; }
  }
  for (i=0; i<data->nb_dof; i++) {
    if (data->vmin_rand[i]>data->vmax_rand[i]) {
      PrintWarning(("!!! ERROR vmin_rand[%i] > vmax_rand[%i] !!!\n", i, i));
      return FALSE;
    }
  }
  return TRUE;
}

/**********************************************************************/

static int psf_build_jnt_data(p3d_read_jnt_data * data)
{
  double * dofs, * dofs_rand;
  double * velocity_torque_max, *velAccJerk_max;
  int i;
  p3d_matrix4 tmp_mat;
  p3d_matrix4 inv_mat;
  p3d_jnt * jntPt;

  if (data->nb_links>0) {
    PrintWarning(("!!! WARNING: Joint links not allowed in the environment description !!!\n"));
    return FALSE;    
  }

  dofs      = MY_ALLOC(double, 3 * (data->nb_dof));
  dofs_rand = MY_ALLOC(double, 2 * (data->nb_dof));
  velocity_torque_max= MY_ALLOC(double, 2 * (data->nb_dof));
  velAccJerk_max = MY_ALLOC(double, data->nb_dof);
  if ((data->nb_dof>0) && ((dofs == NULL) || (dofs_rand == NULL))) {
    PrintError(("Not enough memory !!!\n"));
    return FALSE;
  }

  if (data->flag_relative_pos) {
    p3d_mat4Mult(XYZ_ROBOT->joints[data->prev_jnt]->pos0, data->pos, tmp_mat);
    p3d_mat4Copy(tmp_mat, data->pos);
  }

  for(i=0; i<data->nb_dof; i++) {
    dofs[3*i]        = data->v[i];
    dofs[3*i+1]      = data->vmin[i];
    dofs[3*i+2]      = data->vmax[i];
    dofs_rand[2*i]   = data->vmin_rand[i];
    dofs_rand[2*i+1] = data->vmax_rand[i];
    velocity_torque_max[2*i]= data->velocity_max[i];
    velocity_torque_max[2*i+1]= data->torque_max[i];
    velAccJerk_max[i] = 0.0;
  }

  p3d_add_desc_jnt_deg(data->type, data->pos, dofs, data->prev_jnt,
		       dofs_rand, data->scale, velocity_torque_max, velAccJerk_max, velAccJerk_max, velAccJerk_max);
  MY_FREE(dofs, double, 3 * (data->nb_dof));
  MY_FREE(dofs_rand, double, 2 * (data->nb_dof));

  jntPt = XYZ_ROBOT->joints[XYZ_ROBOT->njoints];

  if (data->flag_v_pos0) {
    for(i=0; i<data->nb_dof; i++)
      { p3d_jnt_set_dof_deg(jntPt, i, data->v_pos0[i]); }
    p3d_jnt_calc_jnt_mat(jntPt);
    p3d_matInvertXform(jntPt->jnt_mat, inv_mat);
    p3d_matMultXform(jntPt->pos0, inv_mat, data->pos);
    p3d_jnt_change_pos0(jntPt, data->pos);
    for(i=0; i<data->nb_dof; i++)
      { p3d_jnt_set_dof_deg(jntPt, i, data->v[i]); }
  }

  if (data->flag_name)
    { p3d_jnt_set_name(jntPt, data->name); }

  if (data->flag_is_user) {
    for(i=0; i<data->nb_dof; i++)      
      { p3d_jnt_set_dof_is_user(jntPt, i, data->is_user[i]); }
  }

  return TRUE;
}

/**********************************************************************/

// PROTEIN ROBOT DESCRIPTION

static int psf_create_p3d_protein_base_joint(psf_protein* protPt, int numprot) {

  int joint_built = FALSE;

  // Joint base (freeflyer) caracteristics
  psf_atom* firstAtom = protPt->chainList[0]->resList[0]->bkbAtomList[0];
  char jointName[PSF_P3D_JNT_MAX_NAME_LENGTH];
  int indRef = 0;
  double posAtom[] = { firstAtom->pos[0],
		       firstAtom->pos[1],
		       firstAtom->pos[2], 
		       0.0,
		       0.0,
		       0.0 };
  double vDof[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double vDofmin[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double vDofmax[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double v0Dof[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  
  // BASE JOINT
  // NOTE : A free-flying joint is defined as base-joint of a protein.
  //        By default, this joint is static.
  //        The frame origin corresponds with the origin of the first backbone atom (in the list)
  
  global_indJ++;

  snprintf(jointName, PSF_P3D_JNT_MAX_NAME_LENGTH-1, ".prot_base.%d", numprot);
  joint_built = psf_create_p3d_joint(P3D_FREEFLYER, jointName, indRef, posAtom, vDof, v0Dof, vDofmin, vDofmax);
 
  return (joint_built);

}

/***************************************************************/

static int psf_create_p3d_joint( p3d_type_joint jointType,
				 char *jointName,
				 int indRef,
				 double* posAtom,
				 double* vDof,
				 double* v0Dof,
				 double* vDofmin,
				 double* vDofmax) {

  int ready_to_build = FALSE;
  int joint_built = FALSE;
  p3d_read_jnt_data *data;

  // init data
  data = psf_beg_jnt_data(jointType, 1.0);   
  // name
  psf_set_jnt_name(data, jointName);
  // parent
  psf_set_jnt_IndRef(data, indRef);
  // axes description
  psf_set_jnt_pos(data, posAtom);
  // degrees of freedom description
  psf_set_jnt_vDof(data,vDof);
  psf_set_jnt_vminDof(data, vDofmin);
  psf_set_jnt_vmaxDof(data, vDofmax);
  psf_set_jnt_v0Dof(data, v0Dof);  
  
  // make joint from "data"
  ready_to_build = psf_check_jnt_data(data);
  
  if (!ready_to_build) {
    p3d_destroy_read_jnt_data(data);
    return FALSE;
  }
  
  joint_built = psf_build_jnt_data(data);
  p3d_destroy_read_jnt_data(data);
  

  if (!joint_built) {
    return FALSE;    
  }

  return TRUE;

}

/***************************************************************/

static void psf_create_p3d_omega_jnt(psf_residue *resPt, int indrefbkb, int art_bkb)
{
  char jointName[PSF_P3D_JNT_MAX_NAME_LENGTH];

  double *prev_CA_pos, *prev_C_pos, *N_pos, *CA_pos;
  double axis1[3], axis2[3], axis3[3];
  double posdiff[3];
  double pos_And_Axis[6];

  double dihedang;

  double vmin, vmax;

  get_CA_pos(resPt->prevResidue, &prev_CA_pos);
  get_C_pos(resPt->prevResidue, &prev_C_pos);
  get_N_pos(resPt, &N_pos);
  get_CA_pos(resPt, &CA_pos);

  vectSub(prev_C_pos, prev_CA_pos, posdiff);
  vectNormalize(posdiff, axis1);
  vectSub(N_pos,prev_C_pos, posdiff);
  vectNormalize(posdiff, axis2);
  vectSub(CA_pos,N_pos, posdiff);
  vectNormalize(posdiff, axis3);

  pos_And_Axis[0] = N_pos[0];
  pos_And_Axis[1] = N_pos[1];
  pos_And_Axis[2] = N_pos[2];
  pos_And_Axis[3] = axis2[0];
  pos_And_Axis[4] = axis2[1];
  pos_And_Axis[5] = axis2[2];

  dihedang = compute_dihedang(axis3,axis2,axis1)*(180/PI);
  
  if(art_bkb) {
    if(dihedang > 0.0) {
      if(dihedang > OMEGAMAX*(180/PI))
	vmax = dihedang;
      else
	vmax = OMEGAMAX*(180/PI);
      if(dihedang < OMEGAMIN*(180/PI))
	vmin = dihedang;
      else
	vmin = OMEGAMIN*(180/PI);      
    }
    else {
      if(dihedang > -OMEGAMIN*(180/PI))
	vmax = dihedang;
      else
	vmax = -OMEGAMIN*(180/PI);
      if(dihedang < -OMEGAMAX*(180/PI))
	vmin = dihedang;
      else
	vmin = -OMEGAMAX*(180/PI); 
    }	

  }
  else {
    vmin = vmax = dihedang;
  } 

  // update global_indJ
  global_indJ ++;

  snprintf(jointName, PSF_P3D_JNT_MAX_NAME_LENGTH-1, "omega.%s.%d", resPt->resName, resPt->resSeq);
  psf_create_p3d_joint(P3D_ROTATE, jointName, bkb_ref_indJ, pos_And_Axis, &dihedang, &dihedang, &vmin, &vmax);
 
  // update bkb_ref_indJ
  bkb_ref_indJ = global_indJ;  
}

/***************************************************************/

static void psf_create_p3d_phi_jnt(psf_residue *resPt, int indrefbkb, int art_bkb)
{
  char jointName[PSF_P3D_JNT_MAX_NAME_LENGTH];

  double *prev_C_pos, *N_pos, *CA_pos, *C_pos;
  double axis1[3], axis2[3], axis3[3];
  double posdiff[3];
  double pos_And_Axis[6];

  double dihedang;

  double vmin, vmax;

  int    canRef = 1;

  if(resPt->prevResidue != NULL) 
    get_C_pos(resPt->prevResidue,&prev_C_pos);
  else 
    canRef = 0;

  get_N_pos(resPt,&N_pos);
  get_CA_pos(resPt,&CA_pos);
  get_C_pos(resPt,&C_pos);

  vectSub(CA_pos,N_pos,posdiff);
  vectNormalize(posdiff,axis2);

  pos_And_Axis[0] = CA_pos[0];
  pos_And_Axis[1] = CA_pos[1];
  pos_And_Axis[2] = CA_pos[2];
  pos_And_Axis[3] = axis2[0];
  pos_And_Axis[4] = axis2[1];
  pos_And_Axis[5] = axis2[2];

  if (canRef) {
    vectSub(N_pos,prev_C_pos,posdiff);
    vectNormalize(posdiff,axis1);
    vectSub(C_pos,CA_pos,posdiff);
    vectNormalize(posdiff,axis3);

    // NOTE : for PROLINE : we consider phi fixed at the modeling value !!!
    //        Other possibility : including all atoms in first rigid
    //                            and do not writing jnt and second rigid

    dihedang = compute_dihedang(axis3,axis2,axis1)*(180/PI);

    if(art_bkb) {
      if((resPt->resType == PRO) || (resPt->resType == PROH)) {
      	vmin = vmax = dihedang;
      }
      else {
	vmin = -180.0;
	vmax =  180.0;
      }
    }
    else {
      vmin = vmax = dihedang;
    } 
  }
  else {
    // NOTE : this angle could be variable, but it can't be refered
    vmin = vmax = dihedang = 0.0;
  }

  // update global_indJ
  global_indJ ++;

  snprintf(jointName, PSF_P3D_JNT_MAX_NAME_LENGTH-1, "phi.%s.%d", resPt->resName, resPt->resSeq);
  psf_create_p3d_joint(P3D_ROTATE, jointName, bkb_ref_indJ, pos_And_Axis, &dihedang, &dihedang, &vmin, &vmax);
 
  // update bkb_ref_indJ
  bkb_ref_indJ = global_indJ;  

}

/***************************************************************/

static void  psf_create_p3d_psi_jnt(psf_residue *resPt, int indrefbkb, int art_bkb)
{
  char jointName[PSF_P3D_JNT_MAX_NAME_LENGTH];

  double *N_pos,*CA_pos,*C_pos,*next_N_pos;
  double axis1[3], axis2[3], axis3[3];
  double posdiff[3];
  double pos_And_Axis[6];

  double dihedang;

  double vmin, vmax;

  int    canRef = 1;

  get_N_pos(resPt,&N_pos);
  get_CA_pos(resPt,&CA_pos);
  get_C_pos(resPt,&C_pos);
  if(resPt->nextResidue != NULL)
    get_N_pos(resPt->nextResidue,&next_N_pos);
  else
    canRef = 0;

  vectSub(C_pos,CA_pos,posdiff);
  vectNormalize(posdiff,axis2);

  pos_And_Axis[0] = C_pos[0];
  pos_And_Axis[1] = C_pos[1];
  pos_And_Axis[2] = C_pos[2];
  pos_And_Axis[3] = axis2[0];
  pos_And_Axis[4] = axis2[1];
  pos_And_Axis[5] = axis2[2];
  
  if(canRef) {
    vectSub(CA_pos,N_pos,posdiff);
    vectNormalize(posdiff,axis1);
    vectSub(next_N_pos,C_pos,posdiff);
    vectNormalize(posdiff,axis3);

    dihedang = compute_dihedang(axis3,axis2,axis1)*(180/PI);
    if(art_bkb) {
      vmin = -180.0;
      vmax =  180.0;
    }
    else {
      vmin = vmax = dihedang;
    } 
  }
  else {
    // NOTE : this angle could be variable, but it can't be refered
    vmin = vmax = dihedang = 0.0;
  }

  // update global_indJ
  global_indJ ++;

  snprintf(jointName, PSF_P3D_JNT_MAX_NAME_LENGTH-1, "psi.%s.%d", resPt->resName, resPt->resSeq);
  psf_create_p3d_joint(P3D_ROTATE, jointName, bkb_ref_indJ, pos_And_Axis, &dihedang, &dihedang, &vmin, &vmax);
 
  // update bkb_ref_indJ
  bkb_ref_indJ = global_indJ;  
}

/***************************************************************/

static void  psf_create_p3d_gamma_jnt(psf_residue *resPt, int indRef, char *name, double *a1pos, double *a2pos, double *a3pos, double *a4pos) {

  char jointName[PSF_P3D_JNT_MAX_NAME_LENGTH];

  double axis1[3],axis2[3],axis3[3];
  double posdiff[3];
  double  pos_And_Axis[6];

  double dihedang;

  double vmin, vmax;

  vectSub(a2pos,a1pos,posdiff);
  vectNormalize(posdiff,axis1);
  vectSub(a3pos,a2pos,posdiff);
  vectNormalize(posdiff,axis2);
  vectSub(a4pos,a3pos,posdiff);
  vectNormalize(posdiff,axis3);

  pos_And_Axis[0] = a3pos[0];
  pos_And_Axis[1] = a3pos[1];
  pos_And_Axis[2] = a3pos[2];
  pos_And_Axis[3] = axis2[0];
  pos_And_Axis[4] = axis2[1];
  pos_And_Axis[5] = axis2[2];
  
  dihedang = compute_dihedang(axis3,axis2,axis1)*(180/PI);
  vmin = -180.0;
  vmax =  180.0;
  
  // update global_indJ
  global_indJ ++;
  
  snprintf(jointName, PSF_P3D_JNT_MAX_NAME_LENGTH-1, "%s.%s.%d", name, resPt->resName, resPt->resSeq);
  psf_create_p3d_joint(P3D_ROTATE, jointName, indRef, pos_And_Axis, &dihedang, &dihedang, &vmin, &vmax);

}

/***************************************************************/

static int psf_create_p3d_residue(psf_residue *resPt, int indRefbkb) {
  
  int AAA_index;
  int art_bkb;
  int art_sch;

  if (!get_AAA_index_by_protein_Name(AAA_protein_List, resPt->chainPt->proteinPt->name, &AAA_index)) {
    printf("ERROR : no AAA datas for %s\n", resPt->chainPt->proteinPt->name);
    return FALSE;    
  }

  if (!read_Art(AAA_protein_List->AAA_proteinList[AAA_index], aaa_ind, &art_bkb, &art_sch)) {
    printf("ERROR : reading aaa data : residue (pdb) serial % d\n", resPt->resSeq);
    return FALSE;
  }
  aaa_ind++;
  
  // joint omega between two residues
  if(resPt->prevResidue != NULL) {
    // next function modifies bkb_ref_indJ
    psf_create_p3d_omega_jnt(resPt, indRefbkb, art_bkb);
  }
  else {
    bkb_ref_indJ = indRefbkb;
  }

  if ((art_bkb == 0)&&(art_sch == 0)) {
    // all the atoms form one onlty body if everithing is rigid
    psf_create_p3d_body_with_all_atoms(resPt);
  }
  else {
    // NOTE : Move3D (function p3d_jnt_set_object) can now 
    //        accept the definition of several bodies without  
    //        joints between them

    // NOTE : BCD requires particular order in the .p3d file
    //        thus, when art_sch==1 (even if art_bkb==0)
    //        the joint psi (fixed or not) is necessary

    psf_create_p3d_body_with_first_bkb_rigid(resPt);
    if(art_bkb == 1) {
      psf_create_p3d_phi_jnt(resPt, indRefbkb, art_bkb);
    }
    psf_create_p3d_body_with_second_bkb_rigid(resPt);
    psf_create_p3d_sidechain(resPt, art_sch, 1);
    psf_create_p3d_psi_jnt(resPt, indRefbkb, art_bkb);
    psf_create_p3d_body_with_third_bkb_rigid(resPt);
  }
  
  return TRUE;
}

/***************************************************************/

static void psf_create_p3d_atom(psf_atom *aPt)
{
  char sphereName[PSF_P3D_SPHERE_MAX_NAME_LENGTH];

  int color;
  double colorVect[3];

  int polyType = P3D_REAL;
  char collision_Detection_Type = COLLISION_DETECTION_CHAR;

  p3d_poly *poly;

  if (aPt->atomType == psf_HYDROGEN_P) {
    collision_Detection_Type = NO_COLLISION_DETECTION_CHAR;
    polyType = P3D_GHOST;
  }

  if (!get_p3d_atom_color(aPt->atomType, &color, colorVect))
    printf("WARNING : no color for atom  %s %d\n", aPt->name, aPt->serial);

  snprintf(sphereName, PSF_P3D_SPHERE_MAX_NAME_LENGTH-1, "%c-%s.%d", collision_Detection_Type, aPt->name, aPt->serial);
  p3d_add_desc_sphere(sphereName, aPt->vdwR, polyType);
  
  // in case the real name of the sphere is shorter than the one
  // constructed with the previous snprintf
  poly = p3d_poly_get_poly_by_name(sphereName);
  
  p3d_set_prim_pos(poly, aPt->pos[0], aPt->pos[1], aPt->pos[2], 0.0, 0.0, 0.0);
  
  if (color == Any)
    p3d_poly_set_color(poly, Any, colorVect); // BE CAREFULL : Any is a constant, defined in 
                                              //               p3d_type.h
  else {
    p3d_poly_set_color(poly, color, NULL); 
  }

}

/**********************************************************************/

static void psf_create_p3d_ALAH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos, *a2pos, *a3pos, *a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : 1HB
    a4pos = resPt->schAtomList[psf_ALAH_1HB]->pos;

    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt, bkb_ref_indJ, (char*)"gamma1", a1pos, a2pos, a3pos, a4pos);
  }

  // write rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
    
  } 
  psf_create_p3d_atom(resPt->schAtomList[psf_ALAH_1HB]);
  psf_create_p3d_atom(resPt->schAtomList[psf_ALAH_2HB]);
  psf_create_p3d_atom(resPt->schAtomList[psf_ALAH_3HB]);
  if(wb) {
    p3d_end_desc();
  }   
}

/***************************************************************/

static void psf_create_p3d_ARG_sidechain(psf_residue *resPt, int art_sch, int wb)
{  
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_ARG_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
    
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ARG_CG]);    
  if(wb) {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_ARG_CG]->pos;
    // NOTE : ref for dihedang : CD
    a4pos = resPt->schAtomList[psf_ARG_CD]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb)  {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
    
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ARG_CD]);    
  if(wb) {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3 
    a1pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a2pos = resPt->schAtomList[psf_ARG_CG]->pos;
    a3pos = resPt->schAtomList[psf_ARG_CD]->pos;
    // NOTE : ref for dihedang : NE
    a4pos = resPt->schAtomList[psf_ARG_NE]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma3",a1pos,a2pos,a3pos,a4pos);
  }

  // write third sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-3.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
    
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ARG_NE]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma4 
    a1pos = resPt->schAtomList[psf_ARG_CG]->pos;
    a2pos = resPt->schAtomList[psf_ARG_CD]->pos;
    a3pos = resPt->schAtomList[psf_ARG_NE]->pos;
    // NOTE : ref for dihedang : CZ
    a4pos = resPt->schAtomList[psf_ARG_CZ]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma4",a1pos,a2pos,a3pos,a4pos);
  }

  // write fourth sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-4.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ARG_CZ]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma5 
    a1pos = resPt->schAtomList[psf_ARG_CD]->pos;
    a2pos = resPt->schAtomList[psf_ARG_NE]->pos;
    a3pos = resPt->schAtomList[psf_ARG_CZ]->pos;
    // NOTE : ref for dihedang : NH1
    a4pos = resPt->schAtomList[psf_ARG_NH1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma5",a1pos,a2pos,a3pos,a4pos);
  }

  // write fifth sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-5.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ARG_NH1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ARG_NH2]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_ARGH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    // NOTE : ref for dihedang : CG
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a4pos = resPt->schAtomList[psf_ARGH_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_2HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_CG]);    
  if(wb)  {
    p3d_end_desc();
  }


  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_ARGH_CG]->pos;
    // NOTE : ref for dihedang : CD
    a4pos = resPt->schAtomList[psf_ARGH_CD]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_1HG]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_2HG]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_CD]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3 
    a1pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a2pos = resPt->schAtomList[psf_ARGH_CG]->pos;
    a3pos = resPt->schAtomList[psf_ARGH_CD]->pos;
    // NOTE : ref for dihedang : NE
    a4pos = resPt->schAtomList[psf_ARGH_NE]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma3",a1pos,a2pos,a3pos,a4pos);
  }

  // write third sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-3.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_1HD]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_2HD]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_NE]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma4 
    a1pos = resPt->schAtomList[psf_ARGH_CG]->pos;
    a2pos = resPt->schAtomList[psf_ARGH_CD]->pos;
    a3pos = resPt->schAtomList[psf_ARGH_NE]->pos;
    // NOTE : ref for dihedang : CZ
    a4pos = resPt->schAtomList[psf_ARGH_CZ]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma4",a1pos,a2pos,a3pos,a4pos);
  }

  // write fourth sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-4.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_CZ]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma5 
    a1pos = resPt->schAtomList[psf_ARGH_CD]->pos;
    a2pos = resPt->schAtomList[psf_ARGH_NE]->pos;
    a3pos = resPt->schAtomList[psf_ARGH_CZ]->pos;
    // NOTE : ref for dihedang : NH1
    a4pos = resPt->schAtomList[psf_ARGH_NH1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma5",a1pos,a2pos,a3pos,a4pos);
  }

  // write fifth sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-5.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_NH1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_NH2]);    
  // H' atoms
  if(resPt->schAtomList[psf_ARGH_1HH1]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_1HH1]);}   
  if(resPt->schAtomList[psf_ARGH_2HH1]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_2HH1]);}    
  if(resPt->schAtomList[psf_ARGH_1HH2]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_1HH2]);}    
  if(resPt->schAtomList[psf_ARGH_2HH2]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_2HH2]);}    
  if(resPt->schAtomList[psf_ARGH_HE]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_ARGH_HE]);}    
  if(wb)  {
    p3d_end_desc();
  }

}


/***************************************************************/
static void psf_create_p3d_ASN_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_ASN_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ASN_CG]);    
  if(wb)  {
    p3d_end_desc();
  }


  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_ASN_CG]->pos;
    // NOTE : ref for dihedang : OD1
    a4pos = resPt->schAtomList[psf_ASN_OD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ASN_OD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ASN_ND2]);      
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_ASNH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_ASNH_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ASNH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ASNH_2HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ASNH_CG]);    
  if(wb)  {
    p3d_end_desc();
  }


  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_ASNH_CG]->pos;
    // NOTE : ref for dihedang : OD1
    a4pos = resPt->schAtomList[psf_ASNH_OD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ASNH_OD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ASNH_ND2]);      
  // H' atoms
  if(resPt->schAtomList[psf_ASNH_1HD2]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_ASNH_1HD2]);}
  if(resPt->schAtomList[psf_ASNH_2HD2]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_ASNH_2HD2]);}
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_ASP_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_ASP_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ASP_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_ASP_CG]->pos;
    // NOTE : ref for dihedang : OD1
    a4pos = resPt->schAtomList[psf_ASP_OD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ASP_OD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ASP_OD2]);      
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_ASPH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_ASPH_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ASPH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ASPH_2HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ASPH_CG]);    
  if(wb)  {
    p3d_end_desc();
  }


  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_ASPH_CG]->pos;
    // NOTE : ref for dihedang : OD1
    a4pos = resPt->schAtomList[psf_ASPH_OD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ASPH_OD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ASPH_OD2]);      
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_CYS_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : SG
    a4pos = resPt->schAtomList[psf_CYS_SG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_CYS_SG]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_CYSH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : SG
    a4pos = resPt->schAtomList[psf_CYSH_SG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_CYSH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_CYSH_2HB]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_CYSH_SG]);    
  // H' atoms
  if (resPt->schAtomList[psf_CYSH_HG]) psf_create_p3d_atom(resPt->schAtomList[psf_CYSH_HG]);    
  if(wb)  {
    p3d_end_desc();
  }

}


/***************************************************************/

static void psf_create_p3d_GLN_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_GLN_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_GLN_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_GLN_CG]->pos;
    // NOTE : ref for dihedang : CD
    a4pos = resPt->schAtomList[psf_GLN_CD]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_GLN_CD]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3 
    a1pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a2pos = resPt->schAtomList[psf_GLN_CG]->pos;
    a3pos = resPt->schAtomList[psf_GLN_CD]->pos;
    // NOTE : ref for dihedang : OE1
    a4pos = resPt->schAtomList[psf_GLN_OE1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma3",a1pos,a2pos,a3pos,a4pos);
  }

  // write third sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-3.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_GLN_OE1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_GLN_NE2]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_GLNH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_GLNH_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_GLNH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_GLNH_2HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_GLNH_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_GLNH_CG]->pos;
    // NOTE : ref for dihedang : CD
    a4pos = resPt->schAtomList[psf_GLNH_CD]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_GLNH_1HG]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_GLNH_2HG]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_GLNH_CD]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3 
    a1pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a2pos = resPt->schAtomList[psf_GLNH_CG]->pos;
    a3pos = resPt->schAtomList[psf_GLNH_CD]->pos;
    // NOTE : ref for dihedang : OE1
    a4pos = resPt->schAtomList[psf_GLNH_OE1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma3",a1pos,a2pos,a3pos,a4pos);
  }

  // write third sch rigid
  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-3.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_GLNH_OE1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_GLNH_NE2]);    
  // H' atoms
  if(resPt->schAtomList[psf_GLNH_1HE2]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_GLNH_1HE2]);}
  if(resPt->schAtomList[psf_GLNH_2HE2]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_GLNH_2HE2]);}
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_GLU_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG
   a4pos = resPt->schAtomList[psf_GLU_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, (char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_GLU_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_GLU_CG]->pos;
    // NOTE : ref for dihedang : CD
    a4pos = resPt->schAtomList[psf_GLU_CD]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_GLU_CD]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3 
    a1pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a2pos = resPt->schAtomList[psf_GLU_CG]->pos;
    a3pos = resPt->schAtomList[psf_GLU_CD]->pos;
    // NOTE : ref for dihedang : OE1
    a4pos = resPt->schAtomList[psf_GLU_OE1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma3",a1pos,a2pos,a3pos,a4pos);
  }

  // write third sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-3.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_GLU_OE1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_GLU_OE2]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_GLUH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_GLUH_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_GLUH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_GLUH_2HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_GLUH_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_GLUH_CG]->pos;
    // NOTE : ref for dihedang : CD
    a4pos = resPt->schAtomList[psf_GLUH_CD]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_GLUH_1HG]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_GLUH_2HG]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_GLUH_CD]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3 
    a1pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a2pos = resPt->schAtomList[psf_GLUH_CG]->pos;
    a3pos = resPt->schAtomList[psf_GLUH_CD]->pos;
    // NOTE : ref for dihedang : OE1
    a4pos = resPt->schAtomList[psf_GLUH_OE1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma3",a1pos,a2pos,a3pos,a4pos);
  }

  // write third sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-3.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_GLUH_OE1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_GLUH_OE2]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_HIS_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_HIS_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_HIS_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_HIS_CG]->pos;
    // NOTE : ref for dihedang : ND1
    a4pos = resPt->schAtomList[psf_HIS_ND1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName,  PSF_P3D_BODY_MAX_NAME_LENGTH-1,"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_HIS_ND1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_HIS_CD2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_HIS_CE1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_HIS_NE2]);      
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_HISH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_HISH_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_HISH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_HISH_2HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_HISH_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_HISH_CG]->pos;
    // NOTE : ref for dihedang : ND1
    a4pos = resPt->schAtomList[psf_HISH_ND1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_HISH_ND1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_HISH_CD2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_HISH_HD2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_HISH_CE1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_HISH_NE2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_HISH_HE1]);
  // H' atoms      
  if (resPt->schAtomList[psf_HISH_HD1]) {psf_create_p3d_atom(resPt->schAtomList[psf_HISH_HD1]);}
  if (resPt->schAtomList[psf_HISH_HE2]) {psf_create_p3d_atom(resPt->schAtomList[psf_HISH_HE2]);}
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_ILE_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG1
    a4pos = resPt->schAtomList[psf_ILE_CG1]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ILE_CG2]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ILE_CG1]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_ILE_CG1]->pos;
    // NOTE : ref for dihedang : CD1
    a4pos = resPt->schAtomList[psf_ILE_CD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ILE_CD1]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_ILEH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : CG1
    a4pos = resPt->schAtomList[psf_ILEH_CG1]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ILEH_CG2]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ILEH_HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ILEH_CG1]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2-2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_ILEH_CG2]->pos;
    // NOTE : ref for dihedang : 1HG2
    a4pos = resPt->schAtomList[psf_ILEH_1HG2]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2-2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second-2 sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ILEH_1HG2]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ILEH_2HG2]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ILEH_3HG2]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2-1 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_ILEH_CG1]->pos;
    // NOTE : ref for dihedang : CD1
    a4pos = resPt->schAtomList[psf_ILEH_CD1]->pos;
    // gamma2-1 is refered to the gamma1 (global_indJ - 1)
    psf_create_p3d_gamma_jnt(resPt,(global_indJ - 1),(char*)"gamma2-1",a1pos,a2pos,a3pos,a4pos);
  }

  // write second-1 sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ILEH_1HG1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ILEH_2HG1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ILEH_CD1]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3 
    a1pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a2pos = resPt->schAtomList[psf_ILEH_CG1]->pos;
    a3pos = resPt->schAtomList[psf_ILEH_CD1]->pos;
    // NOTE : ref for dihedang : 1HD1
    a4pos = resPt->schAtomList[psf_ILEH_1HD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma3",a1pos,a2pos,a3pos,a4pos);
  }

  // write third sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-3.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_ILEH_1HD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ILEH_2HD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_ILEH_3HD1]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_LEU_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_LEU_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LEU_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_LEU_CG]->pos;
    // NOTE : ref for dihedang : CD1
    a4pos = resPt->schAtomList[psf_LEU_CD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LEU_CD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LEU_CD2]);      
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_LEUH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_LEUH_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LEUH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LEUH_2HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LEUH_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_LEUH_CG]->pos;
    // NOTE : ref for dihedang : CD1
    a4pos = resPt->schAtomList[psf_LEUH_CD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LEUH_HG]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LEUH_CD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LEUH_CD2]);      
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3-1 
    a1pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a2pos = resPt->schAtomList[psf_LEUH_CG]->pos;
    a3pos = resPt->schAtomList[psf_LEUH_CD1]->pos;
    // NOTE : ref for dihedang : 1HD1
    a4pos = resPt->schAtomList[psf_LEUH_1HD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma3-1",a1pos,a2pos,a3pos,a4pos);
  }

  // write third-1 sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-3-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LEUH_1HD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LEUH_2HD1]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_LEUH_3HD1]);      
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3-2 
    a1pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a2pos = resPt->schAtomList[psf_LEUH_CG]->pos;
    a3pos = resPt->schAtomList[psf_LEUH_CD2]->pos;
    // NOTE : ref for dihedang : 1HD2
    a4pos = resPt->schAtomList[psf_LEUH_1HD2]->pos;
    // gamma3-2 is refered to the gamma2 (global_indJ - 1)
    psf_create_p3d_gamma_jnt(resPt,(global_indJ - 1),(char*)"gamma3-2",a1pos,a2pos,a3pos,a4pos);
  }

  // write third-2 sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-3-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LEUH_1HD2]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LEUH_2HD2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_LEUH_3HD2]);      
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_LYS_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_LYS_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LYS_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_LYS_CG]->pos;
    // NOTE : ref for dihedang : CD
    a4pos = resPt->schAtomList[psf_LYS_CD]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LYS_CD]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3 
    a1pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a2pos = resPt->schAtomList[psf_LYS_CG]->pos;
    a3pos = resPt->schAtomList[psf_LYS_CD]->pos;
    // NOTE : ref for dihedang : CE
    a4pos = resPt->schAtomList[psf_LYS_CE]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma3",a1pos,a2pos,a3pos,a4pos);
  }

  // write third sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-3.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LYS_CE]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma4 
    a1pos = resPt->schAtomList[psf_LYS_CG]->pos;
    a2pos = resPt->schAtomList[psf_LYS_CD]->pos;
    a3pos = resPt->schAtomList[psf_LYS_CE]->pos;
    // NOTE : ref for dihedang : NZ
    a4pos = resPt->schAtomList[psf_LYS_NZ]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma4",a1pos,a2pos,a3pos,a4pos);
  }

  // write fourth sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-4.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LYS_NZ]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_LYSH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    // NOTE : ref for dihedang : CG
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a4pos = resPt->schAtomList[psf_LYSH_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_2HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_LYSH_CG]->pos;
    // NOTE : ref for dihedang : CD
    a4pos = resPt->schAtomList[psf_LYSH_CD]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_1HG]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_2HG]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_CD]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3 
    a1pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a2pos = resPt->schAtomList[psf_LYSH_CG]->pos;
    a3pos = resPt->schAtomList[psf_LYSH_CD]->pos;
    // NOTE : ref for dihedang : CE
    a4pos = resPt->schAtomList[psf_LYSH_CE]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma3",a1pos,a2pos,a3pos,a4pos);
  }

  // write third sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-3.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_1HD]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_2HD]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_CE]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma4 
    a1pos = resPt->schAtomList[psf_LYSH_CG]->pos;
    a2pos = resPt->schAtomList[psf_LYSH_CD]->pos;
    a3pos = resPt->schAtomList[psf_LYSH_CE]->pos;
    // NOTE : ref for dihedang : NZ
    a4pos = resPt->schAtomList[psf_LYSH_NZ]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma4",a1pos,a2pos,a3pos,a4pos);
  }

  // write fourth sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-4.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_1HE]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_2HE]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_NZ]);    
  // H' atoms
  if(resPt->schAtomList[psf_LYSH_1HZ]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_1HZ]);}
  if(resPt->schAtomList[psf_LYSH_2HZ]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_LYSH_2HZ]);}
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_MET_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_MET_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_MET_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_MET_CG]->pos;
    // NOTE : ref for dihedang : SD
    a4pos = resPt->schAtomList[psf_MET_SD]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_MET_SD]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3 
    a1pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a2pos = resPt->schAtomList[psf_MET_CG]->pos;
    a3pos = resPt->schAtomList[psf_MET_SD]->pos;
    // NOTE : ref for dihedang : CE
    a4pos = resPt->schAtomList[psf_MET_CE]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma3",a1pos,a2pos,a3pos,a4pos);
  }

  // write third sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-3.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_MET_CE]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_METH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    // NOTE : ref for dihedang : CG
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a4pos = resPt->schAtomList[psf_METH_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_METH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_METH_2HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_METH_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_METH_CG]->pos;
    // NOTE : ref for dihedang : SD
    a4pos = resPt->schAtomList[psf_METH_SD]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_METH_1HG]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_METH_2HG]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_METH_SD]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma3 
    a1pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a2pos = resPt->schAtomList[psf_METH_CG]->pos;
    a3pos = resPt->schAtomList[psf_METH_SD]->pos;
    // NOTE : ref for dihedang : CE
    a4pos = resPt->schAtomList[psf_METH_CE]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma3",a1pos,a2pos,a3pos,a4pos);
  }

  // write third sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-3.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_METH_CE]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma4 
    a1pos = resPt->schAtomList[psf_METH_CG]->pos;
    a2pos = resPt->schAtomList[psf_METH_SD]->pos;
    a3pos = resPt->schAtomList[psf_METH_CE]->pos;
    // NOTE : ref for dihedang : 1HE
    a4pos = resPt->schAtomList[psf_METH_1HE]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma4",a1pos,a2pos,a3pos,a4pos);
  }

  // write fourth sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-4.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_METH_1HE]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_METH_2HE]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_METH_3HE]);    
  if(wb)  {
    p3d_end_desc();
  }

}


/***************************************************************/

static void psf_create_p3d_PHE_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_PHE_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_PHE_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_PHE_CG]->pos;
    // NOTE : ref for dihedang : CD1
    a4pos = resPt->schAtomList[psf_PHE_CD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_PHE_CD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_PHE_CD2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_PHE_CE1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_PHE_CE2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_PHE_CZ]);      
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_PHEH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_PHEH_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_2HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_PHEH_CG]->pos;
    // NOTE : ref for dihedang : CD1
    a4pos = resPt->schAtomList[psf_PHEH_CD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_CD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_CD2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_HD1]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_HD2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_CE1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_CE2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_CZ]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_HE1]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_HE2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_PHEH_HZ]);      
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_SER_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : OG
    a4pos = resPt->schAtomList[psf_SER_OG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_SER_OG]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_SERH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : OG
    a4pos = resPt->schAtomList[psf_SERH_OG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_SERH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_SERH_2HB]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_SERH_OG]);
  // H' atoms
  if(resPt->schAtomList[psf_SERH_HG]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_SERH_HG]);}
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_THR_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : OG1
    a4pos = resPt->schAtomList[psf_THR_OG1]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_THR_OG1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_THR_CG2]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_THRH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : OG1
    a4pos = resPt->schAtomList[psf_THRH_OG1]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_THRH_HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_THRH_OG1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_THRH_CG2]);
  // H' atoms
  if(resPt->schAtomList[psf_THRH_HG1]) {psf_create_p3d_atom(resPt->schAtomList[psf_THRH_HG1]);}
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_THRH_CG2]->pos;
    // NOTE : ref for dihedang : 1HG2
    a4pos = resPt->schAtomList[psf_THRH_1HG2]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_THRH_1HG2]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_THRH_2HG2]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_THRH_3HG2]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_TRP_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_TRP_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_TRP_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_TRP_CG]->pos;
    // NOTE : ref for dihedang : CD1
    a4pos = resPt->schAtomList[psf_TRP_CD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_TRP_CD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_TRP_CD2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRP_NE1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_TRP_CE2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRP_CE3]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRP_CZ2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRP_CZ3]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRP_CH2]);      
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_TRPH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_TRPH_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_2HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_TRPH_CG]->pos;
    // NOTE : ref for dihedang : CD1
    a4pos = resPt->schAtomList[psf_TRPH_CD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_CD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_CD2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_HD1]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_NE1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_CE2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_CE3]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_HE3]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_CZ2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_CZ3]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_CH2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_HZ2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_HZ3]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_HH2]);      
  // H' atoms
  if (resPt->schAtomList[psf_TRPH_HE1]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_TRPH_HE1]);}
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_TYR_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_TYR_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_TYR_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    a3pos = resPt->schAtomList[psf_TYR_CG]->pos;
    // NOTE : ref for dihedang : CD1
    a4pos = resPt->schAtomList[psf_TYR_CD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_TYR_CD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_TYR_CD2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TYR_CE1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_TYR_CE2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TYR_CZ]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TYR_OH]);      
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_TYRH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : CG
    a4pos = resPt->schAtomList[psf_TYRH_CG]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_1HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_2HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_CG]);    
  if(wb)  {
    p3d_end_desc();
  }

  if(art_sch) {
    // write joint gamma2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_TYRH_CG]->pos;
    // NOTE : ref for dihedang : CD1
    a4pos = resPt->schAtomList[psf_TYRH_CD1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_CD1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_CD2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_CE1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_CE2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_HD1]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_HD2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_HE1]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_HE2]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_CZ]);      
  psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_OH]);      
  // H' atoms
  if (resPt->schAtomList[psf_TYRH_HH]!=NULL) {psf_create_p3d_atom(resPt->schAtomList[psf_TYRH_HH]);}
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_VAL_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_gen_CB]->pos;
    // NOTE : ref for dihedang : CG1
    a4pos = resPt->schAtomList[psf_VAL_CG1]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_VAL_CG1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_VAL_CG2]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_VALH_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  double *a1pos,*a2pos,*a3pos,*a4pos;

  if(art_sch) {
    // write joint gamma1 
    get_N_pos(resPt,&a1pos);
    get_CA_pos(resPt,&a2pos);
    a3pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    // NOTE : ref for dihedang : CG1
    a4pos = resPt->schAtomList[psf_VALH_CG1]->pos;
    // gamma1 is refered to the last bkb joint 
    psf_create_p3d_gamma_jnt(resPt,bkb_ref_indJ,(char*)"gamma1",a1pos,a2pos,a3pos,a4pos);
  }

  // write first sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_VALH_HB]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_VALH_CG1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_VALH_CG2]);    
  if(wb)  {
    p3d_end_desc();
  }


  if(art_sch) {
    // write joint gamma2-1 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_VALH_CG1]->pos;
    // NOTE : ref for dihedang : 1HG1
    a4pos = resPt->schAtomList[psf_VALH_1HG1]->pos;
    psf_create_p3d_gamma_jnt(resPt,global_indJ,(char*)"gamma2-1",a1pos,a2pos,a3pos,a4pos);
  }

  // write second-1 sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2-1.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_VALH_1HG1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_VALH_2HG1]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_VALH_3HG1]);    
  if(wb)  {
    p3d_end_desc();
  }


  if(art_sch) {
    // write joint gamma2-2 
    get_CA_pos(resPt,&a1pos);
    a2pos = resPt->bkbAtomList[psf_genH_CB]->pos;
    a3pos = resPt->schAtomList[psf_VALH_CG2]->pos;
    // NOTE : ref for dihedang : 1HG2
    a4pos = resPt->schAtomList[psf_VALH_1HG2]->pos;
    // gamma2-2 is refered to the gamma1 (global_indJ - 1)
    psf_create_p3d_gamma_jnt(resPt,(global_indJ - 1),(char*)"gamma2-2",a1pos,a2pos,a3pos,a4pos);
  }

  // write second-2 sch rigid
  if(wb) {
    snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1,(char*)"side-chain-2-2.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
    p3d_beg_desc(P3D_BODY, bodyName);
  }
  psf_create_p3d_atom(resPt->schAtomList[psf_VALH_1HG2]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_VALH_2HG2]);    
  psf_create_p3d_atom(resPt->schAtomList[psf_VALH_3HG2]);    
  if(wb)  {
    p3d_end_desc();
  }

}

/***************************************************************/

static void psf_create_p3d_sidechain(psf_residue *resPt, int art_sch, int wb)
{
  switch(resPt->resType) {
  case psf_ALA:
    // no side-chain in these cases 
    break;
  case psf_ALAH: 
    psf_create_p3d_ALAH_sidechain(resPt,art_sch,wb);
    break;
  case psf_ARG:
    psf_create_p3d_ARG_sidechain(resPt,art_sch,wb);
    break;    
  case psf_ARGH: 
    psf_create_p3d_ARGH_sidechain(resPt,art_sch,wb);
    break;    
  case psf_ASN: 
    psf_create_p3d_ASN_sidechain(resPt,art_sch,wb);
    break;
  case psf_ASNH: 
    psf_create_p3d_ASNH_sidechain(resPt,art_sch,wb);
    break;
  case psf_ASP: 
    psf_create_p3d_ASP_sidechain(resPt,art_sch,wb);
    break;
  case psf_ASPH: 
    psf_create_p3d_ASPH_sidechain(resPt,art_sch,wb);
    break;
  case psf_CYS: 
    psf_create_p3d_CYS_sidechain(resPt,art_sch,wb);
    break;
  case psf_CYSH: 
    psf_create_p3d_CYSH_sidechain(resPt,art_sch,wb);
    break;
  case psf_GLN: 
    psf_create_p3d_GLN_sidechain(resPt,art_sch,wb);
    break;
   case psf_GLNH: 
    psf_create_p3d_GLNH_sidechain(resPt,art_sch,wb);
    break;
  case psf_GLU: 
    psf_create_p3d_GLU_sidechain(resPt,art_sch,wb);
    break;
  case psf_GLUH: 
    psf_create_p3d_GLUH_sidechain(resPt,art_sch,wb);
    break;
  case psf_GLY: 
  case psf_GLYH: 
    // no side-chain in these cases 
    break;
  case psf_HIS: 
    psf_create_p3d_HIS_sidechain(resPt,art_sch,wb);
    break;
   case psf_HISH: 
    psf_create_p3d_HISH_sidechain(resPt,art_sch,wb);
    break;
  case psf_ILE: 
    psf_create_p3d_ILE_sidechain(resPt,art_sch,wb);
    break;
  case psf_ILEH: 
    psf_create_p3d_ILEH_sidechain(resPt,art_sch,wb);
    break;
  case psf_LEU: 
    psf_create_p3d_LEU_sidechain(resPt,art_sch,wb);
    break;
  case psf_LEUH: 
    psf_create_p3d_LEUH_sidechain(resPt,art_sch,wb);
    break;
  case psf_LYS: 
    psf_create_p3d_LYS_sidechain(resPt,art_sch,wb);
    break;
  case psf_LYSH: 
    psf_create_p3d_LYSH_sidechain(resPt,art_sch,wb);
    break;
  case psf_MET: 
    psf_create_p3d_MET_sidechain(resPt,art_sch,wb);
    break;
  case psf_METH: 
    psf_create_p3d_METH_sidechain(resPt,art_sch,wb);
    break;
  case psf_PHE: 
    psf_create_p3d_PHE_sidechain(resPt,art_sch,wb);
    break;
  case psf_PHEH: 
    psf_create_p3d_PHEH_sidechain(resPt,art_sch,wb);
    break;
  case psf_PRO: 
  case psf_PROH: 
    // no side-chain in these cases 
    break;
  case psf_SER: 
    psf_create_p3d_SER_sidechain(resPt,art_sch,wb);
    break;
  case psf_SERH: 
    psf_create_p3d_SERH_sidechain(resPt,art_sch,wb);
    break;
  case psf_THR: 
    psf_create_p3d_THR_sidechain(resPt,art_sch,wb);
    break;
  case psf_THRH: 
    psf_create_p3d_THRH_sidechain(resPt,art_sch,wb);
    break;
  case psf_TRP: 
    psf_create_p3d_TRP_sidechain(resPt,art_sch,wb);
    break;
  case psf_TRPH: 
    psf_create_p3d_TRPH_sidechain(resPt,art_sch,wb);
    break;
  case psf_TYR: 
    psf_create_p3d_TYR_sidechain(resPt,art_sch,wb);
    break;
  case psf_TYRH: 
    psf_create_p3d_TYRH_sidechain(resPt,art_sch,wb);
    break;
  case psf_VAL: 
    psf_create_p3d_VAL_sidechain(resPt,art_sch,wb);
    break;
  case psf_VALH: 
    psf_create_p3d_VALH_sidechain(resPt,art_sch,wb);
    break;
  }
}

/***************************************************************/

static void psf_create_p3d_body_with_all_atoms(psf_residue *resPt)
{
  char wholeResidueName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  // write body head
  snprintf(wholeResidueName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, "all-atoms.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
  p3d_beg_desc(P3D_BODY, wholeResidueName);

  // NOTE : terminal atoms are not considered yet !!!

  // write atoms
  switch(resPt->resType) {
  case GLY: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLY_N]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLY_CA]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLY_C]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLY_O]);
    break;
  case GLYH: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_N]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_CA]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_1HA]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_2HA]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_C]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_O]);
    // H' atoms
    if (resPt->bkbAtomList[psf_GLYH_H]) {psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_H]);}
    if (resPt->bkbAtomList[psf_GLYH_H_Nterm]) {psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_H_Nterm]);}
    break;
  case PRO: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_N]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_CA]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_CB]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_CG]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_CD]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_C]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_O]);
    break;
  case PROH: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_N]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_CA]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_HA]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_CB]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_1HB]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_2HB]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_CG]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_1HG]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_2HG]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_CD]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_1HD]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_2HD]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_C]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_O]);
    break;
  default:
    if(resPt->flagH == 0) {
      psf_create_p3d_atom(resPt->bkbAtomList[psf_gen_N]);
      psf_create_p3d_atom(resPt->bkbAtomList[psf_gen_CA]);
      psf_create_p3d_atom(resPt->bkbAtomList[psf_gen_CB]);
      psf_create_p3d_sidechain(resPt,0,0);   // 0 -> rigid side-chain
      psf_create_p3d_atom(resPt->bkbAtomList[psf_gen_C]);
      psf_create_p3d_atom(resPt->bkbAtomList[psf_gen_O]);
    }
    else {
      psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_N]);
      psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_CA]);
      psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_HA]);
      psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_CB]);
      psf_create_p3d_sidechain(resPt,0,0);   // 0 -> rigid side-chain
      psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_C]);
      psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_O]);
      // H' atoms
      if (resPt->bkbAtomList[psf_genH_H]!=NULL) {psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_H]);}
      if (resPt->bkbAtomList[psf_genH_H_Nterm]!=NULL) {psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_H_Nterm]);}
    }
  }

  p3d_end_desc();
  
}

/***************************************************************/

static void psf_create_p3d_body_with_first_bkb_rigid(psf_residue *resPt)
{
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  // write body head
  snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, "first-bkb-rigid.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
  p3d_beg_desc(P3D_BODY, bodyName);

  // write atoms
  switch(resPt->resType) {
  case GLY: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLY_N]);
    break;
  case GLYH: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_N]);
    // H' atoms
    if (resPt->bkbAtomList[psf_GLYH_H]!=NULL) {psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_H]);}
    if (resPt->bkbAtomList[psf_GLYH_H_Nterm]!=NULL) {psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_H_Nterm]);}
    break;
  case PRO: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_N]);
    break;
  case PROH: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_N]);
    break;
  default:
    if(resPt->flagH == 0) {
      psf_create_p3d_atom(resPt->bkbAtomList[psf_gen_N]);
    }
    else {
      psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_N]);
      // H' atoms
      if (resPt->bkbAtomList[psf_genH_H]) {psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_H]);}
      if (resPt->bkbAtomList[psf_genH_H_Nterm]) {psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_H_Nterm]);}
    }
  }

  p3d_end_desc();   

}

/***************************************************************/

static void psf_create_p3d_body_with_second_bkb_rigid(psf_residue *resPt)
{
  // WARNING : for PROLINE : if joint phi is not writen then
  //           there is no second rigid !!!

   char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  // write body head
  snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, "second-bkb-rigid.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
  p3d_beg_desc(P3D_BODY, bodyName);

  // write atoms
  switch(resPt->resType) {
  case GLY: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLY_CA]);
    break;
  case GLYH: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_CA]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_1HA]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_2HA]);
    break;
  case PRO: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_CA]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_CB]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_CG]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_CD]);
    break;
  case PROH: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_CA]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_HA]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_CB]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_1HB]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_2HB]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_CG]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_1HG]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_2HG]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_CD]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_1HD]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_2HD]);
    break;
  default:
    if(resPt->flagH == 0) {
      psf_create_p3d_atom(resPt->bkbAtomList[psf_gen_CA]);
      psf_create_p3d_atom(resPt->bkbAtomList[psf_gen_CB]);
    }
    else {
      psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_CA]);
      psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_HA]);
      psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_CB]);
    }
  }

   p3d_end_desc();
}

/***************************************************************/

static void psf_create_p3d_body_with_third_bkb_rigid(psf_residue *resPt)
{
   char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

   // write body head
   snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, "third-bkb-rigid.%s.%d.%s", resPt->resName, resPt->resSeq, resPt->chainPt->chainID);
   p3d_beg_desc(P3D_BODY, bodyName);

  // write atoms
  switch(resPt->resType) {
  case GLY: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLY_C]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLY_O]);
    break;
  case GLYH: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_C]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_GLYH_O]);
    break;
  case PRO: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_C]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PRO_O]);
    break;
  case PROH: 
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_C]);
    psf_create_p3d_atom(resPt->bkbAtomList[psf_PROH_O]);
    break;
  default:
    if(resPt->flagH == 0) {
      psf_create_p3d_atom(resPt->bkbAtomList[psf_gen_C]);
      psf_create_p3d_atom(resPt->bkbAtomList[psf_gen_O]);
    }
    else {
      psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_C]);
      psf_create_p3d_atom(resPt->bkbAtomList[psf_genH_O]);
    }
  }

  p3d_end_desc();   
}


/***************************************************************/

static int psf_create_p3d_peptide_chain(psf_AAchain* chain, int numprot) {
  
  int i;
  int joint_built = FALSE; 

  //  First joint frame relative to the protein base-joint
  psf_residue *firstResidue = chain->resList[0];  
  psf_atom* firstAtom = firstResidue->bkbAtomList[0];

  char jointName[PSF_P3D_JNT_MAX_NAME_LENGTH];
  int indRef;
  double posAtom[] = { firstAtom->pos[0],
		       firstAtom->pos[1],
		       firstAtom->pos[2], 
		       0.0,
		       0.0,
		       0.0 };
  double vDof[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double vDofmin[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double vDofmax[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double v0Dof[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  

  // FIRST JOINT FRAME OF THE AA CHAIN
  // NOTE : The first joint frame is relative to the protein base-joint
  //        This joint is a free-flying joint.

  indRef = 1; // the protein base-joint is jnt[1] 

  global_indJ++;

  snprintf(jointName, PSF_P3D_JNT_MAX_NAME_LENGTH-1, ".chain_base.%d.%s.%d", numprot , firstResidue->chainPt->chainID, firstResidue->chainPt->subchainind);
  joint_built = psf_create_p3d_joint(P3D_FREEFLYER, jointName, indRef, posAtom, vDof, v0Dof, vDofmin, vDofmax);


  if(!joint_built) {
    return FALSE;
  }
   
  indRef = global_indJ;

  // Write each residue
  for(i=0; i<chain->nresidues; i++) {

    if(!psf_create_p3d_residue(chain->resList[i],indRef)) {
      return FALSE;
    }
    indRef = bkb_ref_indJ;  // bkb_ref_indJ is modified in function psf_create_p3d_residue
  }
  
  return TRUE;

}


/**********************************************************************/

// PROTEIN ENVIRONNEMENT DESCRIPTION


static void psf_beg_p3d_env(char* envName) {
  
  p3d_beg_desc(P3D_ENV, envName);

}

/***************************************************************/

static void psf_set_p3d_protein_env_box(psf_protein *proteinPt) {

  double dist_fNlC;
  double vdiff[3];
  psf_atom *firstAtom, *lastAtom;
  psf_residue *lastResidue;

  firstAtom = proteinPt->chainList[0]->resList[0]->bkbAtomList[0];
  lastResidue = proteinPt->chainList[proteinPt->nchains - 1]->resList[proteinPt->chainList[proteinPt->nchains - 1]->nresidues - 1];
  lastAtom= lastResidue->bkbAtomList[lastResidue->nbkbatoms - 1];

  // for env-box dimensions
  vectSub(lastAtom->pos,firstAtom->pos,vdiff);
  dist_fNlC = vectNorm(vdiff);

  p3d_set_env_box(firstAtom->pos[0] - 2.0*dist_fNlC, firstAtom->pos[0] + 2.0*dist_fNlC,
		  firstAtom->pos[1] - 2.0*dist_fNlC, firstAtom->pos[1] + 2.0*dist_fNlC,
		  firstAtom->pos[2] - 2.0*dist_fNlC, firstAtom->pos[2] + 2.0*dist_fNlC);

}

/**********************************************************************/
/**********************************************************************/

/**********************************************************************/

// USEFULL FUNCTIONS FOR LIGANDS


static psf_rigid *get_rigid_connected_to_rigid_by_joint(psf_rigid *rigidPt, psf_joint *jointPt)
{
  // NOTE : the connection is made by the atoms in the joint axis
  if(jointPt->dihedralAtoms[1]->rigidPt != rigidPt)
    return jointPt->dihedralAtoms[1]->rigidPt;
  else
    return jointPt->dihedralAtoms[2]->rigidPt;
}


/**********************************************************************/

static void insert_next_rigids_in_list(psf_rigid *rigidPt, psf_rigid ***rigidList, int *nrigids)
{
  psf_rigid *nextrPt;
  int i;
  
  for(i=0; i<rigidPt->noutJnts; i++) {    
    nextrPt = get_rigid_connected_to_rigid_by_joint(rigidPt,rigidPt->outJntsList[i]);
    insert_pointer_in_list((void*)nextrPt,(void***)rigidList,nrigids);
  }
}

/**********************************************************************/

// LIGAND ROBOT DESCRIPTION

static int psf_create_p3d_ligand_base_joint(psf_ligand* ligPt, int numlig) {

  int joint_built = FALSE;

  // Joint base (freeflyer) caracteristics
  char jointName[PSF_P3D_JNT_MAX_NAME_LENGTH];
  int indRef = 0;
  double pos_oref[] = { ligPt->oref[0],
		      ligPt->oref[1],
		      ligPt->oref[2], 
		      0.0,
		      0.0,
		      0.0 };
  double vDof[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double vDofmin[] = { -XBOX_HALF,-YBOX_HALF,-ZBOX_HALF,-180.0,-180.0,-180.0 };
  double vDofmax[] = { XBOX_HALF, YBOX_HALF, ZBOX_HALF, 180.0, 180.0, 180.0 };
  double v0Dof[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  
  // BASE JOINT
  // NOTE : A free-flying joint is defined as base-joint of a protein.
  //        By default, this joint is static.
  //        The frame origin corresponds with the origin of the first backbone atom (in the list)
  
  
  global_indJ++;

  snprintf(jointName, PSF_P3D_JNT_MAX_NAME_LENGTH-1, ".lig_base.%d", numlig);
  joint_built = psf_create_p3d_joint(P3D_FREEFLYER, jointName, indRef, pos_oref, vDof, v0Dof, vDofmin, vDofmax);
 
  return (joint_built);

}

/**********************************************************************/

static int psf_create_p3d_joint_with_updates(psf_joint *jointPt, int indRef){

  int joint_built = FALSE;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double axis1[3],axis2[3],axis3[3];
  double posdiff[3];
  double  pos_And_Axis[6];
  double dihedang;
  char   jointName[PSF_P3D_JNT_MAX_NAME_LENGTH];

  a1pos = jointPt->dihedralAtoms[0]->pos;
  a2pos = jointPt->dihedralAtoms[1]->pos;
  a3pos = jointPt->dihedralAtoms[2]->pos;
  a4pos = jointPt->dihedralAtoms[3]->pos;

  vectSub(a3pos,a2pos,posdiff);
  vectNormalize(posdiff,axis2);
  vectSub(a2pos,a1pos,posdiff);
  vectNormalize(posdiff,axis1);
  vectSub(a4pos,a3pos,posdiff);
  vectNormalize(posdiff,axis3);

  pos_And_Axis[0] = a3pos[0];
  pos_And_Axis[1] = a3pos[1];
  pos_And_Axis[2] = a3pos[2];
  pos_And_Axis[3] = axis2[0];
  pos_And_Axis[4] = axis2[1];
  pos_And_Axis[5] = axis2[2];

  dihedang = compute_dihedang(axis3,axis2,axis1);

  // update global_indJ
  global_indJ++;

  snprintf(jointName, PSF_P3D_JNT_MAX_NAME_LENGTH-1, "joint.%d", global_indJ);
  joint_built = psf_create_p3d_joint(P3D_ROTATE, jointName, indRef, pos_And_Axis, &dihedang, &dihedang, &(jointPt->vmin), &(jointPt->vmax));

  return joint_built;

}

/**********************************************************************/

static void psf_create_p3d_rigid_with_updates(psf_rigid* rigidPt, int numlig){

  int i;

  psf_rigid *nextrPt;
  char bodyName[PSF_P3D_BODY_MAX_NAME_LENGTH];

  // update global_indR
  global_lig_indR++;

  snprintf(bodyName, PSF_P3D_BODY_MAX_NAME_LENGTH-1, "rigid-%d.LIG.%d\n",global_lig_indR, numlig);
  p3d_beg_desc(P3D_BODY, bodyName);
  for(i=0; i<rigidPt->natoms; i++) {    
    psf_create_p3d_atom(rigidPt->atomList[i]);
  }
  p3d_end_desc();

  // idex of parent joint in next rigids
  // WARNING : this translator currently works ONLY with kinematic trees 
  //           (i.e. rigidPt->ninJnts in ALWAYS 1)
  // NOTE : global_indJ corresponds with the index of the current parent joint
  for(i=0; i<rigidPt->noutJnts; i++) {    
    nextrPt = get_rigid_connected_to_rigid_by_joint(rigidPt,rigidPt->outJntsList[i]);
    nextrPt->indexparentj = global_indJ;
  }
}

/**********************************************************************/

static int psf_create_p3d_ligand_kinematic_tree_from_root(psf_ligand *ligPt, int numlig, psf_rigid *rootrPt)
{
  psf_rigid **currigidList;
  psf_rigid **nextrigidList, **auxlist;
  int nrigids;
  int nnextrigids;
  int i;
  int joint_error = FALSE;

  psf_create_p3d_rigid_with_updates(rootrPt, numlig);

  nrigids = 0;
  currigidList = NULL;
  // insert next rigids in list
  insert_next_rigids_in_list(rootrPt,&currigidList,&nrigids);

  while(nrigids > 0 && (!joint_error)) {
    nnextrigids = 0;
    nextrigidList = NULL;

    i = 0;
    while(i<nrigids && (!joint_error)) {

      if(currigidList[i]->ninJnts != 1) {
	printf("ERROR : wrong kinematic structure : a joint can have only one parent\n");
	free(nextrigidList);
	nextrigidList = NULL;
	free(currigidList);
	currigidList = NULL;
	return FALSE;
      }
      
      joint_error = FALSE;

      joint_error = !(psf_create_p3d_joint_with_updates(currigidList[i]->inJntsList[0],currigidList[i]->indexparentj));
      if (!joint_error) {
	psf_create_p3d_rigid_with_updates(currigidList[i], numlig);
	// insert next rigids in list
	insert_next_rigids_in_list(currigidList[i],&nextrigidList,&nnextrigids);
	i++;
      }
    }

    if (!joint_error) {
      nrigids = nnextrigids;
      auxlist = currigidList;
      currigidList = nextrigidList;
      nextrigidList = auxlist;
      free(nextrigidList);
      nextrigidList = NULL;
    }
  }
  free(currigidList);
  currigidList = NULL;
    
  return (!joint_error);
}

/**********************************************************************/

// LIGAND ENVIRONNEMENT DESCRIPTION


static void psf_set_p3d_ligand_env_box (psf_ligand *ligPt) {

  p3d_set_env_box(ligPt->oref[0] - XBOX_HALF, ligPt->oref[0] + XBOX_HALF,
		  ligPt->oref[1] - YBOX_HALF, ligPt->oref[1] + YBOX_HALF,
		  ligPt->oref[2] - ZBOX_HALF, ligPt->oref[2] + ZBOX_HALF);
}


/**********************************************************************/

// PDB AND PSF STRUCTURES TO P3D MODELS

int psf_make_p3d_prot_desc(psf_protein* proteinPt, int numprot) {

  int i;

  // write protein head
  if (!(psf_create_p3d_protein_base_joint(proteinPt, numprot))) {
    return FALSE;
  }

  // write each chain 
  for(i=0; i<proteinPt->nchains; i++) {
    if(!psf_create_p3d_peptide_chain(proteinPt->chainList[i], numprot)) {
      return FALSE;
    }
  }

  return TRUE;
}

/**********************************************************************/

int psf_make_p3d_lig_desc(psf_ligand* ligandPt, int numlig) {

  int kinematic_tree_created = FALSE;

  // write ligand head
  if (!(psf_create_p3d_ligand_base_joint(ligandPt,numlig))) {
    return FALSE;
  }

  // write rigids and joints
  kinematic_tree_created = psf_create_p3d_ligand_kinematic_tree_from_root(ligandPt, numlig, ligandPt->rigidList[0]);

  return kinematic_tree_created;
}

/**********************************************************************/

int psf_make_p3d_desc(psf_molecule* molecule) {

  int indexletter;
  int ligIndex, protIndex;
  int p3d_prot_created, p3d_lig_created;
  char moleculeName[PSF_P3D_MOL_MAX_NAME_LENGTH];


  indexletter = 0;
  if (molecule->nproteins !=0 ) {
    snprintf(moleculeName, PSF_P3D_MOL_MAX_NAME_LENGTH-1, "%s.pep", psf_givemeword(molecule->name,'.',&indexletter));
  }
  else if (molecule->nligands !=0) {
    snprintf(moleculeName, PSF_P3D_MOL_MAX_NAME_LENGTH-1, "%s.lig", psf_givemeword(molecule->name,'.',&indexletter));
  }
  else {
    return FALSE;    
  }

  psf_beg_p3d_env(molecule->name); // BEGIN ENV

  p3d_beg_desc(P3D_ROBOT, moleculeName); // BEGIN ROBOT (ONLY ONE)

  p3d_prot_created = TRUE;
  protIndex = 0;
  while (protIndex<molecule->nproteins && p3d_prot_created) {
    p3d_prot_created = psf_make_p3d_prot_desc(molecule->proteinList[protIndex], (protIndex+1));
    protIndex++;
  }

  p3d_lig_created = TRUE;
  ligIndex = 0;
  while (ligIndex<molecule->nligands && p3d_lig_created) {
    p3d_lig_created = psf_make_p3d_lig_desc(molecule->ligandList[ligIndex], (ligIndex+1));
    ligIndex++;
  }

  p3d_end_desc(); // END ROBOT

  // More sophisticated method to create env box have to
  // be computed
  if (molecule->nproteins != 0) {
    psf_set_p3d_protein_env_box(molecule->proteinList[0]);
  }
  else if (molecule->nligands != 0) {
    psf_set_p3d_ligand_env_box(molecule->ligandList[0]); 
  }

  p3d_end_desc(); // END ENV

  return (p3d_lig_created && p3d_prot_created);

}

/***************************************************************/

static int psf_make_p3d_from_psf(psf_molecule* molecule) {
#ifdef WITH_XFORMS
  int usrAnswer = FALSE;
  int molecules_created = FALSE;

  // get information on articulations //////////////////////////////////////////

  // ligands :  dihedral datas
  if (molecule->nligands != 0) {
    create_joint_lig_Form(molecule->ligandList, molecule->nligands);
    usrAnswer = do_joint_lig_Form();
    
    if (usrAnswer == FALSE)
      return FALSE;
    
    if (!generate_rigids_in_all_ligands(molecule->ligandList, molecule->nligands)) {
      PrintError(("ERROR : unable to generate rigids\n"));
      return FALSE;
    }

  }
  // ligands : end dihedral datas
  
  // proteins : AAA datas
  if (molecule->nproteins != 0) {
    AAA_protein_List = AAA_read_psf_struct(molecule->proteinList, molecule->nproteins);
    if (AAA_protein_List == NULL) {
      PrintError(("ERROR while writing AAA list\n"));
      return (FALSE);
    }
    
    //fl_initialize(&argc, argv, "FormDemo", 0, 0);
    create_AAA_Form(AAA_protein_List);
    usrAnswer = do_AAA_Form(AAA_protein_List);
    
    if (usrAnswer == FALSE) {
      free_AAA_protein_List(AAA_protein_List);
      AAA_protein_List = NULL;
      return FALSE;
    }
  }
  // proteins : end AAA datas
  
  // create P3D description from PSF Structure /////////////////////////////////
  molecules_created = psf_make_p3d_desc(molecule);

  if (molecule->nproteins != 0) {
    free_AAA_protein_List(AAA_protein_List);
    AAA_protein_List = NULL;
  }
   
  return (molecules_created);
#else
  printf("ERROR: the function %s requires the XFORMS module.\n", __FUNCTION__);
  return(0);
#endif
} 

/***************************************************************/

int psf_make_p3d_from_pdb(char *file) { 
#ifdef WITH_XFORMS
  FILE *PDB_file_desc;
  char shortFileName[PSF_MAX_NAME_LENGTH];
  psf_molecule* current_molecule;
  int molecules_created;

  // test of file name length
  if (!get_short_file_name(file, shortFileName, PSF_MAX_NAME_LENGTH)) {
    PrintWarning(("File Name too long !!!\n"));
    return FALSE;
  }

  if(!(PDB_file_desc=fopen(file,"r"))) {
    PrintError(("MP: psf_make_p3dModel_from_pdbFile : can't open %s\n",file));
    return FALSE;
  }

  current_molecule = init_molecule_struct(shortFileName);

  // read PDB and write PSF structure //////////////////////////////////////////

  if(!extract_psf_struct(PDB_file_desc, shortFileName, current_molecule)) {
    free_molecule(current_molecule);
    free(current_molecule);
    current_molecule = NULL;
    return (FALSE);
  }
  //printf("PSF EXTRACTED\n");

  fclose(PDB_file_desc);    

  molecules_created = psf_make_p3d_from_psf(current_molecule);

  free_molecule(current_molecule);
  free(current_molecule);
  current_molecule = NULL;

  if (molecules_created) {
    p3d_BB_init_BB0();
  }

  return molecules_created;
#else
  printf("ERROR: the function %s requires the XFORMS module.\n", __FUNCTION__);
  return(0);
#endif
}

/***************************************************************/

int psf_make_p3d_from_multiple_pdb(file_name_list* file_list, char* name) { 
#ifdef WITH_XFORMS
  FILE *PDB_file_desc;
  int fileIndex;
  psf_molecule* current_molecule = NULL;
  int extracted = FALSE;
  int molecules_created = FALSE;
  char shortFileName[PSF_MAX_NAME_LENGTH];

  current_molecule = init_molecule_struct(name);

  // read PDB and write PSF structure //////////////////////////////////////////

  fileIndex = 0;

  do {

    PDB_file_desc = NULL;

    if(!(PDB_file_desc=fopen(file_list->name_list[fileIndex],"r"))) {
      PrintError(("ERROR : can't open %s\n",file_list->name_list[fileIndex]));
      extracted = FALSE;
    }
    else
    {
      if (get_short_file_name((char*)file_list->name_list[fileIndex], shortFileName, PSF_MAX_NAME_LENGTH)) 
      {
	extracted = extract_psf_struct(PDB_file_desc, shortFileName, current_molecule);
	fileIndex++;
      }
      else {
	PrintWarning(("File Name too long !!!\n"));
      }
      fclose(PDB_file_desc);
    }
    
  } while (extracted && fileIndex<file_list->nb_file_name);

  if (!extracted) {
    free_molecule(current_molecule);
    free(current_molecule);
    current_molecule = NULL;
    return FALSE;    
  }

  molecules_created = psf_make_p3d_from_psf(current_molecule);

  free_molecule(current_molecule);
  free(current_molecule);
  current_molecule = NULL;

  if (molecules_created) {
    p3d_BB_init_BB0();    
  }

  return molecules_created;
#else
  printf("ERROR: the function %s requires the XFORMS module.\n", __FUNCTION__);
  return(0);
#endif
}
