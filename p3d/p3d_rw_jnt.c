/****************************************************************************/
/*! \file p3d_rw_jnt.c
 *
 *  \brief Save and write a joint
 *
 *     Describe the file format use in the scenario or in .p3d.
 */
/****************************************************************************/


#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"


typedef struct {
  char * name;
  p3d_type_joint type;
} p3d_jnt_type_name;

const p3d_jnt_type_name jnt_type_name_array[NB_JNT_TYPE] =
         { { (char*)"P3D_FIXED",     P3D_FIXED },
           { (char*)"P3D_ROTATE",    P3D_ROTATE },
	   { (char*)"P3D_TRANSLATE", P3D_TRANSLATE },
	   { (char*)"P3D_PLAN",      P3D_PLAN },
	   { (char*)"P3D_FREEFLYER", P3D_FREEFLYER },
	   { (char*)"P3D_KNEE",      P3D_KNEE },
	   { (char*)"P3D_BASE",      P3D_BASE } };




/*--------------------------------------------------------------------------*/
/*! \brief Get the joint type with the joint type name.
 *
 * \param  name: the joint type name.
 *
 * \retval type: the joint type.
 *
 * \return TRUE if \a name is valid.
 */
int p3d_rw_jnt_get_type_by_name(const char * name, p3d_type_joint * type)
{
  int i;

  if ((strncmp(name, "P3D_", 4) != 0) && (strncmp(name, "M3D_", 4) != 0))
    { return FALSE; }
  for(i=0; i<NB_JNT_TYPE; i++) {
    if (strcmp(name+4, jnt_type_name_array[i].name+4) == 0) {
      (*type) = jnt_type_name_array[i].type;
      return TRUE;
    }
  }
  return FALSE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Get the joint type with the joint type number.
 *
 * \param  num:  the joint type number (between 0 and ::NB_JNT_TYPE -1).
 *
 * \retval type: the joint type.
 *
 * \return TRUE if the joint type number is between 0 and ::NB_JNT_TYPE -1.
 */
int p3d_rw_jnt_get_type_by_num(int num, p3d_type_joint * type)
{
  if ((num<0) || (num>=NB_JNT_TYPE))
    { return FALSE; }
  
  (*type) = jnt_type_name_array[num].type;
  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Get the joint type name with the joint type.
 *
 * \param  type:  the joint type.
 *
 * \return the joint type name, NULL if \a type is not valid.
 */
const char * p3d_rw_jnt_get_name_by_type(p3d_type_joint type)
{
  int i;
  
  for(i=0; i<NB_JNT_TYPE; i++) {
    if (type == jnt_type_name_array[i].type)
      { return jnt_type_name_array[i].name; }
  }

  return NULL;
}


/*--------------------------------------------------------------------------*/
/*! \brief Get the joint type num with the joint type.
 *
 * \param  type:  the joint type.
 *
 * \return the joint type num, -1 if \a type is not valid.
 */
int p3d_rw_jnt_get_num_by_type(p3d_type_joint type)
{
  int i;
  
  for(i=0; i<NB_JNT_TYPE; i++) {
    if (type == jnt_type_name_array[i].type)
      { return i; }
  }

  return -1;
}


/*--------------------------------------------------------------------------*/
/*! \brief Get the joint type name with the joint type number.
 *
 * \param  num:  the joint type number (between 0 and ::NB_JNT_TYPE -1).
 *
 * \return the joint type name, NULL if \a num is not valid.
 */
const char * p3d_rw_jnt_get_name_by_num(int num)
{
  if ((num<0) || (num>=NB_JNT_TYPE))
    { return NULL; }

  return jnt_type_name_array[num].name;
}


/*--------------------------------------------------------------------------*/
/*! \brief Create a link with a joint.
 *
 *  \param jntPt: the joint to link with
 *
 *  \return the new link with a joint structure
 */
p3d_read_jnt_link_data * p3d_rw_jnt_create_joint_link(void)
{
  p3d_read_jnt_link_data * jnt_linkPt;
  int i;

  jnt_linkPt = MY_ALLOC(p3d_read_jnt_link_data, 1);
  if (jnt_linkPt == NULL)
    { return NULL; }
  jnt_linkPt->jnt_num    = -1;
  jnt_linkPt->robot_num  = -1;
  jnt_linkPt->jnt_link   = NULL;
  jnt_linkPt->flag_prev  = TRUE;
  for(i=0; i<NB_POS_PARAM; i++)
    { jnt_linkPt->pos_param[i] = 0.0; }
  return jnt_linkPt;
}


/*--------------------------------------------------------------------------*/
/*! \brief Destroy a link with a joint.
 *
 *  \param jnt_linkPt: the link to destroy
 */
void p3d_rw_jnt_destroy_joint_link(p3d_read_jnt_link_data * jnt_linkPt)
{
  MY_FREE(jnt_linkPt, p3d_read_jnt_link_data, 1);
}


/*--------------------------------------------------------------------------*/
/*! \brief Create a structure to store the parameters of the joints.
 *
 * \param type: the joint type.
 *
 * \return the joint data.
 */
p3d_read_jnt_data * p3d_create_read_jnt_data(p3d_type_joint type)
{
  p3d_read_jnt_data * data;
  int i;

  data = MY_ALLOC(p3d_read_jnt_data, 1);
  if (data == NULL) {
    PrintError(("Not enough memory !!!\n"));
    return NULL;
  }

  data->type = type;
  p3d_jnt_get_nb_param(type, &(data->nb_dof), &(data->nb_param));
  data->v         = MY_ALLOC(double, data->nb_dof);
  data->v_pos0    = MY_ALLOC(double, data->nb_dof);
  data->vmin      = MY_ALLOC(double, data->nb_dof);
  data->vmax      = MY_ALLOC(double, data->nb_dof);
  data->vmin_rand = MY_ALLOC(double, data->nb_dof);
  data->vmax_rand = MY_ALLOC(double, data->nb_dof);
  data->is_user   = MY_ALLOC(int,    data->nb_dof);
  data->is_active_for_planner   = MY_ALLOC(int,    data->nb_dof);
 
  if ((data->nb_dof>0) && 
      ((data->v == NULL) || (data->v_pos0 == NULL) ||
       (data->vmin == NULL) || (data->vmax == NULL) || 
       (data->vmin_rand == NULL) || (data->vmax_rand == NULL) ||
       (data->is_user == NULL) || (data->is_active_for_planner == NULL))) {
    PrintError(("Not enough memory !!!\n"));
    return NULL;
  }
  data->flag_v         = FALSE;
  data->flag_v_pos0    = FALSE;
  data->flag_vmin      = FALSE;
  data->flag_vmax      = FALSE;
  data->flag_vmin_rand = FALSE;
  data->flag_vmax_rand = FALSE;
  data->flag_is_user   = FALSE;
  data->flag_is_active_for_planner   = FALSE;
  for(i=0; i<data->nb_dof; i++)
    { data->v[i] = 0.0; }

  data->param      = MY_ALLOC(double, data->nb_param);
  if ((data->nb_param>0) && (data->param == NULL)) {
    PrintError(("Not enough memory !!!\n"));
    return NULL;
  }
  data->flag_param = FALSE;
  
  data->flag_pos = FALSE;
  data->flag_relative_pos = FALSE;

  data->prev_jnt = 0;
  data->flag_prev_jnt = FALSE;

  data->flag_name = FALSE;

  data->nb_links = 0;
  data->link_array = NULL;

  return data;
}


/*--------------------------------------------------------------------------*/
/*! \brief Destroy a structure to store the parameters of the joints.
 *
 * \param data: the joint data.
 */
void p3d_destroy_read_jnt_data(p3d_read_jnt_data * data)
{
  int i;

  if (data->nb_dof>0) {
    MY_FREE(data->v,         double, data->nb_dof);
    MY_FREE(data->v_pos0,    double, data->nb_dof);
    MY_FREE(data->vmin,      double, data->nb_dof);
    MY_FREE(data->vmax,      double, data->nb_dof);
    MY_FREE(data->vmin_rand, double, data->nb_dof);
    MY_FREE(data->vmax_rand, double, data->nb_dof);
    MY_FREE(data->is_user,   int,    data->nb_dof);
    MY_FREE(data->is_active_for_planner,   int,    data->nb_dof);
  }

  if (data->nb_dof>0)
    { MY_FREE(data->param, double, data->nb_param); }

  if (data->nb_links>0) {
    for(i=0; i<data->nb_links; i++)
      { p3d_rw_jnt_destroy_joint_link(data->link_array[i]); }
    MY_FREE(data->link_array, p3d_read_jnt_link_data *, data->nb_links);
  }

  MY_FREE(data, p3d_read_jnt_data, 1);
}


/*--------------------------------------------------------------------------*/
/*! \brief Check the data read.
 *
 *  \param  data:  the data used to build a joint.
 *  
 *  \return wether or not it is possible to create this joint.
 */
static int s_p3d_check_data(p3d_read_jnt_data * data, int num_line)
{
  int i;

  if (!(data->flag_pos)) {
    PrintWarning(("!!! ERROR (line %i) position of the joint not given !!!\n", 
		  num_line));
    return FALSE;
  }
  if (data->nb_dof>0) {
    if (!(data->flag_vmin)) {
      PrintWarning(("!!! ERROR (line %i) joint minimum values not given !!!\n",
		    num_line));
      return FALSE;
    }
    if (!(data->flag_vmax)) {
      PrintWarning(("!!! ERROR (line %i) joint maximum values not given !!!\n",
		    num_line));
      return FALSE;
    }
  }
  if (!((data->nb_param==0) || (data->flag_param))) {
    PrintWarning(("!!! ERROR (line %i) joint parameters not given !!!\n", 
		  num_line));
    return FALSE;
  }
  for (i=0; i<data->nb_dof; i++) {
    if (data->vmin[i]>data->vmax[i]) {
      PrintWarning(("!!! ERROR (line %i) vmin[%i] > vmax[%i] !!!\n", 
		    num_line, i, i));
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
      PrintWarning(("!!! ERROR (line %i) vmin_rand[%i] > vmax_rand[%i] !!!\n", 
		    num_line, i, i));
      return FALSE;
    }
  }
  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Build the joint with the data read.
 *
 *  \param  data:  the data used to build a joint.
 *  
 *  \return wether or not it is possible to create this joint.
 *
 *  \internal
 */
static int s_p3d_build_jnt_data(p3d_read_jnt_data * data)
{
  double * dofs, * dofs_rand;
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
  }
  p3d_add_desc_jnt_deg(data->type, data->pos, dofs, data->prev_jnt,
		       dofs_rand, data->scale);
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

  if (data->flag_is_active_for_planner) {
    for(i=0; i<data->nb_dof; i++) { 
      p3d_jnt_set_dof_is_active_for_planner(jntPt, i, data->is_active_for_planner[i]); 
      if(data->is_active_for_planner[i] == 0)
	p3d_set_flag_passive_parameters_for_planner(TRUE);
    }
  }
  else {
    p3d_jnt_set_is_active_for_planner(jntPt,TRUE);
  }

  return TRUE;
}



/****************************************************************************/
/* Joint definition */

/*! \page file_description_jnt_page Commands for the description of joint.

The description of a joint can be done with \ref p3d_add_desc_jnt_subsection or \ref p3d_add_desc_xyz_jnt_subsection. But these functions became complex.
An other way is to define this joint between a 
\ref p3d_beg_desc_jnt_subsection and \ref p3d_end_desc_jnt_subsection.

This definition can be done in the files: .p3d, .macro (and .sce for multi-robots version).

The following functions are defined:

\subsection p3d_beg_desc_jnt_subsection p3d_beg_desc_jnt <type>

	Start the definition of a new joint for the type ::p3d_type_joint:
	\<type\>: Type is ::p3d_type_joint (with the M3D_ or P3D_ beginning):
		- ::P3D_ROTATE, M3D_ROTATE: Joint rotate (1 dof).
			Its default use is one rotation \a Rz.
		- ::P3D_FIXED, M3D_FIXED: Fixed joint (0 dof).
			Used for grasp.
		- ::P3D_TRANSLATE, M3D_TRANSLATE: Joint translate (1 dof).
			Its default use is one translation \a z.
		- ::P3D_PLAN, M3D_PLAN: Joint plan (3 dof). Its default use
			is 2 translation \a x, \a y and then a rotation \a Rz.
		- ::P3D_FREEFLYER, M3D_FREEFLYER: Joint freeflyer (6 dof).
			Its default use is 3 translations \a x, \a y, \a z then
			3 rotations \a Rx, \a Ry, \a Rz.
		- ::P3D_KNEE, M3D_KNEE: Joint knee (3 dof).
			Its default use is 3 rotations \a Rx, \a Ry, \a Rz.
		- ::P3D_PPIVOT, M3D_PPIVOT: Only use for compatibility
			reasons, int fact this is a ::P3D_ROTATE.

       It ends with \ref p3d_end_desc_jnt_subsection
		       
\subsection p3d_end_desc_jnt_subsection p3d_end_desc

	Finish the description began by \ref p3d_beg_desc_jnt_subsection.


\subsection p3d_set_prev_jnt_subsection p3d_set_prev_jnt <number>

        Define the previous joint \<number\>, the default value is 0, 
the ::P3D_BASE joint.


\section joint_position_section Funtions to set the position of one joint.

\subsection p3d_set_pos_axe_subsection p3d_set_pos_axe <x> <y> <z> <axe x> <axe y> <axe z>

        Define the initial position of the joint by its orientation, like \ref p3d_add_desc_jnt_subsection.
	- \<x\> \<y\> \<z\>: This is the absolute position of the joint.
	- \<axe x\> \<axe y\> \<axe z\>: is the new main axis to the joint 
		(default is z axis). We use only one rotation to transform the
		\a z axis to it's new axis. For example a ::P3D_PLAN, (1, 0, 0)
		represent a rotation \a Ry 90 degree (\a z axis becomes
		\a x axis). So we have then two translations (\a -z, \a y),
		one rotation (\a Rx). This parametrage can't represent all
		placement, but it is compatible with previous version of
		Move3D. To specify any rotation use
		\ref p3d_set_pos_xyz_subsection 

\subsection p3d_set_pos_axe_dh_subsection p3d_set_pos_axe_dh <a> <alpha> <r> <theta>
    Define the initial position of the joint folowing Denavit & Hartenberg parameters, like \ref p3d_set_pos_axe_dh_subsection.

\subsection p3d_set_pos_xyz_subsection p3d_set_pos_xyz <x> <y> <z> <Rx> <Ry> <Rz>

        Define the initial position of the joint by three rotations, like \ref p3d_add_desc_xyz_jnt_subsection.
	- \<x\> \<y\> \<z\>: This is the absolute position of the joint.
	- \<Rx\> \<Ry\> \<Rz\>: Three rotations to describe the orientation 
		of the joints.


\subsection p3d_set_pos_mat_subsection p3d_set_pos_mat <M(0,0)> <M(0,1)> ... <M(3,2)> <M(3,3)>

        Define the initial position of the joint by its matrice.
	- \<M(0,0)\> \<M(0,1)\> ... \<M(3,2)\> \<M(3,3)\>: This position matrix of the joint.


\subsection p3d_set_pos_relative_subsection p3d_set_pos_relative

        Flag to show that the initial position of the joint (defined either by \ref p3d_set_pos_axe_subsection, \ref p3d_set_pos_xyz_subsection, \ref p3d_set_pos_dh_subsection or \ref p3d_set_pos_mat_subsection) is defined relativly to the previous joint initial position. If \ref p3d_set_pos_relative_subsection is not define. The initial position is the absolute position in the environment.




\section joint_dof_value_section Funtions to set the values of the degree of freedom of one joint.

\subsection p3d_set_dof_subsection p3d_set_dof <value_1> <value_2> ... <value_n>

        Function to set the initial values of the degree of freedom, all the degree of freedom must be defined.
	   - \<value_i\>: Value of the degree of freedom, in degree for angle.
	   By default, they are null.


\subsection p3d_set_dof_pos0_subsection p3d_set_dof_pos0 <value_1> <value_2> ... <value_n>

        Function to set the initial values of the degree of freedom, corresponding to the initial position of the joint.
	   - \<value_i\>: Value of the degree of freedom, in degree for angle.
	   By default, they are null.


\subsection p3d_set_dof_vmin_subsection p3d_set_dof_vmin <value_1> <value_2> ... <value_n>

        Funtions to set the initial minimal bounds for the values of the degree of freedom of one joint.  All the degree of freedom must be defined.
	   - \<value_i\>: Value of the minimal bounds for the degrees
	   of freedom, in degree for angle.  By default, they are null.


\subsection p3d_set_dof_vmax_subsection p3d_set_dof_vmax <value_1> <value_2> ... <value_n>

        Funtions to set the initial maximal bounds for the values of the degree of freedom of one joint.  All the degree of freedom must be defined.
	   - \<value_i\>: Value of the marimal bounds for the degrees
	   of freedom, in degree for angle.  By default, they are null.


\subsection p3d_set_dof_vmin_rand_subsection p3d_set_dof_vmin_rand <value_1> <value_2> ... <value_n>

        Funtions to set the initial minimal bounds for random sampling of the values of the degree of freedom of one joint.  All the degree of freedom must be defined.
	   - \<value_i\>: Value of the minimal bounds for random sampling 
	   the degrees of freedom, in degree for angle. 
	   By default, they are equal to the minimal bounds.


\subsection p3d_set_dof_vmax_rand_subsection p3d_set_dof_vmax_rand <value_1> <value_2> ... <value_n>

        Funtions to set the initial maximal bounds for random sampling of the values of the degree of freedom of one joint.  All the degree of freedom must be defined.
	   - \<value_i\>: Value of the maximal bounds for random sampling 
	   the degrees of freedom, in degree for angle. 
	   By default, they are equal to the maximal bounds.

  
\subsection p3d_set_is_user_subsection p3d_set_is_user <flag_1> ... <flag_n>

        Funtions to define if the degree of freedom are visible to the user or not. All the degree of freedom must be defined.
	- \<flag_i\>: TRUE if the degree of freedom is visible for user.

    

\section joint_param_value_section Funtions to set parameters to the joint

\subsection p3d_set_param_subsection p3d_set_param <value_1> <value_2> ... <value_n>

        Defined parameter used by the joint (currently, no joint use this).


\subsection p3d_set_name_subsection p3d_set_name <name>

        Define the name of the joint
*/


/*--------------------------------------------------------------------------*/
/*! \brief Read the description of a joint in a file.
 *
 *  \param  f:     the file.
 *  \param  size:  the allocated size of the string.
 *  \param  line:  the string use to store the data of the file.
 *  \param  data:  the data needed to compute the joint.
 *  
 *  \retval size:     the new string that store the line.
 *  \retval num_line: the new line number.
 *  \param  data:     the data needed to compute the joint.
 * 
 *  \return wether or not it is possible to create this joint.
 */
int p3d_parse_jnt_desc(FILE * f, char ** line, int * size, 
		       int * num_line, p3d_read_jnt_data * data)
{
  double dtab[6];
  char * pos, * fct, * name;
  int no_error;
  p3d_read_jnt_link_data *jnt_linkPt = NULL;

  no_error = TRUE;

  while ((((*size) = p3d_read_line_next_function(f, line, *size, num_line))
	  != 0) && (no_error)) {
    pos = (*line);
    p3d_read_string_name(&pos, &fct); /* with p3d_read_line_next_function 
					 before, read_string_name
					 always returns TRUE */

    /*--------------------------------------------------*/
    /* Joint position */

    if (strcmp(fct,"p3d_set_prev_jnt")==0) {
      if (data->flag_prev_jnt) {
	PrintWarning(("!!! WARNING (line %i): redefine the previous"
		      " joint !!!\n", *num_line));
      }
      if(!p3d_read_string_int(&pos, 1, &(data->prev_jnt)))
	{ no_error = FALSE; }
      else
	{ data->flag_prev_jnt = TRUE; }
      continue;
    }
    if (strcmp(fct,"p3d_set_pos_axe")==0) {
      if (data->flag_pos) {
	PrintWarning(("!!! WARNING (line %i): redefine the joint"
		      " position !!!\n", *num_line));
      }
      if(!p3d_read_string_double(&pos, 6, dtab))
	{ no_error = FALSE; }
      else {
	p3d_convert_axe_to_mat(data->pos, dtab);
	data->flag_pos = TRUE;
      }
      continue;
    }
    if (strcmp(fct,"p3d_set_pos_axe_dh")==0) {
      if (data->flag_pos) {
        PrintWarning(("!!! WARNING (line %i): redefine the joint"
        " position !!!\n", *num_line));
      }
      if (data->flag_prev_jnt){ //if the previous jnt is defined
        if(!p3d_read_string_double(&pos, 4, dtab)){
          no_error = FALSE;
        } else {
          p3d_convert_dh_to_mat(data->pos, dtab, XYZ_ROBOT->joints[data->prev_jnt]);
          data->flag_pos = TRUE;
        }
      }else{
         PrintWarning(("!!! WARNING (line %i): Set D-H parameters"
        " before giving the previous joint !!!\n", *num_line));
      }
      continue;
    }
    if (strcmp(fct,"p3d_set_pos_xyz")==0) {
      if (data->flag_pos) {
	PrintWarning(("!!! WARNING (line %i): redefine the joint"
		      " position !!!\n", *num_line));
      }
      if(!p3d_read_string_double(&pos, 6, dtab))
	{ no_error = FALSE; }
      else {
	dtab[3] = DTOR(dtab[3]);
	dtab[4] = DTOR(dtab[4]);
	dtab[5] = DTOR(dtab[5]);
	p3d_mat4Pos(data->pos, dtab[0], dtab[1], dtab[2],
		    dtab[3], dtab[4], dtab[5]);
	data->flag_pos = TRUE;
      }
      continue;
    }
    if (strcmp(fct,"p3d_set_pos_mat")==0) {
      if (data->flag_pos) {
	PrintWarning(("!!! WARNING (line %i): redefine the joint"
		      " position !!!\n", *num_line));
      }
      if(!p3d_read_string_mat(&pos, data->pos))
	{ no_error = FALSE; }
      else
	{ data->flag_pos = TRUE; }
      continue;
    }
    if (strcmp(fct,"p3d_set_pos_relative")==0) {
      if (data->flag_relative_pos) {
	PrintWarning(("!!! WARNING (line %i): redefine the joint position"
		      " relative !!!\n", *num_line));
      }
      data->flag_relative_pos = TRUE;
      continue;
    }
    
    /*--------------------------------------------------*/
    /* Dofs values */

    if (strcmp(fct,"p3d_set_dof")==0) {
      if (data->flag_v) {
	PrintWarning(("!!! WARNING (line %i): redefine the degree of freedom"
		      " values !!!\n", *num_line));
      }
      if(!p3d_read_string_double(&pos, data->nb_dof, data->v))
	{ no_error = FALSE; }
      else
	{ data->flag_v = TRUE; }
      continue;
    }
    if (strcmp(fct,"p3d_set_dof_pos0")==0) {
      if (data->flag_v_pos0) {
	PrintWarning(("!!! WARNING (line %i): redefine the degree of freedom"
		      " values for the initial position !!!\n", *num_line));
      }
      if(!p3d_read_string_double(&pos, data->nb_dof, data->v_pos0))
	{ no_error = FALSE; }
      else
	{ data->flag_v_pos0 = TRUE; }
      continue;
    }
    if (strcmp(fct,"p3d_set_dof_vmin")==0) {
      if (data->flag_vmin) {
	PrintWarning(("!!! WARNING (line %i): redefine the degree of freedom"
		      " minimal values !!!\n", *num_line));
      }
      if(!p3d_read_string_double(&pos, data->nb_dof, data->vmin))
	{ no_error = FALSE; }
      else
	{ data->flag_vmin = TRUE; }
      continue;
    }
    if (strcmp(fct,"p3d_set_dof_vmax")==0) {
      if (data->flag_vmax) {
	PrintWarning(("!!! WARNING (line %i): redefine the degree of freedom"
		      " maximal values !!!\n", *num_line));
      }
      if(!p3d_read_string_double(&pos, data->nb_dof, data->vmax))
	{ no_error = FALSE; }
      else
	{ data->flag_vmax = TRUE; }
      continue;
    }
    if (strcmp(fct,"p3d_set_dof_vmin_rand")==0) {
      if (data->flag_vmin_rand) {
	PrintWarning(("!!! WARNING (line %i): redefine the degree of freedom"
		      " minimal random values !!!\n", *num_line));
      }
      if(!p3d_read_string_double(&pos, data->nb_dof, data->vmin_rand))
	{ no_error = FALSE; }
      else
	{ data->flag_vmin_rand = TRUE; }
      continue;
    }
    if (strcmp(fct,"p3d_set_dof_vmax_rand")==0) {
      if (data->flag_vmax_rand) {
	PrintWarning(("!!! WARNING (line %i): redefine the degree of freedom"
		      " maximal random values !!!\n", *num_line));
      }
      if(!p3d_read_string_double(&pos, data->nb_dof, data->vmax_rand))
	{ no_error = FALSE; }
      else
	{ data->flag_vmax_rand = TRUE; }
      continue;
    }
   
    
    /*--------------------------------------------------*/
    /* Joint parameters */

    if (strcmp(fct,"p3d_set_param")==0) {
      if (data->flag_param) {
	PrintWarning(("!!! WARNING (line %i): redefine the parameters"
		      " of the joint !!!\n", *num_line));
      }
      if(!p3d_read_string_double(&pos, data->nb_param, data->param))
	{ no_error = FALSE; }
      else
	{ data->flag_param = TRUE; }
      continue;
    }
    if (strcmp(fct,"p3d_set_name")==0) {
      if (data->flag_name) {
	PrintWarning(("!!! WARNING (line %i): redefine the name"
		      " of the joint !!!\n", *num_line));
      }
      if(!p3d_read_string_line_string(&pos, &name))
	{ no_error = FALSE; }
      else {
	strncpy(data->name, name, JNT_MAX_SIZE_NAME-1);
	data->flag_name = TRUE; 
      }
      continue;
    }
    if (strcmp(fct,"p3d_set_is_user")==0) {
      if (data->flag_is_user) {
	PrintWarning(("!!! WARNING (line %i): redefine the user"
		      " degree of freedom for the joint !!!\n", *num_line));
      }
      if(!p3d_read_string_boolean(&pos, data->nb_dof, data->is_user))
	{ no_error = FALSE; }
      else
	{ data->flag_is_user = TRUE; }
      continue;
    }
    if (strcmp(fct,"p3d_set_is_active_for_planner")==0) {
      if (data->flag_is_active_for_planner) {
	PrintWarning(("!!! WARNING (line %i): redefine the is_active_for_planner flag"
		      " degree of freedom for the joint !!!\n", *num_line));
      }
      if(!p3d_read_string_boolean(&pos, data->nb_dof, data->is_active_for_planner))
	{ no_error = FALSE; }
      else
	{ data->flag_is_active_for_planner = TRUE; }
      continue;
    }
    
    /*--------------------------------------------------*/
    /* Joint links */

    if (strcmp(fct,"p3d_add_link_to")==0) {
      jnt_linkPt = p3d_rw_jnt_create_joint_link();
      if (jnt_linkPt == NULL) {
	no_error = FALSE;
	PrintError(("Not enough memory !!!\n"));
      } else {
	jnt_linkPt->flag_prev = FALSE;
	if (!p3d_read_string_int(&pos, 1, &(jnt_linkPt->jnt_num))) {
	  no_error = FALSE;
	  p3d_rw_jnt_destroy_joint_link(jnt_linkPt);
	} else if (!p3d_read_string_double(&pos, NB_POS_PARAM, 
					   jnt_linkPt->pos_param)) {
	  no_error = FALSE;
	  p3d_rw_jnt_destroy_joint_link(jnt_linkPt);
	} else {
	  data->link_array = MY_REALLOC(data->link_array, 
					p3d_read_jnt_link_data*,
					data->nb_links, data->nb_links+1);
	  if (data->link_array == NULL) {
	    no_error = FALSE;
	    data->nb_links = 0;
	    PrintError(("Not enough memory !!!\n"));
	  } else {
	    data->link_array[data->nb_links] = jnt_linkPt;
	    data->nb_links ++;
	  }
	}
      }
      continue;
    }
    if (strcmp(fct,"p3d_add_link_from")==0) {
      jnt_linkPt = p3d_rw_jnt_create_joint_link();
      if (jnt_linkPt == NULL) {
	no_error = FALSE;
	PrintError(("Not enough memory !!!\n"));
      } else {
	jnt_linkPt->flag_prev = TRUE;
	if (!p3d_read_string_int(&pos, 1, &(jnt_linkPt->jnt_num))) {
	  no_error = FALSE;
	  p3d_rw_jnt_destroy_joint_link(jnt_linkPt);
	} else if (!p3d_read_string_double(&pos, NB_POS_PARAM, 
					   jnt_linkPt->pos_param)) {
	  no_error = FALSE;
	  p3d_rw_jnt_destroy_joint_link(jnt_linkPt);
	} else {
	  data->link_array = MY_REALLOC(data->link_array, 
					p3d_read_jnt_link_data*,
					data->nb_links, data->nb_links+1);
	  if (data->link_array == NULL) {
	    no_error = FALSE;
	    data->nb_links = 0;
	    PrintError(("Not enough memory !!!\n"));
	  } else {
	    data->link_array[data->nb_links] = jnt_linkPt;
	    data->nb_links ++;
	  }
	}
      }
      continue;
    }
    if (strcmp(fct,"p3d_add_link_to_robot")==0) {
      jnt_linkPt = p3d_rw_jnt_create_joint_link();
      if (jnt_linkPt == NULL) {
	no_error = FALSE;
	PrintError(("Not enough memory !!!\n"));
      } else {
	jnt_linkPt->flag_prev = FALSE;
	if (!p3d_read_string_int(&pos, 1, &(jnt_linkPt->robot_num))) {
	  no_error = FALSE;
	  p3d_rw_jnt_destroy_joint_link(jnt_linkPt);
	} else if (!p3d_read_string_int(&pos, 1, &(jnt_linkPt->jnt_num))) {
	  no_error = FALSE;
	  p3d_rw_jnt_destroy_joint_link(jnt_linkPt);
	} else if (!p3d_read_string_double(&pos, NB_POS_PARAM, 
					   jnt_linkPt->pos_param)) {
	  no_error = FALSE;
	  p3d_rw_jnt_destroy_joint_link(jnt_linkPt);
	} else {
	  data->link_array = MY_REALLOC(data->link_array, 
					p3d_read_jnt_link_data*,
					data->nb_links, data->nb_links+1);
	  if (data->link_array == NULL) {
	    no_error = FALSE;
	    data->nb_links = 0;
	    PrintError(("Not enough memory !!!\n"));
	  } else {
	    data->link_array[data->nb_links] = jnt_linkPt;
	    data->nb_links ++;
	  }
	}
      }
      continue;
    }
    if (strcmp(fct,"p3d_add_link_from_robot")==0) {
      jnt_linkPt = p3d_rw_jnt_create_joint_link();
      if (jnt_linkPt == NULL) {
	no_error = FALSE;
	PrintError(("Not enough memory !!!\n"));
      } else {
	jnt_linkPt->flag_prev = TRUE;
	if (!p3d_read_string_int(&pos, 1, &(jnt_linkPt->robot_num))) {
	  no_error = FALSE;
	  p3d_rw_jnt_destroy_joint_link(jnt_linkPt);
	} else if (!p3d_read_string_int(&pos, 1, &(jnt_linkPt->jnt_num))) {
	  no_error = FALSE;
	  p3d_rw_jnt_destroy_joint_link(jnt_linkPt);
	} else if (!p3d_read_string_double(&pos, NB_POS_PARAM, 
					   jnt_linkPt->pos_param)) {
	  no_error = FALSE;
	  p3d_rw_jnt_destroy_joint_link(jnt_linkPt);
	} else {
	  data->link_array = MY_REALLOC(data->link_array, 
					p3d_read_jnt_link_data*,
					data->nb_links, data->nb_links+1);
	  if (data->link_array == NULL) {
	    no_error = FALSE;
	    data->nb_links = 0;
	    PrintError(("Not enough memory !!!\n"));
	  } else {
	    data->link_array[data->nb_links] = jnt_linkPt;
	    data->nb_links ++;
	  }
	}
      }
      continue;
    }

    if (strcmp(fct,"p3d_end_desc")==0)
      { break; }
    no_error = FALSE;
  }
  
  if (no_error == FALSE) {
    PrintWarning(("ERROR (line %i) function %s not valid !!!\n", 
		  *num_line, fct));
  } else
    { no_error = s_p3d_check_data(data, *num_line); }

  return no_error;
}


/*--------------------------------------------------------------------------*/
/*! \brief Read the description of a joint in a file.
 *
 *  \param  f:     the file.
 *  \param  type:  the type of joint.
 *  \param  scale: the scale factor apply on the joint.
 *  
 *  \return wether or not it is possible to create this joint.
 */
int p3d_env_beg_jnt_desc(FILE * f, p3d_type_joint type, double scale)
{
  char * line = NULL;
  int size = 0;
  int num_line = 0;
  int no_error;
  p3d_read_jnt_data * data;


  data = p3d_create_read_jnt_data(type);
  if (data == NULL)
    { return FALSE; }

  data->scale = scale;

  no_error = p3d_parse_jnt_desc(f, &line, &size, &num_line, data);
  
  if (no_error)
    { no_error = s_p3d_build_jnt_data(data); }

  p3d_destroy_read_jnt_data(data);
  MY_FREE(line, char, size);
  
  return no_error;
}


/*--------------------------------------------------------------------------*/
/*! \brief Write a joint description in a file.
 *
 *  \param  f:     the file.
 *  \param  jntPt: the joint.
 */
void p3d_rw_jnt_write_jnt_desc(FILE * f, p3d_jnt * jntPt)
{
  int i;
  double vmin, vmax;
  p3d_link_between_joint * linkPt;
  double pos_param[6];
  p3d_jnt * jnt2Pt;

  fprintf(f, "\np3d_beg_desc_jnt %s\n\n", 
	  p3d_rw_jnt_get_name_by_type(jntPt->type));
  fprintf(f, "    p3d_set_name \"%s\"\n", jntPt->name);

    
  /*--------------------------------------------------*/
  /* Degree of freedom */

  if (jntPt->dof_equiv_nbr > 0) {
    fprintf(f, "\n#-- Degree of freedom -----\n");
    fprintf(f, "\n    p3d_set_dof");
    for(i=0; i<jntPt->dof_equiv_nbr; i++)
      { fprintf(f, " %f", p3d_jnt_get_dof_deg(jntPt, i)); }
    fprintf(f, "\n    p3d_set_dof_vmin");
    for(i=0; i<jntPt->dof_equiv_nbr; i++) {
      p3d_jnt_get_dof_bounds_deg(jntPt, i, &vmin, &vmax);
      fprintf(f, " %f", vmin);
    }
    fprintf(f, "\n    p3d_set_dof_vmax");
    for(i=0; i<jntPt->dof_equiv_nbr; i++) {
      p3d_jnt_get_dof_bounds_deg(jntPt, i, &vmin, &vmax);
      fprintf(f, " %f", vmax);
    }
    fprintf(f, "\n    p3d_set_dof_vmin_rand");
    for(i=0; i<jntPt->dof_equiv_nbr; i++) {
      p3d_jnt_get_dof_rand_bounds_deg(jntPt, i, &vmin, &vmax);
      fprintf(f, " %f", vmin);
    }
    fprintf(f, "\n    p3d_set_dof_vmax_rand");
    for(i=0; i<jntPt->dof_equiv_nbr; i++) {
      p3d_jnt_get_dof_rand_bounds_deg(jntPt, i, &vmin, &vmax);
      fprintf(f, " %f", vmax);
    }
    fprintf(f, "\n    p3d_set_is_user");
    for(i=0; i<jntPt->dof_equiv_nbr; i++) {
      if (p3d_jnt_get_dof_is_user(jntPt, i))
	{ fprintf(f, " TRUE"); }
      else
	{ fprintf(f, " FALSE"); }
    }
    fprintf(f, "\n    p3d_set_is_active_for_planner");
    for(i=0; i<jntPt->dof_equiv_nbr; i++) {
      if (p3d_jnt_get_dof_is_active_for_planner(jntPt, i))
	{ fprintf(f, " TRUE"); }
      else
	{ fprintf(f, " FALSE"); }
    }
    fprintf(f, "\n");
  }

    
  /*--------------------------------------------------*/
  /* Joint position */

  fprintf(f, "\n#-- Joint position -----\n");
  p3d_mat4ExtractPosReverseOrder(jntPt->pos0, &(pos_param[0]), &(pos_param[1]),
				 &(pos_param[2]), &(pos_param[3]),
				 &(pos_param[4]), &(pos_param[5]));
  fprintf(f, "\n    p3d_set_pos_xyz %f %f %f %f %f %f\n", pos_param[0],
	  pos_param[1], pos_param[2], pos_param[3], pos_param[4],pos_param[5]);
    
  /*--------------------------------------------------*/
  /* Joint links */

  fprintf(f, "\n#-- Joint links -----\n");
  for(i=0; i<jntPt->n_link_jnt; i++) {
    linkPt = jntPt->link_jnt_arr[i];
    if (jntPt == linkPt->prev_jnt)
      { jnt2Pt = linkPt->next_jnt; }
    else
      { jnt2Pt = linkPt->prev_jnt; }

    if (jntPt->num > jnt2Pt->num) {
      if (jntPt == linkPt->prev_jnt)
	{ fprintf(f, "\n    p3d_add_link_to"); }
      else
	{ fprintf(f, "\n    p3d_add_link_from"); }
      p3d_mat4ExtractPosReverseOrder(linkPt->rel_pos,
				     &(pos_param[0]), &(pos_param[1]),
				     &(pos_param[2]), &(pos_param[3]),
				     &(pos_param[4]), &(pos_param[5]));
      fprintf(f, " %i \\\n", jnt2Pt->num);
      fprintf(f, "           %f %f %f %f %f %f\n",
	      pos_param[0], pos_param[1], pos_param[2], 
	      RTOD(pos_param[3]), RTOD(pos_param[4]), RTOD(pos_param[5]));
      
    }
  }

  fprintf(f, "\np3d_end_desc\n");
}
