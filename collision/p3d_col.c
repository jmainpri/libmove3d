#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include <algorithm>



#define Vtag 1 
#define Etag 2 
#define Ftag 3
#define TRUE 1 
#define FALSE 0 
#define featTag(featurePtr) (*((int *) (featurePtr)))


#ifndef MAX_REPORT
#define MAX_REPORT 10000 /*nombre de collisions memorisees dans le report*/
#endif

/* Modif Bio */
extern int bio_all_molecules_col(void);
extern int bio_all_molecules_col_with_report(void);
extern void  p3d_start_bio_col(void);
extern int biocol_robot_report(int nrobot);
/*end Modif Bio */

/* Variables globales */

int test_brunet=0;
int COLLISION_BY_OBJECT = TRUE;
int RETURN_KCD_DISTANCE_ESTIMATE = TRUE;

/* these defines are used in combination with 
   the with_report parameter
   these defined are copied in FORMenv.c 
*/
#define P3D_KCD_ALL_NO_REPORT         ((p3d_type_col_choice)0)
#define P3D_KCD_ALL_JUST_BOOL         ((p3d_type_col_choice)1)
#define P3D_KCD_ALL_DISTANCE_ESTIMATE ((p3d_type_col_choice)2)
#define P3D_KCD_ALL_DISTANCE_EXACT    ((p3d_type_col_choice)3)

/* static var */
static p3d_type_col_choice p3d_kcd_which_test = (p3d_type_col_choice)(P3D_KCD_ALL_DISTANCE_ESTIMATE + P3D_KCD_ROB_ALL);
static int p3d_col_mode=p3d_col_mode_none;
static int p3d_col_last_mode=p3d_col_mode_none;
static int p3d_report_num=0;




void set_kcd_which_test(p3d_type_col_choice val)
{
  //PrintInfo(("we set to mode kcd %d\n", val));
  p3d_kcd_which_test = val;
}

p3d_type_col_choice get_kcd_which_test(void)
{
  return p3d_kcd_which_test;
}

void set_collision_by_object(int boolval)
{
  COLLISION_BY_OBJECT = boolval;
}

int get_collision_by_object(void)
{
  return COLLISION_BY_OBJECT;
}

void set_return_kcd_distance_estimate(int boolval)
{
  RETURN_KCD_DISTANCE_ESTIMATE = boolval;
}

int get_return_kcd_distance_estimate()
{
  return RETURN_KCD_DISTANCE_ESTIMATE;
}

/***************************************************/
/* Fonction indiquant si un objet est purement     */
/* graphique ou non                                */
/* In : l'objet                                    */
/* Out : TREU/FALSE                                */
/***************************************************/
int p3d_col_object_is_pure_graphic(p3d_obj *obj)
{
  int j;

  if (obj!=NULL) {    
    for(j=0;j<obj->np;j++) {
      if(obj->pol[j]->TYPE != P3D_GRAPHIC)
	{ return FALSE; }
    }
  }
  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Function to activate the collision between a pair of objets
 *         in the current context.
 *
 *  \param  obj1: The first object
 *  \param  obj1: The second object
 */
void p3d_col_activate_pair_of_objects(p3d_obj *obj1, p3d_obj *obj2)
{
#ifdef VCOLLIDE_ACT
  int id1,id2,res;
#endif

  switch (p3d_col_mode)
    {
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:
      { 
	id1=p3d_v_collide_object_get_id(obj1);  
	id2=p3d_v_collide_object_get_id(obj2);
	res=vcActivateObject (vc_hand,id1);
	res=vcActivateObject (vc_hand,id2);
	res=vcActivatePair(vc_hand,id1,id2);
	/* Activate the pair in the current context */
	p3d_BB_activate_pair((p3d_BB_handle *)NULL, obj1, obj2);
	break;
      }
#endif
    case p3d_col_mode_kcd: 
      { 
	/* Activate the pair in the current context */
	p3d_col_pair_activate_pair((p3d_collision_pair *)NULL, obj1, obj2);
	break;
      }
#ifdef PQP
    case p3d_col_mode_pqp: 
        pqp_activate_object_object_collision(obj1, obj2);
    break;
#endif
    default:PrintInfo(("\n Erreur p3d_col_activate_pair_of_objects, collision checker=none\n"));
    }
}


/*--------------------------------------------------------------------------*/
/*! \brief Function to activate the collision between an objet
 *         and the environment.
 *
 *  \param  obj:  The object
 *
 *  \note \a obj must be a movable object.
 */
void p3d_col_activate_object_to_env(p3d_obj *obj)
{ 
#ifdef VCOLLIDE_ACT
  int id1, id2, res;
  int io, o, no;
  p3d_obj * obst;
#endif
  

  #ifdef PQP
  if (p3d_col_object_is_pure_graphic(obj) ) {//|| (obj->jnt != NULL)) //Modification necessaire pour utiliser PQP.
    //Ce 2eme test semble une erreur car il sera toujours vrai pour un corps de robot.
    //L'erreur passait inaperçue car, avec kcd, cette fonction n'est jamais appelee.
  #else
  if (p3d_col_object_is_pure_graphic(obj) || (obj->jnt != NULL)) {
  #endif
    PrintWarning(("!!! p3d_col_activate_object_to_env call with no valid object !!!\n"));
  } else {
    switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:
      o = p3d_get_desc_curnum(P3D_OBSTACLE);
      no =  p3d_get_desc_number(P3D_OBSTACLE);
      id1=p3d_v_collide_object_get_id(obj);  
      res=vcActivateObject (vc_hand,id1);

      /* activation du robot par rapport aux obstacles */
      for(io=0;io<no;io++){
	p3d_sel_desc_num(P3D_OBSTACLE,io);
	obst =  (p3d_obj *) p3d_get_desc_curid(P3D_OBSTACLE);
	if (!p3d_col_object_is_pure_graphic(obst)) {
	  id2=p3d_v_collide_object_get_id(obst);
	  res=vcActivateObject (vc_hand,id2);
	  res=vcActivatePair(vc_hand,id1,id2);
	}
      }
      p3d_sel_desc_num(P3D_OBSTACLE,o);
      p3d_BB_activate_env((p3d_BB_handle *)NULL, obj);
      break;
#endif
    case p3d_col_mode_kcd:
      p3d_col_pair_activate_env((p3d_collision_pair *)NULL, obj);
      break;
#ifdef PQP
    case p3d_col_mode_pqp:
       pqp_activate_object_collision(obj);
    break;
#endif
    default:
      PrintInfo(("\n Erreur p3d_col_activate_object_to_env, collision checker=none\n"));
    }
  }
}

/***************************************************/
/* Fonction d'activation d'une paire de polyhedres */
/* In : la paire de polyhedres                     */
/* Out :                                           */
/***************************************************/
void p3d_col_activate_pair(p3d_poly *obj1,p3d_poly *obj2)
{
#ifdef VCOLLIDE_ACT
  int id1,id2,res;
#endif

  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      id1=p3d_v_collide_get_id(obj1);  
      id2=p3d_v_collide_get_id(obj2);
      res=vcActivateObject (vc_hand,id1);
      res=vcActivateObject (vc_hand,id2);
      res=vcActivatePair(vc_hand,id1,id2);
      break;
    }
#endif
#ifdef PQP
    case p3d_col_mode_pqp:
      //PrintInfo(("\n Warning: p3d_col_activate_pair, PQP implementation only works with p3d_obj pairs. Set COLLISION_BY_OBJECT to TRUE.\n"));
    break;
#endif
  default:PrintInfo(("\n Erreur p3d_col_activate_pair, collision checker=none\n"));
  }
}

/****************************************************/
/* Fonction d'activation d'un body par rapport a un */
/* objet                                            */
/* In : le body et l'objet                          */
/* Out :                                            */
/****************************************************/
void p3d_col_activate_body_obj(p3d_obj *bod,p3d_obj *obst)
{
  switch (p3d_col_mode)
  {
#ifdef VCOLLIDE_ACT
      case p3d_col_mode_v_collide:{ 
	int i,j,np1,np2,id,idn,res;
	int nr_pairs_activated=0; 

	if(COLLISION_BY_OBJECT)
	  {
	    id=p3d_v_collide_object_get_id(bod);
	    idn=p3d_v_collide_object_get_id(obst);
	    if((!p3d_v_collide_object_is_non_active(bod)) &&
	       (!p3d_v_collide_object_is_non_active(obst))&& (idn!=id))
	      {
		res=vcActivateObject (vc_hand,id);
		res=vcActivateObject (vc_hand,idn);
		nr_pairs_activated++;
		res=vcActivatePair(vc_hand,id,idn);
	      }
	  }
	else
	  {
	    np1 = bod->np;
	    np2 = obst->np;

	    for(i=0;i<np1;i++){
	      /* this object's vcollide_id: */
	      if(bod->pol[i]->TYPE != P3D_GRAPHIC){
		id=p3d_v_collide_get_id(bod->pol[i]);
		/* activate the object */
		res=vcActivateObject (vc_hand,id);
		for(j=0;j<np2;j++){
		  if(obst->pol[j]->TYPE != P3D_GRAPHIC){
		    /* the other object's vcollide_id */			    
		    idn=p3d_v_collide_get_id(obst->pol[j]);
		    /* activate for all pairs but (id,id): */
		    if((idn!=id)&&(p3d_filter_relevant_pair(obst->pol[j])))
		      {
			/* PrintInfo(("we add: (%i,%i)\n",id,idn))); */
			res=vcActivateObject(vc_hand,idn);
			nr_pairs_activated++;
			res=vcActivatePair(vc_hand,id,idn);
		      }
		  }
		}
	      }
	    }
	  }
	p3d_BB_activate_pair((p3d_BB_handle *)NULL, bod, obst);
	break;
      }
#endif
  case p3d_col_mode_kcd: 
    p3d_col_pair_activate_env((p3d_collision_pair *)NULL, bod);
    break;
#ifdef PQP
   case p3d_col_mode_pqp:
     pqp_activate_object_object_collision(bod, obst);
   break;
#endif
  default:PrintInfo(("\n Erreur p3d_col_activate_body_obj, collision checker=none\n"));
  }
}

/****************************************************/
/* Fonction d'activation d'un polyhedre par rapport */
/* a l'environnement                                */
/* In : le polyhedre                                */
/* Out :                                            */
/****************************************************/
void p3d_col_activate_full(p3d_poly *obj)
{ 
#ifdef VCOLLIDE_ACT
  int id,res;
  int nr_pairs_activated=0; 
#endif
  
  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      int i,nr=0,idn;
      p3d_poly *objn=NULL;

      /* this object's vcollide_id: */
      id=p3d_v_collide_get_id(obj);
      /* activate the object */
      res=vcActivateObject (vc_hand,id);
      /* number of polyhedra in scene: */
      nr=p3d_poly_get_nb();
      /* visit all polyhedra: */
      objn = p3d_poly_get_first();
      for(i=1;i<=nr;i++){
	idn=p3d_v_collide_get_id(objn);
	/* activate for all pairs but (id,id): */
	if((idn!=id)&&(p3d_filter_relevant_pair(objn))){
	  /* PrintInfo(("we add: (%i,%i)\n",id,idn))); */
	  res=vcActivateObject(vc_hand,idn);
	  nr_pairs_activated++;
	  res=vcActivatePair(vc_hand,id,idn);
	}
	/* the other object's vcollide_id */
	objn=p3d_poly_get_next();
      }
      PrintInfo(("nr of polyhedrons (== Vcollide obstacles) = %i, nr_pairs_activated = %i\n",nr,nr_pairs_activated));
      break;
    }
#endif
#ifdef PQP
    case p3d_col_mode_pqp:
      //PrintInfo(("\n Warning: p3d_col_activate_full, PQP implementation only works with p3d_obj pairs. Set COLLISION_BY_OBJECT to TRUE.\n"));
    break;
#endif
    default:
      PrintInfo(("\n Erreur p3d_col_activate_full, collision checker=none\n"));
  }
}

/****************************************************/
/* Fonction d'activation de toutes les paires de    */
/* polyhedres                                       */
/* In :                                             */
/* Out :                                            */
/****************************************************/
void p3d_col_activate_all(void)
{

  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("ERROR: p3d_col_activate_all not implemented for VCOLLIDE\n"));
      break;
    }
#endif
    case p3d_col_mode_kcd:{ 
      PrintInfo(("ERROR: p3d_col_activate_all not implemented for KCD\n"));
      break;
    }
#ifdef PQP
    case p3d_col_mode_pqp:
      pqp_activate_all_collisions();
    break;
#endif
    default:
      PrintInfo(("\n Erreur p3d_col_activate_all, collision checker=none\n"));
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Function to deactivate the collision between a pair of objets
 *         in the current context.
 *
 *  \param  obj1: The first object
 *  \param  obj1: The second object
 */
void p3d_col_deactivate_pair_of_objects(p3d_obj *obj1, p3d_obj *obj2)
{
#ifdef VCOLLIDE_ACT
  int id1,id2,res;
#endif

  switch (p3d_col_mode) {
#ifdef VCOLLIDE_ACT
  case p3d_col_mode_v_collide:
    id1=p3d_v_collide_object_get_id(obj1);  
    id2=p3d_v_collide_object_get_id(obj2);
    res=vcDeactivatePair(vc_hand,id1,id2);
    /* Activate the pair in the current context */
    p3d_BB_deactivate_pair((p3d_BB_handle *)NULL, obj1, obj2);
    break;
#endif
  case p3d_col_mode_kcd: 
    /* Activate the pair in the current context */
    p3d_col_pair_deactivate_pair((p3d_collision_pair *)NULL, obj1, obj2);
    break;
#ifdef PQP
  case p3d_col_mode_pqp:
    pqp_deactivate_object_object_collision(obj1, obj2);
  break;
#endif
  default:
    PrintInfo(("\n Erreur p3d_col_deactivate_pair_of_objects, collision checker=none\n"));
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Function to deactivate the collision between an objet
 *         and the environment.
 *
 *  \param  obj:  The object
 *
 *  \note \a obj must be a movable object.
 */
void p3d_col_deactivate_object_to_env(p3d_obj *obj)
{ 
#ifdef VCOLLIDE_ACT
  int id1, id2, res;
  int io, o, no;
  p3d_obj * obst;
#endif
  
  #ifdef PQP
  if (p3d_col_object_is_pure_graphic(obj) ) {//|| (obj->jnt != NULL)) //Modification necessaire pour utiliser PQP.
    //Ce 2eme test semble une erreur car il sera toujours vrai pour un corps de robot.
    //L'erreur passait inaperçue car, avec kcd, cette fonction n'est jamais appelee.
  #else
  if (p3d_col_object_is_pure_graphic(obj) || (obj->jnt != NULL)) {
  #endif
    PrintWarning(("!!! p3d_col_activate_object_to_env call with no valid object !!!\n"));
  } else {
    switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:
      o = p3d_get_desc_curnum(P3D_OBSTACLE);
      no =  p3d_get_desc_number(P3D_OBSTACLE);
      id1=p3d_v_collide_object_get_id(obj);  

      /* activation du robot par rapport aux obstacles */
      for(io=0;io<no;io++){
	p3d_sel_desc_num(P3D_OBSTACLE,io);
	obst =  (p3d_obj *) p3d_get_desc_curid(P3D_OBSTACLE);
	if (!p3d_col_object_is_pure_graphic(obst)) {
	  id2=p3d_v_collide_object_get_id(obst);
	  res=vcDeactivatePair(vc_hand,id1,id2);
	}
      }
      p3d_sel_desc_num(P3D_OBSTACLE,o);
      p3d_BB_deactivate_env((p3d_BB_handle *)NULL, obj);
      break;
#endif
    case p3d_col_mode_kcd:
      p3d_col_pair_deactivate_env((p3d_collision_pair *)NULL, obj);
      break;
#ifdef PQP
    case p3d_col_mode_pqp:
       pqp_deactivate_object_collision(obj);
    break;
#endif
    default:
      PrintInfo(("\n Erreur p3d_col_desactivate_object_to_env, collision checker=none\n"));
    }
  }
}



//! \brief Function to deactivate all the collision tests of a robot.
//!  \param  robot:  the robot
void p3d_col_deactivate_robot(p3d_rob *robot)
{ 
  switch (p3d_col_mode){ 
#ifdef PQP
  case p3d_col_mode_pqp:
    pqp_deactivate_robot_collision(robot);
  break;
#endif
   default:
    PrintInfo(("\n Erreur p3d_col_deactivate_robot, collision checker=none\n"));
  }
}

//! \brief Function to activate all the collision tests of a robot.
//!  \param  robot:  the robot
void p3d_col_activate_robot(p3d_rob *robot)
{ 

  switch (p3d_col_mode){ 
#ifdef PQP
  case p3d_col_mode_pqp:
    pqp_activate_robot_collision(robot);
  break;
#endif
   default:
    PrintInfo(("\n Erreur p3d_col_activate_robot, collision checker=none\n"));
  }
}



/*******************************************************/
/* Fonction de desactivation d'une paire de polyhedres */
/* In : la paire de polyhedres                         */
/* Out :                                               */
/*******************************************************/
void p3d_col_deactivate_pair(p3d_poly *obj1,p3d_poly *obj2)
{
#ifdef VCOLLIDE_ACT
  int id1,id2,res;
#endif

  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      id1=p3d_v_collide_get_id(obj1);
      id2=p3d_v_collide_get_id(obj2);  
      /* PrintInfo(("deactivate vc_id1 = %i and vc_id2 = %i\n",id1,id2))); */
      res=vcDeactivatePair(vc_hand,id1,id2);
      break;
    }
#endif
#ifdef PQP
    case p3d_col_mode_pqp:
      //PrintInfo(("\n Warning: p3d_col_deactivate_pair, PQP implementation only works with p3d_obj pairs. Set COLLISION_BY_OBJECT to TRUE.\n"));
    break;
#endif
    default:
      PrintInfo(("\n Erreur p3d_col_deactivate_pair, collision checker=none\n"));
  }
}

/********************************************************/
/* Fonction de desactivation d'un polyhedre par rapport */
/* a l'environnement                                    */
/* In : le polyhedre                                    */
/* Out :                                                */
/********************************************************/
void p3d_col_deactivate_full(p3d_poly *obj)
{

  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("ERROR: p3d_col_deactivate_full not implemented for VCOLLIDE\n"));
      break;
    }
#endif
    default:
      PrintInfo(("\n Erreur p3d_col_deactivate_full, collision checker=none\n"));
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Function to deactivate collision between all objects pair
 *         for the current context.
 */
void p3d_col_cur_deactivate_all(void)
{
#ifdef VCOLLIDE_ACT
  int res;
#endif

  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
  case p3d_col_mode_v_collide:
    /* (Carl) ma fonction VCollide a moi */ 
    res=vcDeactivateAllPairs(vc_hand);
    p3d_BB_deactivate_all((p3d_BB_handle *)NULL);
    break;
#endif
  case p3d_col_mode_kcd:
    p3d_col_pair_deactivate_all((p3d_collision_pair *)NULL);
    break;
#ifdef PQP
    case p3d_col_mode_pqp:
      pqp_deactivate_all_collisions();
    break;
#endif
  default:
    PrintInfo(("\n Erreur p3d_col_cur_deactivate_all, collision checker=none\n"));
  }
}


/********************************************************/
/* Fonction de desactivation de toutes les paires de    */
/* polyhedres plus radicale...                          */
/* In :                                                 */
/* Out :                                                */
/********************************************************/
void p3d_col_deactivate_rudely_all(void)
{/* PairNode *pair; */
/*  char overlapping; */

  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("ERROR: p3d_col_deactivate_rudely_all not implemented for VCOLLIDE\n"));
      break;
    }
#endif
#ifdef PQP
    case p3d_col_mode_pqp:
      pqp_deactivate_all_collisions();
    break;
#endif
    default:
      PrintInfo(("\n Erreur p3d_col_deactivate_rudely_all, collision checker=none\n"));
  }
}

/********************************************************/
/* Fonction permettant de passer des Dynamic Bounding   */
/* Box aux cuboides                                     */
/* In : le pointeur sur le polyhedre a changer          */
/* Out :                                                */
/********************************************************/
int p3d_switch_to_cube(p3d_poly *p)
{

  switch (p3d_col_mode){  
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("ERROR: p3d_switch_to_cube not implemented for VCOLLIDE\n"));
      return FALSE;
      break;
    }
#endif
    case p3d_col_mode_kcd:{ 
      PrintInfo(("ERROR: p3d_switch_to_cube not implemented for KCD\n"));
      return FALSE;
      break;
    }
    default:{
      PrintInfo(("\n Erreur p3d_switch_to_cube, collision checker=none\n"));
      return FALSE;
    }
  }
}

/**********************************************************/
/* Fonction permettant de passer des cuboides aux Dynamic */
/* Bounding Box                                           */
/* In : le pointeur sur le polyhedre a changer            */
/* Out :                                                  */
/**********************************************************/
int p3d_switch_to_DBB(p3d_poly *p)
{

  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
  case p3d_col_mode_v_collide:{ 
      PrintInfo(("ERROR: p3d_switch_to_DBB not implemented for VCOLLIDE\n"));
      return FALSE;
      break;
    }
#endif
    case p3d_col_mode_kcd:{ 
      PrintInfo(("ERROR: p3d_switch_to_DBB not implemented for KCD\n"));
      return FALSE;
      break;
    }
    default:{ 
      PrintInfo(("\n Erreur p3d_switch_to_DBB, collision checker=none\n"));
      return FALSE;
    }
  }
}

/********************************************************/
/* Fonction permettant de passer des Dynamic Bounding   */
/* Box aux cuboides pour tous les polyhedres            */
/* In : le pointeur sur le polyhedre a changer          */
/* Out :                                                */
/********************************************************/
int p3d_switch_all_to_cube(void)
{ /*int npoly,i,res;*/
  int res;
  p3d_poly *p=NULL;

  p=p3d_poly_get_first();
  while (p!=NULL)
    { res=p3d_switch_to_cube(p);
    p=p3d_poly_get_next();
    }
  return(TRUE);
}

/**********************************************************/
/* Fonction permettant de passer des cuboides aux Dynamic */
/* Bounding Box pour tous les polyhedres                  */
/* In : le pointeur sur le polyhedre a changer            */
/* Out :                                                  */
/**********************************************************/
int p3d_switch_all_to_DBB(void)
{ /*int npoly,i,res;*/
  int res;
  p3d_poly *p=NULL;

  p=p3d_poly_get_first();
  while (p!=NULL)
    { res=p3d_switch_to_DBB(p);
    p=p3d_poly_get_next();
    }
  return(TRUE);

}

/********************************************************/
/* Fonction permettant d'activer l'algorithme nbody     */
/* In :                                                 */
/* Out :                                                */
/********************************************************/
int p3d_enable_nbody(void)
{int res=0;

  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("ERROR: p3d_enable_nbody not implemented for VCOLLIDE\n"));
      return(res);
      break;
    }
#endif
    case p3d_col_mode_kcd:{ 
      PrintInfo(("ERROR: p3d_enable_nbody not implemented for KCD\n"));
      return(res);
      break;
    }
    default:{ 
      PrintInfo(("\n Erreur p3d_enable_nbody, collision checker=none\n"));
      return FALSE;
    }
  }
}

/********************************************************/
/* Fonction permettant de desactiver l'algorithme nbody */
/* In :                                                 */
/* Out :                                                */
/********************************************************/
int p3d_disable_nbody(void)
{int res=0;

  switch (p3d_col_mode){
#ifdef VCOLLIDE_ACT 
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("ERROR: p3d_disable_nbody not implemented for VCOLLIDE\n"));
      return(res);
      break;
    }
#endif
    case p3d_col_mode_kcd:{ 
      PrintInfo(("ERROR: p3d_disable_nbody not implemented for KCD\n"));
      return(res);
      break;
    }
    default:{ 
      PrintInfo(("\n Erreur p3d_disable_nbody, collision checker=none\n"));
      return FALSE;
    }
  }
}

/* ************************************************ */
/* function sets minimal relevant volume iff        */
/* collision detector allows volume tuning          */
/* ************************************************ */
int p3d_col_set_user_defined_small_volume(double val)
{
  int success;

  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      success = FALSE;
      break;
    }
#endif
    case p3d_col_mode_kcd:{ 
      kcd_init_min_vol_of_obj_detail(val);
      success = TRUE;
      break;
    }
#ifdef GJK_DEBUG
  case p3d_col_mode_gjk:
    {
      success = FALSE;
      break;
    }
#endif /*GJK_DEBUG*/

    default:{ 
      PrintInfo(("\n Erreur p3d_col_set_user_defined_small_volume, collision checker=none\n"));
      success = FALSE;
    }
  }
  return success;
}

/* ************************************************ */
/* function gets minimal relevant volume iff        */
/* collision detector allows volume tuning          */
/* ************************************************ */
int p3d_col_get_user_defined_small_volume(double *val)
{
  int success;

  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      *val = 0.0;
      success = FALSE;
      break;
    }
#endif
    case p3d_col_mode_kcd:{ 
      kcd_get_user_defined_small_volume(val);
/*       PrintInfo(("p3d_col: val = %f\n",*val))); */
      success = TRUE;
      break;
    }
#ifdef GJK_DEBUG
  case p3d_col_mode_gjk:
    {
      *val = 0.0;
      success = FALSE;
      break;
    }
#endif /*GJK_DEBUG*/

    default:{ 
      PrintInfo(("\n Erreur p3d_col_get_user_defined_small_volume, collision checker=none\n"));
      *val = 0.0;
      success = FALSE;
    }
  }
  return success;
}

/***************************************************************************/
/* Modifications Pepijn  June 2001
 * concerning 
 *   - the separation of the notions of dmax and tolerance
 *   - the elimination of the microcollision (pb of Etienne Ferre)
 * see also documentation (KINEO)
 */      

/***************************************************************************/
/* Function p3d_col_set_tolerance()
 * this function will now set the kcd_tolerance equal to tolerance
 * (the real object tolerance used to enlarge the objects)
 * ARGS IN: tolerance value
 * ARGS OUT: returns true in case when KCD with distances is used, 
 *           false otherwise
 * USE: will be used in the initialisation of move3D (option -tol) and when 
        the slider is moved (FORMenv.c)
 */
/***************************************************************************/
int p3d_col_set_tolerance(double value)
{
  int success;

  switch (p3d_col_mode)
    { 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:
      { 
	success = FALSE;
	break;
      }
#endif
    case p3d_col_mode_kcd:
      { 
	if(RETURN_KCD_DISTANCE_ESTIMATE)
	  {
	    kcd_set_tolerance(value);
	    success = TRUE;
	  }      
	else
	  {
	    kcd_set_tolerance(0.0);
	    success = FALSE; 	  
	  }
	break;
      }

#ifdef GJK_DEBUG
  case p3d_col_mode_gjk:
    {
      gjk_debug_set_tolerance(value);
      success = TRUE;
      break;
    }
#endif /*GJK_DEBUG*/

#ifdef PQP
   case p3d_col_mode_pqp:
      success = TRUE;
   break;
#endif

    default:{ 
      PrintInfo(("\n Erreur p3d_set_tolerance, collision checker=none\n"));
      success = FALSE;
    }
  }
  return success;
} 
/***************************************************************************/
/* Function p3d_col_get_tolerance()
 * this function will now get the tolerance in case the current collision
 * checker uses  the tolerance (only KCD for the moment)
 * otherwise zero is returned
 * (the real object tolerance used to enlarge the objects)
 * ARGS IN: /
 * ARGS OUT: returns true and the tolerance in case of KCD
 *           false and zero otherwise
 * USE: retrieve the current tolerance
 */
/***************************************************************************/
int p3d_col_get_tolerance(double *val)
{
  int success;
  switch (p3d_col_mode)
    {
    case p3d_col_mode_kcd:{ 
      if( RETURN_KCD_DISTANCE_ESTIMATE )
	{
	  kcd_get_tolerance(val);
	  success = TRUE;
	}
      else
	{
	  *val = 0.0;
	  success = FALSE;
	}
      break;
    }

#ifdef GJK_DEBUG
  case p3d_col_mode_gjk:
    {
      gjk_debug_get_tolerance(val);
      success = TRUE ;
      break;
    }
#endif /*GJK_DEBUG*/

    default:{
       *val = 0.0;
       success = FALSE;
    }
  }  
/*  PrintInfo(("p3d_col_get_tolerance returns %f\n",*val))); */
  return success;
} 
/***************************************************************************/
/* Function p3d_col_get_dmax()
 * This function will retrieve dmax 
 * ARGS IN: /
 * ARGS OUT: dmax in all cases
 * USE: retrieve the current dmax
 */
/***************************************************************************/
void p3d_col_get_dmax(double *val)
{
      *val = p3d_get_env_dmax();
}

/***************************************************************************/
/* Function p3d_col_set_dmax()
 * This function will change dmax
 * ARGS IN: nex value
 * ARGS OUT: /
 * USE:change the current dmax
 */
/***************************************************************************/
void p3d_col_set_dmax(double val)
{
      p3d_set_env_dmax(val);
}
/***************************************************************************/
/* Function void p3d_col_set_relative_error(double epsilon)
 * This function will change relative error 
 * ARGS IN: relative error (as a percentage)
 * USE: to have a good approximation of the exact distances knowing the 
 *      relative error
 */
/***************************************************************************/
void p3d_col_set_relative_error(double epsilon)
{
  switch (p3d_col_mode)
    {	
    case p3d_col_mode_kcd:
      {
	kcd_set_relative_error(epsilon);
	break;
      }
    default:
      {
	PrintInfo(("WARNING: collision checker does not support a relative errro\n"));
      }
    }
}
/***************************************************************************/
/* Function p3d_col_get_relative_error(double *epsilon)
 * This function will get the value of the current relative error on the 
   exact distances
 * ARGS IN: 
 * ARGS OUT: value of relative error
 */
/***************************************************************************/
void p3d_col_get_relative_error(double *epsilon)
{
  switch (p3d_col_mode)
    {	
    case p3d_col_mode_kcd:
      {
	*epsilon = kcd_get_relative_error();
	break;
      }
    default:
      {
	PrintInfo(("WARNING: collision checker does not support a relative errro\n"));
      }
    }
}




/******************************************/
/* Fonction  test de collision global     */
/* In :                                   */
/* Out :    true or false                 */
/******************************************/
int p3d_col_test(void)
{ 
  double p3d_kcd_dist = P3D_HUGE;
  p3d_type_col_choice  mode_dist;

	switch (p3d_col_mode)
	{
#ifdef VCOLLIDE_ACT
	case p3d_col_mode_v_collide:
		vcCollide(vc_hand); /* VCOLLIDE collision test */
		/* PrintInfo(("num_of_objs = %i, vc_an_int = %i\n",num_of_objs,vc_an_int))); */
		p3d_report_num = vcInCollision(vc_hand); /* VCOLLIDE boolean report */
		/*       if( p3d_report_num ) */
		/* 	PrintInfo(("vcollide in collision in p3d_col_test()\n")); */
		return(p3d_report_num);
		break;
#endif
	case p3d_col_mode_kcd:
		if(RETURN_KCD_DISTANCE_ESTIMATE)
		{
		  mode_dist = get_kcd_which_test();
		  mode_dist = (p3d_type_col_choice) ((int)mode_dist - ((int)mode_dist%10));
		  set_kcd_which_test((p3d_type_col_choice)(P3D_KCD_ALL_DISTANCE_ESTIMATE+(int)mode_dist));
      //only the current robot is tested
// 		  p3d_report_num=p3d_kcd_collision_test_and_distance_estimate(&p3d_kcd_dist);
      p3d_report_num = kcd_robot_collides_something(XYZ_ENV->cur_robot->num, DISTANCE_ESTIMATE, &p3d_kcd_dist);
			/* PrintInfo(("test distance estimate: %f\n",p3d_kcd_dist))); */
		}
		else
		  {
		    set_kcd_which_test(P3D_KCD_ALL_NO_REPORT);
		    p3d_report_num=p3d_kcd_collision_test();
		  }
		/*       if(p3d_report_num) */
		/*         PrintInfo(("boem in p3d_col_test()\n")); */
		return(p3d_report_num);
		break;
#ifdef GJK_DEBUG
	case p3d_col_mode_gjk:
		p3d_report_num=p3d_gjk_collision_test();
		/*       if(p3d_report_num) */
		/* 	PrintInfo(("poum: gjk_collision in p3d_col_test()\n")); */
		return(p3d_report_num);
		break;
#endif /* GJK_DEBUG */

	case p3d_col_mode_bio: /* Modif Bio */
                return p3d_report_num= bio_all_molecules_col();
                return p3d_report_num;
		break;

#ifdef PQP
        case p3d_col_mode_pqp:
           p3d_report_num= pqp_all_collision_test();
           return p3d_report_num;
        break;
#endif
	default:
		PrintInfo(("\n Erreur p3d_col_test, collision checker=none\n"));
		return FALSE;
	}
} 
/******************************************/
/* Fonction  test de collision global     */
/* In :                                   */
/* Out :   all pairs                      */
/******************************************/
int p3d_col_test_all(void)
{ 
  double p3d_kcd_dist;
  p3d_type_col_choice  mode_dist;

  switch (p3d_col_mode)
    { 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:
      vcCollideAll(vc_hand); /* VCOLLIDE collision test */
      /* PrintInfo(("In p3d_col_test_all, vcollide\n")); */
      if(vc_colrep!=NULL)
        free(vc_colrep);
      vc_colrep = NULL;
      vc_colrep = (VCReportType *)malloc(sizeof(VCReportType)*MAX_REPORT);
      p3d_report_num = vcReport(vc_hand,MAX_REPORT,vc_colrep); /* VCOLLIDE report */
      vc_colrep = (VCReportType *)realloc(vc_colrep,sizeof(VCReportType)*p3d_report_num);
      /* PrintInfo(("p3d_report_num = %i\n",p3d_report_num))); */
      return(p3d_report_num);
      break;
#endif
    case p3d_col_mode_kcd:
      {
	mode_dist = get_kcd_which_test(); 
	if(RETURN_KCD_DISTANCE_ESTIMATE)
	  {  
	    set_kcd_which_test(P3D_KCD_ALL_DISTANCE_ESTIMATE);
	    p3d_report_num=p3d_kcd_collision_test_and_distance_estimate(&p3d_kcd_dist);
	    /* begin test: */
	    /* 	  PrintInfo(("distance estimate for robot: %f\n",p3d_kcd_dist))); */
	    /* 	  nj = XYZ_ENV->robot[0]->njoints+1; */
	    /* 	  distances = MY_ALLOC(double,nj); */
	    /* 	  p3d_kcd_report_distance_bodies_obst(0,distances); */
	    /* 	  for(i=0;i<nj;i++) */
	    /* 	    PrintInfo(("distance %i: %f\n",i,distances[i])); */
	    /* 	  MY_FREE(distances,double,nj); */
	    /* end test */
	  }
	else
	  {
	    set_kcd_which_test(P3D_KCD_ALL_JUST_BOOL);
	    p3d_report_num=p3d_kcd_collision_test_all();
	    /* PrintInfo(("just bool\n")); */
	  }
	set_kcd_which_test(mode_dist);
      }
      return p3d_report_num;
      break;
#ifdef GJK_DEBUG
    case p3d_col_mode_gjk:
      p3d_report_num=p3d_gjk_collision_test();
      /*       if(p3d_report_num) */
      /* 	PrintInfo(("poum: gjk_collision in p3d_col_test()\n")); */
      return(p3d_report_num);
      break;
#endif

    case p3d_col_mode_bio: /* Modif Bio */
      return p3d_report_num= bio_all_molecules_col_with_report();
      return p3d_report_num;
      break;

#ifdef PQP
    case p3d_col_mode_pqp:
          p3d_report_num= pqp_all_collision_test();
//          if(p3d_report_num)
//          {
//            p3d_obj *o1, *o2;
//            if(pqp_colliding_pair(&o1, &o2))
//            {
//              printf("Collision between \"%s\" and \"%s\"\n", o1->name, o2->name);
//            }
//          }
//          else
//          {
//        	  printf("No collision\n");
//          }

          return p3d_report_num;
    break;
#endif
    default:
      PrintInfo(("\n Erreur p3d_col_test_all, collision checker=none\n"));
      return FALSE;
    }
} 

/******************************************/
/*! \fn int p3d_col_test_choice()                                                                  
\brief test the different possibilities of a collision checker
\note Before calling this function the wanted mode must be specified
      In Move3D this is done by using the interface
\warning in Move3D this is only implemented for KCD and has only an 
influence on:  static test (e.g 1000 test)
          or   changing the configuration of the robot.
This function is not used in the planner module 
*/
/******************************************/

int p3d_col_test_choice()
{  
  p3d_type_col_choice choice_made =  get_kcd_which_test();
  int p3d_report_num = 0;
  p3d_type_col_choice kcd_with_report;
  p3d_rob *robotPt;

  switch (p3d_col_mode)
    {      
    case p3d_col_mode_kcd:
      {      
	robotPt = XYZ_ENV->cur_robot;
	kcd_with_report = (p3d_type_col_choice) ((int)choice_made%10);
        choice_made = (p3d_type_col_choice) ((int)choice_made - (int)kcd_with_report);
	//PrintInfo(("choice %d, wr %d \n",choice_made,kcd_with_report)));
	switch(choice_made)
	  {
	    
	  case P3D_KCD_ROB_ALL: 
	    {
	       p3d_report_num = p3d_col_test_robot(robotPt, kcd_with_report);	
	       break;
	    }
	  case P3D_KCD_ROB_ROB:
	    {
	      if(XYZ_ENV->nr >= 2)
		{
		  PrintInfo(("We test robot 0 against robot 1 \n"));
		  p3d_report_num = p3d_col_test_robot_other(XYZ_ENV->robot[0], XYZ_ENV->robot[1] , kcd_with_report);
		}
	      else
		{
		  PrintInfo(("WARNING: NO TEST, Not enough robots...\n"));
		}
	      break;	      
	    }
	  case P3D_KCD_ROB_AUTO:
	    {
	       p3d_report_num = p3d_col_test_self_collision(robotPt, kcd_with_report);
	       break;
	    }
	  case P3D_KCD_ROB_ENV:	    
	    {
	      p3d_report_num = p3d_col_test_robot_statics(robotPt, kcd_with_report);
	      break;
	    }
	  case P3D_KCD_EVERYTHING:
	    {
	      p3d_report_num = p3d_kcd_test_everything(kcd_with_report);
	      break;
	    }
	  default:
	    {
	      PrintInfo(("WARNING Wrong KCD choice in p3d_col_test_kcd_choice()\n"));
	    }
	  }
	break;
      }
    default:
      {
	PrintInfo(("ERROR test kcd choice only valid for KCD !\n"));
      }	
    }
  return ( p3d_report_num);
}



/* tests for auto collision of a device only */
int p3d_col_test_self_collision(p3d_rob *robotPt, int with_report)
{
  int p3d_report_num = 0;

  switch (p3d_col_mode)
    {      
    case p3d_col_mode_kcd:
      {
	set_kcd_which_test((p3d_type_col_choice)(P3D_KCD_ROB_AUTO+with_report));
	p3d_report_num = p3d_kcd_test_self_collision(robotPt->num, with_report);
	break;
      }
#ifdef PQP
    case p3d_col_mode_pqp:
      p3d_report_num = pqp_robot_selfcollision_test(robotPt);
    break;
#endif
    default:
      {
	PrintInfo(("\n ERROR p3d_col_test in p3d_col_test_self_collision()\n"));
	break;
      }
    }
  return(p3d_report_num);
}

/* tests for collision between two given devices only */
int p3d_col_test_robot_other(p3d_rob *robotPt1, p3d_rob *robotPt2, int with_report)
{
  int p3d_report_num = 0;
 
  switch (p3d_col_mode)
    {      
    case p3d_col_mode_kcd:
      {
	set_kcd_which_test((p3d_type_col_choice)(P3D_KCD_ROB_ROB+with_report));
	p3d_report_num = p3d_kcd_test_robot_other(robotPt1, robotPt2, with_report);
	break;
      }
#ifdef PQP
    case p3d_col_mode_pqp:
      p3d_report_num = pqp_robot_robot_collision_test(robotPt1, robotPt2);
    break;
#endif
    default:
      {
	PrintInfo(("\n ERROR p3d_col_test in p3d_col_test_robot_other()\n"));
	break;
      }
    }
  return(p3d_report_num);
}

/* tests for collision for given device only (i.e. tests self-collision,
   collision with other devices, collision with static objects */
int p3d_col_test_robot(p3d_rob *robotPt, int with_report)
{
  int p3d_report_num = 0;
  double kcd_distance_estimate = P3D_HUGE;

  switch (p3d_col_mode)
    {      
    case p3d_col_mode_kcd:
      {
	set_kcd_which_test((p3d_type_col_choice)(P3D_KCD_ROB_ALL+with_report));
	p3d_report_num = kcd_robot_collides_something(robotPt->num, with_report, &kcd_distance_estimate);
	break;
      }
#ifdef PQP
    case p3d_col_mode_pqp:
      p3d_report_num = pqp_robot_all_collision_test(robotPt);
    break;
#endif
    default:
      {
	PrintInfo(("\n ERROR p3d_col_test in p3d_col_test_robot()\n"));
	break;
      }
    }
  return(p3d_report_num);
}

/* tests for collision of device with static objects only */
int p3d_col_test_robot_statics(p3d_rob *robotPt, int with_report)
{
  int p3d_report_num = 0;
 
  switch (p3d_col_mode)
    {      
    case p3d_col_mode_kcd:
      {
	set_kcd_which_test((p3d_type_col_choice)(P3D_KCD_ROB_ENV+with_report));
	p3d_report_num = p3d_kcd_test_robot_statics(robotPt, with_report );
	break;
      }
    #ifdef PQP
    case p3d_col_mode_pqp:
       p3d_report_num = pqp_robot_environment_collision_test(robotPt);
    break;
    #endif
    default:
      {
	PrintInfo(("\n ERROR p3d_col_test in p3d_col_test_robot_statics()\n"));
	break;
      }
    }
  return(p3d_report_num);
}


/****************************************************************************/
/* Function returns an array of distances  (*distances) for a given robot   */
/*   iff the collision detector computes any  and TRUE                      */
/*     otherwise the function does not return any distance and FALSE        */
/* In :  robot pointer, space for array of distances                        */
/* Out : distances[i] is distance estimate between body i and the obstacles */
/****************************************************************************/
int p3d_col_report_distance_bodies_obst(p3d_rob *robotPt, double *distances)
{
  int distance_returned = TRUE;

  switch (p3d_col_mode)
    { 
    case p3d_col_mode_kcd:
      {
	if(RETURN_KCD_DISTANCE_ESTIMATE)
	  {	
  	    kcd_get_dist_report_obst(robotPt->num,distances);	
	    /* modification Pepijn june 2001 */
	    /* The returned distances are the distances to the obstacle
             * knowing that the distances returned by kcd are conservative.
             * This means that in general the returned distances are too small
	     */
	  }	  
	else
	  {
	    distance_returned = FALSE;
	  }
	break;
      }extern p3d_vector3 vectMinDist[2];
#ifdef GJK_DEBUG
/* Modification june 2001
 * use distances calculated by GJK
 */
  case p3d_col_mode_gjk:
      {
	 kcd_get_dist_report_obst(robotPt->num,distances);
	 break;
      }
#endif /*GJK_DEBUG*/
    default:
      { 
	distance_returned = FALSE;
      }
    }
return (distance_returned);
}




/**
 * p3d_GetMinDistCost
 * Get the cost of a current configuration based 
 * on its minimal distance to the obstacles. 
 * This function is used for algorithm based on configuration
 * spaces with cost functions
 * @param[In] robotPt: the robot
 * @return: the value of the cost for the current 
 * configuration of the robot 
 * Note: For efficiency reasons we suppose that the robot is not
 * in collision (configurations are node configurations), but it 
 * could change. 
 */

p3d_vector3 vectMinDist[2] = {{0,0,0},{0,0,0}};

double p3d_GetMinDistCost(p3d_rob* robotPt) {
	  int i, nof_bodies = robotPt->no;
	  double* distances;
	  double MinDist = P3D_HUGE;
	  double cost;


	  distances = MY_ALLOC(double, nof_bodies);

	  int settings = get_kcd_which_test();
	  set_kcd_which_test((p3d_type_col_choice)(40+3));
	  // 40 = KCD_ROB_ENV
	  // 3 = DISTANCE_EXACT

	  p3d_col_test_choice();

	  p3d_vector3* body = MY_ALLOC(p3d_vector3,nof_bodies);
	  p3d_vector3* other = MY_ALLOC(p3d_vector3,nof_bodies);

	  p3d_kcd_closest_points_robot_environment(robotPt,body,other,distances);

	 // Pour le manipulateur mettre 7 (dernier corps)
//	  i=7;

	  i = (int)(std::min_element(distances,distances+nof_bodies-1 )-distances);

	  MinDist = distances[i];

	  for(int it=0;it<3;it++)
		  vectMinDist[0][it] = body[i][it];

	  for(int it=0;it<3;it++)
		  vectMinDist[1][it] = other[i][it];

	//  if(MinDist)
	  MY_FREE(distances, double, nof_bodies);

	  cost = 2000/MinDist;
	//  cost = exp(-MinDist/1000);

	  set_kcd_which_test((p3d_type_col_choice)settings);

	  return cost;}

/**
 * p3d_GetAverageDistCost
 * Get the cost of a current configuration based 
 * on its average distance to the obstacles. 
 * This function is used for algorithm based on configuration
 * spaces with cost functions
 * @param[In] robotPt: the robot
 * @return: the value of the cost for the current 
 * configuration of the robot 
 * Note: For efficiency reasons we suppose that the robot is not
 * in collision (configurations are node configurations), but it 
 * could change. 
 */
double p3d_GetAverageDistCost(p3d_rob* robotPt) {
  int i, njnt = robotPt->njoints;
  double* distances;
  double cost;
  double costSum = 0.;
  distances = MY_ALLOC(double, njnt+1);
  p3d_col_test();
  kcd_get_dist_report_obst(robotPt->num, distances);
  for(i=0;i<njnt+1;i++ ) {
    //    PrintInfo(("dist[%d]: %f\n", i, distances[i]  ));
    costSum += exp(-distances[i]);
  }  
  //PrintInfo(("\n"));
  MY_FREE(distances, double, njnt+1);	

  cost = costSum/(njnt+1);
  return cost;
}

/****************************************************************************/
/*! \fn int p3d_col_report_distance(p3d_rob *robotPt, double *distances)
 * \brief retrieve the distances after the collision test is made
 * \param *robotPt pointer to the robot
 * \param *distances array of distances that is returned
 * \note using the variable p3d_kcd_which_test we retrieve the correct distances 
 */
/****************************************************************************/
int p3d_col_report_distance(p3d_rob *robotPt, double *distances)
{
  int distance_returned = TRUE;
  p3d_type_col_choice choice_made =  get_kcd_which_test();
  p3d_type_col_choice kcd_with_report;

  switch (p3d_col_mode) {

  case p3d_col_mode_kcd:

    kcd_with_report = (p3d_type_col_choice) ((int)choice_made%10);
    choice_made = (p3d_type_col_choice)( (int)choice_made - (int)kcd_with_report );

    if( kcd_with_report != DISTANCE_ESTIMATE ) {
      distance_returned = FALSE;
      PrintWarning(("WARNING: Wrong with_report in  p3d_col_report_distance()\n"));
    } else {

      switch(choice_made) {
      case P3D_KCD_COL_TEST:
	kcd_get_dist_report_obst(robotPt->num,distances);
	break;
      case P3D_KCD_ROB_ALL: 
	kcd_get_dist_report_both(robotPt->num,distances);
	break;
      case P3D_KCD_ROB_ROB:
	kcd_get_dist_for_bodies(robotPt->num,distances);  	
	break;	      
      case P3D_KCD_ROB_AUTO:
	kcd_get_dist_for_bodies(robotPt->num,distances);  	
	break;
      case P3D_KCD_ROB_ENV:	    
	kcd_get_dist_report_obst(robotPt->num,distances);
	break;
      case P3D_KCD_EVERYTHING:
	kcd_get_dist_report_both(robotPt->num,distances);    
	break;
      default:
	distance_returned = FALSE;
	PrintInfo(("WARNING Wrong KCD choice in p3d_col_report_distance()\n"));
      }	
    }  
    break;

#ifdef GJK_DEBUG
  case p3d_col_mode_gjk:
    kcd_get_dist_report_obst(robotPt->num,distances);
    break;
#endif /*GJK_DEBUG*/
  default:
    distance_returned = FALSE;

  }

  return (distance_returned);
}



/* modification Pepijn may 2001
 * This function calls the reportmecanism function of KCD
 *
 */
 
/****************************************************************************
 * Function returns an array of distances  (*distances) computed by the      
 * collision detector between the links of a given robot 
 *
 * In :  robot pointer      
 *       space for array of distances ->  [nof_bodies]
 *       space for matrix: points of movable objects ->  [3][nof_bodies]
 *       space for matrix:  points of static  objects ->  [3][nof_bodies] 
 * Out : distances[i] is distance estimate between body i and closest so
 *       points_bodies[3][i] coordinates of the point of the body (mo) that
 *                        is closest to the other object (moving or static)
 *                        (relative to the origine of the coordinate frame 
 *                         of the object = workspace)
 *                        except for GJK_DEBUG (relative to the center of the
 *                        body(mo)) 
 *       points_other[3][i] coordinates of the point of the object (so) that
 *                        is closest to the mo (relative to the origine of the
 *                        coordinate frame of the object)
 ****************************************************************************/
int p3d_col_report_closest_points(p3d_rob *robotPt, p3d_vector3 *points_bodies, 
				  p3d_vector3 *points_other, double *distances)
{
  int distance_returned = TRUE;
  int i, j, nof_bodies;
  double min;
  p3d_vector3 zpa,zpb;
  p3d_type_col_choice choice_made =  get_kcd_which_test();
  p3d_type_col_choice kcd_with_report;

  switch (p3d_col_mode)
    { 
    case p3d_col_mode_kcd:
      {
	kcd_with_report = (p3d_type_col_choice)((int)choice_made%10);
        choice_made = (p3d_type_col_choice) ((int)choice_made - (int)kcd_with_report);
	//PrintInfo(("Get points choice %d, wr %d \n",choice_made,kcd_with_report));
	switch(choice_made)
	  {
	  case P3D_KCD_COL_TEST:
	    {
	      if(kcd_with_report == DISTANCE_EXACT)
		{  
		  p3d_kcd_closest_points_robot_environment(robotPt,points_bodies,points_other,distances);	
		}
	      else
		{
		  distance_returned = FALSE;
		  PrintInfo(("WARNING:Closest points not implemented for with_report %d ()\n",kcd_with_report));
		}
	
	      break;
	    }
	  case P3D_KCD_ROB_ALL: 
	    {
	      p3d_kcd_closest_points_of_both(robotPt,points_bodies,points_other,distances);	
	      break;
	    }
	  case P3D_KCD_ROB_ROB:
	    {
	      p3d_kcd_closest_points_between_bodies(robotPt,points_bodies,points_other,distances);	
	      break;	      
	    }
	  case P3D_KCD_ROB_AUTO:
	    {
	      p3d_kcd_closest_points_between_bodies(robotPt,points_bodies,points_other,distances);
	       break;
	    }
	  case P3D_KCD_ROB_ENV:	    
	    {	   
	      p3d_kcd_closest_points_robot_environment(robotPt,points_bodies,points_other,distances);
	      break;
	    }
	  case P3D_KCD_EVERYTHING:
	    {
	      p3d_kcd_closest_points_of_both(robotPt,points_bodies,points_other,distances);	    
	      break;
	    }
	  default:
	    {	
	      distance_returned = FALSE;
	      PrintInfo(("WARNING Wrong KCD choice in p3d_col_report_closest_points()\n"));
	    }	 
	  }	  
	break;
    }


    case p3d_col_mode_gjk:
      {
	nof_bodies = robotPt->no;
        for(j=0;j<nof_bodies;j++)
	  {
	    kcd_get_points_closest_pair(robotPt->num,j,zpa,zpb);
	    for (i = 0 ; i < 3 ; i++)
	      {
		points_bodies[j][i]  = zpa[i]; 
		points_other[j][i]  = zpb[i]; 
	      }     
	    kcd_get_dist_grp_mo_so(robotPt->num,j,&min);
            distances[j] = min;
	  }
	distance_returned  = TRUE;
	break;
      }
    default:
      { 
	distance_returned  = FALSE;
	PrintInfo(("WARNING:p3d_col_report_closest_points(): collision checker must be GJK\n"));
      }
    }
  return distance_returned;
}







/*********************************************/
/* Fonction  test de collision d'une paire   */
/* de polyhedres                             */
/* In :                                      */
/* Out :                                     */
/*********************************************/
int p3d_col_test_pair(p3d_poly *obj1, p3d_poly *obj2)
{ 
  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("ERROR: p3d_col_test_pair not implemented for VCOLLIDE\n"));
      return(FALSE);
      break;
    }
#endif
    case p3d_col_mode_kcd:{ 
      PrintInfo(("ERROR: p3d_col_test_pair not implemented for KCD\n"));
      return(FALSE);
      break;
    }
#ifdef PQP
    case p3d_col_mode_pqp:
       PrintInfo(("\n Erreur p3d_col_test_pair, PQP implementation only works with p3d_obj pairs. Set COLLISION_BY_OBJECT to TRUE.\n"));
       return FALSE;
    break;
#endif
    default:{ 
      PrintInfo(("\n Erreur p3d_col_test_pair, collision checker=none\n"));
      return FALSE;
    }
  }
}


//! Tests the collision between two p3d_obj (obstacles or robot bodies).
//! Only implemented for PQP.
//! \return  1 in case of collision, 0 otherwise.
int p3d_col_test_pair(p3d_obj *obj1, p3d_obj *obj2)
{ 
  switch (p3d_col_mode)
  { 
#ifdef PQP
    case p3d_col_mode_pqp:
       return pqp_collision_test(obj1, obj2); 
    break;
#endif
    default:
      PrintInfo(("\n Error: p3d_col_test_pair(p3d_obj *, p3d_obj *), this function is only implemented for PQP\n"));
      return FALSE;
    break;
  }
}


//! Tests the collision between a robot and a p3d_obj (that can be a body of the robot).
//! Only implemented for PQP.
//! \return  1 in case of collision, 0 otherwise.
int p3d_col_test_robot_obj(p3d_rob *robot, p3d_obj *obj)
{ 
  switch (p3d_col_mode)
  { 
#ifdef PQP
    case p3d_col_mode_pqp:
      return pqp_robot_obj_collision_test(robot, obj);
    break;
#endif
    default:
      PrintInfo(("\n Error: p3d_col_test_robot_obj(p3d_robot *, p3d_obj *), this function is only implemented for PQP\n"));
      return FALSE;
    break;
  }
}


//! Deactivates all the collision tests for the specified object (that can be a body of the robot).
//! That means collision tests between the object and any other robot or environment body.
//! Only implemented for PQP.
//! \return  1 in case of collision, 0 otherwise.
int p3d_col_deactivate_obj(p3d_obj *obj)
{ 
  switch (p3d_col_mode)
  { 
#ifdef PQP
    case p3d_col_mode_pqp:
      return pqp_deactivate_object_collision(obj);
    break;
#endif
    default:
      PrintInfo(("\n Error: p3d_col_deactivate_full_obj(p3d_obj *), this function is only implemented for PQP\n"));
      return FALSE;
    break;
  }
}

//! Activates all the collision tests for the specified object (that can be a body of the robot).
//! That means collision tests between the object and any other robot or environment body.
//! Only implemented for PQP.
//! \return  1 in case of collision, 0 otherwise.
int p3d_col_activate_obj(p3d_obj *obj)
{ 
  switch (p3d_col_mode)
  { 
#ifdef PQP
    case p3d_col_mode_pqp:
      return pqp_activate_object_collision(obj);
    break;
#endif
    default:
      PrintInfo(("\n Error: p3d_col_activate_full_obj(p3d_obj *), this function is only implemented for PQP\n"));
      return FALSE;
    break;
  }
}

/*********************************************/
/* Fonction  test de collision optimise de   */
/* toutes les paires actives de polyhedres   */
/* In :                                      */
/* Out :                                     */
/*********************************************/
int p3d_col_test_act(void)
{/* PairNode *pair; */

 switch (p3d_col_mode){
#ifdef VCOLLIDE_ACT 
   case p3d_col_mode_v_collide:{ 
     PrintInfo(("ERROR: p3d_col_test_act not implemented for VCOLLIDE\n"));
     return(FALSE);
     break;
   }
#endif
  case p3d_col_mode_kcd:{ 
     PrintInfo(("ERROR: p3d_col_test_act not implemented for KCD\n"));
     return(FALSE);
     break;
   }
#ifdef PQP
    case p3d_col_mode_pqp:
       PrintInfo(("ERROR: p3d_col_test_act is not implemented for PQP.\n"));
       return TRUE;
    break;
#endif
   default:{ 
     PrintInfo(("\n Erreur p3d_test_act, collision checker=none\n"));
     return FALSE;
   }
 }
} 

/***************************************************/
/* Fonction reportant le nombre de collisions      */
/* In :                                            */
/* Out :                                           */
/***************************************************/
int p3d_col_number(void) 
{
  return(p3d_report_num);
}

/*************************************************/
/* Fonction reportant la ieme collisions du test */
/* sans commentaire                              */
/* In : le numero                                */
/* Out : la paire d'objets en collision          */
/*************************************************/
void p3d_col_get(int ind, int *id1, int *id2)
{ 

  switch (p3d_col_mode){
#ifdef VCOLLIDE_ACT 
    case p3d_col_mode_v_collide:{
      *id1 = vc_colrep[ind-1].id1;
      *id2 = vc_colrep[ind-1].id2;
      /* PrintInfo(("VCOLLIDE,p3d_col_get: %i, %i\n",*id1,*id2)); */
      break;
    }
#endif
    case p3d_col_mode_kcd:{
      /* 	p3d_kcd_col_get_id_report(ind-1,id1,id2); */
      /* PrintInfo(("SOLID,p3d_col_get: %i, %i\n",ind, *id1, *id2)); */
      break;
    }
#ifdef PQP
    case p3d_col_mode_pqp:
       PrintInfo(("\n Erreur p3d_col_get, this function is not implemented for PQP.\n"));
    break;
#endif
    default:{ PrintInfo(("\n Erreur p3d_col_get, collision checker=none\n"));
    }
  }
}

/*************************************************/
/* Fonction reportant la ieme collisions du test */
/* In : le numero                                */
/* Out : la paire d'objets en collision          */
/*************************************************/
void p3d_col_get_report(int ind, p3d_poly **p1, p3d_poly **p2)
{ 
#ifdef VCOLLIDE_ACT
  int ncol;
#endif

  switch (p3d_col_mode){
#ifdef VCOLLIDE_ACT
   case p3d_col_mode_v_collide:{ 
      ncol= p3d_col_number();
      if(ind>ncol){PrintInfo(("Dans p3d_col_get : il n'y a pas autant de collisions\n"));}
      p1=p3d_v_collide_get_poly_by_id(vc_colrep[ind-1].id1);
      p2=p3d_v_collide_get_poly_by_id(vc_colrep[ind-1].id2);
      if(!(COLLISION_BY_OBJECT)){PrintInfo(("p3d_col_get_report : collision entre %s et %s\n",p1->poly->name,p2->poly->name));}
      break;
    }
#endif
    case p3d_col_mode_kcd:{
      /*       p3d_kcd_col_get_report(ind-1,p1,p2); */
/*       if(p1&&p2) */
/* 	PrintInfo(("p3d_col_get_report: ind = %i, p1->id = %i, p2->id = %i\n",ind,p1->id, p2->id)); */

      /* B Kineo Carl 22.02.2002 */

      kcd_get_pairInCollision((void **)p1,(void **)p2);
      /* begin test */
/*       if(p1&&p2) */
/* 	printf("kcd: %s clashes %s\n",(*p1)->poly->name,(*p2)->poly->name); */
/*       else */
/* 	printf("kcd: kcd_get_pairInCollision returned NULL pointer(s)\n"); */
      /*  end  test */
      /* E Kineo Carl 22.02.2002 */


      break;
    }
    default:{ PrintInfo(("\n Erreur p3d_col_get_report, collision checker=none\n"));
    }
  } 
}

void p3d_col_get_report_obj(p3d_obj **o1, p3d_obj **o2)
{
  switch (p3d_col_mode){
#ifdef VCOLLIDE_ACT
   case p3d_col_mode_v_collide:{
      break;
    }
#endif
    case p3d_col_mode_kcd:{
      p3d_kcd_get_pairObjInCollision(o1,o2);
      /* begin test */
/*       if(o1&&o2) */
/* 	printf("p3d_col_get_report_obj: %s clashes %s\n",(*o1)->name,(*o2)->name); */
/*       else */
/* 	printf("p3d_col_get_report_obj: kcd_get_pairObjInCollision returned NULL pointer(s)\n"); */
      /*  end  test */
      break;
    }
    case p3d_col_mode_pqp:
      pqp_colliding_pair(o1,o2);
    break;
    default:{ PrintInfo(("\n Erreur p3d_col_get_report, collision checker=none\n"));
    }
  }
}

void p3d_col_init_coll()
{
  p3d_v_collide_set_null();

}

/********************************************************************************/
/* Fonction permettant de supprimer la structure polyhedre du collision checker */
/* In :le polyhedre                                                             */
/* Out :                                                                        */
/********************************************************************************/
void p3d_col_del_poly(p3d_poly *p)
{ 
  /*
    switch (p3d_col_mode)
    {
    case p3d_col_mode_none:break;
    case p3d_col_mode_v_collide:{ p3d_v_collide_del_poly(p);
    break;
    }
    default:PrintInfo(("\n Erreur p3d_col_del_poly, collision checker=none\n"));              
    }
  */ 
  p3d_v_collide_del_poly(p);
}

/********************************************************************************/
/* Fonction permettant de positionner les donnees d'un objet lie au             */
/* collision checker                                                            */
/* In :le polyedre, la matrice de position                                      */
/* Out :                                                                        */
/********************************************************************************/
void p3d_col_set_pos_of_object(p3d_obj *p, p3d_matrix4 mat)
{ 
  switch (p3d_col_mode)
    {
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:
      { 
	p3d_v_collide_set_pos_of_object(p,mat);
	break;
      }
#endif
    case p3d_col_mode_kcd:
      {
	p3d_kcd_set_object_moved(p->o_id_in_env,TRUE);
	break;
      }
#ifdef PQP
    case p3d_col_mode_pqp:
        // PrintInfo(("\n Erreur p3d_col_set_pos_of_object, this function is not implemented for PQP.\n"));
    break;
#endif
    case p3d_col_mode_none: break;
        default:PrintInfo(("\n Erreur p3d_col_set_pos_of_object, collision checker=none\n"));              
    } 
}

/********************************************************************************/
/* Fonction permettant de positionner les donnees d'un polyedre lie au          */
/* collision checker                                                            */
/* In :le polyedre, la matrice de position                                      */
/* Out :                                                                        */
/********************************************************************************/
void p3d_col_set_pos(p3d_poly *p, p3d_matrix4 mat)
{ 
  if(p->TYPE != P3D_GRAPHIC){
    switch (p3d_col_mode){
#ifdef VCOLLIDE_ACT 
      case p3d_col_mode_v_collide:{ 
	p3d_v_collide_set_pos(p,mat);
	break;
      }
#endif
      case p3d_col_mode_kcd:
	{
	  p3d_kcd_set_pos(p,mat);
	  break;
	}
    case p3d_col_mode_gjk: 
	break;
    case p3d_col_mode_bio:
      break;
    case p3d_col_mode_none: 
	break;
#ifdef PQP
      case p3d_col_mode_pqp:
         PrintInfo(("\n Erreur p3d_col_set_pos, this function is not implemented for PQP.\n"));
      break;
#endif
      default:
	PrintInfo(("\n Erreur p3d_col_set_pos, collision checker=none\n"));
    } 
  }
}


/********************************************************************************/
/* Fonction permettant de savoir si l ID du polyhedre correspond a id1 ou id2   */
/* In :le polyhedre ,les ID id1 et id2                                          */
/* Out : TRUE id1 ou id2 est egal a l id du polyhedre                           */
/*       FALSE sinon                                                            */
/********************************************************************************/
int p3d_col_test_poly_id(p3d_poly *p,int id1,int id2)
{ 
  switch (p3d_col_mode)
    { 
#ifdef VCOLLIDE_ACT      
    case p3d_col_mode_v_collide:
      return (p3d_v_collide_get_id(p)==id1 || p3d_v_collide_get_id(p)==id2);
      break;
#endif     
    case p3d_col_mode_kcd:
      if(p)
	return((p->id == id1)||(p->id == id2));
      else
	return FALSE;
      break;
      
    default:	
      PrintInfo(("\n Erreur p3d_col_test_poly_id, collision checker=none\n"));
      return FALSE;
    }
}

/********************************************************************************/
/* Fonction permettant de savoir si l ID de l'objet   correspond a id1 ou id2   */
/* In :l'objet,      les ID id1 et id2                                          */
/* Out : TRUE id1 ou id2 est egal a l id de l'objet                             */
/*       FALSE sinon                                                            */
/********************************************************************************/
int p3d_col_test_object_id(p3d_obj *p,int id1,int id2)
{ 
  switch (p3d_col_mode)
    {
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      return (p3d_v_collide_object_get_id(p)==id1 || p3d_v_collide_object_get_id(p)==id2);
      break;
    }
#endif      
    default:{ 
      PrintInfo(("\n Erreur p3d_col_test_poly_id, collision checker=none\n"));
      return FALSE;
    }
    }
}
/********************************************************************************/
/* Fonction permettant de savoir si robot numero rob_nr est en                  */
/*   collision ou pas.                                                          */
/* In :  rob_nr, numcoll (=total number of collisions)                          */
/* Out:  TRUE si robot en collision, FALSE sinon.                               */
/********************************************************************************/
int p3d_col_does_robot_collide(int rob_nr, int numcoll)
{
#ifdef VCOLLIDE_ACT
  int ib,ip,np;
  int i,coll=0,id1,id2;
  pp3d_obj o;
#endif
  int nb;

  nb= p3d_get_desc_number(P3D_BODY);
  switch (p3d_col_mode)
    {
    case p3d_col_mode_kcd: 
      {
	int val = kcd_check_report(rob_nr);
	return val;
      }
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:
      { 
	/* facon bourrine de voir si ce robot est en collision */
	i=1;
	while(i<=numcoll && coll == 0){
	  p3d_col_get(i,&id1,&id2);
	  ib=0;
	  while(ib<nb && coll == 0){
	    p3d_sel_desc_num(P3D_BODY,ib);
	    o = (p3d_obj *) p3d_get_desc_curid(P3D_BODY);
	    if(COLLISION_BY_OBJECT)
	      {
		coll=p3d_col_test_object_id(o,id1,id2);
	      }
	    else
	      {
		np = o->np; ip = 0;
		while(ip<np && coll == 0){
		  /* tsiano if(o->pol[ip]->id==id1 || o->pol[ip]->id==id2){coll=1;} */
		  coll=p3d_col_test_poly_id(o->pol[ip],id1,id2);         
		  ip=ip+1;
		}
	      }
	    ib=ib+1;	 
	  }
	  i=i+1;
	}
	return coll;
      }
#endif
    case p3d_col_mode_gjk:
      {
	int val = kcd_check_report(rob_nr);
	return val;
      }
    case p3d_col_mode_bio:
      {
	int val = biocol_robot_report(rob_nr);
	return val;
      }
#ifdef PQP
    case p3d_col_mode_pqp:
     {    
       int val =  pqp_robot_all_collision_test(XYZ_ENV->robot[rob_nr]);
       return val;
    }
#endif

    case p3d_col_mode_none:
      {
	PrintInfo(("\n WARNING p3d_col_does_robot_collide, no collision checker active\n")); 
	return FALSE;
      }
    default:
      {
	/* default */
	PrintInfo(("\n Error p3d_col_does_robot_collide, collision checker not defined\n"));
	return FALSE;
      }
    }
}
/********************************************************************************/
/* Fonction permettant de recuperer le pointeur de fonction qui calcul les BB   */
/*   dans le collision checker                                                  */
/* In :                                                                         */
/* Out : le pointeur                                                            */
/********************************************************************************/
void *p3d_col_get_col_BB_poly_fct(void)
{ 
  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("WARNING: p3d_col_get_col_BB_poly_fct does not apply to VCOLLIDE\n"));
      return NULL;
      break;
    }
#endif
    case p3d_col_mode_kcd:{ 
      PrintInfo(("WARNING: p3d_col_get_col_BB_poly_fct does not apply to KCD\n"));
      return NULL;
      break;
    }
#ifdef PQP
    case p3d_col_mode_pqp:{ 
      PrintInfo(("WARNING: p3d_col_get_col_BB_poly_fct does not apply to PQP\n"));
      return NULL;
      break;
    }
#endif
    default:{ 
      PrintInfo(("\n BB_poly calculee a partir de p3d_BB_get_BB_poly\n"));
      return (void*)p3d_BB_get_BB_poly1;
    }
  }
}

/********************************************************************************/
/* Fonction demarrant le collision checker numero  p3d_col_mode                 */
/* In :                                                                         */
/* Out :                                                                        */
/********************************************************************************/
void p3d_col_start_current(void)
{
  /* p3d_BB_start();  Modif Bio */
  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{
      p3d_BB_start(); /* Modif Bio */ 
      p3d_v_collide_start();
      PrintInfo(("\nCollision checker=CC5\n"));
      p3d_col_activate_env();p3d_col_activate_robots(); /* Modif Bio */
      break;
    }
#endif
    case p3d_col_mode_kcd:{   
      p3d_BB_start(); /* Modif Bio */
      p3d_start_kcd();
      PrintInfo(("\nCollision checker=KCD\n"));
      p3d_col_pair_start();
      p3d_col_env_start();
      p3d_col_activate_env();p3d_col_activate_robots(); /* Modif Bio */
      break;
    }
  
    case p3d_col_mode_bio:{ /* Modif Bio */
      p3d_start_bio_col();
      break;
    }

    case p3d_col_mode_gjk:{   
      p3d_BB_start(); /* Modif Bio */
      kcd_init_report();
      PrintInfo(("\nCollision checker=GJK debug mode\n"));
      PrintInfo(("Note: robot self intersection is not tested\n"));
      PrintInfo(("Note: robot1 vs. robot2 not tested\n"));
      PrintInfo(("Note: concave facets are not tested\n"));
      p3d_col_activate_env();p3d_col_activate_robots(); /* Modif Bio */
      break;
    }
    #ifdef PQP
    case p3d_col_mode_pqp:
       //call the following functions BEFORE calling p3d_start_pqp()
       p3d_BB_start(); 
       p3d_col_pair_start();
       p3d_col_env_start();
       p3d_col_activate_env();
       p3d_col_activate_robots();

       PrintInfo(("\n"));
       PrintInfo(("############################\n"));
       PrintInfo(("## Collision checker= PQP ##\n"));
       PrintInfo(("############################\n\n"));
       p3d_start_pqp();
       //pqp_print_collision_pairs();
    break;
    #endif

    default:{ 
      PrintInfo(("p3d_col_start_current\n!!! No collision detector active\n"));
    }
  }
  /* p3d_col_activate_env();p3d_col_activate_robots();  Modif Bio */
}

/********************************************************************************/
/* Fonction demarrant le collision checker numero p3d_col_last_mode                             */
/* In : le numero du collision checker                                          */
/* Out :                                                                        */
/********************************************************************************/
void p3d_col_start_last()
{ 
  p3d_col_mode=p3d_col_last_mode;
  p3d_col_start_current();
}

/********************************************************************************/
/* Fonction demarrant le collision checker numero c                             */
/* In : le numero du collision checker                                          */
/* Out :                                                                        */
/********************************************************************************/

void p3d_col_start(int c)
{ 
  p3d_col_mode=c;
  p3d_col_start_current();
} 

/********************************************************************************/
/* Fonction arretant le collision checker                                       */
/* In : le numero du collision checker                                         */
/* Out :                                                                        */
/********************************************************************************/
void p3d_col_stop(void)
{ /* p3d_col_mode=p3d_col_mode_none; */
  p3d_BB_set_mode_close();
  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      p3d_v_collide_stop();
      break;
    }
#endif
  case p3d_col_mode_kcd:{ 
      p3d_col_env_stop();
      p3d_col_pair_stop();
      p3d_kcd_stop();
      /* PrintInfo(("kcd gestopt!\n")); */
      break;
    }
    default:{ PrintInfo(("p3d_col_stop\n No collision detector active\n"));
    }
  }
  p3d_BB_stop();
}

/* destruct all collision detection data */
void p3d_col_stop_all(void)
{
  p3d_filter_cleanup();
  p3d_v_collide_stop();
  p3d_col_env_stop();
  p3d_col_pair_stop();
  p3d_kcd_stop();
  p3d_BB_stop();
  p3d_col_last_mode=p3d_col_mode;
  p3d_col_mode=p3d_col_mode_none;
}

void p3d_col_BB_set_mode_col(void)
{
  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("Mode col not with V_COLLIDE\n"));
      break;
    }
#endif
    case p3d_col_mode_kcd:{ 
      /*  p3d_BB_set_mode_close(); */
      PrintInfo(("Mode col not with KCD\n"));
      break;
    }
    default:{ 
      PrintInfo(("p3d_col_BB_set_mode_col\n No collision detector active\n"));
    }
  }
}

int p3d_col_get_mode(void)
{
  return p3d_col_mode;
}

void p3d_col_set_mode(int col_mode)
{
  p3d_col_mode = col_mode;
}


/* Author: Carl Van Geem   Date: 6 November 2001   Function: p3d_col_report_mo_closest_points */


/****************************************************************************
 * Function returns a distance   computed by the      
 * collision detector between  a given movable object  and other things
 *
 * In :  robot pointer      
 *       space for array of distances ->  [nof_bodies]
 *       space for matrix: points of movable objects ->  [3][nof_bodies]
 *       space for matrix:  points of static  objects ->  [3][nof_bodies] 
 * Out : distances[i] is distance estimate between body i and closest so
 *       points_bodies[3][i] coordinates of the point of the body (mo) that
 *                        is closest to the other object (moving or static)
 *                        (relative to the origine of the coordinate frame 
 *                         of the object = workspace)
 *                        except for GJK_DEBUG (relative to the center of the
 *                        body(mo)) 
 *       points_other[3][i] coordinates of the point of the object (so) that
 *                        is closest to the mo (relative to the origine of the
 *                        coordinate frame of the object)
 ****************************************************************************/
int p3d_col_report_mo_closest_points(p3d_obj *mobjPt, p3d_vector3 point_mo, 
				  p3d_vector3 point_o, double *distance)
{
  int distance_returned = TRUE;
  p3d_type_col_choice choice_made =  get_kcd_which_test();
  p3d_type_col_choice kcd_with_report;

  switch (p3d_col_mode)
    { 
    case p3d_col_mode_kcd:
      {
	kcd_with_report = (p3d_type_col_choice) ((int)choice_made%10);
        choice_made = (p3d_type_col_choice) ((int)choice_made - (int)kcd_with_report);
	//PrintInfo(("Get points choice %d, wr %d \n",choice_made,kcd_with_report));
	switch(choice_made)
	  {
	  case P3D_KCD_COL_TEST:
	    {
	      if(kcd_with_report == DISTANCE_EXACT)
		{  
		  p3d_kcd_closest_points_mo_of_both(mobjPt,point_mo,point_o,distance);	
		}
	      else
		{
		  distance_returned = FALSE;
		  PrintInfo(("WARNING:Closest points not available for with_report %d ()\n",kcd_with_report));
		}
	
	      break;
	    }
	  case P3D_KCD_ROB_ALL: 
	    {
	      PrintInfo(("WARNING: not for robot but for movable object\n"));
	      p3d_kcd_closest_points_mo_of_both(mobjPt,point_mo,point_o,distance);	
	      break;
	    }
	  case P3D_KCD_ROB_ROB:
	    {
	      PrintInfo(("WARNING:Closest points not available\n"));
	      distance_returned = FALSE;
	      break;	      
	    }
	  case P3D_KCD_ROB_AUTO:
	    {
	      PrintInfo(("WARNING:Closest points not available\n"));
	      distance_returned = FALSE;
	      break;
	    }
	  case P3D_KCD_ROB_ENV:	    
	    {	   
	      PrintInfo(("WARNING: not for robot but for movable object\n"));
	      p3d_kcd_closest_points_mo_environment(mobjPt,point_mo,point_o,distance);
	      break;
	    }
	  case P3D_KCD_EVERYTHING:
	    {
	      p3d_kcd_closest_points_mo_of_both(mobjPt,point_mo,point_o,distance);	    
	      break;
	    }
	  case P3D_KCD_MO_ENV:
	    {
	      p3d_kcd_closest_points_mo_environment(mobjPt,point_mo,point_o,distance);
	      break;
	    }
	  default:
	    {	
	      distance_returned = FALSE;
	      PrintInfo(("WARNING Wrong KCD choice in p3d_col_report_closest_points()\n"));
	    }	 
	  }	  
	break;
    }


    default:
      { 
	distance_returned  = FALSE;
	PrintInfo(("WARNING:p3d_col_report_mo_closest_points(): collision checker must be KCD\n"));
      }
    }
  return distance_returned;
}


/****************************************************/
/* Fonction de desactivation d'un obstacle          */
/* In :     obstaclePt    pointer to obstacle       */
/* Out :                                            */
/****************************************************/
void p3d_col_deactivate_obstacle(p3d_obj *obstaclePt)
{
  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("ERROR: p3d_col_deactivate_obstacle not implemented for VCOLLIDE\n"));
      break;
    }
#endif
    case p3d_col_mode_kcd:{ 
      p3d_kcd_deactivate_obstacle(obstaclePt);
      break;
    }
#ifdef PQP
    case p3d_col_mode_pqp:
      pqp_deactivate_object_collision(obstaclePt);
    break;
#endif
    default:
      PrintInfo(("\n Erreur p3d_col_deactivate_obstacle, collision checker=none\n"));
  }
}

/****************************************************/
/* Fonction de (re-)activation d'un obstacle        */
/* In :     obstaclePt    pointer to obstacle       */
/* Out :                                            */
/****************************************************/
void p3d_col_activate_obstacle(p3d_obj *obstaclePt)
{
  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("ERROR: p3d_col_activate_obstacle not implemented for VCOLLIDE\n"));
      break;
    }
#endif
    case p3d_col_mode_kcd:{ 
      p3d_kcd_activate_obstacle(obstaclePt);
      break;
    }
#ifdef PQP
    case p3d_col_mode_pqp:
      pqp_activate_object_collision(obstaclePt);
    break;
#endif
    default:
      PrintInfo(("\n Erreur p3d_col_activate_obstacle, collision checker=none\n"));
  }
}


/********************************************************************************/
/* Fonction ajoutant un obstacle au collision checker deja initialise           */
/* In :                                                                         */
/* Out : TRUE if obstacle successfully added, FALSE otherwise                   */
/********************************************************************************/
int p3d_col_add_obstacle(p3d_obj *obst)
{
  int success = FALSE;

  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("\nnot implemented for Collision checker=CC5\n"));
      success = FALSE;
      break;
    }
#endif
    case p3d_col_mode_kcd:{   
      success = kcd_add_obstacle(obst);

      p3d_BB_activate(obst);

      PrintInfo(("\nCollision checker=KCD\n"));
      /* MY_ALLOC_INFO("Collision checker=KCD"); */
      break;
    }
    case p3d_col_mode_gjk:{   
      PrintInfo(("\nnot implemented for Collision checker=GJK debug mode\n"));
      success = FALSE;
      break;
    }
    default:{ 
      PrintInfo(("p3d_col_add_obstacle\n!!! No collision detector active\n"));
      success = FALSE;
    }
  }
  return success;
}

/********************************************************************************/
/* Fonction ajoutant plusieures obstacles au collision checker deja initialise  */
/* In :                                                                         */
/* Out : TRUE if obstacles successfully added, FALSE otherwise                  */
/********************************************************************************/
int p3d_col_add_obstacle_list(p3d_obj **obst, int nof_obst)
{
  int success = FALSE;
  int i;

  switch (p3d_col_mode){ 
#ifdef VCOLLIDE_ACT
    case p3d_col_mode_v_collide:{ 
      PrintInfo(("\nnot implemented for Collision checker=CC5\n"));
      success = FALSE;
      break;
    }
#endif
    case p3d_col_mode_kcd:{   
      success = kcd_add_obstacle_list(obst,nof_obst);

      for(i = 0; i < nof_obst; i++)
	p3d_BB_activate(obst[i]);
      PrintInfo(("\nCollision checker=KCD\n"));
      /* MY_ALLOC_INFO("Collision checker=KCD"); */
      break;
    }
    case p3d_col_mode_gjk:{   
      PrintInfo(("\nnot implemented for Collision checker=GJK debug mode\n"));
      success = FALSE;
      break;
    }
    default:{ 
      PrintInfo(("p3d_col_add_obstacle_list\n!!! No collision detector active\n"));
      success = FALSE;
    }
  }
  return success;
}

