/*********************************************/
/* Ce fichier permet de creer des polyhedres */
/*********************************************/

#include "Util-pkg.h"
#include "P3d-pkg.h"
//#include "Planner-pkg.h"
//#include "Localpath-pkg.h"
//#include "Collision-pkg.h"

/*
#define Vtag 1 
#define Etag 2 
#define Ftag 3
#define TRUE 1 
#define FALSE 0 
*/

/* Variables globales */

int p3d_polymax=0;   /* nombre max de polyhedres */

float p3d_xmin=P3D_HUGE,p3d_xmax=-P3D_HUGE,p3d_ymin=P3D_HUGE,p3d_ymax=-P3D_HUGE,p3d_zmin=P3D_HUGE,p3d_zmax=-P3D_HUGE; /* Limite de la boite visible */

int IN_OBSTACLE, LASTr, LASTo, LASTp;

//extern pp3d_obj XYZ_OBSTACLES;
//extern pp3d_env XYZ_ENV;         /* bibliotheque gerant tous les polyhedres obstacles chargees dans p3d_data.h */

/* contraire au poly_get_poly_pos(), cette fonction ne fait pas de copie
   de la matrice pos. Celle-ci retourne un pointer */
void p3d_get_poly_pos_of_polyh(p3d_poly *polyPt, poly_matrix4 **pos) {
  *pos = &(polyPt->poly->pos);
}

void p3d_get_poly_pos0(p3d_poly *polyPt, p3d_matrix4 **pos0) {
  *pos0 = &(polyPt->pos0);
}

/*--------------------------------------------------------------------------*/
/*! \brief Return the matrice of position relative of the polyhedre to
 *         the joint where it is attached.
 *
 *  \param  polyPt: The polyhedre.
 *
 *  \retval pos:    The pointer to the relative position matrix of \a polyPt.
 *
 *  \note If the polyhedre is not linked to a joint the default value of 
 *        \a pos is the absolute position of \a polyPt.
 */
void p3d_get_poly_pos_rel_jnt(p3d_poly *polyPt, p3d_matrix4 **pos) {
  *pos = &(polyPt->pos_rel_jnt);
}

int p3d_get_poly_id(p3d_poly *polyPt) {
  return polyPt->id;
}

/* array of points, returns *(double[3]) */
/* polyh1->poly->the_points */
void p3d_get_poly_point_array(p3d_poly *polyPt, poly_vector3 **the_points) {
  *the_points = polyPt->poly->the_points  ;
}

/* poly->poly->nb_points */
/* unsigned int poly_get_nb_points(poly_polyhedre *polyhedre) */
unsigned int p3d_get_nof_points_of_polyh(p3d_poly *polyPt) {
  return poly_get_nb_points(polyPt->poly);
}

/* parameter 'i' must be at least 1 ! */
unsigned int p3d_get_nof_points_in_face_of_polyh(p3d_poly *polyPt,int i) {
  return poly_get_nb_points_in_face(polyPt->poly,i);
}

void p3d_get_points_in_face_of_polyh(p3d_poly *polyPt,int fid, poly_index_p *the_points) {
  *the_points = p3d_get_points_in_face(polyPt->poly,fid);
}

int p3d_get_index_point_in_face_of_polyh(p3d_poly *polyPt,int fid, int pid) {
  return poly_get_index_point_in_face(polyPt->poly,fid, pid);
}

unsigned int p3d_get_nof_faces_of_polyh(p3d_poly *polyPt) {
  return poly_get_nb_faces(polyPt->poly);
}

int p3d_get_poly_is_convex(p3d_poly *polyPt) {
  return p3d_poly_is_convex(polyPt->poly);
}

int p3d_get_facet_is_convex(p3d_poly *polyPt, int facet_nr) {
  return p3d_poly_facet_is_convex(polyPt->poly,facet_nr);
}

/* poly->primitive_data->radius */
void p3d_get_prim_radius(p3d_poly *polyPt,double *the_datum) {
  if(polyPt->entity_type == CONE_ENTITY)
    *the_datum = (polyPt->primitive_data->radius / 2.0);
  else
    *the_datum = polyPt->primitive_data->radius;
}
/* poly->primitive_data->other_radius */
void p3d_get_prim_other_radius(p3d_poly *polyPt,double *the_datum) {
  if(polyPt->entity_type == CONE_ENTITY)
    *the_datum = (polyPt->primitive_data->other_radius / 2.0);
  else
    *the_datum = polyPt->primitive_data->other_radius;
}
/* poly->primitive_data->sin_slope */
void p3d_get_prim_sin_slope(p3d_poly *polyPt,double *the_datum) {
  *the_datum = polyPt->primitive_data->sin_slope;
}
/* poly->primitive_data->height */
void p3d_get_prim_height(p3d_poly *polyPt,double *height) {
  *height = polyPt->primitive_data->height;
}
/* poly->primitive_data->x_length */
void p3d_get_prim_x_length(p3d_poly *polyPt,double *x) {
  *x = polyPt->primitive_data->x_length;
}
/* poly->primitive_data->y_length */
void p3d_get_prim_y_length(p3d_poly *polyPt,double *y) {
  *y = polyPt->primitive_data->y_length;
}
/* poly->primitive_data->z_length */
void p3d_get_prim_z_length(p3d_poly *polyPt,double *z) {
  *z = polyPt->primitive_data->z_length;
}
void p3d_get_prim_lengths(p3d_poly *polyPt,double *x,double *y,double *z) {
  *x = polyPt->primitive_data->x_length;
  *y = polyPt->primitive_data->y_length;
  *z = polyPt->primitive_data->z_length;
}

/*******************************************/
/* Fonction permettant de creer, d'allouer */
/* et de nommer un polyhedre               */
/* in : le nom du polyhedre                */
/* out : un pointeur sur le polyhedre      */
/*******************************************/
p3d_poly *p3d_poly_beg_poly(char name[20],int type) {
  p3d_poly *poly ;

  if(!(poly = MY_ALLOC(p3d_poly,1))) {
    PrintInfo(("allocation failed in p3d_begin_polyhedre\n"));
  }
  poly->poly=p3d_create_poly(name);

  poly->entity_type = POLYHEDRON_ENTITY;
  poly->primitive_data = NULL;

  poly->id=p3d_polymax;

  poly->TYPE = type;

  poly->color = -1;  // <- modif Juan

  poly->MODIF=TRUE ;
  poly->box.x1  =  P3D_HUGE; poly->box.y1 =  P3D_HUGE; poly->box.z1 =  P3D_HUGE;
  poly->box.x2  = -P3D_HUGE; poly->box.y2 = -P3D_HUGE; poly->box.z2 = -P3D_HUGE;
  p3d_mat4Copy(p3d_mat4IDENTITY,poly->pos0);
  p3d_mat4Copy(p3d_mat4IDENTITY,poly->pos_rel_jnt);
  return(poly);
}

/*******************************************/
/* Fonction permettant d'ajouter un sommet */
/* a un polyhedre donne                    */
/* in : le pointeur sur le polyhedre, les  */
/* coordonnees                             */
/* out :                                   */
/*******************************************/
void p3d_poly_add_vert(p3d_poly *poly, float x, float y, float z) {
  p3d_vector3 v;

  p3d_f_2_v3(x,y,z,&v);
  if (! p3d_add_point(v,poly->poly)) PrintInfo(("\nErreur dans p3d_add_poly_vert\n"));
if(poly->MODIF == FALSE) {PrintInfo(("error : you can t modify a polyhedron already defined...\n")); return;}

  /* Mise a jour de la boite visible */
  if(x>p3d_xmax) {p3d_xmax=x;}
  if(x>poly->box.x2) {poly->box.x2=x;}
  if(x<p3d_xmin) {p3d_xmin=x;}
  if(x<poly->box.x1) {poly->box.x1=x;}
  if(y>p3d_ymax) {p3d_ymax=y;}
  if(y>poly->box.y2) {poly->box.y2=y;}
  if(y<p3d_ymin) {p3d_ymin=y;}
  if(y<poly->box.y1) {poly->box.y1=y;}
  if(z>p3d_zmax) {p3d_zmax=z;}
  if(z>poly->box.z2) {poly->box.z2=z;}
  if(z<p3d_zmin) {p3d_zmin=z;}
  if(z<poly->box.z1) {poly->box.z1=z;}

}


/*******************************************/
/* Fonction permettant d'ajouter une face  */
/* a un polyhedre donne                    */
/* in : le pointeur sur le polyhedre, le   */
/* nom de la face, la liste de sommets     */
/* out :                                   */
/*******************************************/
void p3d_poly_add_face(p3d_poly *poly, int *listeV, int nb_Vert) {
  if(poly->MODIF == FALSE) {PrintInfo(("error : you can t modify a polyhedron already defined...\n")); return;}

  if (! p3d_build_face((p3d_index *)listeV,nb_Vert,poly->poly)) PrintInfo(("\nErreur dans p3d_add_poly_face\n"));;

}


/*******************************************/
/* Fonction terminant la description d'un  */
/* polyhedre donne et en bloquant la       */
/* modification                            */
/* in : le pointeur sur le polyhedre       */
/* out :                                   */
/*******************************************/
void p3d_poly_end_poly(p3d_poly *poly) { /* unsigned int nb_edges; */

  /* creation des arretes */
  /* nb_edges=p3d_get_nb_edges(poly->poly); */  /* Pour avoir un calcul exacte du nombre d arretes sans les calculer */

  p3d_polymax=p3d_polymax+1;
  poly->MODIF=FALSE ;
  poly->color_vect = NULL;
  poly->list = -1;
  poly->listfil = -1;
  poly->listgour = -1;

  poly->is_used_in_object_flag = 1;
}

/************************************************************/
/* Fonction determinant la couleur d'un polyhedre           */
/* In : le pointeur sur polyhedre,la couleur                */
/* Out :                                                    */
/************************************************************/
void p3d_poly_set_color
(p3d_poly *p, int color, double *color_vect) {

  /* si le polyedre a deja ete colore.... */
  if(p->color == Any) {
    MY_FREE(p->color_vect,double,4);
  }

  p->color = color;
  if(color==Any) {
    p->color_vect = MY_ALLOC(double,4);
    p->color_vect[0] = color_vect[0]; p->color_vect[1] = color_vect[1]; p->color_vect[2] = color_vect[2]; p->color_vect[3] = color_vect[3];
  } else {p->color_vect=NULL;}
}


/************************************************************/
/* Fonction determinant le type d'entite  d'un polyhedre    */
/* In : le pointeur sur polyhedre, son type                 */
/* Out :                                                    */
/************************************************************/
void p3d_poly_set_entity_type(p3d_poly *p,int entity_tjip) {
  p->entity_type = entity_tjip;
}

/************************************************************/
/* Fonction retourne    le type d'entite  d'un polyhedre    */
/* In : le pointeur sur polyhedre                           */
/* Out :                                                    */
/************************************************************/
int p3d_poly_get_entity_type(p3d_poly *p) {
  return p->entity_type;
}

/************************************************************/
/* Fonction recuperant les limites de la fenetre visible    */
/* In :                                                     */
/* Out : xmin,xmax,ymin,ymax,zmin,zmax                      */
/************************************************************/
void p3d_get_box(double *x1,double *x2,double *y1,double *y2,double *z1,double *z2) {
  *x1=p3d_xmin;
  *x2=p3d_xmax;
  *y1=p3d_ymin;
  *y2=p3d_ymax;
  *z1=p3d_zmin;
  *z2=p3d_zmax;
}

/************************************************************/
/* Fonction recuperant le nombre de polyhedres d'une        */
/* description                                              */
/* In :                                                     */
/* Out : le nombre de polyhedres                            */
/************************************************************/
int p3d_poly_get_nb(void) {
  return(p3d_polymax);
}


/*******************************************************/
/* Fonction recuperant le ieme polyhedres d'une        */
/* description                                         */
/* In : le numero                                      */
/* Out : le pointeur sur polyhedre                     */
/*******************************************************/
p3d_poly *p3d_poly_get_poly(int i) {
  p3d_poly *p;

  if(i>p3d_polymax) {PrintInfo(("ERREUR dans p3d_get_poly: il n'y a pas autant de polyhedres !\n"));}
  p=p3d_poly_get_first();
  i--;
  while (p->id!=i) {
    p=p3d_poly_get_next();
  }
  return p;
}



/*******************************************************/
/* Fonction recuperant un polyhedres d'une description */
/* par son nom                                         */
/* In : l  'id du polyhedre                            */
/* Out : le pointeur du polyhedre                      */
/*******************************************************/
p3d_poly *p3d_poly_get_poly_by_name(char *name) {
  p3d_poly *p;
  char *name_poly;
  int i;

  /* begin Modif. Carl: start at end rather than at beginning */
  /*   if (XYZ_OBSTACLES!=NULL) */
  /*     { i=0; */
  /*       while(i<XYZ_OBSTACLES->np)     */
  /*         { p=XYZ_OBSTACLES->pol[i]; */
  /*           name_poly=p3d_get_name(p->poly);  */
  /*           if(strcmp(name,name_poly)==0) return p; */
  /*           i++; */
  /*         } */
  /*     }  */
  if (XYZ_OBSTACLES!=NULL) {
    i=XYZ_OBSTACLES->np - 1;
    while(i >= 0) {
      p=XYZ_OBSTACLES->pol[i];
      name_poly=p3d_get_name(p->poly);
      if(strcmp(name,name_poly)==0) return p;
      i--;
    }
  }
  /* end Modif. Carl: start at end rather than at beginning */

  i=FALSE;
  p=p3d_poly_get_first();
  while((p!=NULL) && (i==FALSE)) {
    if (p!=NULL) {
      if(strcmp(name,p3d_get_name(p->poly))==0)
        i=TRUE;
      else
        p=p3d_poly_get_next();
    }
  }

  if (p==NULL) PrintInfo(("\nErreur name %s non valide dans p3d_get_poly_by_name\n",name));
  return p;
}


/*************************************************************/
/* Fonction initialisant la fonction de parcours des         */
/* polyhedres                                                */
/* In :                                                      */
/* Out :                                                     */
/*************************************************************/
p3d_poly *p3d_poly_get_first() {
  IN_OBSTACLE=TRUE;
  LASTr=0;
  LASTo=0;
  LASTp=-1;
  return p3d_poly_get_next();
}

/*************************************************************/
/* Fonction donnant le polyhedre suivant                     */
/* polyhedres                                                */
/* In :                                                      */
/* Out : la valeur du pointeur ou NULL si on est a la fin    */
/*************************************************************/
p3d_poly *p3d_poly_get_next() {
  p3d_obj *obj;
  p3d_rob *rob;
  p3d_poly *p;

  if (XYZ_ENV == NULL) return NULL; //si l'environnement n'existe pas

  if (IN_OBSTACLE) { //Si on est dans la description d'un obstacle
    if (XYZ_ENV->no==0) {// Si il n'y a pas d'obstacles
      LASTp=-1; //initialisation du dernier poly utilise
      LASTo=0; //initialisation du dernier objet utilise
      IN_OBSTACLE=FALSE; //on sort de la descritption de l'obstacle
      LASTr=0; //initialisation du dernier robot utilise
      return p3d_poly_get_next(); //reiteration
    }
    obj=XYZ_ENV->o[LASTo]; //selection de LASTo objet de la liste d'obstacles de l'environement
    if (LASTp<obj->np-1) {//si on a pas encore atteind le dernier poly de l'objet
      LASTp++;
      return obj->pol[LASTp]; //renvois le polyhedre
    } else { //si c'est le dernier poly de l'objet
      LASTp=-1; //reinitialiser le dernier poly
      LASTo++; //voir le prochain objet
      if (LASTo==XYZ_ENV->no) { //si c'est le dernier obstacle
        LASTo=0; //reinitialiser le dernier obstacle parcouru
        IN_OBSTACLE=FALSE; //on sort de la descritpion des obstacles
        LASTr=0; //on reinitialise le robot qu'on parcours
        return p3d_poly_get_next(); //reiteration
      } else
        return p3d_poly_get_next(); //on reitere en passant a l'obstacle suivant
    }
  } else {//si ce n'est pas un obstacle
    if (LASTr<XYZ_ENV->nr) { //si on a pas parcouru tous les robots
      rob=XYZ_ENV->robot[LASTr];//selection du robot num LASTr
      obj=rob->o[LASTo]; //on prend l'obstacle num LASTo
      if (LASTp<obj->np-1) {//si ce n'est pas le dernier polyhedre
        LASTp++;//aller au poly suivant
        return obj->pol[LASTp];// renvois le polyhedre
      } else {//si c'est le dernier poly
        LASTp=-1;//reinitialiser le nombre de poly
        do{
          LASTo++;//passer au prochain obstacle
        }while((LASTo < rob->no) && ((rob->o[LASTo])->concat == 1));
        if (LASTo==rob->no) {//si c'est le dernier obstacle du robot
          LASTo=0; //reinitialiser le nombre d'objets
          LASTr++; //incrementer le nombre de robot
          return p3d_poly_get_next(); //reinterer
        } else return p3d_poly_get_next();//si ce n'est pas le dernier obstacle reiterer
      }
    } else
      return (p=(p3d_poly *)(NULL)); //il n'y a plus d'obstacle dans l'environnement
  }
}

/*******************************************************************************************
 detruit un primitive
*******************************************************************************************/

void p3d_poly_destroy_primitive(p3d_primitive *primaat) {
  if(primaat)
    MY_FREE(primaat, p3d_primitive, 1);
}

/*************************************************************/
/* Fonction detruisant tout le polyhedre                     */
/* In :                                                      */
/* Out : TRUE  si on est a la fin                            */
/*************************************************************/
int p3d_poly_del_poly(p3d_poly *p) { /*p3d_col_del_poly(p);*/
  if(p) {
    if(p->color==Any) {
      MY_FREE(p->color_vect,double,4);
    }
    p3d_poly_destroy_primitive(p->primitive_data);
    if(p->poly){
      p3d_destroy_poly(p->poly);
      MY_FREE(p,p3d_poly,1);
      p->poly = NULL;
      p3d_polymax--;
    }
    /*
    if (p3d_polymax==0) 
    { p3d_col_start(p3d_col_mode_none);
    p3d_BB_set_mode_close();
    }
    */
    return(TRUE);
  }
  return(FALSE);
}
