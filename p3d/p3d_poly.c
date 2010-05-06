/*********************************************/
/* Ce fichier permet de creer des polyhedres */
/*********************************************/

#include "Util-pkg.h"
#include "P3d-pkg.h"

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

//! Function to change the curvature (left to 0 by default) of a vertex.
//! \param poly pointer to the polyhedron
//! \param index index of the vertex (startiong from 1) in the point array of the p3d_poly->poly
//! \param curvature curvature value to set
//! \return TRUE in case of success, FALSE otherwise
int p3d_poly_vert_curv(p3d_poly *poly, int index, double curvature) {
  if ( (index < 1) || (index > (int) poly->poly-> nb_points) )
  {
     PrintInfo(("\nError, wrong vertex index in p3d_poly_vert_curv.\n"));
     return FALSE;
  }

  poly->poly->curvatures[index-1]= curvature;

  return TRUE;
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


//! Computes a set of sample points on a triangle surface.
//! The idea is to sample a minimal area rectangle that bounds the triangle, with a regular grid, and to only keep
//! the points that are inside the triangle.
//! \param p1 first point of the triangle
//! \param p2 second point of the triangle
//! \param p3 third point of the triangle
//! \param step the discretization step of the sampling (if it is bigger than the triangle dimensions, there will be only one sample generated, positioned at the triangle center)
//! \param nb_samples pointer to an integer that will be filled with the number of computed samples
//! \return a 3D point array of size "nb_samples"
p3d_vector3 *sample_triangle_surface(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, double step, unsigned int *nb_samples)
{
  unsigned int i, j, k, n1, n2, cnt, nb_allocs;
  double l1, l2, l3, dotp, du, dv;
  p3d_vector3 p1p2, p2p3, p1p3, p2p1, pipf;
  p3d_vector3 pi, pf, p1s, p2s, crossp1, crossp2, crossp3, crossp4, u, v, s;
  p3d_vector3 *samples= NULL;

  if( step <= 0 )
  {
    printf("%s: %d: sample_triangle_surface(): the \"step\" argument must be > 0.\n", __FILE__,__LINE__);
    return NULL;
  }

  p3d_vectSub(p2, p1, p1p2);
  l1= p3d_vectNorm(p1p2);

  p3d_vectSub(p3, p2, p2p3);
  l2= p3d_vectNorm(p2p3);

  p3d_vectSub(p3, p1, p1p3);
  l3= p3d_vectNorm(p1p3);

  p3d_vectSub(p1, p2, p2p1);


  if( l1 < EPSILON || isnan(l1) || l2 < EPSILON || isnan(l2) || l3 < EPSILON || isnan(l3))
  {
    printf("%s: %d: sample_triangle_surface(): the input triangle has a null length edge.\n", __FILE__,__LINE__);
    return NULL;
  }

  *nb_samples= 0;

  p3d_vectScale(p1p2, u, 1.0/l1);

  dotp= p3d_vectDotProd(p1p3, u);
  for(i=0; i<3; i++)
  {
    v[i]= p3[i] - ( p1[i] + dotp*u[i] );
  }
  l2= p3d_vectNorm(v);
  p3d_vectScale(v, v, 1.0/l2);


  if( dotp > 0)
  {
    if(dotp > l1)
    {
      for(i=0; i<3; i++)
      {
        pi[i]= p1[i];
        pf[i]= p1[i] + dotp*u[i];
      }
    }
    else
    {
      for(i=0; i<3; i++)
      {
        pi[i]= p1[i];
        pf[i]= p2[i];
      }
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      pi[i]= p1[i] + dotp*u[i];
      pf[i]= p2[i];
    }
  }

  p3d_vectSub(pi, pf, pipf);
  l1= p3d_vectNorm(pipf);

  n1= (unsigned int) (l1/step) + 1;
  n2= (unsigned int) (l2/step) + 1;

  du= l1/n1;
  dv= l2/n2;

  if( n1==1 && n2==1 )
  {
    samples= (p3d_vector3 *) malloc(sizeof(p3d_vector3));
    *nb_samples= 1;
    for(i=0; i<3; i++)
    {  samples[0][i]= ( p1[i] + p2[i] + p3[i] ) / 3.0;  }
    return samples;
  }

  nb_allocs= n1*n2;
  samples= (p3d_vector3 *) malloc(nb_allocs*sizeof(p3d_vector3));


  p3d_vectXprod(p1p2, p1p3, crossp1);
  p3d_vectXprod(p2p1, p2p3, crossp2);


  cnt= 0;

  for(i=0; i<n1; i++)
  {
   for(j=0; j<n2; j++)
   {
     for(k=0; k<3; k++)
     {  s[k]= pi[k] + (i+0.5)*du*u[k] + (j+0.5)*dv*v[k];  }


     p3d_vectSub(s, p1, p1s);
     p3d_vectSub(s, p2, p2s);
     p3d_vectXprod(p1s, p1p3, crossp3);
     p3d_vectXprod(p2s, p2p3, crossp4);

     if( (p3d_vectDotProd(crossp1, crossp3) > 0) && (p3d_vectDotProd(crossp2, crossp4) > 0) )
     {
       for(k=0; k<3; k++)
       {  samples[cnt][k]= s[k];  }
       cnt++;
     }
   }
  }

  *nb_samples= cnt;
  return samples;
}


//! Creates a macro file from all the bodies of the given robot in the given configuration.
//! The file will be called [name of the robot]Body.macro.
//! \param robot pointer to the robot
//! \param q configuration in which the robot will be exported (if NULL, uses the current config)
//! \return 0 in case of success, 1 otherwise
int p3d_export_robot_as_one_body(p3d_rob *robot, configPt q)
{
  if(robot==NULL)
  {
    printf("%s: %d: p3d_export_robot_as_one_body(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

  unsigned int k, v;
  int i, j, shift;
  p3d_obj *obj= NULL;
  p3d_vector3 p;
  p3d_matrix4 T;
  configPt q0;
  FILE *file= NULL;
  char filename[256];
  p3d_polyhedre *poly= NULL;

  q0= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q0);

  if(q!=NULL)
  {
    p3d_set_and_update_this_robot_conf(robot, q);
  }

  if(getenv("HOME_MOVE3D")==NULL)
  {
    printf("%s: %d: p3d_export_robot_as_one_OBJ(): the environment variable \"HOME_MOVE3D\" is not defined .\n",__FILE__,__LINE__);
	strcpy(filename, ".");
  }
  else
  {
	strcpy(filename, getenv("HOME_MOVE3D"));
  }
  strcat(filename, "/");
  strcat(filename, robot->name);
  strcat(filename, "Body.macro");

  file= fopen(filename, "w");
  
  if(file==NULL)
  { 
    printf("%s: %d: p3d_export_robot_as_one_body(): can not open file \"%s\".\n", __FILE__, __LINE__,filename);
    return 1; 
  }

  fprintf(file, "p3d_beg_desc P3D_OBSTACLE\n");
  fprintf(file, "\tp3d_add_desc_poly polyhedre1\n");

  //first export the vertices:
  for(i=0; i<robot->no; i++)
  {
    obj= robot->o[i];

    for(j=0; j<obj->np; j++)
    {
//       if(obj->is_used_in_device_flag)
//       {    p3d_matMultXform(obj->jnt->abs_pos, obj->pol[i]->pos_rel_jnt, T);     }
//       else
//       {    p3d_mat4Copy(obj->pol[i]->pos0, T);    }
      if(obj->jnt==NULL)
      {  continue;  }


      p3d_matMultXform(obj->jnt->abs_pos, obj->pol[j]->pos_rel_jnt, T);
      poly=  obj->pol[j]->poly;

      for(k=0; k<poly->nb_points; k++)
      {
        p3d_xformPoint(T, poly->the_points[k], p);
        fprintf(file, "\t\tp3d_add_desc_vert %f %f %f \n", p[0], p[1], p[2]);
      }
    }
  }

  //now the faces:
  shift= 0;
  for(i=0; i<robot->no; i++)
  {
    obj= robot->o[i];

    for(j=0; j<obj->np; j++)
    {
      if(obj->jnt==NULL)
      {  continue;  }

      poly=  obj->pol[j]->poly;


      for(k=0; k<poly->nb_faces; k++)
      {
        fprintf(file, "\t\tp3d_add_desc_face ");
        for(v=0; v<poly->the_faces[k].nb_points; v++)
        {
          fprintf(file, "%d ", poly->the_faces[k].the_indexs_points[v]+shift);
        }
        fprintf(file, "\n");
      }
      shift+= poly->nb_points;
    }
  }

  fprintf(file, "\tp3d_end_desc_poly\n");
  fprintf(file, "p3d_end_desc\n");
  fclose(file);

  p3d_set_and_update_this_robot_conf(robot, q0);
  p3d_destroy_config(robot, q0);


  return 0;
}


//! Exports the bodies of a robot in a single .obj file but with separate groups
//! for each body. The bodies are exported with poses corresponding to a specified
//! configuration of the robot.
//! \param robot pointer to the robot
//! \param q configuration of the robot  (if NULL, uses the current config)
//! \return 0 in case of success, 1 otherwise
int p3d_export_robot_as_multipart_OBJ(p3d_rob *robot, configPt q)
{
  if(robot==NULL)
  {
    printf("%s: %d: p3d_export_robot_as_multipart_OBJ(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

  unsigned int k, v;
  int i, j, shift;
  p3d_obj *obj= NULL;
  p3d_vector3 p;
  p3d_matrix4 T, T1;
  configPt q0;
  FILE *file= NULL;
  char filename[256];
  p3d_polyhedre *poly= NULL;

  q0= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q0);

  if(q!=NULL)
  {
    p3d_set_and_update_this_robot_conf(robot, q);
  }

  if(getenv("HOME_MOVE3D")==NULL)
  {
    printf("%s: %d: p3d_export_robot_as_multipart_OBJ(): the environment variable \"HOME_MOVE3D\" is not defined .\n",__FILE__,__LINE__);
	strcpy(filename, ".");
  }
  else
  {
	strcpy(filename, getenv("HOME_MOVE3D"));
  }
  strcat(filename, "/");
  strcat(filename, robot->name);
  strcat(filename, ".obj");

  file= fopen(filename, "w");
  
  if(file==NULL)
  { 
    printf("%s: %d: p3d_export_robot_as_multipart_OBJ(): can not open file \"%s\".\n", __FILE__, __LINE__,filename);
    return 1; 
  }

  fprintf(file, "# exported by BioMove3D\n");
  
  shift= 0;
  for(i=0; i<robot->no; i++)
  {
    obj= robot->o[i];

    fprintf(file, "\t\tg %s \n", obj->name);

    //first export the vertices:
    for(j=0; j<obj->np; j++)
    {
      if(obj->jnt==NULL || obj->pol[j]->p3d_objPt!=obj)
      {  continue;  }

      poly=  obj->pol[j]->poly;

      p3d_matInvertXform(obj->pol[j]->pos0, T1);

      p3d_matMultXform(obj->jnt->abs_pos, obj->pol[j]->pos_rel_jnt, T);
      
      for(k=0; k<poly->nb_points; k++)
      {
        p3d_xformPoint(T, poly->the_points[k], p);
        fprintf(file, "\t\tv %f %f %f \n", p[0], p[1], p[2]);
      }
    }
    // now the faces:
    for(j=0; j<obj->np; j++)
    {
      if(obj->jnt==NULL || obj->pol[j]->p3d_objPt!=obj)
      {  continue;  }
      poly=  obj->pol[j]->poly;

      for(k=0; k<poly->nb_faces; k++)
      {
        fprintf(file, "\t\tf ");
        for(v=0; v<poly->the_faces[k].nb_points; v++)
        {
          fprintf(file, "%d ", poly->the_faces[k].the_indexs_points[v]+shift);
        }
        fprintf(file, "\n");
      }
      shift+= poly->nb_points;
    }

  }

  fclose(file);

  p3d_set_and_update_this_robot_conf(robot, q0);
  p3d_destroy_config(robot, q0);

  return 0;
}


//! Samples the surface of a robot and exports the resulting point cloud.
//! The points are exported with the body poses corresponding to a specified
//! configuration of the robot.
//! The user can choose to export only the bodies that have a specific name.
//! \param robot pointer to the robot
//! \param step discretization step of the triangle surfaces
//! \param prefix only the bodies whose names have the same beginning will be sampled (if NULL, all the bodies will be exported)
//! \param q configuration of the robot  (if NULL, uses the current config)
//! \return 0 in case of success, 1 otherwise
int p3d_export_robot_as_point_cloud(p3d_rob *robot, double step, char *prefix, configPt q)
{
  if(robot==NULL)
  {
    printf("%s: %d: p3d_export_robot_as_point_cloud(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

  int i, j, i1, i2, i3;
  unsigned int k, v;
  p3d_obj *obj= NULL;
  size_t length;
  p3d_matrix4 T;
  configPt q0;
  FILE *file= NULL;
  char filename[256];
  unsigned int nbSamples;
  p3d_vector3 p;
  p3d_vector3 *samples= NULL;
  p3d_polyhedre *poly= NULL;
  

  q0= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q0);

  if(q!=NULL)
  {
    p3d_set_and_update_this_robot_conf(robot, q);
  }

  if(getenv("HOME_MOVE3D")==NULL)
  {
    printf("%s: %d: p3d_export_robot_as_point_cloud(): the environment variable \"HOME_MOVE3D\" is not defined .\n",__FILE__,__LINE__);
	strcpy(filename, ".");
  }
  else
  {
	strcpy(filename, getenv("HOME_MOVE3D"));
  }
  strcat(filename, "/");
  strcat(filename, robot->name);
  strcat(filename, ".pc");

  file= fopen(filename, "a+");
  
  if(file==NULL)
  { 
    printf("%s: %d: p3d_export_robot_as_point_cloud(): can not open file \"%s\".\n", __FILE__, __LINE__,filename);
    return 1; 
  }

  if(prefix==NULL)
  { length= 0; }
  else
  { length= strlen(prefix); }
	  
  for(i=0; i<robot->no; i++)
  {
    obj= robot->o[i];

    if(length!=0)    {
      if(strncmp(obj->name, prefix, length)!=0) {
    	 continue;
      }	
    }

    for(j=0; j<obj->np; j++)
    {
      if(obj->jnt==NULL || obj->pol[j]->p3d_objPt!=obj)
      {  continue;  }
      poly=  obj->pol[j]->poly;

      p3d_matMultXform(obj->jnt->abs_pos, obj->pol[j]->pos_rel_jnt, T);
      
      for(k=0; k<poly->nb_faces; k++)
      {
    	i1= poly->the_faces[k].the_indexs_points[0] - 1;
    	i2= poly->the_faces[k].the_indexs_points[1] - 1;
    	i3= poly->the_faces[k].the_indexs_points[2] - 1;
    	samples= sample_triangle_surface(poly->the_points[i1],poly->the_points[i2],poly->the_points[i3], step, &nbSamples);

    	
        for(v=0; v<nbSamples; v++)
        {
          p3d_xformPoint(T, samples[v], p);
          fprintf(file, "v %f %f %f\n", p[0], p[1], p[2]);
        }
        if(samples!=NULL)
        { free(samples);  }
      }
    }

  }


  fclose(file);

  p3d_set_and_update_this_robot_conf(robot, q0);
  p3d_destroy_config(robot, q0);

  return 0;
}



