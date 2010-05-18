/*********************************************/
/* Ce fichier permet de creer des primitives */
/*********************************************/

#include "Util-pkg.h"
#include "P3d-pkg.h"

#ifdef LINUX
#define P3D_NPT 15
#else
#define P3D_NPT 15
#endif

/*******************************************************************/
/*********************************************/
/* Fonction creant un cube primitive         */
/* In :   le cote du cube                    */
/* Out : le pointeur sur le p3d_primitive    */
/*********************************************/
p3d_primitive *p3d_create_cube_primitive(double a)
{
  p3d_primitive *ent;

  ent = MY_ALLOC(p3d_primitive,1);
  ent->x_length = a;
  ent->y_length = a;
  ent->z_length = a;
  return ent;
}

/*********************************************/
/* Fonction creant une boite primitive       */
/* In :   les cotes de la boite              */
/* Out : le pointeur sur le p3d_primitive    */
/*********************************************/
p3d_primitive *p3d_create_box_primitive(double x, double y, double z)
{
  p3d_primitive *ent;

  ent = MY_ALLOC(p3d_primitive,1);
  ent->x_length = x;
  ent->y_length = y;
  ent->z_length = z;
  return ent;
}

/*********************************************/
/* Fonction creant un sphere primitive       */
/* In :   le rad  du sphere                  */
/* Out : le pointeur sur le p3d_primitive    */
/*********************************************/
p3d_primitive *p3d_create_sphere_primitive(double r)
{
  p3d_primitive *ent;

  ent = MY_ALLOC(p3d_primitive,1);
  ent->radius = r;
  return ent;
}

/*********************************************/
/* Fonction creant un cylindre primitive     */
/* In :   le rad et hauteur du cylindre      */
/* Out : le pointeur sur le p3d_primitive    */
/*********************************************/
p3d_primitive *p3d_create_cylinder_primitive(double r, double h)
{
  p3d_primitive *ent;

  ent = MY_ALLOC(p3d_primitive,1);
  ent->radius = r;
  ent->height = h;
  return ent;
}
/*********************************************/
/* Fonction creant un cone primitive         */
/* In :   le rad et hauteur du cone          */
/* Out : le pointeur sur le p3d_primitive    */
/*********************************************/
p3d_primitive *p3d_create_cone_primitive(double d1, double d2, double h)
{
  p3d_primitive *ent;

  ent = MY_ALLOC(p3d_primitive,1);
  ent->radius = d1;       /* diameter on top */
  ent->other_radius = d2; /* diameter at bottom */ /* Carl 10 oct.2000 */
  ent->height = h;
  ent->sin_slope = ABS(d1-d2) / ( 2.0 * sqrt((d1-d2)*(d1-d2)/4.0 + h*h) );
  return ent;
}

/*********************************************/
/* Fonction creant un cube                   */
/* In : le nom du polyhedre, le cote du cube */
/* Out : le pointeur sur le p3d_poly         */
/*********************************************/
p3d_poly *p3d_create_cube(char name[20], double a, int type)
{double a2=a/2.;
 p3d_poly *poly ;
 int face[4];

 poly=p3d_poly_beg_poly(name,type);
  
   poly->entity_type = CUBE_ENTITY;
   poly->primitive_data = p3d_create_cube_primitive(a);

   p3d_poly_add_vert(poly,-a2,-a2, a2);
   p3d_poly_add_vert(poly,-a2, a2, a2);
   p3d_poly_add_vert(poly, a2, a2, a2); 
   p3d_poly_add_vert(poly, a2,-a2, a2); 

   p3d_poly_add_vert(poly,-a2,-a2,-a2);
   p3d_poly_add_vert(poly,-a2, a2,-a2);
   p3d_poly_add_vert(poly, a2, a2,-a2); 
   p3d_poly_add_vert(poly, a2,-a2,-a2); 

   face[0]=1;
   face[1]=4;
   face[2]=3;
   face[3]=2;
   p3d_poly_add_face(poly, face, 4);

   face[0]=5;
   face[1]=6;
   face[2]=7;
   face[3]=8;
   p3d_poly_add_face(poly, face, 4);

   
   face[0]=1;
   face[1]=5;
   face[2]=8;
   face[3]=4;
   p3d_poly_add_face(poly, face, 4);

   face[0]=4;
   face[1]=8;
   face[2]=7;
   face[3]=3;
   p3d_poly_add_face(poly, face, 4);

   face[0]=2;
   face[1]=3;
   face[2]=7;
   face[3]=6;
   p3d_poly_add_face(poly, face, 4);

   face[0]=1;
   face[1]=2;
   face[2]=6;
   face[3]=5;
   p3d_poly_add_face(poly, face, 4);

 p3d_poly_end_poly(poly);

 return(poly);
}

/**********************************************************/
/* Fonction creant un parallelepipede rectangle           */
/* In : le nom du polyhedre, les cotes du parallelepipede */
/* Out : le pointeur sur le p3d_poly                      */
/**********************************************************/
p3d_poly *p3d_create_box(char name[20], double a, double b, double c, int type)
{double a2=a/2.,b2=b/2.,c2=c/2.;
 p3d_poly *poly ;
 int face[4];

 poly=p3d_poly_beg_poly(name,type);

   poly->entity_type = BOX_ENTITY;
   poly->primitive_data = p3d_create_box_primitive(a,b,c);
  
   p3d_poly_add_vert(poly,-a2,-b2, c2);
   p3d_poly_add_vert(poly,-a2, b2, c2);
   p3d_poly_add_vert(poly, a2, b2, c2); 
   p3d_poly_add_vert(poly, a2,-b2, c2); 

   p3d_poly_add_vert(poly,-a2,-b2,-c2);
   p3d_poly_add_vert(poly,-a2, b2,-c2);
   p3d_poly_add_vert(poly, a2, b2,-c2); 
   p3d_poly_add_vert(poly, a2,-b2,-c2); 

   face[0]=1;
   face[1]=4;
   face[2]=3;
   face[3]=2;
   p3d_poly_add_face(poly, face, 4);

   face[0]=5;
   face[1]=6;
   face[2]=7;
   face[3]=8;
   p3d_poly_add_face(poly, face, 4);

   
   face[0]=1;
   face[1]=5;
   face[2]=8;
   face[3]=4;
   p3d_poly_add_face(poly, face, 4);

   face[0]=4;
   face[1]=8;
   face[2]=7;
   face[3]=3;
   p3d_poly_add_face(poly, face, 4);

   face[0]=2;
   face[1]=3;
   face[2]=7;
   face[3]=6;
   p3d_poly_add_face(poly, face, 4);

   face[0]=1;
   face[1]=2;
   face[2]=6;
   face[3]=5;
   p3d_poly_add_face(poly, face, 4);

 p3d_poly_end_poly(poly);

 return(poly);
}

/***********************************************************/
/* Fonction creant un swept rectangle with cut top, bottom */
/* In : le nom du polyhedre, les cotes du parallelepipede  */
/* Out : le pointeur sur le p3d_poly                       */
/***********************************************************/
p3d_poly *p3d_create_srect(char name[20], double a, double b, double h,
			   double ux,double uy, double uz,
			   double vx,double vy, double vz, int type)
{double a2=a/2.,b2=b/2.;
 p3d_poly *poly ;
 int face[4];

 poly=p3d_poly_beg_poly(name,type);
 poly->entity_type = CONVEX_POLYHEDRON; 

 p3d_poly_add_vert(poly,-a2,-b2, h+ux/uz*a2+uy/uz*b2);
 p3d_poly_add_vert(poly,-a2, b2, h+ux/uz*a2-uy/uz*b2);
 p3d_poly_add_vert(poly, a2, b2, h-ux/uz*a2-uy/uz*b2); 
 p3d_poly_add_vert(poly, a2,-b2, h-ux/uz*a2+uy/uz*b2); 
 
 p3d_poly_add_vert(poly,-a2,-b2, vx/vz*a2+vy/vz*b2);
 p3d_poly_add_vert(poly,-a2, b2, vx/vz*a2-vy/vz*b2);
 p3d_poly_add_vert(poly, a2, b2,-vx/vz*a2-vy/vz*b2); 
 p3d_poly_add_vert(poly, a2,-b2,-vx/vz*a2+vy/vz*b2); 
 
 face[0]=1;
 face[1]=4;
 face[2]=3;
 face[3]=2;
 p3d_poly_add_face(poly, face, 4);
 
 face[0]=5;
 face[1]=6;
 face[2]=7;
 face[3]=8;
 p3d_poly_add_face(poly, face, 4);

 face[0]=1;
 face[1]=5;
 face[2]=8;
 face[3]=4;
 p3d_poly_add_face(poly, face, 4);
 
 face[0]=4;
 face[1]=8;
 face[2]=7;
 face[3]=3;
 p3d_poly_add_face(poly, face, 4);
 
 face[0]=2;
 face[1]=3;
 face[2]=7;
 face[3]=6;
 p3d_poly_add_face(poly, face, 4);
 
 face[0]=1;
 face[1]=2;
 face[2]=6;
 face[3]=5;
 p3d_poly_add_face(poly, face, 4);
 
 p3d_poly_end_poly(poly);
 return(poly);
}

/**************************************************/
/* Fonction creant une pyramide                   */
/* In : le nom du polyhedre, les parametres       */
/* Out : le pointeur sur le p3d_poly              */
/**************************************************/
p3d_poly *p3d_create_pyramid(char name[20], 
	double xbottom, double ybottom, 
	double xtop, double ytop, 
        double height, double xoff, double yoff, int type)
{
  double xtop2 = xtop/2.0, xoff2 = xoff/2;
  double ytop2 = ytop/2.0, yoff2 = yoff/2;
  double xbottom2 = xbottom/2.0;
  double ybottom2 = ybottom/2.0;
  double h2 = height/2.0;
  p3d_poly *poly ;
  int face[4];

  poly=p3d_poly_beg_poly(name,type);
  poly->entity_type = CONVEX_POLYHEDRON; 
  /* top vertices */
  p3d_poly_add_vert(poly, -xoff2-xtop2 , yoff2-ytop2 ,h2);
  p3d_poly_add_vert(poly, -xoff2-xtop2 , yoff2+ytop2 ,h2);
  p3d_poly_add_vert(poly, -xoff2+xtop2 , yoff2+ytop2 ,h2);
  p3d_poly_add_vert(poly, -xoff2+xtop2 , yoff2-ytop2 ,h2);
  /* bottom vertices */
  p3d_poly_add_vert(poly, xoff2-xbottom2 ,-yoff2-ybottom2 ,-h2);
  p3d_poly_add_vert(poly, xoff2-xbottom2 ,-yoff2+ybottom2 ,-h2);
  p3d_poly_add_vert(poly, xoff2+xbottom2 ,-yoff2+ybottom2 ,-h2);
  p3d_poly_add_vert(poly, xoff2+xbottom2 ,-yoff2-ybottom2 ,-h2);

  /* top face */
  face[0]=1;
  face[1]=4;
  face[2]=3;
  face[3]=2;
  p3d_poly_add_face(poly, face, 4);

  /* bottom face */
  face[0]=5;
  face[1]=6;
  face[2]=7;
  face[3]=8;
  p3d_poly_add_face(poly, face, 4);
  
  /* 4 side faces */
  face[0]=1;
  face[1]=5;
  face[2]=8;
  face[3]=4;
  p3d_poly_add_face(poly, face, 4);
  
  face[0]=4;
  face[1]=8;
  face[2]=7;
  face[3]=3;
  p3d_poly_add_face(poly, face, 4);
  
  face[0]=2;
  face[1]=3;
  face[2]=7;
  face[3]=6;
  p3d_poly_add_face(poly, face, 4);
  
  face[0]=1;
  face[1]=2;
  face[2]=6;
  face[3]=5;
  p3d_poly_add_face(poly, face, 4);

  p3d_poly_end_poly(poly);

  return(poly);
}

/**************************************************/
/* Fonction creant un cylindre                    */
/* In : le nom du polyhedre, le rayon, la hauteur */
/* Out : le pointeur sur le p3d_poly              */
/**************************************************/
p3d_poly *p3d_create_cylindre(char name[20], double r, double l, int type)
{double dt=2*M_PI/P3D_NPT,l2=l/2.;
 p3d_poly *poly ;
 int i,face1[P3D_NPT],face2[P3D_NPT],face[4];

 poly=p3d_poly_beg_poly(name,type);

   poly->entity_type = CYLINDER_ENTITY;
   poly->primitive_data = p3d_create_cylinder_primitive(r,l);

 for(i=0;i<=P3D_NPT-1;i++){
   p3d_poly_add_vert(poly,r*cos(i*dt),r*sin(i*dt), l2);
   p3d_poly_add_vert(poly,r*cos(i*dt),r*sin(i*dt),-l2);
 }

 /* On cree les faces bouchant le "tube" */
 for(i=0;i<=P3D_NPT-1;i++){
   face1[i]=2*i+1;
   face2[i]=2*(P3D_NPT-i);
 }
 p3d_poly_add_face(poly, face1, P3D_NPT);
 p3d_poly_add_face(poly, face2, P3D_NPT);

 /* on cree les facettes du tubes sauf la derniere */
 /* (reliant les derniers points aux premiers) */
 for(i=0;i<=P3D_NPT-2;i++){
   face[0]=2*i+1;
   face[1]=2*i+2;
   face[2]=2*i+4;
   face[3]=2*i+3;
   p3d_poly_add_face(poly, face, 4);
 }

 /* on cree la derniere face */
 face[0]=2*P3D_NPT-1;
 face[1]=2*P3D_NPT;
 face[2]=2;
 face[3]=1;
 p3d_poly_add_face(poly, face, 4);
   
 p3d_poly_end_poly(poly);
 
 return(poly);
}

/**************************************************/
/* Fonction creant un cylindre oval               */
/* In : le nom du polyhedre, le rayon, la hauteur */
/* Out : le pointeur sur le p3d_poly              */
/**************************************************/
p3d_poly *p3d_create_cylindre_oval(char name[20], double a, double b, double l, int type)
{double dt=2*M_PI/P3D_NPT,l2=l/2.;
 p3d_poly *poly ;
 int i,face1[P3D_NPT],face2[P3D_NPT],face[4];

 poly=p3d_poly_beg_poly(name,type);
 poly->entity_type = CONVEX_POLYHEDRON; 

 for(i=0;i<=P3D_NPT-1;i++){
   p3d_poly_add_vert(poly,a*cos(i*dt),b*sin(i*dt), l2);
   p3d_poly_add_vert(poly,a*cos(i*dt),b*sin(i*dt),-l2);
 }

 /* On cree les faces bouchant le "tube" */
 for(i=0;i<=P3D_NPT-1;i++){
   face1[i]=2*i+1;
   face2[i]=2*(P3D_NPT-i);
 }
 p3d_poly_add_face(poly, face1, P3D_NPT);
 p3d_poly_add_face(poly, face2, P3D_NPT);

 /* on cree les facettes du tubes sauf la derniere */
 /* (reliant les derniers points aux premiers) */
 for(i=0;i<=P3D_NPT-2;i++){
   face[0]=2*i+1;
   face[1]=2*i+2;
   face[2]=2*i+4;
   face[3]=2*i+3;
   p3d_poly_add_face(poly, face, 4);
 }

 /* on cree la derniere face */
 face[0]=2*P3D_NPT-1;
 face[1]=2*P3D_NPT;
 face[2]=2;
 face[3]=1;
 p3d_poly_add_face(poly, face, 4);
   
 p3d_poly_end_poly(poly);
 
 return(poly);
}

/*******************************************************/
/* Fonction creant un prisme regulier a section droite */
/* In : le nom du polyhedre, le nombre de sommets,     */
/* la hauteur                                          */
/* Out : le pointeur sur le p3d_poly                   */
/*******************************************************/
p3d_poly *p3d_create_prisme(char name[20], int nvert, double r, double l, int type)
{double dt=2*M_PI/nvert,l2=l/2.;
 p3d_poly *poly ;
 int i,face1[P3D_NPT],face2[P3D_NPT],face[4];

 poly=p3d_poly_beg_poly(name,type);
 poly->entity_type = CONVEX_POLYHEDRON;

 for(i=0;i<=nvert-1;i++){
   p3d_poly_add_vert(poly,r*cos(i*dt),r*sin(i*dt), l2);
   p3d_poly_add_vert(poly,r*cos(i*dt),r*sin(i*dt),-l2);
 }

 /* On cree les faces bouchant le "tube" */
 for(i=0;i<=nvert-1;i++){
   face1[i]=2*i+1;
   face2[i]=2*(nvert-i);
 }
 p3d_poly_add_face(poly, face1, nvert);
 p3d_poly_add_face(poly, face2, nvert);

 /* on cree les facettes du tubes sauf la derniere */
 /* (reliant les derniers points aux premiers) */
 for(i=0;i<=nvert-2;i++){
   face[0]=2*i+1;
   face[1]=2*i+2;
   face[2]=2*i+4;
   face[3]=2*i+3;
   p3d_poly_add_face(poly, face, 4);
 }

 /* on cree la derniere face */
 face[0]=2*nvert-1;
 face[1]=2*nvert;
 face[2]=2;
 face[3]=1;
 p3d_poly_add_face(poly, face, 4);
   
 p3d_poly_end_poly(poly);
 
 return(poly);
}

/*******************************************************/
/* Fonction creant un "rectangular torus slice"        */
/* In : le nom du polyhedre,  radius in, radius out    */
/* la hauteur, angle of slice                          */
/* Out : le pointeur sur le p3d_poly                   */
/*******************************************************/
p3d_poly *p3d_create_rtorusslice(char name[20], double r1, double r2, 
				 double height, double angle, int type)
{
  double step, h2=height/2.;
  int nstep;
  p3d_poly *poly ;
  int i,face1[4],face2[4];

  poly=p3d_poly_beg_poly(name,type);
  poly->entity_type = CONCAVE_POLYHEDRON;
  nstep = 36; /* (int)(angle / 10.0) + 1; */
  step = angle / nstep;
  if(EQ(angle,2*M_PI))
    step = angle/(nstep+1);

 for(i=0;i<=nstep;i++){
   p3d_poly_add_vert(poly,r1*cos(i*step),r1*sin(i*step),-h2);
   p3d_poly_add_vert(poly,r2*cos(i*step),r2*sin(i*step),-h2);
   p3d_poly_add_vert(poly,r2*cos(i*step),r2*sin(i*step), h2);
   p3d_poly_add_vert(poly,r1*cos(i*step),r1*sin(i*step), h2);
 }

 /* end facets */
 if(!EQ(angle,2*M_PI))
   {
     face1[0]=1;
     face1[1]=2;
     face1[2]=3;
     face1[3]=4;
     face2[0]=(nstep+1)*4;
     face2[1]=(nstep+1)*4-1;
     face2[2]=(nstep+1)*4-2;
     face2[3]=(nstep+1)*4-3;
     
     p3d_poly_add_face(poly, face1, 4);
     p3d_poly_add_face(poly, face2, 4);
   }
 else
   {
     face1[0]=1;
     face1[1]=(nstep+1)*4-3;
     face1[2]=(nstep+1)*4;
     face1[3]=4;
     face2[0]=2;
     face2[1]=3;
     face2[2]=(nstep+1)*4-1;
     face2[3]=(nstep+1)*4-2;
 
     p3d_poly_add_face(poly, face1, 4);
     p3d_poly_add_face(poly, face2, 4);

     face1[0]=3;
     face1[1]=4;
     face1[2]=(nstep+1)*4;
     face1[3]=(nstep+1)*4-1;
     face2[0]=2;
     face2[1]=1;
     face2[2]=(nstep+1)*4-3;
     face2[3]=(nstep+1)*4-2;
 
     p3d_poly_add_face(poly, face1, 4);
     p3d_poly_add_face(poly, face2, 4);     
   }

 /* intermediate facets */
 for(i=0;i<nstep;i++)
   {
     face1[0]=1+4*i;
     face1[1]=4+4*i;
     face1[2]=4+4*(i+1);
     face1[3]=1+4*(i+1);
     face2[0]=3+4*i;
     face2[1]=2+4*i;
     face2[2]=2+4*(i+1);
     face2[3]=3+4*(i+1);
 
     p3d_poly_add_face(poly, face1, 4);
     p3d_poly_add_face(poly, face2, 4);

     face1[0]=2+4*i;
     face1[1]=1+4*i;
     face1[2]=1+4*(i+1);
     face1[3]=2+4*(i+1);
     face2[0]=4+4*i;
     face2[1]=3+4*i;
     face2[2]=3+4*(i+1);
     face2[3]=4+4*(i+1);
 
     p3d_poly_add_face(poly, face1, 4);
     p3d_poly_add_face(poly, face2, 4);


   }
 p3d_poly_end_poly(poly);
 
 return(poly);
}

/*******************************************************/
/* Fonction creant un "circular torus slice"           */
/* In : le nom du polyhedre,  radius of torus, radius  */
/*    of circle,         angle of slice                */
/* Out : le pointeur sur le p3d_poly                   */
/*******************************************************/
p3d_poly *p3d_create_ctorusslice(char name[20], double R, double r, 
				 double angle, int type)
{
  double step;
  int nstep;
  p3d_poly *poly ;
  int i,j,face1[12],face2[12], face[4];
  int nvertices=192, i12 = 0;

  poly=p3d_poly_beg_poly(name,type);
  poly->entity_type = CONCAVE_POLYHEDRON;

  nstep = 15;
  step = angle / nstep;
  if(EQ(angle,2*M_PI))
    step = angle/(nstep+1);

 for(i=0;i<=nstep;i++){
   for(j=0;j<12;j++) {
   p3d_poly_add_vert(poly,(R+r*cos(j*M_PI/6))*cos(i*step),
		     (R+r*cos(j*M_PI/6))*sin(i*step),r*sin(j*M_PI/6));
   }
 }

 /* end facets */
 if(!EQ(angle,2*M_PI))
   {
     for(i=0;i<12;i++)
       {
	 face1[i]=i+1;
	 face2[i]=(nstep+1)*12-i;
       }
     p3d_poly_add_face(poly, face1, 12);
     p3d_poly_add_face(poly, face2, 12);
   }
 else
   {
     for(i=0;i<10;i++)
       {
	 face[0] = nvertices - 11 + i ;
	 face[1] = 1 + i ;
	 face[2] = 2 + i ;
	 face[3] = nvertices - 10 + i ;
	 p3d_poly_add_face(poly, face, 4);
       }
     /* i == 10 */
     face[0] = nvertices - 1  ;
     face[1] = 11 ;
     face[2] = 12 ;
     face[3] = nvertices  ;
     p3d_poly_add_face(poly, face, 4);
     /* i == 11 */
     face[0] = nvertices ;
     face[1] = 12 ;
     face[2] = 1 ;
     face[3] = nvertices - 11 ;
     p3d_poly_add_face(poly, face, 4);
   }

 /* intermediate facets */
 for(i=0;i<nstep;i++)
   {
     i12 = i*12;
     for(j=0;j<11;j++)
       {
	 face[0]=  1 + j + i12 ;
	 face[1]= 13 + j + i12 ;
	 face[2]= 14 + j + i12 ;
	 face[3]=  2 + j + i12 ;
	 p3d_poly_add_face(poly, face, 4);
       }
     /* j == 11 */
     face[0]= 12 + i12 ;
     face[1]= 24 + i12 ;
     face[2]= 13 + i12 ;
     face[3]=  1 + i12 ;
     p3d_poly_add_face(poly, face, 4);
   }
 p3d_poly_end_poly(poly);
 
 return(poly);
}

/*******************************************************/
/* Fonction creant un "swept annulus" (review)         */
/* In : le nom du polyhedre,  radius of torus, radius  */
/*    of circle,         angle of slice                */
/*    normals of top and bottom facets                 */
/* Out : le pointeur sur le p3d_poly                   */
/*******************************************************/
p3d_poly *p3d_create_sweptrectslice(char name[20], double r1, double r2, 
				    double height, double angle, 
				    double xtop,double ytop, double ztop,
				    double xbot,double ybot, double zbot, int type)
{
  double step;
  int nstep;
  p3d_poly *poly ;
  int i,face1[4],face2[4];
  int nvertices = 64, i4 = 0;
  double Eb  = 0.0;
  double Et  = 0.0;

  poly=p3d_poly_beg_poly(name,type);
  poly->entity_type = CONVEX_POLYHEDRON; 

  nstep = 15; /* 16 vertices per bow, 4 bows */ /* could be 50 per bow */
  step = angle / nstep;
  if(EQ(angle,2*M_PI))
    step = angle / (nstep+1);

  for(i=0;i<=nstep;i++)
    {
      Eb = ((r2-r1)/2 * (xbot*cos(i*step) + ybot*sin(i*step)) )/zbot;
      Et = ((r1-r2)/2 * (xtop*cos(i*step) + ytop*sin(i*step)) )/ztop;

      p3d_poly_add_vert(poly, r1*cos(i*step), r1*sin(i*step), 0 + Eb);
      p3d_poly_add_vert(poly, r2*cos(i*step), r2*sin(i*step), 0 + Eb);
      p3d_poly_add_vert(poly, r2*cos(i*step), r2*sin(i*step), height + Et);
      p3d_poly_add_vert(poly, r1*cos(i*step), r1*sin(i*step), height + Et);
    }

 /* end facets */
 if(!EQ(angle,2*M_PI))
   {
     face1[0] = 1;
     face1[1] = 2;
     face1[2] = 3;
     face1[3] = 4;
     face2[0] = nvertices;
     face2[1] = nvertices-1;
     face2[2] = nvertices-2;
     face2[3] = nvertices-3;
     p3d_poly_add_face(poly, face1, 4);
     p3d_poly_add_face(poly, face2, 4);
   }
 else
   {
     /* connect to complete the circle */
     face1[0] = 4;
     face1[1] = 1;
     face1[2] = nvertices-3;
     face1[3] = nvertices;
     face2[0] = 3;
     face2[1] = 2;
     face2[2] = nvertices-2;
     face2[3] = nvertices-1;
     p3d_poly_add_face(poly, face1, 4);
     p3d_poly_add_face(poly, face2, 4);

     face1[0] = 4;
     face1[1] = 3;
     face1[2] = nvertices-1;
     face1[3] = nvertices;
     face2[0] = 2;
     face2[1] = 1;
     face2[2] = nvertices-3;
     face2[3] = nvertices-2;
     p3d_poly_add_face(poly, face1, 4);
     p3d_poly_add_face(poly, face2, 4);
   }

 /* intermediate facets */
 for(i=0;i<nstep;i++)
   {
     i4 = i*4;
     face1[0]= 1 + i4 ;
     face1[1]= 4 + i4 ;
     face1[2]= 8 + i4 ;
     face1[3]= 5 + i4 ;
     face2[0]= 6 + i4 ;
     face2[1]= 7 + i4 ;
     face2[2]= 3 + i4 ;
     face2[3]= 2 + i4 ;
     p3d_poly_add_face(poly, face1, 4);
     p3d_poly_add_face(poly, face2, 4);

     face1[0]= 1 + i4 ;
     face1[1]= 5 + i4 ;
     face1[2]= 6 + i4 ;
     face1[3]= 2 + i4 ;
     face2[0]= 3 + i4 ;
     face2[1]= 7 + i4 ;
     face2[2]= 8 + i4 ;
     face2[3]= 4 + i4 ;
     p3d_poly_add_face(poly, face1, 4);
     p3d_poly_add_face(poly, face2, 4);
   }
 p3d_poly_end_poly(poly);
 
 return(poly);
}

/*******************************************************/
/* Fonction creant une approximation d'un "cone"      */
/* In : le nom du polyhedre, le nombre de sommets,     */
/* la hauteur                                          */
/* Out : le pointeur sur le p3d_poly                   */
/*******************************************************/
p3d_poly *p3d_create_cone(char name[20], int nvert,double dtop,
			  double dbottom,  double height, int type)
{
  double r1 = dtop/2, r2 = dbottom/2, height2 = height/2;
  double dt=2*M_PI/nvert;
  p3d_poly *poly ;
  int i,face1[P3D_NPT],face2[P3D_NPT],face[4];

  poly=p3d_poly_beg_poly(name,type);
  poly->entity_type = CONE_ENTITY;
  poly->primitive_data = p3d_create_cone_primitive(dtop,dbottom,height);

  for(i=0;i<=nvert-1;i++){
    p3d_poly_add_vert(poly,r1*cos(i*dt),r1*sin(i*dt), height2);
    p3d_poly_add_vert(poly,r2*cos(i*dt),r2*sin(i*dt),-height2);
  }

  /* On cree les faces bouchant le "tube" */
  for(i=0;i<=nvert-1;i++){
    face1[i]=2*i+1;
    face2[i]=2*(nvert-i);
  }
  p3d_poly_add_face(poly, face1, nvert);
  p3d_poly_add_face(poly, face2, nvert);

  /* on cree les facettes du tubes sauf la derniere */
  /* (reliant les derniers points aux premiers) */
  for(i=0;i<=nvert-2;i++){
    face[0]=2*i+1;
    face[1]=2*i+2;
    face[2]=2*i+4;
    face[3]=2*i+3;
    p3d_poly_add_face(poly, face, 4);
  }

  /* on cree la derniere face */
  face[0]=2*nvert-1;
  face[1]=2*nvert;
  face[2]=2;
  face[3]=1;
  p3d_poly_add_face(poly, face, 4);
  
  p3d_poly_end_poly(poly);
  
  return(poly);
}

/*******************************************************/
/* Fonction creant une approximation d'un "snout"      */
/* In : le nom du polyhedre, le nombre de sommets,     */
/* la hauteur                                          */
/* Out : le pointeur sur le p3d_poly                   */
/*******************************************************/
p3d_poly *p3d_create_snout(char name[20], int nvert,double dtop,double dbottom,
			    double height, double xoff, double yoff, int type)
{
  double r1 = dtop/2, r2 = dbottom/2, height2 = height/2;
  double xoff2 = xoff/2, yoff2 = yoff/2;
  double dt=2*M_PI/nvert;
  p3d_poly *poly ;
  int i,face1[P3D_NPT],face2[P3D_NPT],face[4];

  poly=p3d_poly_beg_poly(name,type);
  poly->entity_type = CONVEX_POLYHEDRON; 

  for(i=0;i<=nvert-1;i++){
    p3d_poly_add_vert(poly,-xoff2+r1*cos(i*dt),yoff2+r1*sin(i*dt), height2);
    p3d_poly_add_vert(poly,xoff2+r2*cos(i*dt),-yoff2+r2*sin(i*dt),-height2);
  }

  /* On cree les faces bouchant le "tube" */
  for(i=0;i<=nvert-1;i++){
    face1[i]=2*i+1;
    face2[i]=2*(nvert-i);
  }
  p3d_poly_add_face(poly, face1, nvert);
  p3d_poly_add_face(poly, face2, nvert);

  /* on cree les facettes du tubes sauf la derniere */
  /* (reliant les derniers points aux premiers) */
  for(i=0;i<=nvert-2;i++){
    face[0]=2*i+1;
    face[1]=2*i+2;
    face[2]=2*i+4;
    face[3]=2*i+3;
    p3d_poly_add_face(poly, face, 4);
  }

  /* on cree la derniere face */
  face[0]=2*nvert-1;
  face[1]=2*nvert;
  face[2]=2;
  face[3]=1;
  p3d_poly_add_face(poly, face, 4);
  
  p3d_poly_end_poly(poly);
  
  return(poly);
}

/**************************************************************/
/* Fonction creant une approximation d'un review "snout"      */
/* In : le nom du polyhedre, le nombre de sommets,            */
/* la hauteur                                                 */
/* Out : le pointeur sur le p3d_poly                          */
/**************************************************************/
p3d_poly *p3d_create_skew_snout(char name[20], int nvert,double dtop,double dbottom,
			    double height, double xoff, double yoff, double ta1,
				double ta2, double ba1, double ba2, int type)
{
  double r1 = dtop/2, r2 = dbottom/2, height2 = height/2;
  double xoff2 = xoff/2, yoff2 = yoff/2;
  double dt=2*M_PI/nvert;
  double ux = -sin(ta1);
  double uy = -cos(ta1)*sin(ta2);
  double uz = cos(ta1)*cos(ta2);
  double vx = -sin(ba1);
  double vy = -cos(ba1)*sin(ba2);
  double vz = cos(ba1)*cos(ba2);
  double X,Y,Z,N;
  p3d_poly *poly ;
  int i,face1[P3D_NPT],face2[P3D_NPT],face[4];

  poly=p3d_poly_beg_poly(name,type);
  poly->entity_type = CONVEX_POLYHEDRON; 
  for(i=0;i<=nvert-1;i++){
    N = - 2*uy*yoff2 + uy*r2*sin(i*dt) - uy*r1*sin(i*dt) + 2*xoff2*ux
      + r2*cos(i*dt)*ux - r1*cos(i*dt)*ux - 2*uz*height2;
    X =  - (uy*r1*cos(i*dt)*yoff2 + uy*yoff2*r2*cos(i*dt) 
	    + uy*xoff2*r1*sin(i*dt) + uy*xoff2*r2*sin(i*dt) 
	    + 2*uz*height2*r1*cos(i*dt) - 2*uz*height2*xoff2)/N;
    Y = (- 2*uz*height2*yoff2 - 2*uz*height2*r1*sin(i*dt) 
	 + r1*cos(i*dt)*yoff2*ux + yoff2*r2*cos(i*dt)*ux + 
	 xoff2*r1*sin(i*dt)*ux + xoff2*r2*sin(i*dt)*ux)/N;
    Z = (uy*r1*sin(i*dt) + uy*r2*sin(i*dt) - 2*uz*height2 + r1*cos(i*dt)*ux
	 + r2*cos(i*dt)*ux)*height2/N;

    p3d_poly_add_vert(poly,X,Y,Z);

    N = - 2*vy*yoff2 + vy*r2*sin(i*dt) - vy*r1*sin(i*dt) + 2*xoff2*vx
      + r2*cos(i*dt)*vx - r1*cos(i*dt)*vx - 2*vz*height2;
    X = - (vy*r1*cos(i*dt)*yoff2 + vy*yoff2*r2*cos(i*dt) 
	   + vy*xoff2*r1*sin(i*dt) + vy*xoff2*r2*sin(i*dt) 
	   + 2*vz*height2*r2*cos(i*dt) + 2*vz*height2*xoff2)/N;
    Y = (2*vz*height2*yoff2 - 2*vz*height2*r2*sin(i*dt) 
	 + r1*cos(i*dt)*yoff2*vx + yoff2*r2*cos(i*dt)*vx 
	 + xoff2*r1*sin(i*dt)*vx + xoff2*r2*sin(i*dt)*vx)/N;
    Z = (vy*r1*sin(i*dt) + vy*r2*sin(i*dt) + 2*vz*height2 + r1*cos(i*dt)*vx
	 + r2*cos(i*dt)*vx)*height2/N;
 
    p3d_poly_add_vert(poly,X,Y,Z);
  }

  /* On cree les faces bouchant le "tube" */
  for(i=0;i<=nvert-1;i++){
    face1[i]=2*i+1;
    face2[i]=2*(nvert-i);
  }
  p3d_poly_add_face(poly, face1, nvert);
  p3d_poly_add_face(poly, face2, nvert);

  /* on cree les facettes du tubes sauf la derniere */
  /* (reliant les derniers points aux premiers) */
  for(i=0;i<=nvert-2;i++){
    face[0]=2*i+1;
    face[1]=2*i+2;
    face[2]=2*i+4;
    face[3]=2*i+3;
    p3d_poly_add_face(poly, face, 4);
  }

  /* on cree la derniere face */
  face[0]=2*nvert-1;
  face[1]=2*nvert;
  face[2]=2;
  face[3]=1;
  p3d_poly_add_face(poly, face, 4);
  
  p3d_poly_end_poly(poly);
  
  return(poly);
}

/**************************************************/
/* Fonction creant une sphere                     */
/* In : le nom du polyhedre, le rayon, la hauteur */
/* Out : le pointeur sur le p3d_poly              */
/**************************************************/
p3d_poly *p3d_create_sphere(char name[20], double r, int type)
{double dt1=M_PI/(P3D_NPT-1),dt2=2*M_PI/P3D_NPT;
 p3d_poly *poly ;
 int i,j,face[4],compt=0;

 poly=p3d_poly_beg_poly(name,type);

 poly->entity_type = SPHERE_ENTITY;
 poly->primitive_data = p3d_create_sphere_primitive(r);

 p3d_poly_add_vert(poly,0.,0.,r);

 for(i=1;i<=P3D_NPT-2;i++){
   for(j=0;j<=P3D_NPT-1;j++){
     p3d_poly_add_vert(poly,r*cos(M_PI/2.-i*dt1)*cos(j*dt2),r*cos(M_PI/2.-i*dt1)*sin(j*dt2),r*sin(M_PI/2.-i*dt1));
     compt=compt+1;
   }
 }
 p3d_poly_add_vert(poly,0.,0.,-r);

 /* on cree les facettes triangulaires de la premiere couche */
 for(i=2;i<=P3D_NPT;i++){
   face[0]=1;
   face[1]=i;
   face[2]=i+1;
   p3d_poly_add_face(poly, face, 3);
 }
 /* la derniere face de la premiere couche */
 face[0]=1;
 face[1]=P3D_NPT+1;
 face[2]=2;
 p3d_poly_add_face(poly, face, 3);

 /* on cree les facettes rectangulaires des autres couches */
 for(i=0;i<=P3D_NPT-4;i++){
   for(j=i*P3D_NPT+2;j<=(i+1)*P3D_NPT;j++){
     face[0]=j;
     face[1]=j+P3D_NPT;
     face[2]=j+P3D_NPT+1;
     face[3]=j+1;
     p3d_poly_add_face(poly, face, 4);
   }
   face[0]=(i+1)*P3D_NPT+1;
   face[1]=(i+2)*P3D_NPT+1;;
   face[2]=(i+1)*P3D_NPT+2;
   face[3]=i*P3D_NPT+2;
   p3d_poly_add_face(poly, face, 4);
   
 }

 /* on cree les facettes triangulaires de la derniere couche */
 for(i=(P3D_NPT-3)*P3D_NPT+2;i<=(P3D_NPT-2)*P3D_NPT;i++){
   face[0]=i;
   face[1]=(P3D_NPT-2)*P3D_NPT+2;
   face[2]=i+1;
   p3d_poly_add_face(poly, face, 3);
 }
 face[0]=(P3D_NPT-2)*P3D_NPT+1;
 face[1]=(P3D_NPT-2)*P3D_NPT+2;
 face[2]=(P3D_NPT-3)*P3D_NPT+2;
 p3d_poly_add_face(poly, face, 3);
   
 p3d_poly_end_poly(poly);
 
 return(poly);
}

/**************************************************/
/* Fonction creant un ellipsoide                 */
/* In : le nom du polyhedre, les dimensions       */
/* Out : le pointeur sur le p3d_poly              */
/**************************************************/
p3d_poly *p3d_create_oval(char name[20], double a, double b, double c, int type)
{double dt1=M_PI/(P3D_NPT-1),dt2=2*M_PI/P3D_NPT;
 p3d_poly *poly ;
 int i,j,face[4],compt=0;

 poly=p3d_poly_beg_poly(name,type);
 poly->entity_type = CONVEX_POLYHEDRON; 

 p3d_poly_add_vert(poly,0.,0.,c);

 for(i=1;i<=P3D_NPT-2;i++){
   for(j=0;j<=P3D_NPT-1;j++){
     p3d_poly_add_vert(poly,a*cos(M_PI/2.-i*dt1)*cos(j*dt2),b*cos(M_PI/2.-i*dt1)*sin(j*dt2),c*sin(M_PI/2.-i*dt1));
     compt=compt+1;
   }
 }
 p3d_poly_add_vert(poly,0.,0.,-c);

 /* on cree les facettes triangulaires de la premiere couche */
 for(i=2;i<=P3D_NPT;i++){
   face[0]=1;
   face[1]=i;
   face[2]=i+1;
   p3d_poly_add_face(poly, face, 3);
 }
 /* la derniere face de la premiere couche */
 face[0]=1;
 face[1]=P3D_NPT+1;
 face[2]=2;
 p3d_poly_add_face(poly, face, 3);

 /* on cree les facettes rectangulaires des autres couches */
 for(i=0;i<=P3D_NPT-4;i++){
   for(j=i*P3D_NPT+2;j<=(i+1)*P3D_NPT;j++){
     face[0]=j;
     face[1]=j+P3D_NPT;
     face[2]=j+P3D_NPT+1;
     face[3]=j+1;
     p3d_poly_add_face(poly, face, 4);
   }
   face[0]=(i+1)*P3D_NPT+1;
   face[1]=(i+2)*P3D_NPT+1;;
   face[2]=(i+1)*P3D_NPT+2;
   face[3]=i*P3D_NPT+2;
   p3d_poly_add_face(poly, face, 4);
   
 }

 /* on cree les facettes triangulaires de la derniere couche */
 for(i=(P3D_NPT-3)*P3D_NPT+2;i<=(P3D_NPT-2)*P3D_NPT;i++){
   face[0]=i;
   face[1]=(P3D_NPT-2)*P3D_NPT+2;
   face[2]=i+1;
   p3d_poly_add_face(poly, face, 3);
 }
 face[0]=(P3D_NPT-2)*P3D_NPT+1;
 face[1]=(P3D_NPT-2)*P3D_NPT+2;
 face[2]=(P3D_NPT-3)*P3D_NPT+2;
 p3d_poly_add_face(poly, face, 3);
   
 p3d_poly_end_poly(poly);
 
 return(poly);
}

/**************************************************/
/* Fonction creant une demi sphere                */
/* In : le nom du polyhedre, le rayon, la hauteur */
/* Out : le pointeur sur le p3d_poly              */
/**************************************************/
p3d_poly *p3d_create_half_sphere(char name[20], double r, int type)
{double dt1=M_PI/(P3D_NPT-1),dt2=2*M_PI/P3D_NPT;
 p3d_poly *poly ;
 int i,j,face[4],botface[P3D_NPT],compt=0;

 poly=p3d_poly_beg_poly(name,type);
 poly->entity_type = CONVEX_POLYHEDRON; 

 p3d_poly_add_vert(poly,0.,0.,r);

 for(i=1;i<=(P3D_NPT-1)/2;i++){
   for(j=0;j<=P3D_NPT-1;j++){
     p3d_poly_add_vert(poly,r*cos(M_PI/2.-i*dt1)*cos(j*dt2),
		            r*cos(M_PI/2.-i*dt1)*sin(j*dt2),
		            r*sin(M_PI/2.-i*dt1));
     compt=compt+1;
   }
 }

 /* on cree les facettes triangulaires de la premiere couche */
 for(i=2;i<=P3D_NPT;i++){
   face[0]=1;
   face[1]=i;
   face[2]=i+1;
   p3d_poly_add_face(poly, face, 3);
 }
 /* la derniere face de la premiere couche */
 face[0]=1;
 face[1]=P3D_NPT+1;
 face[2]=2;
 p3d_poly_add_face(poly, face, 3);

 /* on cree les facettes rectangulaires des autres couches */
 for(i=0;i<(P3D_NPT-1)/2-1;i++){
   for(j=i*P3D_NPT+2;j<=(i+1)*P3D_NPT;j++){
     face[0]=j;
     face[1]=j+P3D_NPT;
     face[2]=j+P3D_NPT+1;
     face[3]=j+1;
     p3d_poly_add_face(poly, face, 4);
   }
   face[0]=(i+1)*P3D_NPT+1;
   face[1]=(i+2)*P3D_NPT+1;;
   face[2]=(i+1)*P3D_NPT+2;
   face[3]=i*P3D_NPT+2;
   p3d_poly_add_face(poly, face, 4);
   
 }

 /* on cree la facette en bas */
 for(i=0;i<P3D_NPT;i++)
   botface[i]= (P3D_NPT-1)/2*P3D_NPT + 1 - i ; 
 p3d_poly_add_face(poly, botface, P3D_NPT);

 p3d_poly_end_poly(poly);
 
 return(poly);
}




/**************************************************/
/* Fonction creant une demi review sphere         */
/* In : le nom du polyhedre, le rayon, la hauteur */
/* Out : le pointeur sur le p3d_poly              */
/**************************************************/
p3d_poly *p3d_create_sphere_shell(char name[20], double radius, double h, int type)
{
  p3d_poly *poly ;
  double r =  ( h * h + radius * radius )/ 2. / h ;
  double alpha = asin( (r-h) / h);
  double dt1=(M_PI - 2 * alpha)/(P3D_NPT-1),dt2=2*M_PI/P3D_NPT;
  int i,j,face[4],botface[P3D_NPT],compt=0;
 
  poly=p3d_poly_beg_poly(name,type);
  poly->entity_type = CONCAVE_POLYHEDRON;

  p3d_poly_add_vert(poly,0.,0.,r);

  for(i=1;i<=(P3D_NPT-1)/2;i++){
    for(j=0;j<=P3D_NPT-1;j++){
      p3d_poly_add_vert(poly,r*cos(M_PI/2.-i*dt1)*cos(j*dt2),
			r*cos(M_PI/2.-i*dt1)*sin(j*dt2),
			r*sin(M_PI/2.-i*dt1) + r - h);
      compt=compt+1;
    }
  }

  /* on cree les facettes triangulaires de la premiere couche */
  for(i=2;i<=P3D_NPT;i++){
    face[0]=1;
    face[1]=i;
    face[2]=i+1;
    p3d_poly_add_face(poly, face, 3);
  }
  /* la derniere face de la premiere couche */
  face[0]=1;
  face[1]=P3D_NPT+1;
  face[2]=2;
  p3d_poly_add_face(poly, face, 3);
  
  /* on cree les facettes rectangulaires des autres couches */
  for(i=0;i<(P3D_NPT-1)/2-1;i++){
    for(j=i*P3D_NPT+2;j<=(i+1)*P3D_NPT;j++){
      face[0]=j;
      face[1]=j+P3D_NPT;
      face[2]=j+P3D_NPT+1;
      face[3]=j+1;
      p3d_poly_add_face(poly, face, 4);
    }
    face[0]=(i+1)*P3D_NPT+1;
    face[1]=(i+2)*P3D_NPT+1;;
    face[2]=(i+1)*P3D_NPT+2;
    face[3]=i*P3D_NPT+2;
    p3d_poly_add_face(poly, face, 4);
    
  }
  
  /* on cree la facette en bas */
  for(i=0;i<P3D_NPT;i++)
    botface[i]= (P3D_NPT-1)/2*P3D_NPT + 1 - i ; 
  p3d_poly_add_face(poly, botface, P3D_NPT);
  
  p3d_poly_end_poly(poly);
 

  return(poly);
}

/**************************************************/
/* Fonction creant un demi ellipsoide             */
/* In : le nom du polyhedre, les dimensions       */
/* Out : le pointeur sur le p3d_poly              */
/**************************************************/
p3d_poly *p3d_create_half_oval(char name[20], double a, double b, double c, int type)
{double dt1=M_PI/(P3D_NPT-1),dt2=2*M_PI/P3D_NPT;
 p3d_poly *poly ;
 int i,j,face[4],botface[P3D_NPT],compt=0;

 poly=p3d_poly_beg_poly(name,type);
 poly->entity_type = CONVEX_POLYHEDRON; 

 p3d_poly_add_vert(poly,0.,0.,c);

 for(i=1;i<=(P3D_NPT-1)/2;i++){
   for(j=0;j<P3D_NPT;j++){
     p3d_poly_add_vert(poly,a*cos(M_PI/2.-i*dt1)*cos(j*dt2),
		            b*cos(M_PI/2.-i*dt1)*sin(j*dt2),
		            c*sin(M_PI/2.-i*dt1));
     compt=compt+1;
   }
 }

 /* on cree les facettes triangulaires de la premiere couche */
 for(i=2;i<=P3D_NPT;i++){
   face[0]=1;
   face[1]=i;
   face[2]=i+1;
   p3d_poly_add_face(poly, face, 3);
 }
 /* la derniere face de la premiere couche */
 face[0]=1;
 face[1]=P3D_NPT+1;
 face[2]=2;
 p3d_poly_add_face(poly, face, 3);

 /* on cree les facettes rectangulaires des autres couches */
 for(i=0;i<(P3D_NPT-1)/2-1;i++){
   for(j=i*P3D_NPT+2;j<=(i+1)*P3D_NPT;j++){
     face[0]=j;
     face[1]=j+P3D_NPT;
     face[2]=j+P3D_NPT+1;
     face[3]=j+1;
     p3d_poly_add_face(poly, face, 4);
   }
   face[0]=(i+1)*P3D_NPT+1;
   face[1]=(i+2)*P3D_NPT+1;;
   face[2]=(i+1)*P3D_NPT+2;
   face[3]=i*P3D_NPT+2;
   p3d_poly_add_face(poly, face, 4);
 }

 /* on cree la facette en bas */
 for(i=0;i<P3D_NPT;i++)
   botface[i]= (P3D_NPT-1)/2*P3D_NPT + 1 - i ; 
 p3d_poly_add_face(poly, botface, P3D_NPT);
 
 p3d_poly_end_poly(poly);
 
 return(poly);
}

/**************************************************/
/* Fonction creant une portion de tore            */
/* In : le nom du polyhedre, les rayons, l'angle  */
/* Out : le pointeur sur le p3d_poly              */
/**************************************************/
p3d_poly *p3d_create_tore(char name[20], double r1, double r2, double a, int type)
{double dt1=a/(P3D_NPT-1),dt2=2*M_PI/P3D_NPT;
 p3d_poly *poly ;
 int i,j,face[4],face1[P3D_NPT],face2[P3D_NPT],compt=1;

 poly=p3d_poly_beg_poly(name,type);
 poly->entity_type = CONCAVE_POLYHEDRON;

 /* le premier point est commun a toutes les facettes.... */
 p3d_poly_add_vert(poly,r1*(cos(a/2.)-1.)-r2*cos(a/2.),0.,r1*sin(a/2.)-r2*sin(a/2.));

 for(i=0;i<=P3D_NPT-1;i++){
   for(j=1;j<=P3D_NPT-1;j++){
     p3d_poly_add_vert(poly,
		       r1*(cos((a/2.)-(i*dt1))-1.) + r2*cos((a/2.)-(i*dt1))*cos(M_PI+(j*dt2)),
		       r2*sin(M_PI+(j*dt2)),
		       r1*sin((a/2.)-(i*dt1)) + r2*sin((a/2.)-i*dt1)*cos(M_PI+(j*dt2)));
     compt=compt+1;
   }
 }

 /* on cree les facettes bouchant le "tube" */
 face1[0]=1;
 face2[0]=1;

 for(i=1;i<=P3D_NPT-1;i++){
   face1[i]=i+1;
   face2[i]=P3D_NPT*P3D_NPT-P3D_NPT+2-i;
 }
 p3d_poly_add_face(poly, face1, P3D_NPT);
 p3d_poly_add_face(poly, face2, P3D_NPT);


 /* on cree les facettes du tube*/
 for(i=0;i<=P3D_NPT-2;i++){

   /* on cree les facettes triangulaires partant de 1 */
   face[0]=1;
   face[1]= i*(P3D_NPT-1) + P3D_NPT + 1;
   face[2]=(i-1)*(P3D_NPT-1) + P3D_NPT + 1;
   p3d_poly_add_face(poly, face, 3);

   face[0]=1;
   face[1]=i*(P3D_NPT-1) + P3D_NPT;
   face[2]=(i+1)*(P3D_NPT-1) + P3D_NPT;
   p3d_poly_add_face(poly, face, 3);

   /* on cree les facettes rectangulaires qui restent... */
   for(j=(i-1)*(P3D_NPT-1)+P3D_NPT+1;j<=i*(P3D_NPT-1)+P3D_NPT-1;j++){
     face[0]=j;
     face[1]=j+P3D_NPT-1;
     face[2]=j+P3D_NPT;
     face[3]=j+1;
     p3d_poly_add_face(poly, face, 4);
   }
 }
   
 p3d_poly_end_poly(poly);
 
 return(poly);
}  

/********************************************************************/
/*!\fn void p3d_scale_prim(p3d_poly *poly, double scale)
 *
 * \brief scale a primitive
 *
 * \param poly the primitive
 * \param scale scale factor
 *
 */
/********************************************************************/
void p3d_scale_prim(p3d_poly *poly, double scale)
{
  p3d_scale_poly(poly->poly, scale, scale, scale);
  if (poly->primitive_data)
    {
      poly->primitive_data->radius *= scale;
      poly->primitive_data->other_radius *= scale;
      poly->primitive_data->height *= scale;
      poly->primitive_data->x_length *= scale;
      poly->primitive_data->y_length *= scale;
      poly->primitive_data->z_length *= scale;
    }
  poly->box.x1 *= scale;
  poly->box.x2 *= scale;
  poly->box.y1 *= scale;
  poly->box.y2 *= scale;
  poly->box.z1 *= scale;
  poly->box.z2 *= scale;

}
