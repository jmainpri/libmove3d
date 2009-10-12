/*************************************************************************/
/********************FONCTIONS GEOMETRIQUES DE BASE***********************/
/*************************************************************************/


#include <math.h>
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "GraspPlanning-pkg.h"
#include <string.h>


//! Cette fonction calcule l'intersection entre la droite (c1c2) et le triangle (p1p2p3).
//! Retourne 1 ou 0 selon qu'il y a intersection ou pas et recopie l'intersection si elle existe dans "intersection".
//! L'intersection pint avec le plan du triangle (a$*x + b*y + c*z + d = 0 ) doit verifier
//! pint.n + d = 0 et  pint= c1 + alpha*c1c2 avec n= (a,b,c) et alpha un reel.
//! c1.n + alpha*(c1c2.n) + d = 0
//! alpha= -(c1.n + d)/(c1c2.n)
//! La fonction teste ensuite si l'intersection est bien dans le triangle.
//! Computes the intersection between a triangle and a line.
//! \param c1 first point used to define the line
//! \param c2 second point used to define the line
//! \param p1 first point of the triangle
//! \param p2 second point of the triangle
//! \param p3 third point of the triangle
//! \return 1 if there is an intersection, 0 otherwise
int gpLine_triangle_intersection(p3d_vector3 c1, p3d_vector3 c2, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, p3d_vector3 intersection)
{
    int i;
    double a0, a, alpha, x1, x2;
    poly_plane plane= gpPlane_from_points(p1, p2, p3);

    p3d_vector3 u;
    p3d_vectSub(c2, c1, u); //u= c1c2;
    p3d_vectNormalize(u, u);

    a= p3d_vectDotProd(plane.normale, u);

    if( fabs(a) < EPSILON ) //droite et normale du triangle sont paralleles
       return FALSE;        //-> pas d'intersection (on elimine le cas où ligne et triangle seraient coplanaires)


    a0= p3d_vectDotProd(plane.normale, c1);

    alpha= -(plane.d + a0)/a;

    for(i=0;i<3;i++)
         intersection[i]= c1[i] + alpha*u[i]; //intersection droite-plan du triangle


    //il faut tester si l'intersection droite-(plan du triangle) est incluse dans le triangle:

    //on teste si l'intersection est du même côte du plan (c1p1p2) que p3:
    plane= gpPlane_from_points(c1, p1, p2);
    x1= p3d_vectDotProd(plane.normale, p3) + plane.d;
    x2= p3d_vectDotProd(plane.normale, intersection) + plane.d;
    if( SIGN(x1) != SIGN(x2) )
       return FALSE;

    //on teste si l'intersection est du même côte du plan (c1p1p3) que p2:
    plane= gpPlane_from_points(c1, p1, p3);
    x1= p3d_vectDotProd(plane.normale, p2) + plane.d;
    x2= p3d_vectDotProd(plane.normale, intersection) + plane.d;
    if( SIGN(x1) != SIGN(x2) )
       return FALSE;

    //on teste si l'intersection est du même côte du plan (c1p2p3) que p1:
    plane= gpPlane_from_points(c1, p2, p3);
    x1= p3d_vectDotProd(plane.normale, p1) + plane.d;
    x2= p3d_vectDotProd(plane.normale, intersection) + plane.d;
    if( SIGN(x1) != SIGN(x2) )
       return FALSE;

    return TRUE;
}



//! Cette fonction calcule l'intersection entre le rayon (demi-droite) partant de "origin" dans la direction "direction"
//! et le triangle (p1p2p3). Le rayon est une DEMI-droite.
//! Retourne 1 ou 0 selon qu'il y a intersection ou pas et recopie l'intersection si elle existe dans intersection.
//! L'intersection pint avec le plan du triangle (a*x + b*y + c*z + d = 0 ) doit verifier
//! pint.n + d = 0 et  pint= origin + alpha*direction avec n= (a,b,c) et alpha un reel >= 0.
//! origin.n + alpha*(direction.n) + d = 0
//! alpha= -(origin.n + d)/(direction.n)
//! La fonction teste ensuite si l'intersection est bien dans le triangle.
int gpRay_triangle_intersection(p3d_vector3 origin, p3d_vector3 direction, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, p3d_vector3 intersection)
{
    int i;
    double a0, a, alpha, x1, x2;
    poly_plane plane= gpPlane_from_points(p1, p2, p3);

    p3d_vectNormalize(direction, direction);
    a= p3d_vectDotProd(plane.normale, direction);

    if( fabs(a) < EPSILON ) //rayon et normale du triangle sont paralleles
       return FALSE;        //-> pas d'intersection (on elimine le cas où rayon et triangle seraient coplanaires)


    a0= p3d_vectDotProd(plane.normale, origin);

    alpha= -(plane.d + a0)/a;

    //On teste si l'intersection est dans le demi-plan contenant la demi-droite et de même normale:
    if( alpha <= 0 )
    { return FALSE; }

    for(i=0;i<3;i++)
         intersection[i]= origin[i] + alpha*direction[i]; //intersection droite-plan du triangle


    //il faut tester si l'intersection est incluse dans le triangle:

    //on teste si l'intersection est du même côte du plan (origine-p1-p2) que p3:
    plane= gpPlane_from_points(origin, p1, p2);
    x1= p3d_vectDotProd(plane.normale, p3) + plane.d;
    x2= p3d_vectDotProd(plane.normale, intersection) + plane.d;
    if( SIGN(x1) != SIGN(x2) )
       return FALSE;

    //on teste si l'intersection est du même côte du plan (origine-p1-p3) que p2:
    plane= gpPlane_from_points(origin, p1, p3);
    x1= p3d_vectDotProd(plane.normale, p2) + plane.d;
    x2= p3d_vectDotProd(plane.normale, intersection) + plane.d;
    if( SIGN(x1) != SIGN(x2) )
       return FALSE;

    //on teste si l'intersection est du même côte du plan (origine-p2-p3) que p1:
    plane= gpPlane_from_points(origin, p2, p3);
    x1= p3d_vectDotProd(plane.normale, p1) + plane.d;
    x2= p3d_vectDotProd(plane.normale, intersection) + plane.d;
    if( SIGN(x1) != SIGN(x2) )
       return FALSE;

    return TRUE;
}





//!  Cette fonction calcule l'intersection entre un plan et un segment donne par deux points.
//!  Il peut y avoir 0, 1 ou 2 points d'intersection (le segment est dans le plan).
//!  S'il n'y en a qu'un, le point d'intersection est recopie en sortie.
//!  S'il y en a deux, les points d'intersection sont directement les points du segment.
//!  La fonction retourne le nombre de points d'intersection.
int gpLine_segment_plane_intersection(poly_plane plane, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 result)
{
   int i;
   double alpha;
   p3d_vector3 p1p2, n12;

   p3d_vectSub(p2, p1, p1p2);

   #ifdef DEBUG
     if( p3d_vectNorm(p1p2) < EPSILON )
     {
       printf("%s: %d: gpLine_segment_plane_intersection(): error: the points defining the segment are the same.\n",__FILE__,__LINE__);
       return 0;
     }
   #endif

   p3d_vectNormalize(plane.normale, plane.normale);
   p3d_vectNormalize(p1p2, n12);

   if( fabs(p3d_vectDotProd(plane.normale, n12)) < EPSILON ) //segment parallele au plan
   {
     if( fabs( p3d_vectDotProd(plane.normale,p1) + plane.d ) < EPSILON ) //un point du segment (donc les deux) appartient au plan
     {
        return 2;
     }
     else
     {  return 0; }
   }

   //Un point du segment s'ecrit p= p1 + alpha*(p2-p1) = p1 + alpha*p1p2, avec alpha dans [O;1].
   //Un point du plan verifie p.normale + d = 0.
   //On en deduit alpha => alpha = - (d + normale.p1) / (normale.p1p2)
   alpha= - ( p3d_vectDotProd(plane.normale, p1) + plane.d )/ p3d_vectDotProd(plane.normale, p1p2);


   if( alpha > 1 || alpha < 0 )  //Les points sont du même côte du plan => pas d'intersection segment-plan
      return 0;

   for(i=0;i<3;i++)
     result[i] = p1[i] + alpha*p1p2[i];

   return 1;
}


//! Computes the closest point of a point to a segment line.
//! \param p the point
//! \param p1 the first point of the segment line
//! \param p2 the second point of the segment line
//! \return the minimimal distance between the segment and the point
double gpPoint_to_line_segment_distance(p3d_vector3 p, p3d_vector3 p1, p3d_vector3 p2)
{
  double alpha, d1, d2, d3, result;
  p3d_vector3 p1p2, p1p2_n, p1p, proj;

  p3d_vectSub(p2, p1, p1p2);
  p3d_vectNormalize(p1p2, p1p2_n);

  p3d_vectSub(p, p1, p1p);
  
  alpha= p3d_vectDotProd(p1p, p1p2_n);

  proj[0]= p1[0] + alpha*p1p2_n[0];
  proj[1]= p1[1] + alpha*p1p2_n[1];
  proj[2]= p1[2] + alpha*p1p2_n[2];

  d1= sqrt( pow(proj[0]-p[0],2) + pow(proj[1]-p[1],2) + pow(proj[2]-p[2],2) );
  d2= sqrt( pow(p1[0]-p[0],2) + pow(p1[1]-p[1],2) + pow(p1[2]-p[2],2) );
  d3= sqrt( pow(p2[0]-p[0],2) + pow(p2[1]-p[1],2) + pow(p2[2]-p[2],2) );

  if(d2 < d1)
  {  result= d2;  }
  else
  {  result= d1;  }

  if(d3 < result)
  {  result= d3;  }

  return result;
}


//! Cette fonction calcule l'intersection entre le triangle p1p2p3 et un plan.
//! Elle retourne le nombre d'intersections (0, 1, 2 ou 3 (triangle dans le plan)).
//! NB: le cas d'une seule intersection correspond a la situation où seul un des sommets
//! du triangle appartient au plan.
//! Si on a 3 intersections, les intersections sont les points d'origine du triangle.
//! Computes the intersection between a triangle and a plane.
//! \param p1 first point of the triangle
//! \param p2 second point of the triangle
//! \param p3 third point of the triangle
//! \param plane the plane's equation
//! \param result1 the first intersection (if it exists)
//! \param result2 the second intersection (if it exists)
//! \return the number of intersections (0, 1, 2 or 3 (the triangle is in the plane))
int gpTriangle_plane_intersection(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3,poly_plane plane,
			       p3d_vector3 result1, p3d_vector3 result2)
{
   int nbIntersectionPoints= 0;
   p3d_vector3 result;
   int ninter;

   ninter= gpLine_segment_plane_intersection(plane, p1, p2, result);
   if(ninter==2)
   {
      if( fabs( p3d_vectDotProd(plane.normale,p3) + plane.d ) < GP_EPSILON )
        return 3;
      else
      {
        p3d_vectCopy(p1, result1);
        p3d_vectCopy(p2, result2);
        return 2;
      }
   }
   if(ninter==1)
   {
      p3d_vectCopy(result, result1);
      nbIntersectionPoints++;
   }


   ninter= gpLine_segment_plane_intersection(plane, p2, p3, result);
   if(ninter==2)
   {
      if( fabs( p3d_vectDotProd(plane.normale,p1) + plane.d ) < GP_EPSILON )
        return 3;
      else
      {
        p3d_vectCopy(p2, result1);
        p3d_vectCopy(p3, result2);
        return 2;
      }
   }
   if(ninter==1)
   {
      if( nbIntersectionPoints==0 )
         p3d_vectCopy(result, result1);
      else
         p3d_vectCopy(result, result2);
      nbIntersectionPoints++;
   }


   ninter= gpLine_segment_plane_intersection(plane, p1, p3, result);
   if(ninter==2)
   {
      if( fabs( p3d_vectDotProd(plane.normale,p2) + plane.d ) < GP_EPSILON )
        return 3;
      else
      {
        p3d_vectCopy(p1, result1);
        p3d_vectCopy(p3, result2);
        return 2;
      }
   }
   if(ninter==1)
   {
      if( nbIntersectionPoints==0 )
         p3d_vectCopy(result, result1);
      else
         p3d_vectCopy(result, result2);
      nbIntersectionPoints++;
   }

   //Si on a deux intersection extrêmement proches, on les assimile:
   if( nbIntersectionPoints == 2 )
   {
     if( sqrt( pow(result1[0]-result2[0],2) + pow(result1[1]-result2[1],2) + pow(result1[2]-result2[2],2) ) < GP_EPSILON )
        return 1;
   }

    return nbIntersectionPoints;

}

//! Cette fonction retourne 1 si le plan coupe le triangle (p1p2p3), 0 sinon.
int gpCheck_triangle_plane_intersection(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, poly_plane plane)
{
   int cnt= 0;
   if( p3d_vectDotProd(p1, plane.normale) + plane.d > EPSILON )
      cnt++;
   else
      cnt--;
   if( p3d_vectDotProd(p2, plane.normale) + plane.d > EPSILON )
      cnt++;
   else
      cnt--;
   if( p3d_vectDotProd(p3, plane.normale) + plane.d > EPSILON )
      cnt++;
   else
      cnt--;

   if( cnt == 3 || cnt==-3) return 0;
   else return 1;

   cnt+= SIGN( p3d_vectDotProd(p1, plane.normale) + plane.d );
   cnt+= SIGN( p3d_vectDotProd(p2, plane.normale) + plane.d );
   if( cnt == 0 ) return 1;
   cnt+= SIGN( p3d_vectDotProd(p3, plane.normale) + plane.d );
   if( cnt == 3 || cnt==-3) return 0;
   else return 1;
}

//! Retourne 1 si le point est au-dessus du plan (i.e. du côte vers lequel pointe la normale du plan),
//! 0 sinon.
inline int gpIs_point_above_plane(p3d_vector3 point, poly_plane plane)
{
  if( p3d_vectDotProd(plane.normale, point) + plane.d > 0 )
    return 1;
  else
    return 0;
}


//! Cette fonction calcule l'intersection entre deux plans d'equation plane1 et plane2.
//! Le resultat est une droite passant par "point_on_line" de vecteur directeur "line_direction".
//! La fonction retourne 0 si les plans sont paralleles, 1 sinon.
//! NOTE: not tested.
int gpPlane_plane_intersection(poly_plane *plane1, poly_plane *plane2, p3d_vector3 point_on_line, p3d_vector3 line_direction)
{
   double norm;
   p3d_vectXprod(plane1->normale, plane2->normale, line_direction);
   norm= p3d_vectNorm(line_direction);
   if( norm < 10e-6) //if planes are parallel
     return 0;
   else
     p3d_vectNormalize(line_direction, line_direction);

   int i;
   double n1n1= p3d_vectDotProd(plane1->normale, plane1->normale);
   double n2n2= p3d_vectDotProd(plane2->normale, plane2->normale);
   double n1n2= p3d_vectDotProd(plane1->normale , plane2->normale);

   double determinant= n1n1*n2n2 - n1n2*n1n2;
   double c1, c2;

   c1= ( (plane2->d)*n1n2 - (plane1->d)*n2n2 ) / determinant;
   c2= ( (plane1->d)*n1n2 - (plane2->d)*n1n1 ) / determinant;

   for(i=0;i<3;i++)
     point_on_line[i]= c1*(plane1->normale[i]) + c2*(plane2->normale[i]);

   p3d_vectXprod(plane1->normale, plane2->normale,line_direction);
   p3d_vectNormalize(line_direction,line_direction);

   return 1;
}



//! Construit un plan (poly_plane) a partir des coordonnees de trois points.
//! Les parametres de l'equation du plan sont definis tels que:
//! normale.p + d = 0 pour tout point p appartenant au plan (avec "." le produit scalaire).
poly_plane gpPlane_from_points(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3)
{
  poly_plane plane;
  p3d_vector3 p1p2, p1p3, normal;
  p3d_vectSub(p2, p1, p1p2);
  p3d_vectSub(p3, p1, p1p3);
  p3d_vectXprod(p1p2, p1p3, normal);
  p3d_vectNormalize(normal, plane.normale);

  plane.d= -plane.normale[0]*p1[0] - plane.normale[1]*p1[1] - plane.normale[2]*p1[2];

  return plane;
}

//! Construit un plan (poly_plane) a partir de la normale au plan et des coordonnees
//! d'un point du plan.
//! Les parametres de l'equation du plan sont definis tels que:
//! normale.p + d = 0 pour tout point p appartenant au plan (avec "." le produit scalaire).
poly_plane gpPlane_from_point_and_normal(p3d_vector3 p, p3d_vector3 normal)
{
  poly_plane plane;

  p3d_vectNormalize(normal, plane.normale);
  plane.d= -plane.normale[0]*p[0] - plane.normale[1]*p[1] - plane.normale[2]*p[2];

  return plane;
}


//! A partir de l'equation d'un plan "plane", definie pour des coordonnees exprimees dans le repere de matrice 
//! de transformation T, retourne l'equation du plan dans le repere global.
poly_plane gpTransform_plane_equation(p3d_matrix4 T, poly_plane plane)
{
   poly_plane result;
   p3d_vector3 point, point2;

   p3d_xformVect(T, plane.normale, result.normale);

   p3d_vectScale(plane.normale, point, -plane.d);
   p3d_xformPoint(T, point, point2);
   result.d= -p3d_vectDotProd(result.normale, point2);
  
   return result;
}



//! Cette fonction calcule l'intersection entre un segment de droite et une sphere.
//! Le segment est donne par les points p1 et p2 et la sphere par son centre "center" et son rayon "radius".
//! Il peut y avoir 0, 1 ou 2 points d'intersection. Ce nombre est retourne par la fonction.
//! S'ils existent, les points d'intersection sont recopies dans result1 et result2.
//! S'il n'y en a qu'un, il est recopie dans result1.
int gpLine_segment_sphere_intersection(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 center, double radius, p3d_vector3 result1, p3d_vector3 result2)
{
  // test prealable rapide: si les deux extremites du segment sont a l'interieur de la sphere,
  // il n'y a pas d'intersection:
  if(SQR( p1[0]-center[0] ) + SQR( p1[1]-center[1] ) + SQR( p1[2]-center[2] ) < SQR(radius) && 
SQR( p2[0]-center[0] ) + SQR( p2[1]-center[1] ) + SQR( p2[2]-center[2] ) < SQR(radius) )
   return 0;

  // Principe:
  // p1= (x1,y1,z1)    p2= (x2,y2,z2)    center= (x3,y3,z3)
  // Un point p= (x,y,z) de la droite associee au segment verifient l'equation:
  // p= p1 + u*(p2 - p1)   avec  0 <= u <= 1
  // donc
  // x = x1 + u*(x2 - x1)
  // y = y1 + u*(y2 - y1)
  // z = z1 + u*(z2 - z1)
  // S'il appartient a la sphere, il verifie aussi:
  // (x - x3)^2 + (y - y3)^2 + (z - z3)^2 = r^2


  // En substituant l'equation de la droite dans celle de la sphere, on obtient une equation du second degre:
  //   a*u^2 + b*u + c = 0
  // avec
  //   a = (x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2
  //
  //   b = 2*[ (x2 - x1)*(x1 - x3) + (y2 - y1)*(y1 - y3) + (z2 - z1)*(z1 - z3) ]
  //
  //   c = x1^2 + y1^2 + z1^2  + x3^2 + y3^2 + z3^2 - 2*[x1*x3 + y1*y3 + z1*z3] - r^2
  //   qui a 0, 1 ou 2 solutions suivant le cas.
  //   Il faut ensuite verifier que ces solutions sont bien sur le segment.


  int nbSolutions= 0;
  double u, a, b, c, delta;
  a=  SQR( p2[0]-p1[0] ) + SQR( p2[1]-p1[1] ) + SQR( p2[2]-p1[2] );

  b=  2* ( (p2[0] - p1[0])*(p1[0] - center[0]) + (p2[1] - p1[1])*(p1[1] - center[1])  + (p2[2] - p1[2])*(p1[2] - center[2]) ) ;

  c= p3d_vectDotProd(p1, p1) + p3d_vectDotProd(center, center) - 2*p3d_vectDotProd(p1, center) - SQR(radius);

  delta =   b*b - 4*a*c ;

  if( delta < 0 ) //pas de solution
  { return 0; }

  if( fabs(delta) < EPSILON ) //au maximum une seule solution
  {
     u = -b/(2*a);
     if( u < 0 || u > 1 ) //L'intersection n'est pas sur le segment
        return 0;
     else
     {
        result1[0]= p1[0] + u*(p2[0] - p1[0]);
        result1[1]= p1[1] + u*(p2[1] - p1[1]);
        result1[2]= p1[2] + u*(p2[2] - p1[2]);
        return 1;
     }
  }
  else //au maximum deux solutions
  {
    // premiere intersection:
    u = (-b + sqrt( delta )) / (2*a);
    if( 0 <= u && u <= 1 ) //L'intersection est sur le segment
    {
       result1[0]= p1[0] + u*(p2[0] - p1[0]);
       result1[1]= p1[1] + u*(p2[1] - p1[1]);
       result1[2]= p1[2] + u*(p2[2] - p1[2]);
       nbSolutions++;
    }

    // deuxieme intersection:
    u = (-b - sqrt( delta )) / (2*a);
    if( 0 <= u && u <= 1 ) //L'intersection est sur le segment
    {
       if(nbSolutions==0)
       {
         result1[0]= p1[0] + u*(p2[0] - p1[0]);
         result1[1]= p1[1] + u*(p2[1] - p1[1]);
         result1[2]= p1[2] + u*(p2[2] - p1[2]);
         nbSolutions++;
       }
       else
       {
         result2[0]= p1[0] + u*(p2[0] - p1[0]);
         result2[1]= p1[1] + u*(p2[1] - p1[1]);
         result2[2]= p1[2] + u*(p2[2] - p1[2]);
         nbSolutions++;
       }
    }

    return nbSolutions;
  }



}


//! Cette fonction calcule les coordonnees 3D d'un point a l'interieur du triangle (p1p2p3)
//! a partir de deux coordonnees (alpha,beta) comprises entre 0.0 et 1.0.
//! Les coordonnees calculees sont retournees dans result.
void gpPoint_in_triangle_from_parameters(double alpha, double beta, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, p3d_vector3 result)
{
  #ifdef DEBUG
  if( alpha < 0 || alpha > 1 || beta < 0 || beta > 1 )
    printf("%s: %d: bad input parameters of the position in the triangle.\n",__FILE__,__LINE__);
  #endif
  //Le principe est le suivant: on calcule les points D,E et F (vecteurs OD, OE et OF)
  //selon
  //OD= p1 + alpha*p1p2  = (1-alpha)*p1 + alpha*p2
  //OE= p1 + alpha*p1p3  = (1-alpha)*p1 + alpha*p3
  //D et E sont tels que DE // p2p3
  //OF= OD + beta*DE= OD + beta*(-OD+OE)= (1-beta)*OD + beta*OE
  //OF= p1 + alpha*p1p2 + alpha*beta*p2p3
  //Le point F appartient a la surface du triangle
  //->  c'est le point resultat.
   result[0]= p1[0] + alpha*(p2[0]-p1[0]) + alpha*beta*(p3[0]-p2[0]);
   result[1]= p1[1] + alpha*(p2[1]-p1[1]) + alpha*beta*(p3[1]-p2[1]);
   result[2]= p1[2] + alpha*(p2[2]-p1[2]) + alpha*beta*(p3[2]-p2[2]);
}

/*
// Cette fonction reçoit le point p dont on sait qu'il appartient au triangle (p1p2p3).
// Elle retourne sa parametrisation (alpha,beta) sur la surface du triangle.
int parameters_in_triangle_from_point(p3d_vector3 p, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, double *alpha, double *beta)
{
   p3d_vector3 p1p2, p2p3, p1p3, p1p;
   p3d_vector3 n12, n13;

   p3d_vectSub(p2, p1, p1p2);
   p3d_vectSub(p3, p2, p2p3);
   p3d_vectSub(p3, p1, p1p3);
   p3d_vectSub( p, p1, p1p);
   p3d_vectNormalize(p1p2, n12);
   p3d_vectNormalize(p1p3, n13);


   #ifdef DEBUG
   p3d_vector3 z;
   p3d_vectXprod(n12, n13, z);
   if( fabs( p3d_vectDotProd(p1p, z) ) > EPSILON  )
   {
      printf("%s: %d: parameters_in_triangle_from_point(): erreur 3: le point n'appartient pas au plan du triangle.\n",__FILE__,__LINE__);
      return 0;
   }
   #endif


   //On fait un simple changement de base pour obtenir la parametrisation du point:
   utl_matrix *M= NULL, *pinvM= NULL;
   utl_vector *vect_p1p= NULL, *result= NULL;

   gsl_matrix *M= gsl_matrix_alloc(3, 2);
   gsl_vector *vect_p1p= gsl_vector_alloc(3);
   gsl_vector *result= gsl_vector_alloc(2);
   gsl_vector *S= gsl_vector_alloc(M->size2);
   gsl_matrix *V= gsl_matrix_alloc(M->size2, M->size2);
   gsl_vector *work= gsl_vector_alloc(2);

   gsl_matrix_set(M, 0, 0, p1p2[0]);     gsl_matrix_set(M, 0, 1, p2p3[0]);
   gsl_matrix_set(M, 1, 0, p1p2[1]);     gsl_matrix_set(M, 1, 1, p2p3[1]);
   gsl_matrix_set(M, 2, 0, p1p2[2]);     gsl_matrix_set(M, 2, 1, p2p3[2]);


   gsl_linalg_SV_decomp(M, V, S, work);
   gsl_linalg_SV_solve(M, V, S, vect_p1p, result);

   utl_matrix_psedo_inverse(M, pinvM);

   vect_p1p->ve[0]= p[0] - p1[0];
   vect_p1p->ve[1]= p[1] - p1[1];
   vect_p1p->ve[2]= p[2] - p1[2];

   utl_matrix_prod_vect(pinvM, vect_p1p, result);

   *alpha= gsl_vector_get(result, 0);
   if((*alpha)> 1)
   {
    if(*alpha > 1 + EPSILON)
     printf("%s: %d: parameters_in_triangle_from_point(): le point n'appartient pas au triangle (alpha=%f).\n",__FILE__,__LINE__, *alpha);
    (*alpha)=1;
   }
   if((*alpha)<=0)
   {
    if(*alpha < -EPSILON)
     printf("%s: %d: parameters_in_triangle_from_point(): le point n'appartient pas au triangle (alpha=%f).\n",__FILE__,__LINE__, *alpha);
     (*alpha)= 0;
     (*beta) = 0;
   }
   *beta= gsl_vector_get(result, 1)/(*alpha);

   if((*beta)> 1)
   {
    if(*beta > 1 + EPSILON)
     printf("%s: %d: parameters_in_triangle_from_point(): le point n'appartient pas au triangle (beta=%f).\n",__FILE__,__LINE__, *beta);
    (*beta)= 1;
   }
   if((*beta) < 0)
   {
    if(*beta < -EPSILON)
     printf("%s: %d: parameters_in_triangle_from_point(): le point n'appartient pas au triangle (beta=%f).\n",__FILE__,__LINE__, *beta);
     (*beta)=0;
   }

   gsl_matrix_free(M);
   gsl_matrix_free(S);
   gsl_matrix_free(V);
   gsl_vector_free(vect_p1p);
   gsl_vector_free(result);
   gsl_vector_free(work);

}*/


//! Cette fonction reçoit le point p dont on sait qu'il appartient au triangle (p1p2p3).
//! Elle retourne sa parametrisation (alpha,beta) sur la surface du triangle.
int gpParameters_in_triangle_from_point(p3d_vector3 p, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, double *alpha, double *beta)
{
   p3d_vector3 p1p2, p2p3, p1p3, p1p;
   p3d_vector3 n12, n13, cross1, cross2;

   p3d_vectSub(p2, p1, p1p2);
   p3d_vectSub(p3, p2, p2p3);
   p3d_vectSub(p3, p1, p1p3);
   p3d_vectSub( p, p1, p1p);
   p3d_vectNormalize(p1p2, n12);
   p3d_vectNormalize(p1p3, n13);


   #ifdef DEBUG
   p3d_vector3 z;
   p3d_vectXprod(n12, n13, z);
   if( fabs( p3d_vectDotProd(p1p, z) ) > EPSILON  )
   {
      printf("%s: %d: gpParameters_in_triangle_from_point(): erreur 3: the point does not belong to the triangle's plane.\n",__FILE__,__LINE__);
      return 0;
   }
   #endif


   p3d_vectXprod(p1p2, p2p3, cross1);
   p3d_vectXprod(p1p, p2p3, cross2);

   if(p3d_vectNorm(cross1) < EPSILON) //triangle degenere (aplati)
   {
     (*alpha)= p3d_vectDotProd(p1p, p1p2);
     (*beta)= 0;
     return 1;
   }
   if(p3d_vectNorm(cross2) < EPSILON) //triangle degenere (aplati)
   {
     (*alpha)= 0;
     (*beta)= p3d_vectDotProd(p1p, p2p3);
     return 1;
   }

   (*alpha)= p3d_vectNorm(cross2)/p3d_vectNorm(cross1);

   (*beta)= ( p3d_vectDotProd(p1p, p2p3) - (*alpha)*p3d_vectDotProd(p1p2, p2p3) )/ ((*alpha)*p3d_vectDotProd(p2p3, p2p3) );


   if((*alpha)> 1)
   {
    if(*alpha > 1 + EPSILON)
     printf("%s: %d: gpParameters_in_triangle_from_point(): the point is not inside to the triangle (alpha=%f).\n",__FILE__,__LINE__, *alpha);
    (*alpha)=1;
   }
   if((*alpha)<=0)
   {
    if(*alpha < -EPSILON)
     printf("%s: %d: gpParameters_in_triangle_from_point():  the point is not inside to the triangle (alpha=%f).\n",__FILE__,__LINE__, *alpha);
     (*alpha)= 0;
     (*beta) = 0;
   }

   if((*beta)> 1)
   {
    if(*beta > 1 + EPSILON)
     printf("%s: %d: gpParameters_in_triangle_from_point():  the point is not inside to the triangle (beta=%f).\n",__FILE__,__LINE__, *beta);
    (*beta)= 1;
   }
   if((*beta) < 0)
   {
    if(*beta < -EPSILON)
     printf("%s: %d: gpParameters_in_triangle_from_point():  the point is not inside to the triangle (beta=%f).\n",__FILE__,__LINE__, *beta);
     (*beta)=0;
   }

  p3d_vector3 result, diff;
  gpPoint_in_triangle_from_parameters(*alpha, *beta, p1, p2, p3, result);

  p3d_vectSub(p, result, diff);
  printf("diff= %g\n", p3d_vectNorm(diff));



  return 1;
}


/*
// Cette fonction reçoit le point p dont on sait qu'il appartient au triangle (p1p2p3).
// Elle retourne sa parametrisation (alpha,beta) sur la surface du triangle.
int parameters_in_triangle_from_point(p3d_vector3 p, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, double *alpha, double *beta)
{
   p3d_vector3 p1p2, p2p3, p1p3, p1p;
   p3d_vector3 n12, n13;

   p3d_vectSub(p2, p1, p1p2);
   p3d_vectSub(p3, p2, p2p3);
   p3d_vectSub(p3, p1, p1p3);
   p3d_vectSub( p, p1, p1p);
   p3d_vectNormalize(p1p2, n12);
   p3d_vectNormalize(p1p3, n13);


   #ifdef DEBUG
   p3d_vector3 z;
   p3d_vectXprod(n12, n13, z);
   if( fabs( p3d_vectDotProd(p1p, z) ) > EPSILON  )
   {
      printf("%s: %d: parameters_in_triangle_from_point(): erreur 3: le point n'appartient pas au plan du triangle.\n",__FILE__,__LINE__);
      return 0;
   }
   #endif

   //On fait un simple changement de base pour obtenir la parametrisation du point:
   utl_matrix *M= NULL, *pinvM= NULL;
   utl_vector *vect_p1p= NULL, *result= NULL;

   UTL_MATRIX_REG_RESIZE(M, 3, 2);
   UTL_MATRIX_REG_RESIZE(pinvM, 2, 3);
   UTL_VECTOR_REG_RESIZE( vect_p1p, 3);
   UTL_VECTOR_REG_RESIZE( result, 2);

   if( M==NULL || pinvM==NULL || vect_p1p==NULL || result==NULL)
   {  printf("%s: %d: parameters_in_triangle_from_point():ERREUR D'ALLOCATION MEMOIRE\n",__FILE__,__LINE__);
      return 0; }


   M->me[0][0]= p1p2[0];  M->me[0][1]= p2p3[0];
   M->me[1][0]= p1p2[1];  M->me[1][1]= p2p3[1];
   M->me[2][0]= p1p2[2];  M->me[2][1]= p2p3[2];

   utl_matrix_psedo_inverse(M, pinvM);

   vect_p1p->ve[0]= p[0] - p1[0];
   vect_p1p->ve[1]= p[1] - p1[1];
   vect_p1p->ve[2]= p[2] - p1[2];

   utl_matrix_prod_vect(pinvM, vect_p1p, result);

   *alpha= result->ve[0];
   if((*alpha)> 1)
   {
    if(*alpha > 1 + EPSILON)
     printf("%s: %d: parameters_in_triangle_from_point(): le point n'appartient pas au triangle (alpha=%f).\n",__FILE__,__LINE__, *alpha);
    (*alpha)=1;
   }
   if((*alpha)<=0)
   {
    if(*alpha < -EPSILON)
     printf("%s: %d: parameters_in_triangle_from_point(): le point n'appartient pas au triangle (alpha=%f).\n",__FILE__,__LINE__, *alpha);
     (*alpha)= 0;
     (*beta) = 0;
   }
   *beta= result->ve[1]/(*alpha);

   if((*beta)> 1)
   {
    if(*beta > 1 + EPSILON)
     printf("%s: %d: parameters_in_triangle_from_point(): le point n'appartient pas au triangle (beta=%f).\n",__FILE__,__LINE__, *beta);
    (*beta)= 1;
   }
   if((*beta) < 0)
   {
    if(*beta < -EPSILON)
     printf("%s: %d: parameters_in_triangle_from_point(): le point n'appartient pas au triangle (beta=%f).\n",__FILE__,__LINE__, *beta);
     (*beta)=0;
   }


   utl_matrix_destroy(M);
   utl_matrix_destroy(pinvM);
   utl_vector_destroy(vect_p1p);
   utl_vector_destroy(result);

}*/


//! Cette fonction calcule la projection orthogonale
//! d'un point sur un plan d'equation ( ax + by + cz + d = 0) dont
//! les parametres sont dans une structure poly_plane.
void gpOrthogonal_projection_point_onto_plane(p3d_vector3 point, poly_plane plane, p3d_vector3 result)
{
   p3d_vector3 normal;
   p3d_vectCopy(plane.normale, normal);
   p3d_vectNormalize(normal, normal); //par precaution

   // Soit p2= (x2,y2,z2) le projete orthogonal sur le plan d'equation
   //  ax + by + cz + d= 0, du point p1= (x1,y1,z1).
   // Soit n= (a,b,c).
   // On a n.p2 + d = 0
   // et (p1-p2) = r*n avec r un reel
   // donc r*(n.n) = n.p1 - n.p2 = n.p1 + d
   // => r = (n.p1 + d)/(n.n)= (n.p1 + d)
   // p2= p1 - r*n

   double r= p3d_vectDotProd(normal, point) + plane.d;
   p3d_vectScale(normal, normal, r);
   p3d_vectSub( point, normal, result);
}

//! Cette fonction calcule un vecteur orthogonal (quelconque) au vecteur v et le normalise.
//! Elle retourne un choix parmi l'infinite possible.
void gpOrthogonal_vector(p3d_vector3 v, p3d_vector3 result)
{
   #ifdef DEBUG
   if( p3d_vectNorm(v) < EPSILON )
   {
      printf("%s: %d: gpOrthogonal_vector(): bad input (vector of null norm).\n",__FILE__,__LINE__);
      result[0]= 1; result[1]= 0; result[2]= 0;
      return;
   }
   #endif

   if( fabs(v[2]) <= EPSILON )
   {
     result[0]= 0; result[1]= 0; result[2]= 1;
     return;
   }
   else
   {
      result[0]= 0;
      result[1]= 1;
      result[2]= -v[1]/v[2];
      p3d_vectNormalize(result, result);
      return;
   }
}

//! Cette fonction calcule les vecteurs v et w tels que (u,v,w) soit une base orthonormal directe.
//! Elle retourne un choix parmi l'infinite possible.
void gpOrthonormal_basis(p3d_vector3 u, p3d_vector3 v, p3d_vector3 w)
{
    p3d_vectNormalize(u, u);
    gpOrthogonal_vector(u, v);
    p3d_vectXprod(u, v, w);
    p3d_vectNormalize(v, v);
    p3d_vectNormalize(w, w);
}


//! Fonction calculant la distance entre un point "point" et le point C du triangle (p0p1p2)
//! tel que la distance de "point" a C soit minimale.
//! La fonction retourne cette distance et recopie les coordonnees de C dans "closestPoint".
//! Cette fonction est une adaptation (passage de C++ a C) de la fonction "DistVector3Triangle3"
//! de la bibliotheque "Wild Magic Library" (WM3), http://www.geometrictools.com de David Eberly.
double gpPoint_to_triangle_distance( p3d_vector3 point, p3d_vector3 p0, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 closestPoint)
{
    p3d_vector3 diff, edge0, edge1;
    p3d_vectSub(p0, point, diff);
    p3d_vectSub(p1, p0, edge0);
    p3d_vectSub(p2, p0, edge1);

    double fA00= p3d_vectDotProd(edge0, edge0);
    double fA01= p3d_vectDotProd(edge0, edge1);
    double fA11= p3d_vectDotProd(edge1, edge1);
    double fB0 = p3d_vectDotProd(diff, edge0);
    double fB1 = p3d_vectDotProd(diff, edge1);
    double fC = p3d_vectDotProd(diff, diff);

    double fDet = fabs(fA00*fA11-fA01*fA01);
    double fS = fA01*fB1-fA11*fB0;
    double fT = fA01*fB0-fA00*fB1;
    double fSqrDistance;


    if (fS + fT <= fDet)
    {
        if (fS < 0.0)
        {
            if (fT < 0.0)  // region 4
            {
                if (fB0 < 0.0)
                {
                    fT = 0.0;
                    if (-fB0 >= fA00)
                    {
                        fS = 1.0;
                        fSqrDistance = fA00+(2.0)*fB0+fC;
                    }
                    else
                    {
                        fS = -fB0/fA00;
                        fSqrDistance = fB0*fS+fC;
                    }
                }
                else
                {
                    fS = 0.0;
                    if (fB1 >= 0.0)
                    {
                        fT = 0.0;
                        fSqrDistance = fC;
                    }
                    else if (-fB1 >= fA11)
                    {
                        fT = 1.0;
                        fSqrDistance = fA11+(2.0)*fB1+fC;
                    }
                    else
                    {
                        fT = -fB1/fA11;
                        fSqrDistance = fB1*fT+fC;
                    }
                }
            }
            else  // region 3
            {
                fS = 0.0;
                if (fB1 >= 0.0)
                {
                    fT = 0.0;
                    fSqrDistance = fC;
                }
                else if (-fB1 >= fA11)
                {
                    fT = 1.0;
                    fSqrDistance = fA11+(2.0)*fB1+fC;
                }
                else
                {
                    fT = -fB1/fA11;
                    fSqrDistance = fB1*fT+fC;
                }
            }
        }
        else if (fT < 0.0)  // region 5
        {
            fT = 0.0;
            if (fB0 >= 0.0)
            {
                fS = 0.0;
                fSqrDistance = fC;
            }
            else if (-fB0 >= fA00)
            {
                fS = 1.0;
                fSqrDistance = fA00+(2.0)*fB0+fC;
            }
            else
            {
                fS = -fB0/fA00;
                fSqrDistance = fB0*fS+fC;
            }
        }
        else  // region 0
        {
            // minimum at interior point
            double fInvDet = (1.0)/fDet;
            fS *= fInvDet;
            fT *= fInvDet;
            fSqrDistance = fS*(fA00*fS+fA01*fT+(2.0)*fB0) +
                fT*(fA01*fS+fA11*fT+(2.0)*fB1)+fC;
        }
    }
    else
    {
        double fTmp0, fTmp1, fNumer, fDenom;

        if (fS < 0.0)  // region 2
        {
            fTmp0 = fA01 + fB0;
            fTmp1 = fA11 + fB1;
            if (fTmp1 > fTmp0)
            {
                fNumer = fTmp1 - fTmp0;
                fDenom = fA00-2.0f*fA01+fA11;
                if (fNumer >= fDenom)
                {
                    fS = 1.0;
                    fT = 0.0;
                    fSqrDistance = fA00+(2.0)*fB0+fC;
                }
                else
                {
                    fS = fNumer/fDenom;
                    fT = 1.0 - fS;
                    fSqrDistance = fS*(fA00*fS+fA01*fT+2.0f*fB0) +
                        fT*(fA01*fS+fA11*fT+(2.0)*fB1)+fC;
                }
            }
            else
            {
                fS = 0.0;
                if (fTmp1 <= 0.0)
                {
                    fT = 1.0;
                    fSqrDistance = fA11+(2.0)*fB1+fC;
                }
                else if (fB1 >= 0.0)
                {
                    fT = 0.0;
                    fSqrDistance = fC;
                }
                else
                {
                    fT = -fB1/fA11;
                    fSqrDistance = fB1*fT+fC;
                }
            }
        }
        else if (fT < 0.0)  // region 6
        {
            fTmp0 = fA01 + fB1;
            fTmp1 = fA00 + fB0;
            if (fTmp1 > fTmp0)
            {
                fNumer = fTmp1 - fTmp0;
                fDenom = fA00-(2.0)*fA01+fA11;
                if (fNumer >= fDenom)
                {
                    fT = 1.0;
                    fS = 0.0;
                    fSqrDistance = fA11+(2.0)*fB1+fC;
                }
                else
                {
                    fT = fNumer/fDenom;
                    fS = 1.0 - fT;
                    fSqrDistance = fS*(fA00*fS+fA01*fT+(2.0)*fB0) +
                        fT*(fA01*fS+fA11*fT+(2.0)*fB1)+fC;
                }
            }
            else
            {
                fT = 0.0;
                if (fTmp1 <= 0.0)
                {
                    fS = 1.0;
                    fSqrDistance = fA00+(2.0)*fB0+fC;
                }
                else if (fB0 >= 0.0)
                {
                    fS = 0.0;
                    fSqrDistance = fC;
                }
                else
                {
                    fS = -fB0/fA00;
                    fSqrDistance = fB0*fS+fC;
                }
            }
        }
        else  // region 1
        {
            fNumer = fA11 + fB1 - fA01 - fB0;
            if (fNumer <= 0.0)
            {
                fS = 0.0;
                fT = 1.0;
                fSqrDistance = fA11+(2.0)*fB1+fC;
            }
            else
            {
                fDenom = fA00-2.0f*fA01+fA11;
                if (fNumer >= fDenom)
                {
                    fS = 1.0;
                    fT = 0.0;
                    fSqrDistance = fA00+(2.0)*fB0+fC;
                }
                else
                {
                    fS = fNumer/fDenom;
                    fT = 1.0 - fS;
                    fSqrDistance = fS*(fA00*fS+fA01*fT+(2.0)*fB0) +
                        fT*(fA01*fS+fA11*fT+(2.0)*fB1)+fC;
                }
            }
        }
    }


    // account for numerical round-off error
    if (fSqrDistance < 0.0)
    {
        fSqrDistance = 0.0;
    }


    int i;
    for(i= 0; i<3; i++)
    { closestPoint[i]= p0[i] + fS*edge0[i] + fT*edge1[i]; }


    return sqrt(fSqrDistance);
}

//! Fonction d'affichage d'un plan (structure poly_plane).
//! a*x + b*y + c*z + d = 0
//! A utiliser dans une fonction d'affichage OpenGL.
void gpDraw_plane(poly_plane plane)
{
   #ifdef DEBUG
   if( fabs( p3d_vectNorm(plane.normale) - 1 ) > EPSILON )
   {  //printf("%s: %d: draw_plane(): erreur: la norme de poly_plane.normale est differente de 1 (%f)\n",__FILE__,__LINE__,   p3d_vectNorm(plane.normale));
      p3d_vectNormalize(plane.normale, plane.normale);
   }
   #endif
   p3d_vector3 c, u, v;
   p3d_vectScale(plane.normale, c, -plane.d); //c= -d*normale (point du plan le plus proche de l'origine)

   gpOrthonormal_basis(plane.normale, u, v);

   double s= 100;
   //g3d_set_color_mat(Blue, NULL);
   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);
   glDisable(GL_CULL_FACE);
   glBegin(GL_QUADS);
       glNormal3d(plane.normale[0], plane.normale[1], plane.normale[2]);
       glVertex3d( c[0] - s*u[0] - s*v[0], c[1] - s*u[1] - s*v[1], c[2] - s*u[2] - s*v[2] );
       glVertex3d( c[0] + s*u[0] - s*v[0], c[1] + s*u[1] - s*v[1], c[2] + s*u[2] - s*v[2] );
       glVertex3d( c[0] + s*u[0] + s*v[0], c[1] + s*u[1] + s*v[1], c[2] + s*u[2] + s*v[2] );
       glVertex3d( c[0] - s*u[0] + s*v[0], c[1] - s*u[1] + s*v[1], c[2] - s*u[2] + s*v[2] );
   glEnd();
   glEnable(GL_CULL_FACE);

   g3d_set_color_mat(Red, NULL);
   glBegin(GL_LINE);
       glVertex3d( c[0], c[1], c[2] );
       glVertex3d( c[0] + s*plane.normale[0], c[1] + s*plane.normale[1], c[2] + s*plane.normale[2] );
   glEnd();
}



//! Fonction d'affichage d'un plan (structure poly_plane) 
//! a*x + b*y + c*z + d = 0, sous forme de grille (carres de côte d).
//! A utiliser dans une fonction d'affichage OpenGL.
void gpDraw_plane2(poly_plane plane, double d)
{
   #ifdef DEBUG
   if( fabs( p3d_vectNorm(plane.normale) - 1 ) > EPSILON )
   {  //printf("%s: %d: draw_plane2(): erreur: la norme de poly_plane.normale est differente de 1 (%f)\n",__FILE__,__LINE__,   p3d_vectNorm(plane.normale));
      p3d_vectNormalize(plane.normale, plane.normale);
   }
   #endif
   int i;
   p3d_vector3 c, u, v;
   p3d_vectScale(plane.normale, c, -plane.d); //c= -d*normale (point du plan le plus proche de l'origine)

   gpOrthonormal_basis(plane.normale, u, v);

   int n= 50;

   glColor3f(0, 0, 0);
   glDisable(GL_LIGHTING);
   glBegin(GL_LINES);
     for(i= -n; i<n; i++)
     {
       glVertex3d( c[0] - n*d*u[0] + i*d*v[0], c[1] - n*d*u[1] + i*d*v[1], c[2] - n*d*u[2] + i*d*v[2] );
       glVertex3d( c[0] + n*d*u[0] + i*d*v[0], c[1] + n*d*u[1] + i*d*v[1], c[2] + n*d*u[2] + i*d*v[2] );
       glVertex3d( c[0] + i*d*u[0] - n*d*v[0], c[1] + i*d*u[1] - n*d*v[1], c[2] + i*d*u[2] - n*d*v[2] );
       glVertex3d( c[0] + i*d*u[0] + n*d*v[0], c[1] + i*d*u[1] + n*d*v[1], c[2] + i*d*u[2] + n*d*v[2] );
     }
   glEnd();

   glLineWidth(6);
   glColor3f(1, 0, 0);
   glBegin(GL_LINE);
       glVertex3d( c[0], c[1], c[2] );
       glVertex3d( c[0] + 20*d*plane.normale[0], c[1] + 20*d*plane.normale[1], c[2] + 20*d*plane.normale[2] );
   glEnd();

   glEnable(GL_LIGHTING);
}


//! Displays a plane as a grid (squares of length d)
//! normal_x*x + normal_y*y + normal_z*z + offset = 0, sous forme de grille (squares of length d).
//! \param normal plane's normal
//! \param offset plane's offset
//! \param d size of the grid's squares
//! Use in an OpenGL context.
void gpDraw_plane(p3d_vector3 normal, double offset, double d)
{
   #ifdef DEBUG
   if( fabs( p3d_vectNorm(normal) - 1 ) > EPSILON )
   {  
      p3d_vectNormalize(normal, normal);
   }
   #endif
   int i;
   p3d_vector3 c, u, v;
   p3d_vectScale(normal, c, -offset);

   gpOrthonormal_basis(normal, u, v);

   int n= 50;

   glColor3f(0, 0, 0);
   glDisable(GL_LIGHTING);
   glBegin(GL_LINES);
     for(i= -n; i<n; i++)
     {
       glVertex3d( c[0] - n*d*u[0] + i*d*v[0], c[1] - n*d*u[1] + i*d*v[1], c[2] - n*d*u[2] + i*d*v[2] );
       glVertex3d( c[0] + n*d*u[0] + i*d*v[0], c[1] + n*d*u[1] + i*d*v[1], c[2] + n*d*u[2] + i*d*v[2] );
       glVertex3d( c[0] + i*d*u[0] - n*d*v[0], c[1] + i*d*u[1] - n*d*v[1], c[2] + i*d*u[2] - n*d*v[2] );
       glVertex3d( c[0] + i*d*u[0] + n*d*v[0], c[1] + i*d*u[1] + n*d*v[1], c[2] + i*d*u[2] + n*d*v[2] );
     }
   glEnd();

   glLineWidth(6);
   glColor3f(1, 0, 0);
   glBegin(GL_LINE);
       glVertex3d( c[0], c[1], c[2] );
       glVertex3d( c[0] + 20*d*normal[0], c[1] + 20*d*normal[1], c[2] + 20*d*normal[2] );
   glEnd();

   glEnable(GL_LIGHTING);
}

//! Cette fonction calcule l'intersection entre deux droites dans le plan.
//! Ces droites sont caracterisees par un de leur point et leur direction.
//! Retourne le nombre d'intersections (0 ou 1).
int gpLine_line_intersection2D(double point1[2], double direction1[2], double point2[2], double direction2[2], double result[2])
{
   double lambda1;
   p3d_matrix2 M, invM;
   M[0][0]=   direction1[0];     M[0][1]=   -direction2[0];
   M[1][0]=   direction1[1];     M[1][1]=   -direction2[1];

   if( p3d_mat2Invert(M, invM)==0 )
     return 0;
   else
   {
      lambda1= invM[0][0]*(point2[0] - point1[0]) + invM[0][1]*(point2[1] - point1[1]);

      result[0]= point1[0] + lambda1*direction1[0];
      result[1]= point1[1] + lambda1*direction1[1];

      return 1;
   }
}


//! Cette fonction calcule l'intersection entre deux segments [a1b1] et [a2b2] dans le plan.
//! Retourne le nombre d'intersections (0, 1 ou 2).
//! Le cas 2 a lieu quand les deux segments sont paralleles et se recoupent partiellement.
//! Dans ce cas, les deux extremites de l'intersection sont recopies dans result1 et result2.
int gpSegment_segment_intersection2D(double a1[2], double b1[2], double a2[2], double b2[2], double result1[2], double result2[2])
{
   double lambda1, lambda2;
   double M[2][2], invM[2][2];

   // p= a1 + lambda1*(b1 - a1)
   // p= a2 + lambda2*(b2 - a2)

   // a1 + lambda1*(b1 - a1) = a2 + lambda2*(b2 - a2)

   // lambda1*(b1 - a1)  - lambda2*(b2 - a2)= a2 - a1


   M[0][0]=   b1[0]-a1[0];     M[0][1]=   -(b2[0]-a2[0]);
   M[1][0]=   b1[1]-a1[1];     M[1][1]=   -(b2[1]-a2[1]);


   if( (a1[0]<a2[0]+EPSILON) && (a1[0]<b2[0]+EPSILON) && (b1[0]<a2[0]+EPSILON) && (b1[0]<b2[0]+EPSILON) )
    return 0;

   if( (a1[0]>a2[0]-EPSILON) && (a1[0]>b2[0]-EPSILON) && (b1[0]>a2[0]-EPSILON) && (b1[0]>b2[0]-EPSILON) )
    return 0;

   if( (a1[1]<a2[1]+EPSILON) && (a1[1]<b2[1]+EPSILON) && (b1[1]<a2[1]+EPSILON) && (b1[1]<b2[1]+EPSILON) )
    return 0;
   if( (a1[1]>a2[1]-EPSILON) && (a1[1]>b2[1]-EPSILON) && (b1[1]>a2[1]-EPSILON) && (b1[1]>b2[1]-EPSILON) )
    return 0;



   if( p3d_mat2Invert(M, invM)==0 ) //si les droites des segments sont paralleles
   {

     if( gpIs_point_on_segment2D(a1,a2,b2) )
     {
       result1[0]= a1[0];
       result1[1]= a1[1];
       if( gpIs_point_on_segment2D(b1,a2,b2) )
       {
         result2[0]= b1[0];
         result2[1]= b1[1];

         if( sqrt( pow(result2[0]-result1[0],2) + pow(result2[1]-result1[1],2) ) < EPSILON )
           return 1;
         else
           return 2;
       }
       if( gpIs_point_on_segment2D(a2,a1,b1) )
       {
         result2[0]= a2[0];
         result2[1]= a2[1];
         if( sqrt( pow(result2[0]-result1[0],2) + pow(result2[1]-result1[1],2) ) < EPSILON )
           return 1;
         else
           return 2;
       }
     }

     if( gpIs_point_on_segment2D(a2,a1,b1) )
     {
       result1[0]= a2[0];
       result1[1]= a2[1];
       if( gpIs_point_on_segment2D(b2,a1,b1) )
       {
         result2[0]= b2[0];
         result2[1]= b2[1];
         if( sqrt( pow(result2[0]-result1[0],2) + pow(result2[1]-result1[1],2) ) < EPSILON )
           return 1;
         else
           return 2;
       }
       if( gpIs_point_on_segment2D(b1,a2,b2) )
       {
         result2[0]= b1[0];
         result2[1]= b1[1];
         if( sqrt( pow(result2[0]-result1[0],2) + pow(result2[1]-result1[1],2) ) < EPSILON )
           return 1;
         else
           return 2;
       }
     }

     if( gpIs_point_on_segment2D(b2,a1,b1) )
     {
       result1[0]= b2[0];
       result1[1]= b2[1];
       if( gpIs_point_on_segment2D(a1,a2,b2) )
       {
         result2[0]= a1[0];
         result2[1]= a1[1];
         if( sqrt( pow(result2[0]-result1[0],2) + pow(result2[1]-result1[1],2) ) < EPSILON )
           return 1;
         else
           return 2;
       }
       if( gpIs_point_on_segment2D(b1,a2,b2) )
       {
         result2[0]= b1[0];
         result2[1]= b1[1];
         if( sqrt( pow(result2[0]-result1[0],2) + pow(result2[1]-result1[1],2) ) < EPSILON )
           return 1;
         else
           return 2;
       }
     }

     return 0;
   }

   else
   {
      lambda1= invM[0][0]*(a2[0] - a1[0]) + invM[0][1]*(a2[1] - a1[1]);
      lambda2= invM[1][0]*(a2[0] - a1[0]) + invM[1][1]*(a2[1] - a1[1]);

      if( lambda1<0 || lambda1>1 || lambda2<0 || lambda2>1 )
        return 0;
      else
      {
        result1[0]= a1[0] + lambda1*(b1[0]-a1[0]);
        result1[1]= a1[1] + lambda1*(b1[1]-a1[1]);
        return 1;
      }
   }
}


//! Cette fonction teste si le point p est sur le segment[ab].
//! Elle retourne 1 si c'est le cas, 0 sinon.
int gpIs_point_on_segment2D(double p[2], double a[2], double b[2])
{
  // px= ax + lambda*(bx - ax)   -> lambda= (px-ax) / (bx-ax)
  // py= ay + lambda*(by - ay)   -> lambda= (py-ay) / (by-ay)

  if( fabs(b[0]-a[0]) < EPSILON )
  {
    if( fabs(p[0]-a[0]) > EPSILON )
      return 0;
    else
    {
      if( ( a[1]<=p[1] && p[1]<=b[1]) || ( b[1]<=p[1] && p[1]<=a[1]) )
        return 1;
      else
        return 0;
    }
  }

  if( fabs(b[1]-a[1]) < EPSILON )
  {
    if( fabs(p[1]-a[1]) > EPSILON )
      return 0;
    else
    {
      if( ( a[0]<=p[0] && p[0]<=b[0]) || ( b[0]<=p[0] && p[0]<=a[0]) )
        return 1;
      else
        return 0;
    }
  }

  if( fabs( (p[0]-a[0])/(b[0]-a[0]) - (p[1]-a[1])/(b[1]-a[1]) ) < EPSILON  )
    return 1;
  else
    return 0;

}


//! Cette fonction calcule et retourne l'aire du triangle (p1p2p3).
//! Elle utilise la formule de Heron d'Alexandrie.
double gpTriangle_area(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3)
{
    double a, b, c, s;
    p3d_vector3 p1p2, p2p3, p3p1;

    p3d_vectSub(p2, p1, p1p2);
    p3d_vectSub(p3, p2, p2p3);
    p3d_vectSub(p1, p3, p3p1);

    //calcul des longueurs des arêtes du triangle:
    a= p3d_vectNorm(p1p2);
    b= p3d_vectNorm(p2p3);
    c= p3d_vectNorm(p3p1);

    //calcul de l'aire du triangle
    s= ( a + b + c )/2;

    return sqrt( s*(s-a)*(s-b)*(s-c) );  //aire du triangle
}


//! Cette fonction calcule l'aire d'un polygone dont les sommets sont ranges comme suit:
//! ( vertices[0][0]  vertices[1][0] ... vertices[nb_vertices-1][0] ) = ( x1 x2 ... xn )
//! ( vertices[0][1]  vertices[1][1] ... vertices[nb_vertices-1][1] ) = ( y1 y2 ... yn )
//! L'aire A du polygone est donnee par la formule
//! A= 0.5 * (x1*y2 - y1*x2  +  x2*y3 - y2*x3  +  ...  +  xn*y1 - yn*x1 )
//! Elle est positive si les points sont donnes dans le sens trigonometrique, negative sinon.
//! Weisstein, Eric W. "Polygon Area." From MathWorld--A Wolfram Web Resource.
//! http://mathworld.wolfram.com/PolygonArea.html
double gpPolygon_area(double (*vertices)[2], int nb_vertices)
{
  int i;
  double area= 0;
  for(i=0; i<nb_vertices-1; i++)
  {
    area+= vertices[i][0]*vertices[i+1][1] - vertices[i][1]*vertices[i+1][0];
  }
  area+= vertices[nb_vertices-1][0]*vertices[0][1] - vertices[nb_vertices-1][1]*vertices[0][0];
  area *= 0.5;

  return area;
}

int gpSave_polygon(char *name, double (*vertices)[2], int nb_vertices)
{
   #ifdef DEBUG
   if(name==NULL || vertices==NULL)
   {
     printf("%s: %d: gpSave_polygon(): NULL input(s) (%p %p).\n",__FILE__,__LINE__,name,vertices);
     return 0;
   }
   #endif

   int i;
   FILE *file= NULL;
   file= fopen(name, "w");
   if(file==NULL)
   { printf("%s: %d: gpSave_polygon(): can not open file.\n",__FILE__,__LINE__);
     return 0;  }

   for(i= 0; i<nb_vertices; i++)
   {
    fprintf(file, "polygon1[%d][0]= %g; polygon1[%d][1]= %g;\n", i+1, vertices[i][0], i+1, vertices[i][1]);
   }

   fclose(file);

   return 1;
}

//! Cette fonction verifie si un polygone est simple ou non (un polygone est simple
//! si les seuls points d'intersection de ses arêtes sont ses sommets).
//! Les sommets sont ranges comme suit:
//! ( vertices[0][0]  vertices[1][0] ... vertices[nb_vertices-1][0] ) = ( x1 x2 ... xn )
//! ( vertices[0][1]  vertices[1][1] ... vertices[nb_vertices-1][1] ) = ( y1 y2 ... yn )
//! La fonction retourne 1 si le polygone est simple, 0 sinon.
int gpIs_polygon_simple(double (*vertices)[2], int nb_vertices)
{
  int i, j, k, nbinter;
  int i1, i2, j1, j2;
  int match_vertex;
  double inter[2],inter2[2], d;

  for(i=0; i<nb_vertices; i++)
  {
    for(j=i+2; j<nb_vertices; j++)
    {
      if( i==(nb_vertices-1) )
      {
         i1= nb_vertices-1;
         i2= 0;
      }
      else
      {
         i1= i;
         i2= i+1;
      }
      if( j==(nb_vertices-1) )
      {
         j1= nb_vertices-1;
         j2= 0;
      }
      else
      {
         j1= j;
         j2= j+1;
      }

      if( sqrt(pow(vertices[i1][0]-vertices[i2][0],2) + pow(vertices[i1][1]-vertices[i2][1],2) ) < EPSILON )
      {
        printf("%s: %d: gpIs_polygon_simple(): the polygon has a degenerate edge (its two vertices are identical).\n",__FILE__,__LINE__);
        return 0;
      }
      if( sqrt(pow(vertices[j1][0]-vertices[j2][0],2) + pow(vertices[j1][1]-vertices[j2][1],2) ) < EPSILON )
      {
        printf("%s: %d: gpIs_polygon_simple(): the polygon has a degenerate edge (its two vertices are identical).\n",__FILE__,__LINE__);
        return 0;
      }

      nbinter= gpSegment_segment_intersection2D(vertices[i1], vertices[i2], vertices[j1], vertices[j2], inter, inter2);

      if(nbinter==0)
        continue;
      else
      {
         if(nbinter==2)
         {

           printf("%s: %d: gpIs_polygon_simple(): an edge is included in another one.\n",__FILE__,__LINE__);
           printf("\t edge 1: x= %g y= %g x= %g y= %g \n",vertices[i1][0], vertices[i1][1], vertices[i2][0], vertices[i2][1]);
           printf("\t edge 2: x= %g y= %g x= %g y= %g \n",vertices[j1][0], vertices[j1][1], vertices[j2][0], vertices[j2][1]);
           printf("\t inter: x= %g y= %g x= %g y= %g \n",inter[0], inter[1], inter2[0], inter2[1]);

           return 0;
         }

         match_vertex= FALSE;
         for(k=0; k<nb_vertices; k++)
         {
           d= sqrt( pow(vertices[k][0]-inter[0],2) + pow(vertices[k][1]-inter[1],2) );
           if(d<=EPSILON)
           {
             match_vertex= TRUE;
             break;             }
         }
         if( match_vertex==FALSE )
         {
           printf("%s: %d: gpIs_polygon_simple(): two edges are crossing.\n",__FILE__,__LINE__);
           return 0;
         }
      }

    }

  }

  return 1;

}

//! Cette fonction teste si l'orientation d'un polygone est CCW (counterclockwise=sens direct) ou non
//! et retourne respectivement 1 ou 0.
//! Les sommets sont ranges comme suit:
//! ( vertices[0][0]  vertices[1][0] ... vertices[nb_vertices-1][0] ) = ( x1 x2 ... xn )
//! ( vertices[0][1]  vertices[1][1] ... vertices[nb_vertices-1][1] ) = ( y1 y2 ... yn )
inline int gpIs_polygon_ccw(double (*vertices)[2], int nb_vertices)
{
  if( gpPolygon_area(vertices, nb_vertices) > 0 ) return 1;
  else return 0;
}



//! Fonction testant l'inclusion d'un point dans un polygone. Les coordonnees des points du polygone
//! sont rangees dans le tableau vertices comme suit:
//! ( vertices[0][0]  vertices[1][0] ... vertices[nb_vertices-1][0] ) = ( x1 x2 ... xn )
//! ( vertices[0][1]  vertices[1][1] ... vertices[nb_vertices-1][1] ) = ( y1 y2 ... yn )
//! Code original de W Randolph Franklin du Rensselaer Polytechnic Institute.
//! NOTE: Remarque importante de l'auteur:
//! "My code uses the fact that, in the C language, when executing the code a&&b, if a is false,
//! then b must not be evaluated.
//! If your compiler doesn't do this, then it's not implementing C, and you will get a divide-by-zero, i.a.,
//! when the test point is vertically in line with a vertical edge. When translating this code to another
//! language with different semantics, then you must implement this test explicitly".
int gpIs_point_in_polygon(double point[2], double (*vertices)[2], int nb_vertices)
{
  int i, j, c = 0;
  for (i = 0, j = nb_vertices-1; i < nb_vertices; j = i++)
  {
    if ( ( (vertices[i][1]>point[1]) != (vertices[j][1]>point[1]) ) &&
	 (point[0] < (vertices[j][0]-vertices[i][0])*(point[1]-vertices[i][1]) / (vertices[j][1]-vertices[i][1]) + vertices[i][0])  )
      c = !c;
  }
  return c;
}


//! Retourne 1 si le polygone dont les sommets sont donnes a la suite dans vertices1 est inclus
//! dans le polygone dont les sommets sont donnes a la suite dans vertices2, 0 sinon.
int gpPolygon_polygon_inclusion(double (*vertices1)[2], int nb_vertices1, double (*vertices2)[2], int nb_vertices2)
{
  int i, i1, i2, j, j1, j2;
  int nbinter;
  double area1, area2;
  double inter1[2],inter2[2];

  area1= gpPolygon_area(vertices1, nb_vertices1);
  area2= gpPolygon_area(vertices2, nb_vertices2);

  if( fabs(area1) >= fabs(area2) )
  { return 0; }

  for(i=0; i<nb_vertices1; i++)
  {
     if( i==(nb_vertices1-1) )
     {
       i1= nb_vertices1-1;
       i2= 0;
     }
     else
     {
       i1= i;
       i2= i+1;
     }

     for(j=0; j<nb_vertices2; j++)
     {
        if( j==(nb_vertices2-1) )
        {
           j1= nb_vertices2-1;
           j2= 0;
        }
        else
        {
           j1= j;
           j2= j+1;
        }

        nbinter= gpSegment_segment_intersection2D(vertices1[i1], vertices1[i2], vertices2[j1], vertices2[j2], inter1, inter2);
        if(nbinter!=0)
        { return 0; }

     }
  }


  for(i=0; i<nb_vertices1; i++)
  {
    if( !gpIs_point_in_polygon(vertices1[i], vertices2, nb_vertices2) )
    { return 0; }
  }

  return 1;
}

//! Calcule le volume d'un tetraedre dont les sommets sont les points a, b, c, d.
//! On a les formules suivantes: volume= (1/6)*| det(a-b,b-c,c-d) |
//! et volume= (1/6)*| (a-b).((b-c)x(c-d)) |
double gpTetrahedron_volume(p3d_vector3 a, p3d_vector3 b, p3d_vector3 c, p3d_vector3 d)
{
   p3d_vector3 ba, cb, dc, cross;
   p3d_vectSub(a, b, ba);
   p3d_vectSub(b, c, cb);
   p3d_vectSub(c, d, dc);

   p3d_vectXprod(cb, dc, cross);

   return ( fabs( p3d_vectDotProd(ba, cross) ) / (6.0f) );
}


void gpSpherical_edge_projection(p3d_vector3 x1, p3d_vector3 x2, double a, p3d_vector3 result)
{
  double theta, k1, k2;
  p3d_vectNormalize(x1, x1);
  p3d_vectNormalize(x2, x2);
  theta= acos(p3d_vectDotProd(x1, x2) );
  k1= sin((1 - a)*theta)/sin(theta);
  k2= sin(a*theta)/sin(theta);
  result[0]= k1*x1[0] + k2*x2[0];
  result[1]= k1*x1[1] + k2*x2[1];
  result[2]= k1*x1[2] + k2*x2[2];
}


//! Calcule un echantillonnage de la surface de la sphere de rayon radius.
//! La fonction retourne un tableau de nb_samples points (alloue dans la fonction).
//! L'echantillonnage est calcule selon une methode de grille multi-resolution
//! (cf Deterministic Sampling Methods for Spheres and SO(3), A. Yershova, S. Lavalle, ICRA 2004).
//! \param nb_samples the number of samples to compute
//! \param radius the radius of the sphere 
//! \return a 3D point array of size "nb_samples"
p3d_vector3 *gpSample_sphere_surface(int nb_samples, double radius)
{
  int i, count;
  double factor;
  p3d_vector2 square_sample, origin;
  p3d_vector3 v[2], cube[8], *samples;

  cube[0][0]= -1;    cube[1][0]=  1;    cube[2][0]=  1;    cube[3][0]= -1;
  cube[0][1]= -1;    cube[1][1]= -1;    cube[2][1]=  1;    cube[3][1]=  1;
  cube[0][2]= -1;    cube[1][2]= -1;    cube[2][2]= -1;    cube[3][2]= -1;

  cube[4][0]= -1;    cube[5][0]=  1;    cube[6][0]=  1;    cube[7][0]= -1;
  cube[4][1]= -1;    cube[5][1]= -1;    cube[6][1]=  1;    cube[7][1]=  1;
  cube[4][2]=  1;    cube[5][2]=  1;    cube[6][2]=  1;    cube[7][2]=  1;


  samples= (p3d_vector3 *) malloc(nb_samples*sizeof(p3d_vector3));
  count= 0;
  
  origin[0]= 0.0;
  origin[1]= 0.0;
  factor= 1.0;

  int nb_iters= nb_samples;
  for(i=1; i<=nb_iters; i++)
  {
    get_sample2D(i, origin, factor, square_sample);

    gpSpherical_edge_projection(cube[1], cube[2], square_sample[0], v[0]); 
    gpSpherical_edge_projection(cube[5], cube[6], square_sample[0], v[1]); 
    gpSpherical_edge_projection(v[0], v[1], square_sample[1], samples[count]);
    samples[count][0]*= radius;
    samples[count][1]*= radius;
    samples[count][2]*= radius;
    count++;
    if(count>=nb_samples)
    { break; }

    gpSpherical_edge_projection(cube[6], cube[5], square_sample[0], v[0]); 
    gpSpherical_edge_projection(cube[7], cube[4], square_sample[0], v[1]); 
    gpSpherical_edge_projection(v[0], v[1], square_sample[1], samples[count]);
    samples[count][0]*= radius;
    samples[count][1]*= radius;
    samples[count][2]*= radius;
    count++;
    if(count>=nb_samples)
    { break; }
  
    gpSpherical_edge_projection(cube[0], cube[1], square_sample[0], v[0]); 
    gpSpherical_edge_projection(cube[3], cube[2], square_sample[0], v[1]); 
    gpSpherical_edge_projection(v[0], v[1], square_sample[1], samples[count]);
    samples[count][0]*= radius;
    samples[count][1]*= radius;
    samples[count][2]*= radius;
    count++;
    if(count>=nb_samples)
    { break; }

    gpSpherical_edge_projection(cube[7], cube[3], square_sample[0], v[0]); 
    gpSpherical_edge_projection(cube[4], cube[0], square_sample[0], v[1]); 
    gpSpherical_edge_projection(v[0], v[1], square_sample[1], samples[count]);
    samples[count][0]*= radius;
    samples[count][1]*= radius;
    samples[count][2]*= radius;
    count++;
    if(count>=nb_samples)
    { break; }
  
    gpSpherical_edge_projection(cube[4], cube[5], square_sample[0], v[0]); 
    gpSpherical_edge_projection(cube[0], cube[1], square_sample[0], v[1]); 
    gpSpherical_edge_projection(v[0], v[1], square_sample[1], samples[count]);
    samples[count][0]*= radius;
    samples[count][1]*= radius;
    samples[count][2]*= radius;
    count++;
    if(count>=nb_samples)
    { break; }
  
    gpSpherical_edge_projection(cube[2], cube[6], square_sample[0], v[0]); 
    gpSpherical_edge_projection(cube[3], cube[7], square_sample[0], v[1]); 
    gpSpherical_edge_projection(v[0], v[1], square_sample[1], samples[count]);
    samples[count][0]*= radius;
    samples[count][1]*= radius;
    samples[count][2]*= radius;
    count++;
    if(count>=nb_samples)
    { break; }

  }


  return samples;
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
p3d_vector3 *gpSample_triangle_surface(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, double step, int *nb_samples)
{
  unsigned int i, j, k, n1, n2, cnt, nb_allocs;
  double l1, l2, l3, dotp, du, dv;
  p3d_vector3 p1p2, p2p3, p1p3, p2p1, pipf;
  p3d_vector3 pi, pf, p1s, p2s, crossp1, crossp2, crossp3, crossp4, u, v, s;
  p3d_vector3 *samples= NULL;

  if( step <= 0 )
  {
    printf("%s: %d: gpSample_triangle_surface(): the \"step\" argument must be > 0.\n", __FILE__,__LINE__);
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
    printf("%s: %d: gpSample_triangle_surface(): the input triangle has a null length edge.\n", __FILE__,__LINE__);
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

