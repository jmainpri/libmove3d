/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */


#include "P3d-pkg.h"
#include "Util-pkg.h"
#include <stdio.h>
#include <math.h>
#include "GraspPlanning-pkg.h"
#include "glpk.h"



//! \deprecated {This function is not used anymore but the code might be of some interest.}
// Cette fonction calcule l'intersection d'un cône de frottement avec le plan de contact
// défini par la variable contactPlane.
// Le cône a pour sommet vertex (qui doit appartenir au plan) et pour normale "normal".
// Son coefficient de frottement est mu.
// Elle retourne le nombre de segment d'intersection (0, 1 ou 2), recopie en sortie la nouvelle normale
// (projection de l'ancienne normale sur le plan de contact) et le nouveau µ.
// La normale initiale doit être dirigée vers l'intérieur du cône.
int gpFriction_cone_contact_plane_intersection(p3d_vector3 vertex, p3d_vector3 normal, double mu, p3d_plane contactPlane, p3d_vector3 new_normal, double *new_mu)
{
   #ifdef GP_DEBUG
     if( fabs( p3d_vectDotProd(vertex, contactPlane.normale) + contactPlane.d ) > EPSILON )
     {
       printf("%s: %d: gpFriction_cone_contact_plane_intersection(): the cone vertex must belong to the plane.\n",__FILE__,__LINE__);
       return 0;
     }
   #endif

   p3d_vector3 center, p, u;

   p3d_vectNormalize(normal, normal);

   p3d_vectAdd( vertex, normal, center); //centre de la section (disque de rayon mu) perpendiculaire
                                         //à l'axe du cône à distance 1 de son sommet

   gpOrthogonal_projection_point_onto_plane(center, contactPlane, p);
   p3d_vectSub( p, vertex, p);
   p3d_vectNormalize(p, p);

   double dot= p3d_vectDotProd(p, normal);
   if( dot < EPSILON ) //Si le cône est perpendiculaire au plan des contacts
   {  return 0; }
   p3d_vectScale(p, p, 1.0/dot); //p est le milieu de l'intersection entre le disque et le plan de contact


   p3d_vectXprod( normal, contactPlane.normale, u);
   p3d_vectNormalize(u, u);


   double a;
   p3d_vector3 diff;
   p3d_vectSub( p, normal, diff);
   a= p3d_vectNorm(diff); //a est la distance entre p et le centre du disque


   //Le rayon du cône est "mu" quand on est à une distance
   //unité de son sommet.
   double x= mu*mu - a*a; //carré de la moitié  de la longueur de  l'intersection (segment) entre le disque et le plan
   double b;
   if( x < 0) //pas d'intersection entre le cône et le plan
   {
     return 0;
   }
   else
   {
     if( fabs(x) < EPSILON ) //intersection entre le cône et le plan: un seul segment
     {
       //p3d_vectCopy(p, n11);
       //p3d_vectNormalize(n11, n11);
       p3d_vectNormalize(p, new_normal);
       *new_mu= 0;
       return 1;
     }
     else
     {
         p3d_vectNormalize(p, new_normal);
         b= sqrt(x);
         *new_mu= b;
         return 2;
     }
   }

}


//! @ingroup graspPlanning 
//! Computes the quality of a 2D grasp.
//! The algorithm comes from an article by Belkacem Bounab:
//! "Central Axis Approach for Computing n-Finger Force-closure Grasps", proceedings of ICRA 2008, May 2008.
//!
//! ( contact[0][0] contact[1][0] contact[2][0] ... ) = ( C1x C2x C3x ... )
//! ( contact[0][1] contact[1][1] contact[2][1] ... )   ( C2y C2y C3y ... )
//!
//! ( normal[0][0] normal[1][0] normal[2][0] ... ) = ( N1x N2x N3x ... )
//! ( normal[0][1] normal[1][1] normal[2][1] ... )   ( N1y N2y N3y ... )
//!
//! Les frontières des cônes sont (V11, V12), (V21, V22), ..., (Vn1, Vn2)
//!
//! Le problème se ramène au problème d'optimisation linéaire suivant:
//!
//! minimize   a11 + a12 + a21 + a22 + ... + an1 + an2 - delta
//!    a
//!
//! subject to
//!          a11*V11 + a12*V12 + ... + an1*Vn1 + an2*Vn2 + delta*N1 = 0
//!          (C2 - C1)x(a21*V21 + a22*V22) + (C3 - C1)x(a31*V31 + a32*V32) + ... = 0
//!
//! avec a= ( a11, a12, a21, a22, ... ,an1, an2, delta )
//! et N la normale au premier point de contact.
//! The problem is solved with the GLPK (GNU Linear Programming Kit) library 
//! (http://www.gnu.org/software/glpk/)
//! \param position an array of dimensions [nbContacts][2] containing the contact positions (wrt the object's center of mass)
//! \param normal an array of dimensions [nbContacts][2] containing the contact normals
//! \param mu an array of dimensions [nbContacts] containing the contact friction coefficients
//! \param nbContacts the number of contacts of the grasp
//! \return the quality of the grasp (>0) if it is stable, 0 otherwise
double gpForce_closure_2D_grasp(double (*position)[2], double (*normal)[2], double mu[], unsigned int nbContacts)
{
  if(nbContacts < 2)
  {
    printf("%s: %d: gpForce_closure_2D_grasp(): at least 2 contacts are required for a 2D-grasp to statisfy the force closure property.\n",__FILE__,__LINE__);
    return 0;
  }

  unsigned int i;
  double epsil= 0.00001;

  //tableaux des bords des cônes de frottement:
  double (*vi1)[2]= (double (*)[2]) malloc(2*nbContacts*sizeof(double));
  double (*vi2)[2]= (double (*)[2]) malloc(2*nbContacts*sizeof(double));

  //calcul des bords des cônes de frottement
  double alpha, norm;
  for(i= 0; i<nbContacts; i++)
  {
    norm= normal[i][0]*normal[i][0] + normal[i][1]*normal[i][1];
    normal[i][0] /= norm;
    normal[i][1] /= norm;

    alpha= atan(mu[i]);
    vi1[i][0]=  cos(alpha)*normal[i][0] + sin(alpha)*normal[i][1];
    vi1[i][1]= -sin(alpha)*normal[i][0] + cos(alpha)*normal[i][1];
    norm= sqrt(1+mu[i]*mu[i]); // ne pas oublier que les bords ne sont pas de norme 1 mais de norme sqrt(1+mu*mu)
    vi1[i][0] *= norm;
    vi1[i][1] *= norm;

    vi2[i][0]=  cos(-alpha)*normal[i][0] + sin(-alpha)*normal[i][1];
    vi2[i][1]= -sin(-alpha)*normal[i][0] + cos(-alpha)*normal[i][1];
    vi2[i][0] *= norm; // ne pas oublier que les bords ne sont pas de norme 1 mais de norme sqrt(1+mu*mu)
    vi2[i][1] *= norm;
  }

  double sum, quality, delta;

  glp_prob *lp;
  glp_smcp parm;
  int nbNonZeroElements= 1+6*nbContacts; //nombre d'éléments non nuls de la matrice de contrainte
  int ia[nbNonZeroElements], ja[nbNonZeroElements];
  double ar[nbNonZeroElements];
  lp = glp_create_prob();
  glp_set_obj_dir(lp, GLP_MIN);

  //On rentre les limites des variables auxiliaires:
  glp_add_rows(lp, 3);
  glp_set_row_name(lp, 1, "fx");
  glp_set_row_bnds(lp, 1, GLP_FX, 0.0, 0.0); //contrainte sur les forces selon X
  glp_set_row_name(lp, 2, "fy");
  glp_set_row_bnds(lp, 2, GLP_FX, 0.0, 0.0); //contrainte sur les forces selon Y
  glp_set_row_name(lp, 3, "tz");
  glp_set_row_bnds(lp, 3, GLP_FX, 0.0, 0.0); //contrainte sur les moments par rapport au contact 1


  glp_add_cols(lp, 2*nbContacts+1);

  //On rentre les coefficients de la fonction à minimiser et les limites des variables structurelles:
    //attention: les indices commencent à 1 avec GLPK
    for(i=1; i<=2*nbContacts; i++)
    {
       glp_set_col_bnds(lp, i, GLP_DB, 0.0, 1.0); //limites sur les aii
       glp_set_obj_coef(lp, i, 1.0);
    }
    glp_set_col_bnds(lp, 2*nbContacts+1, GLP_DB, epsil, 1.0); //limites sur delta
    glp_set_obj_coef(lp, 2*nbContacts+1, -1.0);


  //On rentre les termes non nuls de la matrice de contrainte:
     int k= 1;
    //équations:
    // a11*V11x + a12*V12x + ... + am1*Vn1x + am2*Vn2x + delta*Nx = 0
    // a11*V11y + a12*V12y + ... + am1*Vn1y + am2*Vn2y + delta*Ny = 0
    //attention: les indices commencent à 1 avec GLPK
      for(i=1; i<=nbContacts; i++)
      {
          // X:
          ia[k] = 1;    ja[k] = 2*i-1;     ar[k]=  vi1[i-1][0];
          k++;
          ia[k] = 1;    ja[k] = 2*i;       ar[k]=  vi2[i-1][0];
          k++;

          // Y:
          ia[k] = 2;    ja[k] = 2*i-1;     ar[k]=  vi1[i-1][1];
          k++;
          ia[k] = 2;    ja[k] = 2*i;       ar[k]=  vi2[i-1][1];
          k++;

      }
      ia[k]   = 1;      ja[k] = 2*nbContacts+1;     ar[k]=  normal[0][0];
      k++;
      ia[k]   = 2;      ja[k] = 2*nbContacts+1;     ar[k]=  normal[0][1];
      k++;


    //équation:
      // (C2 - C1)x(a21*V21 + a22*V22) + (C3 - C1)x(a31*V31 + a32*V32) + ... = 0
      double c1cix, c1ciy;
      for(i=2; i<=nbContacts; i++)
      {
          c1cix= position[i-1][0] - position[0][0];
          c1ciy= position[i-1][1] - position[0][1];
          // (Ci-C1) x Vi1
          ia[k] = 3;    ja[k] = 2*i-1;
          ar[k]= c1cix*vi1[i-1][1] - c1ciy*vi1[i-1][0] ;
          k++;
          // (Ci-C1) x Vi2
          ia[k] = 3;    ja[k] = 2*i;
          ar[k]= c1cix*vi2[i-1][1] - c1ciy*vi2[i-1][0] ;
          k++;

      }

  glp_load_matrix(lp, k-1, ia, ja, ar);
  int result;

  glp_init_smcp(&parm);
  parm.msg_lev= GLP_MSG_ERR; // pour que seuls les messages d'erreur soient affichés

  result= glp_simplex(lp, &parm);

  int status= glp_get_status(lp);

/*
  switch(status)
  {
     case 0:           printf("0\n");            break;
     case GLP_OPT:     printf("GLP_OPT\n");      break;
     case GLP_FEAS:    printf("GLP_FEAS\n");     break;
     case GLP_INFEAS:  printf("GLP_INFEAS\n");   break;
     case GLP_NOFEAS:  printf("GLP_NOFEAS\n");   break;
     case GLP_UNBND:   printf("GLP_UNBND\n");    break;
     case GLP_UNDEF:   printf("GLP_UNDEF\n");    break;
     default:          printf("???\n");          break;
  }
*/

  if( status!=GLP_OPT && status!=GLP_FEAS ) //problème infaisable -> prise non force closure
  {
       glp_delete_prob(lp);
       glp_free_env();
       free(vi1);
       free(vi2);
       return 0;
  }
  else
  {
       sum= 0;
       for(i=1; i<=2*nbContacts; i++) //calcul de la somme des aii
       {
          sum += glp_get_col_prim(lp, i);
          //printf("\ta%i= %g\n", i, glp_get_col_prim(lp, i));
       }
       delta = glp_get_col_prim(lp, 2*nbContacts+1);
       quality= delta / sum;

  ////////////////pour vérifier que les contraintes sont bien respectées//////////////////////////////
  ////////////////à commenter ou décommenter/////////////////////////////////////////////////////////
/*
       double sum1= 0, sum2= 0, sum3= 0;
       for(i=1; i<=nbContacts; i++)
       {
         sum1 += glp_get_col_prim(lp, 2*i-1)*vi1[i-1][0] + glp_get_col_prim(lp, 2*i)*vi2[i-1][0];
         sum2 += glp_get_col_prim(lp, 2*i-1)*vi1[i-1][1] + glp_get_col_prim(lp, 2*i)*vi2[i-1][1];
       }
       for(i=2; i<=nbContacts; i++)
       {
         c1cix= position[i-1][0] - position[0][0];
         c1ciy= position[i-1][1] - position[0][1];

         sum3 += ( c1cix*vi1[i-1][1] - c1ciy*vi1[i-1][0] )*glp_get_col_prim(lp, 2*i-1);
         sum3 += ( c1cix*vi2[i-1][1] - c1ciy*vi2[i-1][0] )*glp_get_col_prim(lp, 2*i);
       }
       sum1+= normal[0][0]*delta;
       sum2+= normal[0][1]*delta;

       for(i=1; i<=nbContacts; i++)
       {
          printf("a%d1= %g a%i2=%g \n", i, glp_get_col_prim(lp, 2*i-1), i, glp_get_col_prim(lp, 2*i));
       }
       printf("delta= %g\n", glp_get_col_prim(lp, 2*nbContacts+1));
       printf("sums %g %g %g\n", sum1, sum2, sum3);
*/

  /////////////////////////////////////////////////////////////////////////////


       glp_delete_prob(lp);
       glp_free_env();
       free(vi1);
       free(vi2);

       return quality;
  }



}

//! @ingroup graspPlanning 
//! Computes the quality of a 3D grasp.
//! The algorithm comes from an article by Belkacem Bounab:
//! "Central Axis Approach for Computing n-Finger Force-closure Grasps", proceedings of ICRA 2008, May 2008.
//!
//! ( contact[0][0] contact[1][0] contact[2][0] ... )   ( C1x C2x C3x ... )
//! ( contact[0][1] contact[1][1] contact[2][0] ... ) = ( C2y C2y C3y ... )
//! ( contact[0][2] contact[1][2] contact[2][0] ... )   ( C2z C2z C3z ... )
//!
//! ( normal[0][0] normal[1][0] normal[2][0] ... )   ( N1x N2x N3x ... )
//! ( normal[0][1] normal[1][1] normal[2][0] ... ) = ( N1y N2y N3y ... )
//! ( normal[0][2] normal[1][2] normal[2][0] ... )   ( N1z N2z N3z ... )
//!
//! Les arêtes des cônes de frottement discrétisés en m segments chacun sont
//! (  V11x, V12x, ..., V1mx, V21x, V22x, ..., V2mx, ..., Vn1x, Vn2x, ..., Vnmx  )
//! (  V11y, V12y, ..., V1my, V21y, V22y, ..., V2my, ..., Vn1y, Vn2y, ..., Vnmy  )
//! (  V11z, V12z, ..., V1mz, V21z, V22z, ..., V2mz, ..., Vn1z, Vn2z, ..., Vnmz  )
//!
//! Le problème se ramène au problème d'optimisation linéaire suivant:
//!
//! minimize   a11 + a12 + a21 + a22 + ... + am1 + am2 - delta
//!    a
//!
//! subject to
//!          sum(j=1..m)(a1j*V1j) + sum(j=1..m)(a2j*V2j) + ... + sum(j=1..m)(anj*Vnj) + delta*N1 = 0
//!
//!         sum(j=1..m)(C2 - C1)x(a2j*V2j) + sum(j=1..m)(C3 - C1)x(a3j*V3j) + ...+ sum(j=1..m)(Cn - C1)x(anj*Vnj) = 0
//!
//! avec a= ( a11, a12,..., a1m, a21, a22,..., a2m, ... ,anm, delta )
//! The problem is solved with the GLPK (GNU Linear Programming Kit) library 
//! (http://www.gnu.org/software/glpk/)
//! \param position an array of dimensions [nbContacts][3] containing the contact positions (wrt the object's center of mass)
//! \param normal an array of dimensions [nbContacts][3] containing the contact normals
//! \param mu an array of dimensions [nbContacts] containing the contact friction coefficients
//! \param nbContacts the number of contacts of the grasp
//! \param nbSlices number of segments of the linearized friction cones
//! \return the quality of the grasp (>0) if it is stable, 0 otherwise
double gpForce_closure_3D_grasp(double (*position)[3], double (*normal)[3], double mu[], unsigned int nbContacts, unsigned int nbSlices)
{
  if(nbContacts < 3)
  {
    printf("%s: %d: gpForce_closure_3D_grasp(): at least 3 contacts are required for a 3D-grasp to statisfy the force closure property.\n",__FILE__,__LINE__);
    return 0;
  }

  unsigned int i, j;
  double epsil= 0.0001;
  double sum, delta, quality;

  //tableau des arêtes des cônes de frottement linéarisés:
  double (*vij)[3]= (double (*)[3]) malloc(3*nbSlices*nbContacts*sizeof(double));

  p3d_vector3 axis, normal_i, u;
  p3d_matrix3 R, Ri, Rtmp;
  double norm;

  //linéarisation des cônes de frottement
  for(i= 0; i<nbContacts; i++)
  {
    normal_i[0]= normal[i][0];
    normal_i[1]= normal[i][1];
    normal_i[2]= normal[i][2];

    p3d_vectNormalize(normal_i, normal_i);
    gpOrthogonal_vector(normal_i, axis);

    p3d_mat3Rot(R, axis, atan(mu[i]));
    p3d_vec3Mat3Mult(R, normal_i, u);
    p3d_mat3Rot(Ri, normal_i, 2*M_PI/nbSlices);
    norm= sqrt( 1 + mu[i]*mu[i] ); //ne pas oublier que les arêtes du cône linéarisé sont de norme sqrt( 1 + mu*mu )
    for(j=0; j<nbSlices; j++)
    {
      vij[nbSlices*i + j][0]= norm*u[0];
      vij[nbSlices*i + j][1]= norm*u[1];
      vij[nbSlices*i + j][2]= norm*u[2];
      p3d_mat3Mult( Ri, R, Rtmp );
      p3d_mat3Copy ( Rtmp, R );
      p3d_vec3Mat3Mult(R, normal_i, u);
    }
  }

  glp_prob *lp;
  glp_smcp parm;
  int nbNonZeroElements= 1+6*nbSlices*nbContacts - 3*nbSlices + 3; //nombre d'éléments non nuls de la matrice de contrainte
  int ia[nbNonZeroElements], ja[nbNonZeroElements];
  double ar[nbNonZeroElements];
  lp = glp_create_prob();
  glp_set_obj_dir(lp, GLP_MIN);

  //On rentre les limites des variables auxiliaires:
  glp_add_rows(lp, 6);
  glp_set_row_name(lp, 1, "fx");
  glp_set_row_bnds(lp, 1, GLP_FX, 0.0, 0.0); //contrainte sur les forces selon X
  glp_set_row_name(lp, 2, "fy");
  glp_set_row_bnds(lp, 2, GLP_FX, 0.0, 0.0); //contrainte sur les forces selon Y
  glp_set_row_name(lp, 3, "fz");
  glp_set_row_bnds(lp, 3, GLP_FX, 0.0, 0.0); //contrainte sur les forces selon Z
  glp_set_row_name(lp, 4, "tx");
  glp_set_row_bnds(lp, 4, GLP_FX, 0.0, 0.0); //contrainte sur les moments autour de X par rapport au contact 1
  glp_set_row_name(lp, 5, "ty");
  glp_set_row_bnds(lp, 5, GLP_FX, 0.0, 0.0); //contrainte sur les moments autour de Y par rapport au contact 1
  glp_set_row_name(lp, 6, "tz");
  glp_set_row_bnds(lp, 6, GLP_FX, 0.0, 0.0); //contrainte sur les moments autour de Z par rapport au contact 1



  glp_add_cols(lp, nbSlices*nbContacts+1);

  //On rentre les coefficients de la fonction f à minimiser et les limites des variables structurelles:
  //f= a11 + a12 + a21 + a22 + ... + am1 + am2 - delta
    //attention: les indices commencent à 1 avec GLPK
    for(i=1; i<=nbSlices*nbContacts; i++)
    {
       glp_set_col_bnds(lp, i, GLP_DB, 0.0, 1.0); //limites sur les aii
       glp_set_obj_coef(lp, i, 1.0);
    }
    glp_set_col_bnds(lp, nbSlices*nbContacts+1, GLP_DB, epsil, 1.0); //limites sur delta
    glp_set_obj_coef(lp, nbSlices*nbContacts+1, -1.0);


  //On rentre les termes non nuls de la matrice de contrainte:
     int k= 1;
    //équations:
    //sum(j=1..m)(a1j*V1jx) + sum(j=1..m)(a2j*V2jx) + ... + sum(j=1..m)(anj*Vnjx) + delta*N1x = 0
    //sum(j=1..m)(a1j*V1jy) + sum(j=1..m)(a2j*V2jy) + ... + sum(j=1..m)(anj*Vnjy) + delta*N1y = 0
    //sum(j=1..m)(a1j*V1jz) + sum(j=1..m)(a2j*V2jz) + ... + sum(j=1..m)(anj*Vnjz) + delta*N1z = 0
    //attention: les indices commencent à 1 avec GLPK
      for(i=0; i<nbContacts; i++)
      {
          for(j= 1; j<=nbSlices; j++)
          {
              // X:
              ia[k] = 1;    ja[k] = nbSlices*i + j;     ar[k]=  vij[nbSlices*i + j - 1][0];
              k++;
              // Y:
              ia[k] = 2;    ja[k] = nbSlices*i + j;     ar[k]=  vij[nbSlices*i + j - 1][1];
              k++;
              // Z:
              ia[k] = 3;    ja[k] = nbSlices*i + j;     ar[k]=  vij[nbSlices*i + j - 1][2];
              k++;

          }
      }
      ia[k]   = 1;      ja[k] = nbSlices*nbContacts + 1;     ar[k]=  normal[0][0];
      k++;
      ia[k]   = 2;      ja[k] = nbSlices*nbContacts + 1;     ar[k]=  normal[0][1];
      k++;
      ia[k]   = 3;      ja[k] = nbSlices*nbContacts + 1;     ar[k]=  normal[0][2];
      k++;

    //équations:
      // [ sum(j=1..m)(C2 - C1)x(a2j*V2j) + sum(j=1..m)(C3 - C1)x(a3j*V3j) + ...+ sum(j=1..m)(Cn - C1)x(anj*Vnj) ] . X = 0
      // [ sum(j=1..m)(C2 - C1)x(a2j*V2j) + sum(j=1..m)(C3 - C1)x(a3j*V3j) + ...+ sum(j=1..m)(Cn - C1)x(anj*Vnj) ] . Y = 0
      // [ sum(j=1..m)(C2 - C1)x(a2j*V2j) + sum(j=1..m)(C3 - C1)x(a3j*V3j) + ...+ sum(j=1..m)(Cn - C1)x(anj*Vnj) ] . Z = 0
      double C1Cix, C1Ciy, C1Ciz;
      for(i=1; i<nbContacts; i++)
      {
          C1Cix= position[i][0] - position[0][0];
          C1Ciy= position[i][1] - position[0][1];
          C1Ciz= position[i][2] - position[0][2];
          for(j=1; j<=nbSlices; j++)
          {
              // X:
              ia[k] = 4;    ja[k] = nbSlices*i + j;
              ar[k]= C1Ciy*vij[nbSlices*i + j - 1][2] - C1Ciz*vij[nbSlices*i + j - 1][1];
              k++;

              // Y:
              ia[k] = 5;    ja[k] = nbSlices*i + j;
              ar[k]= C1Ciz*vij[nbSlices*i + j - 1][0] - C1Cix*vij[nbSlices*i + j - 1][2];
              k++;

              // Z:
              ia[k] = 6;    ja[k] = nbSlices*i + j;
              ar[k]= C1Cix*vij[nbSlices*i + j - 1][1] - C1Ciy*vij[nbSlices*i + j - 1][0];
              k++;

          }
      }

  glp_load_matrix(lp, k-1, ia, ja, ar);
  int result;

  glp_init_smcp(&parm);
  parm.msg_lev= GLP_MSG_ERR; // pour que seuls les messages d'erreur soient affichés

  result= glp_simplex(lp, &parm);

  int status= glp_get_status(lp);
//   switch(status)
//   {
//      case 0:           printf("0\n");            break;
//      case GLP_OPT:     printf("GLP_OPT\n");      break;
//      case GLP_FEAS:    printf("GLP_FEAS\n");     break;
//      case GLP_INFEAS:  printf("GLP_INFEAS\n");   break;
//      case GLP_NOFEAS:  printf("GLP_NOFEAS\n");   break;
//      case GLP_UNBND:   printf("GLP_UNBND\n");    break;
//      case GLP_UNDEF:   printf("GLP_UNDEF\n");    break;
//      default:          printf("???\n");          break;
//   }


  if( status!=GLP_OPT && status!=GLP_FEAS ) //problème infaisable -> prise non force closure
  {
       glp_delete_prob(lp);
       glp_free_env();
       free(vij);
       return 0;
  }
  else
  {

       sum= 0;
       for(i=1; i<=nbSlices*nbContacts; i++) //calcul de la somme des aii
       {
          sum += glp_get_col_prim(lp, i);
          //printf("\ta%i= %g\n", i, glp_get_col_prim(lp, i));
       }
       delta = glp_get_col_prim(lp, nbSlices*nbContacts+1);
       quality= delta / sum;
       //printf("\tdelta= %g \n", delta);

  ////////////////pour vérifier que les contraintes sont bien respectées//////////////////////////////
  ///////////////à commenter ou décommenter///////////////////////////////////////////////////////////
//        double sum1= 0, sum2= 0, sum3= 0, sum4= 0, sum5= 0, sum6= 0;
//        for(i=1; i<=nbSlices*nbContacts; i++)
//        {
//          sum1 += glp_get_col_prim(lp, i)*vij[i-1][0];
//          sum2 += glp_get_col_prim(lp, i)*vij[i-1][1];
//          sum3 += glp_get_col_prim(lp, i)*vij[i-1][2];
//        }
//        for(i=1; i<nbContacts; i++)
//        {
//           C1Cix= position[i][0] - position[0][0];
//           C1Ciy= position[i][1] - position[0][1];
//           C1Ciz= position[i][2] - position[0][2];
// 
//           for(j=1; j<=nbSlices; j++)
//           {
//             sum4 += ( C1Ciy*vij[nbSlices*i + j - 1][2] - C1Ciz*vij[nbSlices*i + j - 1][1] ) * glp_get_col_prim(lp, nbSlices*i + j);
//             sum5 += ( C1Ciz*vij[nbSlices*i + j - 1][0] - C1Cix*vij[nbSlices*i + j - 1][2] ) * glp_get_col_prim(lp, nbSlices*i + j);
//             sum6 += ( C1Cix*vij[nbSlices*i + j - 1][1] - C1Ciy*vij[nbSlices*i + j - 1][0] ) * glp_get_col_prim(lp, nbSlices*i + j);
//           }
//        }
// 
//        sum1+= normal[0][0]*delta;
//        sum2+= normal[0][1]*delta;
//        sum3+= normal[0][2]*delta;
//        printf(" sums %f %f %f %f %f %f\n", sum1, sum2, sum3, sum4, sum5, sum6);
  /////////////////////////////////////////////////////////////////////////////


       glp_delete_prob(lp);
       glp_free_env();
       free(vij);

       return quality;
  }

}


//! @ingroup graspPlanning 
//! Tests the force closure property of a 2D grasp.
//! \param position an array of dimensions [nbContacts][2] containing the contact positions (wrt the object's center of mass)
//! \param normal an array of dimensions [nbContacts][2] containing the contact normals
//! \param mu an array of dimensions [nbContacts] containing the contact friction coefficients
//! \param nbContacts the number of contacts of the grasp
//! \return the quality of the grasp (>0) if it is stable, 0 otherwise
double gpForce_closure_2D_grasp2(double (*position)[2], double (*normal)[2], double mu[], unsigned int nbContacts)
{
  if(nbContacts < 2)
  {
    printf("%s: %d: gpForce_closure_2D_grasp(): at least 2 contacts are required for a 2D-grasp to statisfy the force closure property.\n",__FILE__,__LINE__);
    return 0;
  }

  unsigned int i, nb_points;
  int result;
  double torque;
  double (*vi1)[2]= NULL, (*vi2)[2]= NULL;
  double (*point_array)[3]= NULL;
  gpConvexHull3D *chull= NULL;

  nb_points= 2*nbContacts;

  //arrays containing the bounds of the friction cones:
  vi1= (double (*)[2]) malloc(2*nbContacts*sizeof(double));
  vi2= (double (*)[2]) malloc(2*nbContacts*sizeof(double));

  //compute the the bounds of the friction cones:
  double alpha, norm;
  for(i= 0; i<nbContacts; i++)
  {
    norm= normal[i][0]*normal[i][0] + normal[i][1]*normal[i][1];
    normal[i][0] /= norm;
    normal[i][1] /= norm;

    alpha= atan(mu[i]);
    vi1[i][0]=  cos(alpha)*normal[i][0] + sin(alpha)*normal[i][1];
    vi1[i][1]= -sin(alpha)*normal[i][0] + cos(alpha)*normal[i][1];
    norm= sqrt(1+mu[i]*mu[i]); // do not forget that the norms of linearized cone edges are sqrt( 1 + mu*mu )
    vi1[i][0] *= norm;
    vi1[i][1] *= norm;

    vi2[i][0]=  cos(-alpha)*normal[i][0] + sin(-alpha)*normal[i][1];
    vi2[i][1]= -sin(-alpha)*normal[i][0] + cos(-alpha)*normal[i][1];
    vi2[i][0] *= norm; // do not forget that the norms of linearized cone edges are sqrt( 1 + mu*mu )
    vi2[i][1] *= norm;

    torque= position[i][1]*vi1[i][0] - position[i][0]*vi1[i][1];

    point_array[2*i][0]= vi1[i][0];
    point_array[2*i][1]= vi1[i][1];
    point_array[2*i][2]= torque;

    torque= position[i][1]*vi2[i][0] - position[i][0]*vi2[i][1];

    point_array[2*i+1][0]= vi2[i][0];
    point_array[2*i+1][1]= vi2[i][1];
    point_array[2*i+1][2]= torque;
  }

  chull= new gpConvexHull3D(point_array, nb_points);

  chull->compute(false, false);
 
  result= (chull->largest_ball_radius() > 1e-7);

  delete chull;
  
  return result;
}


//! @ingroup graspPlanning 
//! Tests the force closure property of a 3D grasp.
//! \param position an array of dimensions [nbContacts][3] containing the contact positions (wrt the object's center of mass)
//! \param normal an array of dimensions [nbContacts][3] containing the contact normals
//! \param mu an array of dimensions [nbContacts] containing the contact friction coefficients
//! \param nbContacts the number of contacts of the grasp
//! \param nbSlices number of segments of the linearized friction cones
//! \return 1 if the grasp verifies the force closure property, 0 otherwise
int gpForce_closure_3D_grasp2(double (*position)[3], double (*normal)[3], double mu[], unsigned int nbContacts, unsigned int nbSlices)
{
  if(nbContacts < 3)
  {
    printf("%s: %d: gpForce_closure_3D_grasp(): at least 3 contacts are required for a 3D-grasp to statisfy the force closure property.\n",__FILE__,__LINE__);
    return 0;
  }

  int result;
  unsigned int i, j, k, nb_points= 0;
  double norm, (*vij)[3]= NULL;
  p3d_vector3 axis, normal_i, u, torque;
  p3d_matrix3 R, Ri, Rtmp;
  double (*point_array)[6]= NULL;
  gpConvexHull6D *chull= NULL;

  nb_points= nbContacts*nbSlices;

  // array of the edges of the linearized friction cones:
  vij= (double (*)[3]) malloc(3*nb_points*sizeof(double));

  // array of the elementary wrenches:
  point_array= (double (*)[6]) malloc(6*nb_points*sizeof(double));


  //linearize the friction cones and compute the elementary wrenches:
  for(i= 0; i<nbContacts; i++)
  {
    normal_i[0]= normal[i][0];
    normal_i[1]= normal[i][1];
    normal_i[2]= normal[i][2];

    p3d_vectNormalize(normal_i, normal_i);
    gpOrthogonal_vector(normal_i, axis);

    p3d_mat3Rot(R, axis, atan(mu[i]));
    p3d_vec3Mat3Mult(R, normal_i, u);
    p3d_mat3Rot(Ri, normal_i, 2*M_PI/nbSlices);
    norm= sqrt( 1 + mu[i]*mu[i] ); // do not forget that the norms of linearized cone edges are sqrt( 1 + mu*mu )
    for(j=0; j<nbSlices; j++)
    {
      k= nbSlices*i + j;

      vij[k][0]= norm*u[0];
      vij[k][1]= norm*u[1];
      vij[k][2]= norm*u[2];
      p3d_mat3Mult( Ri, R, Rtmp );
      p3d_mat3Copy ( Rtmp, R );
      p3d_vec3Mat3Mult(R, normal_i, u);

      //force:
      point_array[k][0]= vij[k][0];
      point_array[k][1]= vij[k][1];
      point_array[k][2]= vij[k][2];

      //torque:
      p3d_vectXprod(position[i], vij[k], torque);
      point_array[k][3]= torque[0];
      point_array[k][4]= torque[1];
      point_array[k][5]= torque[2];
    }
  }

  chull= new gpConvexHull6D(point_array, nb_points);

  if(chull->compute(false, 0, false)==0)
  {    result= 0;    }
  else
  {    result= (chull->largest_ball_radius() > 1e-7);   }

  delete chull;
  
  return result;
}
