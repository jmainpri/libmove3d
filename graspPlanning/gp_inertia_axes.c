/* Module for the generation of buttons and functions callbacks*/
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "GraspPlanning-pkg.h"

#include <math.h>
#include <stdio.h>
#include <string.h>


//Macros
#define SQR(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))



/*****************************************************************************/
static int gauss_jordan (p3d_matrix3 a, int n, p3d_vector3 sol)
/*---------------------------------------------------------------------------*
 * FONCTION : Elimination de Gauss-Jordan avec pivotage total.               *
 * NOTA.    : Cette routine est basee sur 'gaussj()' de "Numerical Recipes". *
 *            Elle a ete arrangee pour accepter 'b' = vecteur et non plus une*
 *            une matrice.                                                   *
 * RETOUR   : 1 Singular Matrix-1                                            *
 *            2 Singular Matrix-2                                            *
 *            0 OK                                                           *
 *---------------------------------------------------------------------------*/

{
#define _SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
  //   int    *indxc, *indxr, *ipiv;
   int     i, j, k, l, ll;
   int     icol = 0;
   int     irow = 0;
   double  big, dum, pivinv, temp;
  

   int indxc[3];
   int indxr[3];
   int ipiv[3];
   for (j=0; j<n; j++) ipiv[j] = 0;
	
   //-- Boucle principale sur les colonnes a reduire. -----------------------
   for (i=0; i<n; i++) 
    {
     big=0.0;
     
     //-- Boucle de recherche de l'element pivot. ---------------------------
     for (j=0; j<n; j++)
      if (ipiv[j] != 1)
       for (k=0; k<n; k++) 
        {
	 if (ipiv[k] == 0) 
          {
	   if (fabs(a[j][k]) >= big) 
            {
	     big  = fabs (a[j][k]);
	     irow = j;
	     icol = k;
	    } 
          }
         else if (ipiv[k] > 1) 
          {
	
           return(1);
          }
	}
       ++(ipiv[icol]);

     if (irow != icol) 
        {
         for (l=0; l<n; l++) _SWAP(a[irow][l], a[icol][l])
         _SWAP(sol[irow], sol[icol])
        }
       indxr[i] = irow;
       indxc[i] = icol;
       if (a[icol][icol] == 0.0) 
	{
       
        return(2);
        }
       pivinv = 1.0 / a[icol][icol];
       a[icol][icol] = 1.0;
       for (l=0; l<n; l++) a[icol][l] *= pivinv;
       sol[icol] *= pivinv;
       for (ll=0;ll<n;ll++)
        if (ll != icol) 
         {
          dum = a[ll][icol];
          a[ll][icol] = 0.0;
          for (l=0; l<n; l++) a[ll][l] -= a[icol][l] * dum;
          sol[ll] -= sol[icol] * dum;
         }
      }
     
   for (l=n-1; l>=0; l--) 
    {
     if (indxr[l] != indxc[l])
      for (k=0; k<n; k++)
       _SWAP(a[k][indxr[l]], a[k][indxc[l]]);
    }
  
   return(0);

#undef _SWAP
}



// Fonction tirée de "Numerical Recipes in C" chapter 11-1.
// Attention: la matrice d'origine est modifiée par cette fonction (voir plus bas).
// Attention: les tableaux sont parcourus avec des indices allant de 1 à n à l'intérieur de la fonction.
// Il faut donc les réserver, les remplir ou les lire en conséquence. 
#define ROTATE(a,i,j,k,l)\
g= a[i][j];\
h= a[k][l];\
a[i][j]= g - s*(h+g*tau);\
a[k][l]= h + s*(g-h*tau);
// Compute all eigenvalues and eigenvectors of real SYMMETRIC matrix a[1..n][1..n]. 
// On output, elements of a above the diagonal are destroyed.
// d[1..n] returns the eigenvalues of a. 
//v[1..n][1..n] is a matrix whose columns contain, on output, the normalized eigenvectors of a.
// nrot returns the number of Jacobi rotations that were required.
// Note that the above routine assumes that underflows are set to zero.
// On machines where this is not true, the program must be modified.
int jacobi(float **a, int n, float d[], float **v, int *nrot)
{
  int j,iq,ip,i;
  float tresh,theta,tau,t,sm,s,h,g,c,*b,*z;
  b= (float *) malloc((n+1)*sizeof(float)); 
  z= (float *) malloc((n+1)*sizeof(float)); 
 
  #ifdef GP_DEBUG
  for (ip=1;ip<=n;ip++) 
  {  
     for (iq=ip+1;iq<=n;iq++)
     {   
        if( fabs( a[ip][iq]-a[iq][ip] ) > EPSILON )
        {
          printf("%s: %d: jacobi(): la matrice d'entrée n'est pas symétrique\n",__FILE__,__LINE__);
          return 0;
        }
     }
  }
  #endif


  //Initialize to the identity matrix.
  for (ip=1;ip<=n;ip++) 
  {  
     for (iq=1;iq<=n;iq++)
     {   v[ip][iq]= 0.0;  }
     v[ip][ip]= 1.0;
  }

  //Initialize b and d to the diagonal of a.
  for (ip=1;ip<=n;ip++) 
  {
    b[ip]= d[ip]= a[ip][ip];
    //This vector will accumulate terms
    //of the form tapq as in equa-tion (11.1.14)
    z[ip]= 0.0;
  }

  *nrot= 0;
  for (i=1;i<=50;i++)
  {
      sm= 0.0;
     //Sum off-diagonal elements.
      for (ip=1;ip<=n-1;ip++) 
      {
        for (iq=ip+1;iq<=n;iq++)
        { sm += fabs(a[ip][iq]); }
      }

      //The normal return, which relies
      //on quadratic convergence to
      //machine underflow.
      if (sm == 0.0) 
      {
        free(z); 
        free(b);
        return 1;
      }

      if (i < 4)           
        tresh= 0.2*sm/(n*n); //...on the first three sweeps.
      else
        tresh= 0.0;    //...thereafter

      for (ip=1;ip<=n-1;ip++) 
      {
          for (iq=ip+1;iq<=n;iq++)
          {
              g= 100.0*fabs(a[ip][iq]);
             //After four sweeps, skip the rotation if the off-diagonal element is small.
             if ( i > 4 && (float)(fabs(d[ip])+g) == (float)fabs(d[ip])
                        && (float)(fabs(d[iq])+g) == (float)fabs(d[iq]) )
             a[ip][iq]=0.0;
             else if (fabs(a[ip][iq]) > tresh) 
                  {
                     h=d[iq]-d[ip];
                     if ( (float)(fabs(h)+g) == (float)fabs(h) )
                          t= (a[ip][iq])/h;  ///t = 1/(2theta)
                     else 
                     {
                          theta= 0.5*h/(a[ip][iq]);
                          t= 1.0/(fabs(theta)+sqrt(1.0+theta*theta));
                          if (theta < 0.0) t = -t;
                     }
                     c= 1.0/sqrt(1+t*t);
                     s= t*c;
                     tau= s/(1.0+c);
                     h= t*a[ip][iq];
                     z[ip] -= h;
                     z[iq] += h;
                     d[ip] -= h;
                     d[iq] += h;
                     a[ip][iq]= 0.0;

                     //Case of rotations 1 =< j < p.
                     for (j=1;j<=ip-1;j++)
                     {  ROTATE(a,j,ip,j,iq)  }

                     //Case of rotations p < j < q.
                     for (j=ip+1;j<=iq-1;j++) 
                     {   ROTATE(a,ip,j,j,iq) }

                     //Case of rotations q < j <= n.
                     for (j=iq+1;j<=n;j++) 
                     {    ROTATE(a,ip,j,iq,j) }

                     for (j=1;j<=n;j++) 
                     {    ROTATE(v,j,ip,j,iq) }

                     ++(*nrot);
                 }
          }
      }

      for (ip=1;ip<=n;ip++) 
      {
        b[ip] += z[ip];
        //Update d with the sum of tapq,
        d[ip]=b[ip];
        //and reinitialize z.
        z[ip]= 0.0;
      }

  }

  printf("%s: %d: jacobi(): too many iterations in routine jacobi\n",__FILE__,__LINE__);
  return 0;
}


// Fonction tirée de "Numerical Recipes in C" chapter 11-1.
// À utiliser sur les résultats de la fonction jacobi().
// Attention: les tableaux sont parcourus avec des indices allant de 1 à n à l'intérieur de la fonction.
// Il faut donc les réserver, les remplir ou les lire en conséquence. 

// Given the eigenvalues d[1..n] and eigenvectors v[1..n][1..n] as output from jacobi
// (§11.1) or tqli (§11.3), this routine sorts the eigenvalues into descending order, and rearranges
// the columns of v correspondingly. The method is straight insertion.
void eigsrt(float d[], float **v, int n)
{                      
   int k,j,i;
   float p;
   for (i=1;i<n;i++) 
   {
       p= d[k=i];
       for (j=i+1;j<=n;j++)
          if (d[j] >= p) p= d[k=j];
       if (k != i)
       {
          d[k]= d[i];
          d[i]= p;
          for (j=1;j<=n;j++) 
          {
            p= v[j][i];
            v[j][i]= v[j][k];
            v[j][k]= p;
         }
    
       }  
   }
}



// Cette fonction calcule les axes principaux (copiés dans la matrice inertia_axes) de la matrice d'inertie
// contenue dans une structure mass_properties et les range de sorte qu'ils forment une base directe.
// --> inertia_axes= ( axe1x axe2x axe3x )
//                   ( axe1y axe2y axe3y )
//                   ( axe1z axe2z axe3z )
int gpCompute_inertia_axes(Mass_properties *mass_prop, p3d_matrix3 inertia_axes)
{
  #ifdef GP_DEBUG
  if(mass_prop==NULL)
  {
    printf("%s: %d: gpCompute_inertia_axes(): entrée NULL.\n",__FILE__,__LINE__);
    return 0;
  }
  #endif

  int i, nrot;
  int result;
  float d[4];
  p3d_vector3 axis1, axis2, axis3, cross;
  float **a, **v;
  a= (float **) malloc(4*sizeof(float *));
  v= (float **) malloc(4*sizeof(float *));
  for(i=1;i<4;i++)
  {
     a[i]= (float *) malloc(4*sizeof(float));
     v[i]= (float *) malloc(4*sizeof(float));
  }

  a[1][1]= mass_prop->J[0][0];   a[1][2]= mass_prop->J[0][1];    a[1][3]= mass_prop->J[0][2]; 
  a[2][1]= mass_prop->J[1][0];   a[2][2]= mass_prop->J[1][1];    a[2][3]= mass_prop->J[1][2]; 
  a[3][1]= mass_prop->J[2][0];   a[3][2]= mass_prop->J[2][1];    a[3][3]= mass_prop->J[2][2]; 


  result= jacobi(a, 3, d, v, &nrot); //calcule les valeurs propres et les vecteurs propres de la matrice d'inertie
  if(result==0)
  {
    printf("%s: %d: gpCompute_inertia_axes(): le calcul des valeurs propres de la matrice d'inertie a échoué. .\n",__FILE__,__LINE__);
    printf("\tJ[0][0]= %g; J[0][1]= %g; J[0][2]= %g; \n",mass_prop->J[0][0],mass_prop->J[0][1],mass_prop->J[0][2]);
    printf("\tJ[1][0]= %g; J[1][1]= %g; J[1][2]= %g; \n",mass_prop->J[1][0],mass_prop->J[1][1],mass_prop->J[1][2]);
    printf("\tJ[2][0]= %g; J[2][1]= %g; J[2][2]= %g; \n",mass_prop->J[2][0],mass_prop->J[2][1],mass_prop->J[2][2]);
    return 0;
  }


  eigsrt(d, v, 3);  //range les valeurs propres par ordre décroissant et réarrange les vecteurs propres de façon correspondante

  for(i=0; i<3; i++)
  {
    axis1[i]= v[i+1][1];
    axis2[i]= v[i+1][2];
    axis3[i]= v[i+1][3];
  }

  //On range les axes de sorte qu'ils forment une base directe (le 1er reste celui de plus grande valeur propre):
  p3d_vectXprod(axis1, axis2, cross);
  if( p3d_vectDotProd( cross, axis3 ) > 0 )
  {
     for(i=0; i<3; i++)
     {
       inertia_axes[i][0]= axis1[i];
       inertia_axes[i][1]= axis2[i];
       inertia_axes[i][2]= axis3[i];
     }
  }
  else
  {
     for(i=0; i<3; i++)
     {
       inertia_axes[i][0]= axis1[i];
       inertia_axes[i][1]= axis3[i];
       inertia_axes[i][2]= axis2[i];
     }
  }

  for(i=1; i<4;i++)
  { 
    free(a[i]);
    free(v[i]);
  }
  free(a);
  free(v);

  return 1;
}


  

// Cette fonction calcule les plus grandes distances des points du polyèdre dans les directions de chacun des trois axes
// d'inertie.
// On a les limites d'une boîte englobante dont les axes sont alignés sur les axes d'inertie ("inertia axes aligned
// bounding box").
// La fonction reçoit le centre de masse et les axes d'inertie et retourne les limites de la boîte
// (6 nombres algébriques)
// selon les 3 axes dans iaabb (xmin, xmax, ymin, ymax, zmin, zmax).
// iaxes= ( axe1x axe2x axe3x )
//        ( axe1y axe2y axe3y )
//        ( axe1z axe2z axe3z )
int gpInertia_AABB(p3d_polyhedre *polyhedron, p3d_vector3 cmass, p3d_matrix3 iaxes, double iaabb[6])
{
   #ifdef GP_DEBUG
   if(polyhedron==NULL || iaabb==NULL)
   {
     printf("%s: %d: gpInertia_AABB(): entré(s) NULL (%p %p).\n",__FILE__,__LINE__,polyhedron,iaabb);
     return 0;
   }
   #endif

   unsigned int i;
   double dot, axis1min, axis1max, axis2min, axis2max, axis3min, axis3max;
   p3d_vector3 point, axis1, axis2, axis3;

   for(i=0; i<3; i++)
   {
      axis1[i]= iaxes[i][0];
      axis2[i]= iaxes[i][1];
      axis3[i]= iaxes[i][2];
   }


   p3d_vectSub(polyhedron->the_points[0], cmass, point);

   dot= p3d_vectDotProd(point, axis1);
   axis1min= axis1max= dot;

   dot= p3d_vectDotProd(point, axis2);
   axis2min= axis2max= dot;

   dot= p3d_vectDotProd(point, axis3);
   axis3min= axis3max= dot;

   for (i=1; i<polyhedron->nb_points; i++)
   {
      p3d_vectSub(polyhedron->the_points[i], cmass, point);

      dot= p3d_vectDotProd(point, axis1);
      if(dot < axis1min) axis1min= dot;
      if(dot > axis1max) axis1max= dot;

      dot= p3d_vectDotProd(point, axis2);
      if(dot < axis2min) axis2min= dot;
      if(dot > axis2max) axis2max= dot;

      dot= p3d_vectDotProd(point, axis3);
      if(dot < axis3min) axis3min= dot;
      if(dot > axis3max) axis3max= dot;
   }

   iaabb[0]= axis1min;
   iaabb[1]= axis1max;
   iaabb[2]= axis2min;
   iaabb[3]= axis2max;
   iaabb[4]= axis3min;
   iaabb[5]= axis3max;

   return 1;
}


// Fonction d'affichage de la boîte englobante alignée selon les axes d'inertie.
// Elle reçoit le centre de masse de l'objet, ses axes d'inertie et les dimensions de la boîte (xmin, xmax, ymin, ymax, zmin, zmax).
// À utiliser dans une fonction d'affichage OpenGL.
// iaxes= ( axe1x axe2x axe3x )
//        ( axe1y axe2y axe3y )
//        ( axe1z axe2z axe3z )
void gpDraw_inertia_AABB(p3d_vector3 cmass, double iaxes[3][3], double iaabb[6])
{
   int i;
   double a[3], b[3], c[3], d[3], e[3], f[3], g[3], h[3];
   //double length= 1.0, norm;
   //p3d_vector3 v1, v2;
   p3d_vector3 n, nx, ny, nz;

   for(i=0; i<3; i++)
   {
     a[i]= cmass[i] + iaabb[0]*iaxes[i][0]  + iaabb[2]*iaxes[i][1] + iaabb[4]*iaxes[i][2];
     b[i]= cmass[i] + iaabb[1]*iaxes[i][0]  + iaabb[2]*iaxes[i][1] + iaabb[4]*iaxes[i][2];
     c[i]= cmass[i] + iaabb[1]*iaxes[i][0]  + iaabb[3]*iaxes[i][1] + iaabb[4]*iaxes[i][2];
     d[i]= cmass[i] + iaabb[0]*iaxes[i][0]  + iaabb[3]*iaxes[i][1] + iaabb[4]*iaxes[i][2];

     e[i]= cmass[i] + iaabb[0]*iaxes[i][0]  + iaabb[2]*iaxes[i][1] + iaabb[5]*iaxes[i][2];
     f[i]= cmass[i] + iaabb[1]*iaxes[i][0]  + iaabb[2]*iaxes[i][1] + iaabb[5]*iaxes[i][2];
     g[i]= cmass[i] + iaabb[1]*iaxes[i][0]  + iaabb[3]*iaxes[i][1] + iaabb[5]*iaxes[i][2];
     h[i]= cmass[i] + iaabb[0]*iaxes[i][0]  + iaabb[3]*iaxes[i][1] + iaabb[5]*iaxes[i][2];
   }

//glCullFace(GL_FRONT);

   glEnable(GL_LIGHTING);
/*
   v1[0]= cmass[0] + length*iaabb[0]*iaxes[0][0];
   v1[1]= cmass[1] + length*iaabb[0]*iaxes[1][0];
   v1[2]= cmass[2] + length*iaabb[0]*iaxes[2][0];
   v2[0]= cmass[0] + length*iaabb[1]*iaxes[0][0];
   v2[1]= cmass[1] + length*iaabb[1]*iaxes[1][0];
   v2[2]= cmass[2] + length*iaabb[1]*iaxes[2][0];
   draw_arrow(cmass, v1, 1, 0, 0);
   draw_arrow(cmass, v2, 1, 0, 0);

   v1[0]= cmass[0] + length*iaabb[2]*iaxes[0][1];
   v1[1]= cmass[1] + length*iaabb[2]*iaxes[1][1];
   v1[2]= cmass[2] + length*iaabb[2]*iaxes[2][1];
   v2[0]= cmass[0] + length*iaabb[3]*iaxes[0][1];
   v2[1]= cmass[1] + length*iaabb[3]*iaxes[1][1];
   v2[2]= cmass[2] + length*iaabb[3]*iaxes[2][1];
   draw_arrow(cmass, v1, 0, 1, 0);
   draw_arrow(cmass, v2, 0, 1, 0);

   v1[0]= cmass[0] + length*iaabb[4]*iaxes[0][2];
   v1[1]= cmass[1] + length*iaabb[4]*iaxes[1][2];
   v1[2]= cmass[2] + length*iaabb[4]*iaxes[2][2];
   v2[0]= cmass[0] + length*iaabb[5]*iaxes[0][2];
   v2[1]= cmass[1] + length*iaabb[5]*iaxes[1][2];
   v2[2]= cmass[2] + length*iaabb[5]*iaxes[2][2];
   draw_arrow(cmass, v1, 0, 0, 1);
   draw_arrow(cmass, v2, 0, 0, 1);
*/

   n[0]= iaxes[0][0];
   n[1]= iaxes[1][0];
   n[2]= iaxes[2][0];
   p3d_vectNormalize(n, nx);

   n[0]= iaxes[0][1];
   n[1]= iaxes[1][1];
   n[2]= iaxes[2][1];
   p3d_vectNormalize(n, ny);

   n[0]= iaxes[0][2];
   n[1]= iaxes[1][2];
   n[2]= iaxes[2][2];
   p3d_vectNormalize(n, nz);

   glEnable(GL_CULL_FACE);
   glBegin(GL_TRIANGLES);
       glNormal3d(-nz[0], -nz[1], -nz[2]);
       glVertex3dv(a);    glVertex3dv(b);     glVertex3dv(c);
       glVertex3dv(a);    glVertex3dv(c);     glVertex3dv(d);

       glNormal3d(nz[0], nz[1], nz[2]);
       glVertex3dv(e);    glVertex3dv(g);     glVertex3dv(f);
       glVertex3dv(e);    glVertex3dv(h);     glVertex3dv(g);

       glNormal3d(-ny[0], -ny[1], -ny[2]);
       glVertex3dv(a);    glVertex3dv(f);     glVertex3dv(b);
       glVertex3dv(a);    glVertex3dv(e);     glVertex3dv(f);

       glNormal3d(ny[0], ny[1], ny[2]);
       glVertex3dv(g);    glVertex3dv(d);     glVertex3dv(c);
       glVertex3dv(g);    glVertex3dv(h);     glVertex3dv(d);

       glNormal3d(-nx[0], -nx[1], -nx[2]);
       glVertex3dv(b);    glVertex3dv(f);     glVertex3dv(g);
       glVertex3dv(b);    glVertex3dv(g);     glVertex3dv(c);

       glNormal3d(nx[0], nx[1], nx[2]);
       glVertex3dv(a);    glVertex3dv(h);     glVertex3dv(e);
       glVertex3dv(a);    glVertex3dv(d);     glVertex3dv(h);
   glEnd();
//glCullFace(GL_BACK);
glDisable(GL_CULL_FACE);
}

