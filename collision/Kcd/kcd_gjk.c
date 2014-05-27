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
/** **************************************************************************************** **
 ** **************************************************************************************** **
 **     Functions for GJK (all but the support functions)                                    **
 ** **************************************************************************************** **
 ** **************************************************************************************** **/

#include "Collision-pkg.h"


static double rel_error = 1e-6;   /* relative error in the computed distance */

/* support points of object A in local coordinates */ 
static kcd_vector3  p[4];
/* support points of object B in local coordinates */
static kcd_vector3  q[4];

/* the support points, at most 4 */
kcd_vector3  y[4];

/* dot products  d[i][j] = p3d_vectDotProd(y[i],y[j]) */
double d[4][4];

/*  for Wk, Delta[i][x]   is stored in   det[x][i] */
double  det[16][4];

/* index in  y[] of new support point wk */
int wk_index;

static int bits;      /* id of current simplex X (subset of Wk) */
static int last;      /* id of last found support point 
			 (index in  y[] of new support point wk) */
static int last_bit;  /* last_bit = 1 <<= last (<<= : left shift) */
static int all_bits;  /* all_bits = bits | last_bit  ( | : set union) */




/* multiply v with transpose of M and put in v2: tM * (v 0) => (v2 0)*/
static void kcd_gjk_xformVect(kcd_matrix4 M, kcd_vector3 v, kcd_vector3 v2)
{
  int i;
  
  for (i = 0; i < 3; i++) 
    v2[i] = M[0][i] * v[0] + M[1][i] * v[1] + M[2][i] * v[2];
}

/* computes closest points from intermediate support points */
static void kcd_gjk_compute_points(int bits, kcd_vector3 *pa, kcd_vector3 *pb)
{
  double gjk_sum = 0,s;
  int i,bit;
  kcd_vector3 qr;

  (*pa)[0]=0.0;
  (*pa)[1]=0.0;
  (*pa)[2]=0.0;
  (*pb)[0]=0.0;
  (*pb)[1]=0.0;
  (*pb)[2]=0.0;

  for (i = 0, bit = 1; i < 4; ++i, bit <<= 1) 
    {
      if (bits & bit) 
	{
	  gjk_sum += det[bits][i];
	  kcd_vectScale(p[i],qr,det[bits][i]);
	  kcd_vectAdd(*pa,qr,*pa);
	  kcd_vectScale(q[i],qr,det[bits][i]);
	  kcd_vectAdd(*pb,qr,*pb);
	}
    }
   s = 1.0 / gjk_sum;
  kcd_vectScale(*pa,*pa,s);
  kcd_vectScale(*pb,*pb,s);
}


/* Checks if subset  s  is the right candidate for 
   the new subset X of the new set Wk */
static int kcd_gjk_valid(int s) 
{ 
  int i,bit;

  for (i = 0, bit = 1; i < 4; ++i, bit <<= 1) 
    {
      /* bit represents the i-th element of a set,
	 set all_bits contains at most 4 elements (i=0..3) */
      if (all_bits & bit) 
	{
	  /* bit is element of set all_bits */
	  if (s & bit) 
	    { 
	      /* bit is element of set s */
	      if (det[s][i] <= 0)    
		{
		  /* Delta_i(set s) should be negative => 
		     set s  is not valid */
		  return FALSE; 
		}
	    }
	  else 
	    { 
	      /* bit is not element of set s */
	      if (det[s|bit][i] > 0) 
		{
		  /* Delta_i(set s) should be positive => 
		     set s  is not valid */
		  return FALSE; 
		}
	    }
	}
    }
  return TRUE;
}

/* Checks for degenerate cases due to rounding errors,
   Returns TRUE if  w  was already in  W_{k-1} U {w_{k-1}} */
static int kcd_gjk_degenerate(kcd_vector3 w)
{
  int i,bit;

  for (i = 0, bit = 1; i < 4; ++i, bit <<= 1) 
    {
      /* bit represents the i-th element of a set,
	 set all_bits contains at most 4 elements (i=0..3) */
      if ((all_bits & bit) && kcd_vectEqual(y[i],w) )  
	{
	  /* there is an i-th element of all_bits and vector w actually is 
	     the i-th support vector => we have a degenerate case */
	  return TRUE;
	}
    }
  return FALSE;
}

static void kcd_gjk_compute_vector(int bits, kcd_vector3 *v)
{
  double gjk_sum = 0;
  kcd_vector3 gjk_sy;
  int i,bit;

  /* initialize  v */
  (*v)[0] = 0.0;
  (*v)[1] = 0.0;
  (*v)[2] = 0.0;
  gjk_sy[0] = 0.0;
  gjk_sy[1] = 0.0;
  gjk_sy[2] = 0.0;

  for (i = 0, bit = 1; i < 4; ++i, bit <<= 1) 
    {
      /* bit represents the i-th element of a set,
	 set  bits  contains at most 4 elements (i=0..3) */     
      if (bits & bit) 
	{
	  /* there is an element on the i-th place of y[] (and set bits) */
	  gjk_sum += det[bits][i];
	  /* p3d_vectScale( y[i], gjk_sy, det[bits][i]); */
	  gjk_sy[0] = y[i][0] * det[bits][i];
	  gjk_sy[1] = y[i][1] * det[bits][i];
	  gjk_sy[2] = y[i][2] * det[bits][i];
	  kcd_vectAdd( (*v), gjk_sy, (*v) );
	}
    }
   kcd_vectScale( (*v), (*v), 1.0/gjk_sum) ;
}

static void kcd_gjk_compute_det()
{
  static double dp[4][4]; /* dot products  dp[i][j] = y[i].y[j] */
  int i,j,k,bit,sj,sk,s2,s3;
  

  /* compute new dot products for new point y[last] */
  for (i = 0, bit = 1; i < 4; ++i, bit <<=1) 
    {
      if (bits & bit) 
	{
	  /* there is a i-th element in the old set  bits  (hence y[i] exists) */
	  dp[last][i] = kcd_vectDotProd(y[i], y[last]);
	  dp[i][last] = dp[last][i];
	}
    }
  dp[last][last] = kcd_vectDotProd(y[last], y[last]);

  /* update determinant, there where we changed a dot product 
     and propagate the new determinants for the larger sets    */
  det[last_bit][last] = 1.0;
  for (j = 0, sj = 1; j < 4; ++j, sj <<= 1) 
    {
      if (bits & sj) 
	{
	  /* there is a  j-th  element in set  bits */
	  s2 = sj|last_bit; /* set s2  is union of  {sj} and {new-element} */
	  det[s2][j] = dp[last][last] - dp[last][j]; 
	  det[s2][last] = dp[j][j] - dp[j][last];
	  for (k = 0, sk = 1; k < j; ++k, sk <<= 1) 
	    {
	      if (bits & sk) 
		{
		  /* there is a  k-th  element in set  bits */
		  s3 = sk|s2; /* set {s3} is {sk, sj, new-element } */
		  det[s3][k] = det[s2][j] * (dp[j][j] - dp[j][k]) + 
		    det[s2][last] * (dp[last][j] - dp[last][k]);
		  det[s3][j] = det[sk|last_bit][k] * (dp[k][k] - dp[k][j]) + 
		    det[sk|last_bit][last] * (dp[last][k] - dp[last][j]);
		  det[s3][last] = det[sk|sj][k] * (dp[k][k] - dp[k][last]) + 
		    det[sk|sj][j] * (dp[j][k] - dp[j][last]);
		}
	    }
	}
    }
  if (all_bits == 15) 
    {
      /* we even have a fourth element in  all_bits, hence propagate changes */
      det[15][0] = det[14][1] * (dp[1][1] - dp[1][0]) + 
	det[14][2] * (dp[2][1] - dp[2][0]) + 
	det[14][3] * (dp[3][1] - dp[3][0]);
      det[15][1] = det[13][0] * (dp[0][0] - dp[0][1]) + 
	det[13][2] * (dp[2][0] - dp[2][1]) + 
	det[13][3] * (dp[3][0] - dp[3][1]);
      det[15][2] = det[11][0] * (dp[0][0] - dp[0][2]) + 
	det[11][1] * (dp[1][0] - dp[1][2]) +  
	det[11][3] * (dp[3][0] - dp[3][2]);
      det[15][3] = det[7][0] * (dp[0][0] - dp[0][3]) + 
	det[7][1] * (dp[1][0] - dp[1][3]) + 
	det[7][2] * (dp[2][0] - dp[2][3]);
    }
}

/* important side effect: v changes to new vector for next iteration step */
static int kcd_gjk_closest(kcd_vector3 *v) 
{
  int s;

  kcd_gjk_compute_det();
  for (s = bits; s; --s) /* s is supposed to be a subset of bits */
    {
      if ((s & bits) == s) 
	{ /* s indeed is a subset of bits */
	  if ( kcd_gjk_valid(s|last_bit) ) 
	    { /* s  U  {last_bit}  is valid set */
	      bits = s|last_bit;                /* we found the right new subset X */
	      kcd_gjk_compute_vector(bits, v);  /* get new vector v */
	      return TRUE;
	    }
	}
    }
  if ( kcd_gjk_valid(last_bit) ) 
    { /* case of the trivial subset  s  (s is empty) */
      bits = last_bit;      /* we finally found the right new subset X (singleton) */
       kcd_vectCopy(y[last], (*v) );          /* get new vector v */
      return TRUE;
    }
  // Original GJK calls the backup procedure at this point.

#ifdef USE_BACKUP_PROCEDURE
#endif 

  return FALSE;
}

/* GJK method to determine collision only
   DOES NOT return distance */
int kcd_gjk_just_intersect(int box_id1, int nof_vertices1, int *facet_ids1, void *polyh1,
			   int box_id2, int nof_vertices2, int *facet_ids2, void *polyh2, 
			   kcd_matrix4 BwrtA, kcd_vector3 *v)
{
  kcd_vector3 inp1,inp2,w,minv;
  kcd_vector3  supobj1, supobj2; 
  double vnormhat2 = 0.0;

  /* initialize the sets */
  bits = 0;
  all_bits = 0;

  /* the iteration loop */
  do
    {
      last = 0;
      last_bit = 1;


      /* find first empty spot in array y[] (hence in bits): 
	 here we will add the new  w  */
      while (bits & last_bit) 
	{ 
	  ++last; 
	  last_bit <<= 1; 
	}

      /* compute -v */
      minv[0] = -(*v)[0];
      minv[1] = -(*v)[1];
      minv[2] = -(*v)[2];
      /* place v properly, using BwrtA */
      kcd_gjk_xformVect (BwrtA, *v, inp2);
      /* compute support vector  w  */
      kcd_gjk_support(box_id1,nof_vertices1,facet_ids1,polyh1,minv,&supobj1);
      kcd_gjk_support(box_id2,nof_vertices2,facet_ids2,polyh2,inp2,&supobj2);
      kcd_TransfoPoint(BwrtA, supobj2, inp1); 
      kcd_vectSub(supobj1,inp1,w);

    if( GNEQ(kcd_vectDotProd((*v),w),0.0) )
      {
	/* termination condition */
	return FALSE;
      }
    if( kcd_gjk_degenerate(w) )
      {
	/* numerical problem */
	return FALSE;
      }
    kcd_vectCopy(w,y[last]);
    all_bits = bits|last_bit;

    /* select good subset of  bits  and compute new vector  v
       for termination test and/or continuation of the iteration loop */
    if( !kcd_gjk_closest(v) )
      {
	/* numerical problem */
	return FALSE;
      }
    /* side effects of kcd_gjk_closest: v  and  bits  got new values */
    vnormhat2 = (*v)[0] * (*v)[0] + (*v)[1] * (*v)[1] + (*v)[2] * (*v)[2];
  }while( bits < 15 && !(EQ(vnormhat2,0.0)) );
  return TRUE;
}

/* GJK method to determine collision (ignores distance), 
   returning also a point common to both objects in case of intersection */
int kcd_gjk_intersect_and_common_point(int box_id1, int nof_vertices1, int *facet_ids1, void *polyh1,
				       int box_id2, int nof_vertices2, int *facet_ids2, void *polyh2,
				       kcd_vector3 *pa, kcd_vector3 *pb, kcd_matrix4 BwrtA, kcd_vector3 *v)
{
  kcd_vector3 inp1,inp2,w,minv;
/*  kcd_vector3  supobj1, supobj2; */
  double vnormhat2 = 0.0;

  /* initialize the sets */
  bits = 0;
  all_bits = 0;



  /* the iteration loop */
  do
    {
      last = 0;
      last_bit = 1;

      /* find first empty spot in array y[] (hence in bits): 
	 here we will add the new  w  */
      while (bits & last_bit) 
	{ 
	  ++last; 
	  last_bit <<= 1; 
	}
      
      /* compute -v */ 
      minv[0] = -(*v)[0];
      minv[1] = -(*v)[1];
      minv[2] = -(*v)[2];
      /* place v properly, using BwrtA */
      kcd_gjk_xformVect (BwrtA, *v, inp2);
      /* compute support vector  w  */
      kcd_gjk_support(box_id1,nof_vertices1,facet_ids1,polyh1,minv,&(p[last]));
      kcd_gjk_support(box_id2,nof_vertices2,facet_ids2,polyh2,inp2,&(q[last]));
      kcd_TransfoPoint(BwrtA, q[last], inp1);
      kcd_vectSub(p[last],inp1,w);


      if( GNEQ(kcd_vectDotProd((*v),w),0.0) )
	{
	  /* termination condition */
	  return FALSE;
	}
      if( kcd_gjk_degenerate(w) )
	{
	  /* numerical problem */
	  return FALSE;
	}
      kcd_vectCopy(w,y[last]);
      all_bits = bits|last_bit;
      
      /* select good subset of  bits  and compute new vector  v
	 for termination test and/or continuation of the iteration loop */
      if( !kcd_gjk_closest(v) )
	{
	  /* numerical problem */
	  return FALSE;
	}
      /* side effects of kcd_gjk_closest: v  and  bits  got new values */
      vnormhat2 = (*v)[0] * (*v)[0] + (*v)[1] * (*v)[1] + (*v)[2] * (*v)[2];
    }while( bits < 15 && !(EQ(vnormhat2,0.0)) );
  kcd_gjk_compute_points(bits, pa, pb);
  return TRUE;
}


/* GJK method to determine collision and distance, returning also
   the closest points in case of separation */
int kcd_gjk_closest_pair(int box_id1, int nof_vertices1, int *facet_ids1, void *polyh1,
			 int box_id2, int nof_vertices2, int *facet_ids2, void *polyh2, 
			 kcd_matrix4 AwrtW, kcd_matrix4 BwrtW,
			 kcd_vector3 *pa, kcd_vector3 *pb, double *gjk_distance, double gjk_tolerance)
{
  kcd_vector3 inp1,inp2,w,minv; /*wpa,wpb;*/
  kcd_vector3  supobj1, supobj2; 
  kcd_vector3 v;
  kcd_vector3 zero_vec;
  double mu=0.0,mu1; /*dist;*/



  /* initialize the sets */
  bits = 0;
  all_bits = 0;

  /* initialize v */
  kcd_vectSub(*pa,*pb,zero_vec);
  kcd_gjk_support(box_id1,nof_vertices1,facet_ids1,polyh1,zero_vec,&supobj1);
  kcd_TransfoPoint(AwrtW, supobj1, inp1);
  kcd_gjk_support(box_id2,nof_vertices2,facet_ids2,polyh2,zero_vec,&supobj2);
  kcd_TransfoPoint(BwrtW, supobj2, inp2);
  kcd_vectSub(inp1,inp2,v);
  (*gjk_distance) = kcd_vectNorm(v);
 
  /* the iteration loop */
  while(bits < 15 &&  GNEQ((*gjk_distance),gjk_tolerance) )
    {
      last = 0;
      last_bit = 1;

      /* find first empty spot in array y[] (hence in bits): 
	 here we will add the new  w  */
      while (bits & last_bit) 
	{ 
	  ++last; 
	  last_bit <<= 1; 
	}
      
      /* compute -v */
      minv[0] = -v[0];
      minv[1] = -v[1];
      minv[2] = -v[2];
      /* place v properly */
      kcd_gjk_xformVect (AwrtW, minv, inp1);
      kcd_gjk_xformVect (BwrtW,    v, inp2);
      /* compute support vector  w  */
      kcd_gjk_support(box_id1,nof_vertices1,facet_ids1,polyh1,inp1,&p[last]);
      kcd_gjk_support(box_id2,nof_vertices2,facet_ids2,polyh2,inp2,&q[last]);
      kcd_TransfoPoint(AwrtW, p[last],supobj1);
      kcd_TransfoPoint(BwrtW, q[last],supobj2);
      kcd_vectSub(supobj1,supobj2,w);

      mu1 = kcd_vectDotProd(v, w) / (*gjk_distance);
      if(mu < mu1) 
	mu = mu1;


      /* other termination criterion */
      if ((*gjk_distance) - mu <= (*gjk_distance) * rel_error) 
	{
	  break; /* termination criterion */
	}
      if( kcd_gjk_degenerate(w) )
	{
	  /* numerical problem */
	  break;
	}
      kcd_vectCopy(w,y[last]);
      all_bits = bits|last_bit;

      /* select good subset of  bits  and compute new vector  v
	 for termination test and/or continuation of the iteration loop */
      if( !kcd_gjk_closest(&v) )
	{
	  /* numerical problem */
	  break;
	  /* return FALSE; */
	}
      /* side effects of kcd_gjk_closest: v  and  bits  got new values */
      (*gjk_distance) = kcd_vectNorm(v);
    }
  kcd_gjk_compute_points(bits, pa, pb);
  return (LEQ(*gjk_distance,gjk_tolerance));
}




