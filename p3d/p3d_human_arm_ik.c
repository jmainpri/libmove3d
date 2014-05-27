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
/* (garechav) */
#include "Util-pkg.h"
#include "P3d-pkg.h"

#define AINT_EPSILON     (1e-5)

#define cpmatrixIKAN(u,v) memcpy(u,v,sizeof(p3d_matrix4))

// Stores projection axis for determining u and the positive 
// direction axis for determining positive direction of angle
p3d_vector3 proj_axis;
p3d_vector3 pos_axis;

//
// Stores equation of circle for a given problem
//
p3d_vector3 u;
p3d_vector3 v;
p3d_vector3 n;
p3d_vector3 c;
double radius;

double upper_len;   // Len of T pos vector
double lower_len;   // Len of S pos vector
double reciprocal_upper_len; 

double r_angle;

double swivel_angle;

//
// Stores end effector position in world frame and R1 frame
//
p3d_vector3 ee;
p3d_vector3 ee_r1; 

//
// Stores position of middle revolute joint in R1 frame
//
p3d_vector3 p_r1;

//
// elbow rotation axe
//
p3d_vector3 axeRy;

//
// Stores constant matrices and rotation of revolute joint
// and their product S*Ry*T. 
p3d_matrix4 Te;
p3d_matrix4 S;
p3d_matrix4 SRT;

p3d_matrix4 R1 =  { { 1,  0,  0,  0 },
		    { 0,  1,  0,  0 },
		    { 0,  0,  1,  0 },
		    { 0,  0,  0,  1 }};

p3d_matrix4 R1Tmp = { { 1,  0,  0,  0 },
		      { 0,  1,  0,  0 },
		      { 0,  0,  1,  0 },
		      { 0,  0,  0,  1 }};

p3d_matrix4 R2 =  { { 1,  0,  0,  0 },
		    { 0,  1,  0,  0 },
		    { 0,  0,  1,  0 },
		    { 0,  0,  0,  1 }};

// 
// Stores goal transformation
//
p3d_matrix4 G = {{1, 0, 0, 0},
		 {0, 1, 0, 0},
		 {0, 0, 1, 0},
		 {0, 0, 0, 1}};

p3d_matrix4 TbaseRot = {{1, 0, 0, 0},
		        {0, 1, 0, 0},
		        {0, 0, 1, 0},
		        {0, 0, 0, 1}};

void get_translation(p3d_matrix4 M, p3d_vector3 p)
{
  p[0] = M[3][0];
  p[1] = M[3][1];
  p[2] = M[3][2];
}

double angle_normal(double x)
{
  while (x > M_PI) x -= 2*M_PI;
  while (x < -M_PI) x += 2*M_PI;
  
  return x;
}

double norm(p3d_vector3 v)
{
  return sqrt(DOT(v,v));
}

double iszero(float x) 
{
    return x*x < 1e-6;
}

//
// Solve a*cos(theta) + b*sin(theta) = c
// Either one or two solutions. Return the answer in radians
//

int solve_trig1(double a, double b, double c, double theta[2])
{
    double temp  = (double) a*a+b*b-c*c;
    int num;

    if (temp < 0)
    {
	// temp is practically zero
	if (fabs(temp / (fabs(a*a) + fabs(b*b) + fabs(c*c))) < 1e-6)
	{
	    // printf("Special case\n");
	    theta[0] = (double) 2*atan(-b/(-a-c));
	    return 1;
	}
	else
	    return 0;
    }

    temp  = (double) atan2(sqrt(temp), c);
    num =  (!iszero(temp)) ? 2 : 1;

    // Calculate answer in radians
    theta[0] = (double) atan2(b,a);
    if (num == 2)
    {
        theta[1] = theta[0] - temp;
        theta[0] += temp;

	theta[0] = angle_normal(theta[0]);
	theta[1] = angle_normal(theta[1]);
    }
    return num;
}


//
// Multiplies only the rotational components of B*C 
// and stores the result into A
//
void rmatmult(p3d_matrix4 A, p3d_matrix4 B, p3d_matrix4 C)
{
    p3d_matrix4 Temp1;
    p3d_matrix4 Temp2;

    double c11; 
    double c12; 
    double c13;
    double c21; 
    double c22; 
    double c23;
    double c31; 
    double c32; 
    double c33; 
    double v1, v2, v3;


    register double *a = (double *) A; 
    register double *b;
    register double *c; 

    if (A == B)
      {
	cpmatrixIKAN(Temp1, B);
	b = (double *) Temp1;
      }
    else
      b = (double *) B; 
    if (A == C)
      {
	cpmatrixIKAN(Temp2, C);
	c = (double *) Temp2;
      }
    else
      c = (double *) C; 
    
    c11 = *c++; 
    c12 = *c++; 
    c13 = *c++; c++;
    c21 = *c++; 
    c22 = *c++; 
    c23 = *c++; c++;
    c31 = *c++; 
    c32 = *c++; 
    c33 = *c++; 

    v1 = *b++; v2 = *b++; v3 = *b++; b++;    
    *a++ = v1*c11 + v2*c21 + v3*c31;
    *a++ = v1*c12 + v2*c22 + v3*c32;
    *a++ = v1*c13 + v2*c23 + v3*c33;
    *a++ = 0;

    v1 = *b++; v2 = *b++; v3 = *b++; b++;    
    *a++ = v1*c11 + v2*c21 + v3*c31;
    *a++ = v1*c12 + v2*c22 + v3*c32;
    *a++ = v1*c13 + v2*c23 + v3*c33;
    *a++ = 0;

    v1 = *b++; v2 = *b++; v3 = *b++;     
    *a++ = v1*c11 + v2*c21 + v3*c31;
    *a++ = v1*c12 + v2*c22 + v3*c32;
    *a++ = v1*c13 + v2*c23 + v3*c33;
    *a++ = 0;

    *a++ = 0;
    *a++ = 0;
    *a++ = 0;
    *a = 1;
}

void invertrmatrix(p3d_matrix4 N,p3d_matrix4 M)
/*
 * Invert a rotation matrix 
 * n = inverse of m
 */
{
    register double *n,*m,*nmax,*C;
    
    nmax = &N[2][3];
    n = &N[0][0];
    C = &M[0][0];
    while (n < nmax) {
	m = C;
	*n++ = *m;
	m += 4;
	*n++ = *m;
	m += 4;
	*n++ = *m;
	*n++ = 0.0;
	C++;
    }
    
    *n++ = 0;
    *n++ = 0;
    *n++ = 0;
    *n++ = 1.0;
}

void hmatmult(p3d_matrix4 A,p3d_matrix4 B,p3d_matrix4 C)
/*
 * Homogeneous transformation multiplication:
 *
 * A = B * C
 *
 * This routine is optimized for homogeneous transformations. It does
 * *NOT* work on arbitrary 4x4 matrices.
 *
 * A *CAN* point to the same matrix as B or C.
 */
{
    register double	*a,*b,*c,*bp,*cp;
    register double	*bmax,*cmax,*cpmax;
    register double	*b32,*c00,*c03;
    p3d_matrix4		Bt,Ct;

    if (A == B) {
      cpmatrixIKAN(Bt,B);
      bmax = &Bt[3][0];
      b = &Bt[0][0];
      b32 = &Bt[3][2];
    } else {
      bmax = &B[3][0];
      b = &B[0][0];
      b32 = &B[3][2];
    }
    
    if (A == C) {
      cpmatrixIKAN(Ct,C);
      c00 = &Ct[0][0];
      c03 = &Ct[0][3];
    } else {
      c00 = &C[0][0];
      c03 = &C[0][3];
    }
    
    a = (double *) &A[0][0];
    
    while (b < bmax) {
      c = c00;
      cmax = c03;
      while (c < cmax) {
	cp = c;
	cpmax = c + 8;
	bp = b;
	*a = (*bp++) * (*cp);
	do {
	  cp += 4;
	  *a += *bp++ * (*cp);
	} while (cp < cpmax);
	a++;
	c++;
      }
      b += 4;
      *a++ = 0;
    }
    
    c = c00;
    cmax = c03;
    while (c < cmax) {
      cp = c + 12;
      bp = b32;
      *a = *cp;
      do {
	cp -= 4;
	*a += *bp-- * (*cp);
      } while (cp > c);
      a++;
      c++;
    }
    *a = 1;
}

void crossproduct(p3d_vector3 r, p3d_vector3 a, p3d_vector3 b)
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

void rotation_principal_axis_to_matrix(char axis, double angle, p3d_matrix4 m)
{
  double cos_a, sin_a;

  cpmatrixIKAN(m, p3d_mat4IDENTITY);
  cos_a = cos(angle);
  sin_a = sin(angle);
  
  switch (axis)
    {
    case 'x':
    case 'X':
      m[1][1] = cos_a; m[2][1] = -sin_a;
      m[1][2] = sin_a; m[2][2] = cos_a;
      break;
    
    case 'y':
    case 'Y':
      m[0][0] = cos_a;  m[2][0] = sin_a;
      m[0][2] = -sin_a; m[2][2] = cos_a;
      break;
      
    default:
      m[0][0] = cos_a; m[1][0] = -sin_a;
      m[0][1] = sin_a; m[1][1] = cos_a;
      break;
    }
}

//
// Form local coordinate system {x,y} from points p,q relative to the
// implicit origin 0. pscale is the reciprocal length of the p vector
// which as it turns out is already known. If the invert flag is true
// construct the transpose of the rotation matrix instead
//
void make_frame(p3d_vector3 p,
		double p_scale,
		p3d_vector3 q,
		p3d_matrix4 R,
		int invert)
{
  p3d_vector3 x, y, t;  

  // x vector is unit vector from origin to p
  p3d_vectScale((double *)p, x, p_scale);

  // y vector is unit perpendicular projection of q onto x
  p3d_vectScale(x, t, DOT(q,x));  
  p3d_vectSub( (double *)q, t, y);
  p3d_vectNormalize(y,y);

  // z vector is x cross y
  
  if (invert)
    {
      R[0][0] = x[0]; R[1][0] = x[1]; R[2][0] = x[2];
      R[0][1] = y[0]; R[1][1] = y[1]; R[2][1] = y[2];

      R[0][2] = x[1]*y[2] - x[2]*y[1];
      R[1][2] = x[2]*y[0] - x[0]*y[2];
      R[2][2] = x[0]*y[1] - x[1]*y[0];
    }
  else
    {
      R[0][0] = x[0]; 
      R[0][1] = x[1]; 
      R[0][2] = x[2];
      R[1][0] = y[0]; 
      R[1][1] = y[1]; 
      R[1][2] = y[2];

      R[2][0] = x[1]*y[2] - x[2]*y[1];
      R[2][1] = x[2]*y[0] - x[0]*y[2];
      R[2][2] = x[0]*y[1] - x[1]*y[0];
    }
  
  R[3][0] = R[3][1] = R[3][2] =
  R[0][3] = R[1][3] = R[2][3] = 0;
  
  R[3][3] = 1;
}


//
// Evaluate a point on the circle given the swivel angle 
//
void evalcircle(double angle, p3d_vector3 p)
{
    // p = o + r*cos(f)*u + r*sin(f)*v

    p3d_vector3 temp;

    p3d_vectCopy(c, p);
    p3d_vectScale((double*)u, temp, radius*cos(angle));
    p3d_vectAdd(p, temp, p);
    p3d_vectScale((double*)v, temp, radius*sin(angle));
    p3d_vectAdd(p, temp, p);
}

//
// Use formula a^2 + b^2 - 2abcos(theta) = c^2 to get theta
//
int law_of_cosines(double *angle, double a, double b, double c)
{
    float temp = (a*a+b*b-c*c)/(2*a*b);
    
    if (fabs(temp) > 1)
      return 0;
    else {
      *angle = (double) acos(temp);
    }
    return 1;
}


//
// Computes the equation of the circle given the ee position,
// an axis to project the local 2d coordinate system {u,v} onto,
// and the upper and lower lengths of the mechanism.
//
// Outputs
//	c: center of circle
//      u: local x axis
//      v: local y axis
//      n: normal to plane of circle = cross(u,v)
// Returns radius of circle
//

double get_circle_equation(p3d_vector3 ee,
			   p3d_vector3 axis,
			   p3d_vector3 pos_axis,
			   double upper_len,
			   double lower_len,
			   p3d_vector3 c,
			   p3d_vector3 u,
			   p3d_vector3 v,
			   p3d_vector3 n)
{

  double wn = norm(ee);
  double radius;
  double alpha;  

  p3d_vector3 temp;

  p3d_vectCopy(ee,n);  
  p3d_vectNormalize(n,n);

  // Use law of cosines to get angle between first spherical joint 
  // and revolute joint  
  if (!law_of_cosines(&alpha, wn, upper_len, lower_len))
    {
      return 0;
    }

  // center of circle (origin is location of first S joint)
  p3d_vectScale(n, c, cos(alpha) * upper_len);
  
  radius = sin(alpha) * upper_len;
  
  //
  // A little kludgy. If the goal is behind the joint instead
  // of in front of it, we reverse the angle measurement by 
  // inverting the normal vector
  //
  
  if (DOT(n, pos_axis) < 0)
    p3d_vectScale(n,n, -1);

  p3d_vectScale(n,temp, DOT(axis,n));

  // axis is the proj_axis to give the direction of the normal vector u

  p3d_vectSub(axis,temp,u);
  p3d_vectNormalize(u,u);
  
  crossproduct(v, n, u);
  
  return radius;
}

//
// Given the goal position and the position vectors s and t of 
// the S and T matrices, solve for the angle of the R joint 
// according to the formula (here g and s are row vectors)
//
// ([s,1]*Ry*T*R1)*([s,1]*Ry*T*R1)' = g*g'
// 
// which says the distance from R1 to the tip of the last link
// is equal to the distance to the goal.
//
// This equation simplifies to 
//      s*Rot(Ry)*(Rot(T))*t' = g'*g - s'*s - t'*t
// where Rot(M) is the 3x3 rotation matrix of M
// and this equation is of the form
//     alpha * cos(theta_y) * beta * sin(theta_y) = gamma
//
// Only return positive solution
//

int solve_R_angle(p3d_vector3 g,  
		  p3d_vector3 s,  
		  p3d_vector3 t,  
		  p3d_matrix4 TT,  
		  double *angl) 
{ 
  int n;
  double rhs; 
  double a;
  double b;
  double c;
  double temp[2];
  double alpha[3]; 
  int i,j;
  
  // alpha = Rot(T)*t' NOT alpha = t*Rot(T) 
  
  rhs = DOT(g,g) - DOT(s,s) - DOT(t,t);

  for (j = 0; j < 3; j++) 
    { 
      alpha[j] = 0; 
      for (i = 0; i < 3; i++) 
	alpha[j] += TT[j][i]*t[i]; 
    } 
  
  a = alpha[0]*s[0] + alpha[2]*s[2]; 
  b = alpha[0]*s[2] - alpha[2]*s[0]; 
  c = alpha[1]*s[1];   

  a += a; 
  b += b; 
  c = rhs - (c+c); 

  n = solve_trig1(a, b, c, temp); 

  if (n == 2) 
    { 
      // Two positive solutions choose first 
      if (temp[0] > 0 && temp[1] > 0) 
	{ 
	  *angl = temp[0]; 
	  n = 1;
	} 
      else if (temp[0] > 0) 
	{ 
	  n = 1; 
	  *angl = temp[0]; 
	} 
      else if (temp[1] > 0) 
	{ 
	  n = 1; 
	  *angl = temp[1]; 
	} 
      else  
	n = 0; 
    } 
  else if (n == 1) 
    { 
      // Is solution positive? 
      if (temp[0] < 0) 
	n = 0; 
      else  
	*angl = temp[0]; 
    }
  return n; 
}


//
// Given the goal matrix and the projection axis, find the position
// of the end effector and the equation of the circle that defines
// how the R joint can swivel. 
//
// Also compute the matrix S*RY*T and save it for future computations
//
int SetGoal(p3d_matrix4 GG, double *rangle)
{
  p3d_matrix4 RY;
  p3d_vector3 s;
  
  get_translation(G, ee);

  get_translation(Te, p_r1);
  get_translation(S, s);

  // if (project_to_workspace && scale_goal(p_r1,s,ee))
  //     set_translation(G,ee);
  
  radius = get_circle_equation(ee, proj_axis, pos_axis, upper_len,
			       lower_len, c, u, v, n);

  //
  // Build rotation matrix about the R joint (elbow)
  //  
  if (!solve_R_angle(ee, s, p_r1, Te, &r_angle))
    return 0;
  
  *rangle = (double)r_angle;

  // Find RY, and store the positions of the R jt and 
  // the ee in the R1 frame as p_r1 and ee_r1
  // Also save matrix product S*RY*T
  rotation_principal_axis_to_matrix('y', r_angle, RY);

  hmatmult(SRT, S, RY);
  hmatmult(SRT, SRT, Te);
  get_translation(SRT, ee_r1);

  return 1;
}

void solve_R1(p3d_vector3 p, p3d_vector3 q,
	      p3d_vector3 p2, p3d_vector3 q2,
	      double p_scale, p3d_matrix4 R1, int orientacio)
{
  // pot haver-hi problemes de noms
  p3d_matrix4 Ti, Si, TiTmp;
  p3d_vector3 pTmp,qTmp;

  // Construct two local coordinate systems
  // and find the transformation between them

  if(orientacio == 0)
  {
    pTmp[0] = p[0];
    pTmp[1] = -p[2];
    pTmp[2] = 0.0;
    qTmp[0] = q[0];
    qTmp[1] = -q[2];
    qTmp[2] = 0.0;

    make_frame(pTmp, p_scale, qTmp, TiTmp, 1);

  }
  if(orientacio == 1)
  {
    pTmp[0] = p[0];
    pTmp[1] = p[2];
    pTmp[2] = 0.0;
    qTmp[0] = q[0];
    qTmp[1] = q[2];
    qTmp[2] = 0.0;

    make_frame(pTmp, p_scale, qTmp, TiTmp, 1);

  }

  make_frame(p, p_scale, q, Ti, 1);
  make_frame(p2, p_scale, q2, Si, 0);

  if(orientacio == 0 || orientacio == 1)
  {
    rmatmult(R1Tmp,Ti,Si);
    rmatmult(R1,TiTmp,Si);
  }
  else
    rmatmult(R1,Ti,Si);

}

void SolveR1(double angle, p3d_matrix4 R1,int orientacio)
{
  p3d_vector3 p;

  evalcircle(angle, p);
  solve_R1(p_r1, ee_r1, p, ee, reciprocal_upper_len, R1, orientacio);
}

void SolveR1R2(double rangle, p3d_matrix4 R1, p3d_matrix4 R2, int orientacio)
{
  p3d_matrix4 temp;

  SolveR1(rangle, R1, orientacio);
  rmatmult(R2, SRT, R1Tmp);
  invertrmatrix(temp, R2);
  rmatmult(R2, G, temp);
}

//
// Constructor stores the T and S matrices and the 
// lengths of the upper and lower links
// 
int compute_angles(double angles_R1[3], double angles_R2[3], int  orientacio)
{
  double eangle;
  double Tx,Ty,Tz;
  double Rx,Ry,Rz,rRx,rRy,rRz;
  p3d_matrix4 R1Trans;
  p3d_matrix4 R2Trans;


  if (!SetGoal(G, &eangle))
  {
    //printf("No solution found\n");
    return 0;
  }
  else
  {
    SolveR1R2(swivel_angle,R1,R2,orientacio);
    p3d_mat4Transpose(R1, R1Trans);
    p3d_mat4Transpose(R2, R2Trans);

    p3d_mat4ExtractPosReverseOrder(TbaseRot, &Tx, &Ty, &Tz, &rRx, &rRy, &rRz);

    p3d_mat4ExtractPosReverseOrder(R1Trans, &Tx, &Ty, &Tz, &Rx, &Ry, &Rz);

    angles_R1[0] = Rx + rRx;
    angles_R1[1] = Ry - rRy;
    angles_R1[2] = Rz - rRz;

//     p3d_mat4ExtractPosReverseOrder(R2Trans, &Tx, &Ty, &Tz, &Rx, &Ry, &Rz);
// 
//     angles_R2[0] = Rx;
//     angles_R2[1] = Rz;
//     angles_R2[2] = Ry;

    return 1;
  }       
} 

void initialization(p3d_matrix4 T1, p3d_matrix4 T2, p3d_vector3 a, p3d_vector3 p)
{  
  p3d_vector3 temp;

  cpmatrixIKAN(Te,T1);
  cpmatrixIKAN(S,T2);  
  p3d_vectCopy(a, proj_axis);
  p3d_vectCopy(p, pos_axis);
  
  get_translation(Te, temp);

  upper_len = norm(temp);
  reciprocal_upper_len = 1 / upper_len;
 
  get_translation(S, temp);
  lower_len = norm(temp);
  
}

/* 
 * Compute inverse kinematics of human arm
 *
 * Output: q  
 * Inputs: d1 distance entre shoulder  elbow
 *         d2 distance entre elbow et wrist 
 */
int compute_inverse_kinematics_R7_human_arm (double *q,
					     p3d_matrix4 Tgrip,  // wTg
					     p3d_matrix4 Tbase,  // wTb 	  
					     p3d_matrix4 Tdiff,  // Tdiff->rotBase 	  
					     double s_angle,
					     double dis1,
					     double dis2,
					     int orientacio)
				      
{

  p3d_matrix4 RotGrip;
  double angles_R1[3] = {0, 0, 0};
  double angles_R2[3] = {0, 0, 0};
  p3d_matrix4 Tinv;
  p3d_matrix4 bTg;
  p3d_vector3 Xaxis = {1, 0, 0}; // p
  p3d_vector3 Yaxis = {0, 1, 0}; // a
  p3d_vector3 negYaxis = {0, -1, 0}; //a

  p3d_matrix4 T1 = {{ 1, 0, 0, 0 },
		    { 0, 1, 0, 0 },
		    { 0, 0, 1, 0 },
		    { 0, 0, dis1, 1}};
  
  p3d_matrix4 T2 = {{ 1, 0, 0, 0 },
		    { 0, 1, 0, 0 },
		    { 0, 0, 1, 0 },
		    { 0, 0, dis2, 1}};

  swivel_angle = s_angle;
  p3d_mat4Copy(Tdiff,TbaseRot);
  p3d_mat4Copy(Tgrip,RotGrip);

  if(orientacio == 0)
  {
    initialization(T1, T2, negYaxis, Xaxis);
  }
  else if(orientacio == 1)
  {
    initialization(T1, T2, Yaxis, Xaxis);
  }
  else
    return 0;

  p3d_matInvertXform(Tbase,Tinv);
  p3d_mat4Mult(Tinv,RotGrip,bTg);
  p3d_mat4Transpose(bTg,G);

  if (!compute_angles(angles_R1,angles_R2,orientacio))
    return 0;

  q[0] = angles_R1[0]; q[1] = angles_R1[1]; q[2] = angles_R1[2];
  q[3] = r_angle;
  q[4] = angles_R2[0]; q[5] = angles_R2[1]; q[6] = angles_R2[2];

  /* Ok */
  return 1;  
}
