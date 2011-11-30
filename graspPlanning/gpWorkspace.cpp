
#include "GraspPlanning-pkg.h"
#include <vector>

gpSAHandInfo::gpSAHandInfo()
{
  q1min= -15*DEGTORAD;
  q1max=  15*DEGTORAD;
  q2min= -4*DEGTORAD;
  q2max=  75*DEGTORAD;
  q3min=   4*DEGTORAD;
  q3max=  75*DEGTORAD;
    // these bounds are reduced because a finger has no chance to ensure a good contact
    // close to the initial bounds:
//      q1min= -20*DEGTORAD;
//      q1max=  20*DEGTORAD;
//      q2min= 10*DEGTORAD;
//      q2max=  70*DEGTORAD;
//      q3min=   20*DEGTORAD;
//      q3max=  60*DEGTORAD;

  length1= 0.067816;
  length2= 0.029980;
  length3= 0.029;
}


//! @ingroup workspace 
//! Computes the position of the fingertip center point of the Schunk Anthropomorphic Hand finger.
//! NB: The coupling between the 3rd and 4th joints is taken into account in the computation.
//! \param length1 length of the first phalanx
//! \param length2 length of the second phalanx
//! \param length3 length of the third phalanx
//! \param q1 first joint parameter (controls the (palm)-(proximal phalanx) joint first DOF (abduction))
//! \param q2 second joint (controls the (palm)-(proximal phalanx) joint second DOF (subduction))
//! \param q3 third joint parameter (controls the DOFs of the last two joints that are coupled)
//! \param position the computed position of the fingertip center
//! \param normal a vector giving the direction of the fingertip contact surface (orthogonal to the medial axis of the distal phalanx and directed towards the inside of the hand)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSAHfinger_forward_kinematics(double length1, double length2, double length3, double q1, double q2, double q3, p3d_vector3 position, p3d_vector3 normal)
{
  double a;

  //reverse the abduction angle to fit the new model
  q1= -q1;

//   x= -sin(q1)*(  length1*cos(q2) + length2*cos(q2+q3) + length3*cos(q2+2*q3)  ); 
//   y=  cos(q1)*(  length1*cos(q2) + length2*cos(q2+q3) + length3*cos(q2+2*q3)  );
//   z= -length1*sin(q2) - length2*sin(q2+q3) - length3*sin(q2+2*q3);

  a= length1*cos(q2) + length2*cos(q2+q3) + length3*cos(q2+2*q3);

  position[0]= -sin(q1)*( a ); 
  position[1]=  cos(q1)*( a );
  position[2]= -length1*sin(q2) - length2*sin(q2+q3) - length3*sin(q2+2*q3);

  normal[0]=  sin(q1)*(sin(q2 + 2*q3));
  normal[1]= -cos(q1)*(sin(q2 + 2*q3));
  normal[2]= -cos(q2 + 2*q3);

  return GP_OK;
}

//! Computes the jacobian matrix of the SAH finger.
//! NB: The coupling between the 3rd and 4th joints is taken into account in the computation.
//! \param length1 length of the first phalanx (proximal phalanx)
//! \param length2 length of the second phalanx (middle phalanx)
//! \param length3 length of the fthird phalanx (distal phalanx)
//! \param q1 value of the first joint angle (abduction)
//! \param q2 value of the second joint angle (subduction)
//! \param q3 value of the thirs joint angle (flexion between proximal and middle phalanx)
//! \parm J the computed jacobian
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSAHfinger_jacobian(double length1, double length2, double length3, double q1, double q2, double q3, p3d_matrix3 J)
{
  double a, b, c;
  double dx_dq1, dx_dq2, dx_dq3;
  double dy_dq1, dy_dq2, dy_dq3;
  double dz_dq1, dz_dq2, dz_dq3;

  //reverse the abduction angle to fit the new model
  q1= -q1;

//   x= -sin(q1)*(  length1*cos(q2) + length2*cos(q2+q3) + length3*cos(q2+2*q3)  ); 
//   y=  cos(q1)*(  length1*cos(q2) + length2*cos(q2+q3) + length3*cos(q2+2*q3)  );
//   z= -length1*sin(q2) - length2*sin(q2+q3) - length3*sin(q2+2*q3);

  a= length1*cos(q2) + length2*cos(q2+q3) + length3*cos(q2+2*q3);
  b= length1*sin(q2) + length2*sin(q2+q3) + length3*sin(q2+2*q3);
  c= length2*sin(q2+q3) + 2*length3*sin(q2+2*q3);

  dx_dq1= cos(q1)*( a );
  dx_dq2= -sin(q1)*( b );
  dx_dq3= -sin(q1)*( c );

  dy_dq1= sin(q1)*( a );
  dy_dq2= cos(q1)*( b );
  dy_dq3= cos(q1)*( c );

  dz_dq1= 0;
  dz_dq2= a;
  dz_dq3= length2*cos(q2+q3) + 2*length3*cos(q2+2*q3);

  J[0][0]= dx_dq1;   J[0][1]= dx_dq2;   J[0][2]= dx_dq3;
  J[1][0]= dy_dq1;   J[1][1]= dy_dq2;   J[1][2]= dy_dq3;
  J[2][0]= dz_dq1;   J[2][1]= dz_dq2;   J[2][2]= dz_dq3;

  return GP_OK;
}

int gpSAHfinger_manipulability_ellipsoid(double length1, double length2, double length3, double q1, double q2, double q3)
{
  p3d_matrix3 J, U, V;
  p3d_matrix4 T;
  p3d_vector3 S;
  GLfloat mat[16];

  gpSAHfinger_jacobian(length1, length2, length3, q1, q2, q3, J);

  p3d_mat3SVD(J, U, S, V);

//   gpGet_SAHfinger_joint_angles(robot, handProp, double q[4], int finger_index, int handID)
  p3d_mat4Copy(p3d_mat4IDENTITY, T);
  p3d_mat4SetRotMatrix(U, T);
  p3d_to_gl_matrix(T, mat);

  glPushMatrix();
   glMultMatrixf(mat);
    g3d_draw_ellipsoid(S[0], S[1], S[2], 10);
  glPopMatrix();

  return GP_OK;
}

static float *new_vector(long nl, long nh);
static void free_vector(float *v, long nl, long nh);
static float **matrix(long nrl, long nrh, long ncl, long nch);
static void free_matrix(float **m, long nrl, long nrh, long ncl, long nch);
static float pythag(float a, float b);

//! The following macros come from "Numerical Recipes in C", (nrutil.h, appendix B).
#define FSIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define FMAX(a,b) ( (a) > (b) ? (a) : (b) )
#define IMIN(a,b) ( (a) < (b) ? (a) : (b) )

//! The following function comes from "Numerical Recipes in C", chapter 2.
//! Computes (a2 + b2)1/2 without destructive underflow or overflow.
float pythag(float a, float b)
{
  float absa, absb;
  absa= fabs(a);
  absb= fabs(b);
  if (absa > absb) return absa*sqrt(1.0 + SQR(absb/absa));
  else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0 + SQR(absa/absb)));
}

//! The following function comes from "Numerical Recipes in C". (nrutil.c, appendix B).
//! Numerical Recipes standard error handler
void nrerror(char error_text[])
{
  fprintf(stderr,"Numerical Recipes run-time error...\n");
  fprintf(stderr,"%s\n",error_text);
  fprintf(stderr,"...now exiting to system...\n");
  exit(1);
}

//! The following function comes from "Numerical Recipes in C". (nrutil.c, appendix B).
//! allocate a float vector with subscript range v[nl..nh]
float *new_vector(long nl, long nh)
{
  float *v;
  v=(float *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(float)));
  if (!v) nrerror((char*)"allocation failure in vector()");
  return v-nl+NR_END;
}

//! The following function comes from "Numerical Recipes in C". (nrutil.c, appendix B).
//! free a float vector allocated with new_vector() 
void free_vector(float *v, long nl, long nh)
{
  free((FREE_ARG) (v+nl-NR_END));
}
//! The following function comes from "Numerical Recipes in C". (nrutil.c, appendix B).
//! allocate a float matrix with subscript range m[nrl..nrh][ncl..nch]
float **matrix(long nrl, long nrh, long ncl, long nch)
{
  long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
  float **m;
  /* allocate pointers to rows */
  m=(float **) malloc((size_t)((nrow+NR_END)*sizeof(float*)));
  if (!m) nrerror((char*)"allocation failure 1 in matrix()");
  m += NR_END;
  m -= nrl;
  /* allocate rows and set pointers to them */
  m[nrl]=(float *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(float)));
  if (!m[nrl]) nrerror((char*)"allocation failure 2 in matrix()");
  m[nrl] += NR_END;
  m[nrl] -= ncl;
  for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;
  /* return pointer to array of pointers to rows */
  return m;
}

//! The following function comes from "Numerical Recipes in C". (nrutil.c, appendix B).
//! free a float matrix allocated by matrix()
void free_matrix(float **m, long nrl, long nrh, long ncl, long nch)
{
  free((FREE_ARG) (m[nrl]+ncl-NR_END));
  free((FREE_ARG) (m+nrl-NR_END));
}

//! The following function comes from "Numerical Recipes in C", chapter 2.
//! Given a matrix a[1..m][1..n], this routine computes its singular value decomposition, A =
//! U·W·V T. The matrix U replaces a on output. The diagonal matrix of singular values W is output
//! as a vector w[1..n]. The matrix V (not the transpose V T ) is output as v[1..n][1..n].
void svdcmp(float **a, int m, int n, float w[], float **v)
{
  float pythag(float a, float b);
  int flag,i,its,j,jj,k,l,nm;
  float anorm,c,f,g,h,s,scale,x,y,z,*rv1;
  rv1= new_vector(1,n);
  g=scale=anorm=0.0; //Householder reduction to bidiagonal form.
  for (i=1;i<=n;i++) {
    l=i+1;
    rv1[i]=scale*g;
    g=s=scale=0.0;
    if (i <= m) {
       for (k=i;k<=m;k++) scale += fabs(a[k][i]);
         if (scale) {
           for (k=i;k<=m;k++) {
            a[k][i] /= scale;
            s += a[k][i]*a[k][i];
           }
           f=a[i][i];
           g = -FSIGN(sqrt(s),f);
           h=f*g-s;
           a[i][i]=f-g;
           for (j=l;j<=n;j++) {
             for (s=0.0,k=i;k<=m;k++) s += a[k][i]*a[k][j];
             f=s/h;
             for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
           }
           for (k=i;k<=m;k++) a[k][i] *= scale;
         }
    }
    w[i]=scale *g;
    g=s=scale=0.0;
    if (i <= m && i != n) {
      for (k=l;k<=n;k++) scale += fabs(a[i][k]);
      if (scale) {
        for (k=l;k<=n;k++) {
          a[i][k] /= scale;
          s += a[i][k]*a[i][k];
        }
        f=a[i][l];
        g = -FSIGN(sqrt(s),f);
        h=f*g-s;
        a[i][l]=f-g;
        for (k=l;k<=n;k++) rv1[k]=a[i][k]/h;
        for (j=l;j<=m;j++) {
          for (s=0.0,k=l;k<=n;k++) s += a[j][k]*a[i][k];
          for (k=l;k<=n;k++) a[j][k] += s*rv1[k];
        }
        for (k=l;k<=n;k++) a[i][k] *= scale;
      }
    }
    anorm=FMAX(anorm,(fabs(w[i])+fabs(rv1[i])));
  }

  for (i=n;i>=1;i--) { //Accumulation of right-hand transformations.
    if (i < n) {
      if (g) {
        for (j=l;j<=n;j++) //Double division to avoid possible underflow.
          v[j][i]=(a[i][j]/a[i][l])/g;
        for (j=l;j<=n;j++) {
          for (s=0.0,k=l;k<=n;k++) s += a[i][k]*v[k][j];
          for (k=l;k<=n;k++) v[k][j] += s*v[k][i];
        }
      }
      for (j=l;j<=n;j++) v[i][j]=v[j][i]=0.0;
    }
    v[i][i]=1.0;
    g=rv1[i];
    l=i;
  }

  for (i=IMIN(m,n);i>=1;i--) { //Accumulation of left-hand transformations.
   l=i+1;
   g=w[i];
   for (j=l;j<=n;j++) a[i][j]=0.0;
   if (g) {
     g=1.0/g;
     for (j=l;j<=n;j++) {
       for (s=0.0,k=l;k<=m;k++) s += a[k][i]*a[k][j];
       f=(s/a[i][i])*g;
       for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
     }
     for (j=i;j<=m;j++) a[j][i] *= g;
   } else for (j=i;j<=m;j++) a[j][i]=0.0;
   ++a[i][i];
  }

  for (k=n;k>=1;k--) { //Diagonalization of the bidiagonal form: Loop over
//singular values, 
    for (its=1;its<=30;its++) { //and over allowed iterations.
      flag=1;
      for (l=k;l>=1;l--) { //Test for splitting.
        nm=l-1; //Note that rv1[1] is always zero.
        if ((float)(fabs(rv1[l])+anorm) == anorm) {
          flag=0;
          break;
        }
        if ((float)(fabs(w[nm])+anorm) == anorm) break;
      }
      if (flag) {
        c=0.0; //Cancellation of rv1[l], if l > 1.
        s=1.0;
        for (i=l;i<=k;i++) {
          f=s*rv1[i];
          rv1[i]=c*rv1[i];
          if ((float)(fabs(f)+anorm) == anorm) break;
          g=w[i];
          h=pythag(f,g);
          w[i]=h;
          h=1.0/h;
          c=g*h;
          s = -f*h;
          for (j=1;j<=m;j++) {
            y=a[j][nm];
            z=a[j][i];
            a[j][nm]=y*c+z*s;
            a[j][i]=z*c-y*s;
          }
        }
      }
      z=w[k];
      if (l == k) { //Convergence.
        if (z < 0.0) { //Singular value is made nonnegative.
         w[k] = -z;
         for (j=1;j<=n;j++) v[j][k] = -v[j][k];
        }
        break;
      }
      if (its == 30) nrerror((char*)"no convergence in 30 svdcmp iterations");
         x=w[l]; //Shift from bottom 2-by-2 minor.
         nm=k-1;
         y=w[nm];
         g=rv1[nm];
         h=rv1[k];
         f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
         g=pythag(f,1.0);
         f=((x-z)*(x+z)+h*((y/(f+FSIGN(g,f)))-h))/x;
         c=s=1.0; //Next QR transformation:
         for (j=l;j<=nm;j++) {
            i=j+1;
            g=rv1[i];
            y=w[i];
            h=s*g;
            g=c*g;
            z=pythag(f,h);
            rv1[j]=z;
            c=f/z;
            s=h/z;
            f=x*c+g*s;
            g = g*c-x*s;
            h=y*s;
            y *= c;
            for (jj=1;jj<=n;jj++) {
               x=v[jj][j];
               z=v[jj][i];
               v[jj][j]=x*c+z*s;
               v[jj][i]=z*c-x*s;
            }
            z=pythag(f,h);
            w[j]=z; //Rotation can be arbitrary if z = 0.
            if (z) {
              z=1.0/z;
              c=f*z;
              s=h*z;
            }
            f=c*g+s*y;
            x=c*y-s*g;
            for (jj=1;jj<=m;jj++) {
              y=a[jj][j];
              z=a[jj][i];
              a[jj][j]=y*c+z*s;
              a[jj][i]=z*c-y*s;
            }
         }
         rv1[l]=0.0;
         rv1[k]=f;
         w[k]=x;
      } 
  }
  free_vector(rv1,1,n);
}

//! Computes the SVD of a 3x3 matrix.
//! i.e. from a matrix M,  computes matrices U,S,V such that M= U * S * transpose(V),
//! with S a diagonal matrix.
//! \param M input matrix
//! \param U eigenvectors matrix
//! \param S vector of the eigenvalues (diagonal terms of matrix S)
//! \param V eigenvectors matrix
void p3d_mat3SVD(p3d_matrix3 M, p3d_matrix3 U, p3d_vector3 S, p3d_matrix3 V)
{
  int nbRows, nbCols;

  nbRows= nbCols= 3;

  float **a;
  float **v;
  float *w;

  a= matrix(1, nbRows, 1, nbCols);
  v= matrix(1, nbCols, 1, nbCols);
  w= new_vector(1, nbRows);

 
  a[1][1]= M[0][0];  a[1][2]= M[0][1];   a[1][3]= M[0][2]; 
  a[2][1]= M[1][0];  a[2][2]= M[1][1];   a[2][3]= M[1][2]; 
  a[3][1]= M[2][0];  a[3][2]= M[2][1];   a[3][3]= M[2][2]; 

  svdcmp(a, nbRows, nbCols, w, v);

  U[0][0]= a[1][1];  U[0][1]= a[1][2];   U[0][2]= a[1][3]; 
  U[1][0]= a[2][1];  U[1][1]= a[2][2];   U[1][2]= a[2][3]; 
  U[2][0]= a[3][1];  U[2][1]= a[3][2];   U[2][2]= a[3][3]; 

  V[0][0]= v[1][1];  V[0][1]= v[1][2];   V[0][2]= v[1][3]; 
  V[1][0]= v[2][1];  V[1][1]= v[2][2];   V[1][2]= v[2][3]; 
  V[2][0]= v[3][1];  V[2][1]= v[3][2];   V[2][2]= v[3][3];

  S[0]= w[1];
  S[1]= w[2];
  S[2]= w[3];

  free_matrix(a, 1, nbRows, 1, nbCols);
  free_matrix(v, 1, nbCols, 1, nbCols);
  free_vector(w, 1, nbRows);
}

//! Computes the SVD of a 4x4 matrix.
//! i.e. from a matrix M,  computes matrices U,S,V such that M= U * S * transpose(V),
//! with S a diagonal matrix.
//! \param M input matrix
//! \param U eigenvectors matrix
//! \param S vector of the eigenvalues (diagonal terms of matrix S)
//! \param V eigenvectors matrix
void p3d_mat4SVD(p3d_matrix4 M, p3d_matrix4 U, p3d_vector4 S, p3d_matrix4 V)
{
  int nbRows, nbCols;

  nbRows= nbCols= 4;

  float **a;
  float **v;
  float *w;

  a= matrix(1, nbRows, 1, nbCols);
  v= matrix(1, nbCols, 1, nbCols);
  w= new_vector(1, nbRows);

 
  a[1][1]= M[0][0];  a[1][2]= M[0][1];   a[1][3]= M[0][2];  a[1][4]= M[0][3]; 
  a[2][1]= M[1][0];  a[2][2]= M[1][1];   a[2][3]= M[1][2];  a[2][4]= M[1][3]; 
  a[3][1]= M[2][0];  a[3][2]= M[2][1];   a[3][3]= M[2][2];  a[3][4]= M[2][3];  

  svdcmp(a, nbRows, nbCols, w, v);

  U[0][0]= a[1][1];  U[0][1]= a[1][2];   U[0][2]= a[1][3];  U[0][3]= a[1][4]; 
  U[1][0]= a[2][1];  U[1][1]= a[2][2];   U[1][2]= a[2][3];  U[1][3]= a[2][4]; 
  U[2][0]= a[3][1];  U[2][1]= a[3][2];   U[2][2]= a[3][3];  U[2][3]= a[3][4]; 

  V[0][0]= v[1][1];  V[0][1]= v[1][2];   V[0][2]= v[1][3];  V[0][3]= v[1][4]; 
  V[1][0]= v[2][1];  V[1][1]= v[2][2];   V[1][2]= v[2][3];  V[1][3]= v[2][4]; 
  V[2][0]= v[3][1];  V[2][1]= v[3][2];   V[2][2]= v[3][3];  V[2][3]= v[3][4]; 

  S[0]= w[1];
  S[1]= w[2];
  S[2]= w[3];
  S[3]= w[4];

  free_matrix(a, 1, nbRows, 1, nbCols);
  free_matrix(v, 1, nbCols, 1, nbCols);
  free_vector(w, 1, nbRows);
}

//! @ingroup workspace 
//! Computes the outer envelope of the workspace of a SAHand finger as a point cloud.
//! \param data geometrical info about the finger
//! \param dq discretization step of the joint parameters (in radians)
//! \param points computed point cloud
//! \param normal fingertip surface normal at each point of the workspace 
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSAHfinger_outer_workspace(gpSAHandInfo data, double dq, std::vector<gpVector3D> &points, std::vector<gpVector3D> &normals)
{
  unsigned int i, j, n1, n2, n3;
  double q1, q2, q3;
  double q1min, q1max, q2min, q2max, q3min, q3max;
  p3d_vector3 p, n;
  gpVector3D point, normal;
  std::list<gpVector3D> pointList, normalList;
  std::list<gpVector3D>::iterator iter;

  q1min= data.q1min;
  q1max= data.q1max;
  q2min= data.q2min;
  q2max= data.q2max;
  q3min= data.q3min;
  q3max= data.q3max;

  n1= (unsigned int) ( (q1max-q1min)/dq );
  n2= (unsigned int) ( (q2max-q2min)/dq );
  n3= (unsigned int) ( (q3max-q3min)/dq );

  for(i=0; i<=n1; ++i)
  {
    q1= q1min + i*dq;

    for(j=0; j<=n3; ++j)
    {
      q3= q3min + j*dq;

      gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1, q2min, q3, p, n);
      point.x= p[0];
      point.y= p[1];
      point.z= p[2];
      pointList.push_back(point);
      normal.x= n[0];
      normal.y= n[1];
      normal.z= n[2];
      normalList.push_back(normal);

      gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1, q2max, q3, p, n);
      point.x= p[0];
      point.y= p[1];
      point.z= p[2];
      pointList.push_back(point);
      normal.x= n[0];
      normal.y= n[1];
      normal.z= n[2];
      normalList.push_back(normal);
    }  

    for(j=0; j<=n2; ++j)
    {
      q2= q2min + j*dq;
      gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1, q2, q3min, p, n);
      point.x= p[0];
      point.y= p[1];
      point.z= p[2];
      pointList.push_back(point);
      normal.x= n[0];
      normal.y= n[1];
      normal.z= n[2];
      normalList.push_back(normal);

      gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1, q2, q3max, p, n);
      point.x= p[0];
      point.y= p[1];
      point.z= p[2];
      pointList.push_back(point);
      normal.x= n[0];
      normal.y= n[1];
      normal.z= n[2];
      normalList.push_back(normal);
    }
  }

  for(i=0; i<=n2; ++i)
  {
    for(j=0; j<=n3; ++j)
    {
      q2= q2min + i*dq;
      q3= q3min + j*dq;
      q1= q1min;
      gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1min, q2, q3, p, n);
      point.x= p[0];
      point.y= p[1];
      point.z= p[2];
      pointList.push_back(point);
      normal.x= n[0];
      normal.y= n[1];
      normal.z= n[2];
      normalList.push_back(normal);

      gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1max, q2, q3, p, n);
      point.x= p[0];
      point.y= p[1];
      point.z= p[2];
      pointList.push_back(point);
      normal.x= n[0];
      normal.y= n[1];
      normal.z= n[2];
      normalList.push_back(normal);
    }
  }

  points.resize(pointList.size());
  i= 0;
  for(iter=pointList.begin(); iter!=pointList.end(); iter++)
  {
    points.at(i)= *iter;
    i++;
  }
  normals.resize(normalList.size());
  i= 0;
  for(iter=normalList.begin(); iter!=normalList.end(); iter++)
  {
    normals.at(i)= *iter;
    i++;
  }

  return GP_OK;
}


//! @ingroup workspace 
//! Draws the outer envelope of the workspace of a SAHand finger as a point cloud.
//! \param data geometrical info about the finger
//! \param dq discretization step of the joint parameters (in radians)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDraw_SAHfinger_outer_workspace(gpSAHandInfo data, double dq)
{
  std::vector<gpVector3D> points, normals;

  gpSAHfinger_outer_workspace(data, dq, points, normals);

  unsigned int i;

  glPushAttrib(GL_POINT_BIT);
  glPointSize(4);

  glBegin(GL_POINTS);
   for(i=0; i<points.size(); ++i)
   {
//      glNormal3d(normals[i].x, normals[i].y, normals[i].z);
     glNormal3d(points[i].x, points[i].y, points[i].z);
     glVertex3d(points[i].x, points[i].y, points[i].z);
   }
  glEnd();

//   glColor3f(1,0,0);
//   glBegin(GL_LINES);
//    for(i=0; i<points.size(); ++i)
//    {
//      glVertex3d(points[i].x, points[i].y, points[i].z);
//      glVertex3d(points[i].x+0.01*normals[i].x, points[i].y+0.01*normals[i].y, points[i].z+0.01*normals[i].z);
//    }
//   glEnd();

  glPopAttrib();

  return GP_OK;
}


//! @ingroup workspace 
//! Computes the workspace of a SAHand finger as a point cloud.
//! \param data geometrical info about the finger
//! \param dq discretization step of the joint parameters (in radians)
//! \param points computed point cloud
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSAHfinger_workspace(gpSAHandInfo data, double dq, std::vector<gpVector3D> &points)
{
  unsigned int i, j, k, n1, n2, n3;
  double q1, q2, q3;
  double q1min, q1max, q2min, q2max, q3min, q3max;
  p3d_vector3 p, n;
  gpVector3D point;
  std::list<gpVector3D> pointList;
  std::list<gpVector3D>::iterator iter;

  q1min= data.q1min;
  q1max= data.q1max;
  q2min= data.q2min;
  q2max= data.q2max;
  q3min= data.q3min;
  q3max= data.q3max;

  n1= (unsigned int) ( (q1max-q1min)/dq );
  n2= (unsigned int) ( (q2max-q2min)/dq );
  n3= (unsigned int) ( (q3max-q3min)/dq );

  for(i=0; i<=n1; ++i)
  {
    q1= q1min + i*dq;
    for(j=0; j<=n2; ++j)
    {
      q2= q2min + j*dq;
      for(k=0; k<=n3; ++k)
      {
        q3= q3min + k*dq;
        gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1, q2, q3, p, n);
        point.x= p[0];
        point.y= p[1];
        point.z= p[2];
        pointList.push_back(point);
      }
    }  
  }

  points.resize(pointList.size());
  i= 0;
  for(iter=pointList.begin(); iter!=pointList.end(); iter++)
  {
    points.at(i)= *iter;
    i++;
  }

  return GP_OK;
}


//! @ingroup workspace 
//! Computes an approximation of the SAHand finger workspace as a set of spheres.
//! All the spheres are completely included in the real workspace.
//! The function builds a set of points inside the workspace (inner points) and a set of points
//! on the workspace boundary (boundary points).
//! The minimal distance from each inner point to the boundary points is computed.
//! The first sphere center is the inner point with the maximal distance (that will be the sphere radius).
//! All the inner points that are inside the sphere are removed. 
//! The new distance of each remaining points is the minimum of the old distance and the distance
//! from the point to the sphere.
//! The new sphere center is the inner point with the maximal distance and the algorithm continues
//! with the same principle until the maximal number is reached ro the last computed sphere radius
//! is smaller than the given threshold.
//! \param data geometrical info about the finger
//! \param dq discretization step of the joint parameters (in radians). Do not use less than ~2 degrees (pi/180 rads)
//! to avoid memory overload and excessive computation time.
//! \param dr the algorithm will end when it has found a sphere with radius < dr
//! \param nb_spheres_max maximal number of spheres that will be computed
//! \param spheres computed sphere set
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSAHfinger_workspace_approximation(gpSAHandInfo data, double dq, double dr, unsigned int nb_spheres_max, std::vector<gpSphere> &spheres)
{
  unsigned int i, j;
  double distance, dmin;
  gpVector3D best;
  gpSphere sphere;
  std::vector<gpVector3D> inner, boundary, normals;
  std::list<gpVector3D> innerList;
  std::list<gpVector3D>::iterator iterPoint;
  std::list<gpSphere> sphereList;
  std::list<gpSphere>::iterator iterSphere;

  gpSAHfinger_workspace(data, dq, inner);
  gpSAHfinger_outer_workspace(data, dq, boundary, normals);

  for(i=0; i<inner.size(); ++i)
  {
    for(j=0; j<boundary.size(); ++j)
    {
      distance= sqrt( pow(inner[i].x-boundary[j].x, 2) + pow(inner[i].y-boundary[j].y, 2) + pow(inner[i].z-boundary[j].z, 2) );
      if(distance < dmin || j==0)
      {
        dmin= distance;
        inner[i].cost= distance;
      }
    }
    innerList.push_back(inner[i]);
  }

  innerList.sort();

  while(true && !innerList.empty())
  {
    best= innerList.back();
    sphere.center[0]= best.x;
    sphere.center[1]= best.y;
    sphere.center[2]= best.z;
    sphere.radius   = best.cost;
    
    sphereList.push_back(sphere);
    if(sphere.radius < dr || sphereList.size() > nb_spheres_max)
    {  break;  }

    innerList.pop_back();

    //removes points that are inside the sphere and computes the noew distance:
    iterPoint= innerList.begin();
    while(iterPoint!=innerList.end())
    {
      distance= sqrt( pow(iterPoint->x-sphere.center[0], 2) + pow(iterPoint->y-sphere.center[1], 2) + pow(iterPoint->z-sphere.center[2], 2) );
      if( distance <= sphere.radius)
      {
        iterPoint= innerList.erase(iterPoint);
        continue;
      }
      if( (distance-sphere.radius) < iterPoint->cost )
      {
        iterPoint->cost= (distance-sphere.radius);
      }
      iterPoint++;
    }
    innerList.sort();
  }

  spheres.resize(sphereList.size());
  i= 0;
  for(iterSphere=sphereList.begin(); iterSphere!=sphereList.end(); iterSphere++)
  {
    spheres.at(i)= *iterSphere;
    printf("       workspace.at(%d).setCenter(%f, %f, %f); \n", i, spheres.at(i).center[0], spheres.at(i).center[1], spheres.at(i).center[2]);
    printf("       workspace.at(%d).radius= %f; \n", i, spheres.at(i).radius);
    i++;
  }


  return GP_OK;
}


//! Computes the direction with the best force transmission at the fingertip (main axis of the force ellipsoid) of a finger of the SAH hand.
//! \param Twrist hand pose (frame of the wrist center) in the world frame
//! \param handProp structure containing information about the hand geometry
//! \param q the finger joint parameters (angles in radians). Only the three last elements are used. The first one is assumed to be 0 for the thumb.
//! \param finger_index index of the finger (thumb=1,forefinger=2,middlefinger=3,ringfinger=4)
//! \param direction the computed direction
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSAHfinger_main_force_direction(p3d_matrix4 Twrist, gpHand_properties &handProp, double q[4], int finger_index, p3d_vector3 direction)
{
  if(finger_index<1 || finger_index>4 )
  {
    printf("%s: %d: gpSAHfinger_main_force_direction(): finger_index must be >= 1 and <=4 (finger_index= %d).\n",__FILE__,__LINE__, finger_index);
    return GP_ERROR;
  }

  double l0, l1, l2, l3;
  p3d_vector3 S, dir_finger;
  p3d_matrix3 J, U, V;
  p3d_matrix4 Tfinger_world;

  switch(handProp.type)
  {
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
     l0= handProp.length_thumbBase;
     l1= handProp.length_proxPha;
     l2= handProp.length_midPha;
     l3= handProp.length_distPha;

     p3d_mat4Mult(Twrist, handProp.Twrist_finger[finger_index-1], Tfinger_world);
    break;
    default:
       printf("%s: %d: gpSAHfinger_main_force_direction(): this function only applies to GP_SAHAND_RIGHT et GP_SAHAND_LEFT hands.\n", __FILE__, __LINE__);
       return GP_ERROR;
    break;
  }

  gpSAHfinger_jacobian(l1, l2, l3, q[1], q[2], q[3], J);

  // to remove the abduction movement:
  J[0][0]= 0.0;
  J[1][0]= 0.0;
  J[2][0]= 0.0;

  p3d_mat3SVD(J, U, S, V);

  dir_finger[0]= U[0][1];
  dir_finger[1]= U[1][1];
  dir_finger[2]= U[2][1];
  p3d_vectNormalize(dir_finger, dir_finger);

  p3d_xformVect(Tfinger_world, dir_finger, direction);
  

  return GP_OK;
}

//! Draws the force ellipsoid of a finger of the SAH hand.
//! \param robot pointer to the robot hand
//! \param handProp geometric data of the hand; must be initialized with initialize(GP_SAHAND_RIGHT);
//! \param finger_index index of the finger (thumb=1,forefinger=2,middlefinger=3,ringfinger=4)
//! \param handID used to find the finger joints in the robot
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDraw_SAHfinger_manipulability_ellipsoid(p3d_rob *robot, gpHand_properties &handProp, int finger_index, int handID)
{
  if(robot==NULL)
  {
    printf("%s: %d: gpDraw_SAHfinger_manipulability_ellipsoid(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(finger_index<1 || finger_index>4 )
  {
    printf("%s: %d: gpDraw_SAHfinger_manipulability_ellipsoid(): finger_index must be >= 1 and <=4 (finger_index= %d).\n",__FILE__,__LINE__, finger_index);
    return GP_ERROR;
  }

  double q[4];
  p3d_matrix3 J, U, V;
  p3d_matrix4 T;
  p3d_vector3 S;
  p3d_vector3 position, normal;
  GLfloat mat1[16], mat2[16], mat3[16];
  p3d_matrix4 Twrist;

  gpGet_wrist_frame(robot, Twrist);

  p3d_to_gl_matrix(Twrist, mat1);
  p3d_to_gl_matrix(handProp.Twrist_finger[finger_index-1], mat2);

  gpGet_SAHfinger_joint_angles(robot, handProp, q, finger_index, handID);

  gpSAHfinger_forward_kinematics(handProp.length_proxPha, handProp.length_midPha, handProp.length_distPha, q[1], q[2], q[3], position, normal);

  gpSAHfinger_jacobian(handProp.length_proxPha, handProp.length_midPha, handProp.length_distPha, q[1], q[2], q[3], J);

  // to remove the abduction movement:
  J[0][0]= 0.0;
  J[1][0]= 0.0;
  J[2][0]= 0.0;

  p3d_mat3SVD(J, U, S, V);

  for(int i=0; i<3; ++i)
  {
   for(int j=0; j<3; ++j)
   {  U[i][j]= -U[i][j];   }
  }
  //p3d_mat3Print(U, "U");


  p3d_mat4Copy(p3d_mat4IDENTITY, T);
  p3d_mat4SetRotMatrix(U, T);
  p3d_to_gl_matrix(T, mat3);


//   switch(finger_index) 
//   {
//     case 1:   glColor4f(1.0, 0.0, 0.0, 0.5);  break;
//     case 2:   glColor4f(0.0, 1.0, 0.0, 0.5);  break;
//     case 3:   glColor4f(1.0, 0.0, 1.0, 0.5);  break;
//     case 4:   glColor4f(1.0, 1.0, 0.0, 0.5);  break;
//     default:  glColor4f(1.0, 0.0, 1.0, 0.5);  break;
//   }

 p3d_vector3 p1={0.0,0.0,0.0}, p2;
 p3d_mat4ExtractColumnY(T, p2);
 p2[0]*= -0.1;
 p2[1]*= -0.1;
 p2[2]*= -0.1;

 glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT);
  
 glColor4f(0.0, 1.0, 0.0, 0.6);
 glDisable(GL_LIGHTING);
 glEnable(GL_CULL_FACE);
 glEnable(GL_BLEND);
 glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 glDepthMask(GL_FALSE);
  glPushMatrix();
   glMultMatrixf(mat1);
   glMultMatrixf(mat2);
   glTranslatef(position[0], position[1], position[2]);

   g3d_draw_cylinder(p1, p2, 0.002, 8);

   glMultMatrixf(mat3);
//     g3d_draw_ellipsoid(S[0]/3.3, S[1]/3.3, S[2]/3.3, 30);
    g3d_draw_ellipsoid(0.01, S[1]/3.3, S[2]/3.3, 30);
  glPopMatrix();
// glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
  glDepthMask(GL_TRUE);
  glDisable(GL_CULL_FACE);

  glPopAttrib();

  return GP_OK;
}

//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDraw_reachable_points(p3d_rob *robot, p3d_rob *object, gpHand_properties &handProp)
{
  if(robot==NULL)
  {
    printf("%s: %d: gpDraw_reachable_points(): input robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(object==NULL)
  {
    printf("%s: %d: gpDraw_reachable_points(): input object is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  unsigned int i, j;
  static bool firstTime= true;
  static gpKdTree kdtree;
  static std::list<gpContact> contactList;
  std::list<gpContact> points;
  std::list<gpContact>::iterator iter;
  double colors[4][3]= {{1.0,0.0,0.0},{0.0,1.0,0.0},{0.0,0.0,1.0},{1.0,0.0,1.0}};
  p3d_vector3 center;
  p3d_matrix4 Twrist, Tobject, Tobject_inv, T, Ttmp;
  GLfloat mat[16];

  if(firstTime)
  {
//     gpSample_obj_surface(object->o[0], 0.005, handProp.fingertip_radius, contactList);
    gpSample_obj_surface(object->o[0], 0.002, 0.0, contactList);

    FILE *file= fopen("/home/jpsaut/BioMove3Dgit/BioMove3D/pointCLoud", "w+");
    for(std::list<gpContact>::iterator iter=contactList.begin(); iter!=contactList.end(); ++iter)
    {
     fprintf(file, "v %f %f %f\n", iter->position[0], iter->position[1], iter->position[2]);
    }
    fclose(file);

    kdtree.build(contactList);
    firstTime= false;
printf("contacts %lu\n",(unsigned long int)contactList.size());
  }



  p3d_get_freeflyer_pose(robot, Twrist);
  p3d_get_freeflyer_pose(object, Tobject);
  p3d_matInvertXform(Tobject, Tobject_inv);
  p3d_to_gl_matrix(Tobject, mat);

//   glPushAttrib(GL_LIGHTING_BIT | GL_POINT_BIT);
//   glDisable(GL_LIGHTING);
//   glPointSize(5);
//   glPushMatrix();
//   glMultMatrixf(mat);
//     glBegin(GL_POINTS);
//       for(iter=contactList.begin(); iter!=contactList.end(); ++iter)
//       {
//         glVertex3dv(iter->position);
//       }
//     glEnd();
//   glPopMatrix();
//   glPopAttrib();
// return GP_OK;


  glPushAttrib(GL_LIGHTING_BIT | GL_POINT_BIT);
  glDisable(GL_LIGHTING);
  glPointSize(5);

  for(i=0; i<4; ++i) //for each finger:
  {
    glColor3f(colors[i][0], colors[i][1], colors[i][2]);

    p3d_mat4Mult(Twrist, handProp.Twrist_finger[i], Ttmp);
    p3d_mat4Mult(Tobject_inv, Ttmp, T);

    p3d_to_gl_matrix(Tobject, mat);

    for(j=0; j<handProp.workspace.size(); ++j)
    {
      p3d_xformPoint(T, handProp.workspace[j].center, center);
      points.clear();
      kdtree.sphereIntersection(center, handProp.workspace[j].radius, points);

      glPushMatrix();
      glMultMatrixf(mat);
        glBegin(GL_POINTS);
         for(iter=points.begin(); iter!=points.end(); ++iter)
         {
           glVertex3dv(iter->position);
         }
        glEnd();
      glPopMatrix();
    }
  }

  glPopAttrib();

  return GP_OK;
}
