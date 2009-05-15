
/*************************************************************************\

  Copyright 1995 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             S. Gottschalk
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:              {gottscha}@cs.unc.edu


\**************************************************************************/

#ifndef RAPID_H
#define RAPID_H

/****************************************************************************/


// RAPID API RETURN VALUES


const int RAPID_OK = 0; 
  // Used by all API routines except constructors and destructors.


const int RAPID_ERR_MODEL_OUT_OF_MEMORY = 1; 
  // Returned when an API function cannot obtain enough memory to store
  // or process a RAPID_model object.


const int RAPID_ERR_COLLIDE_OUT_OF_MEMORY = 2;
  // Returned when Collide() cannot allocate enough storage to hold
  // collision information.  In this case, there is as much collision
  // detection information available as could be allocated for.


const int RAPID_ERR_UNPROCESSED_MODEL = 3;
  // Returned when an unprocessed model is passed to a function which
  // expects only processed models, such as Collide().


const int RAPID_ERR_BUILD_OUT_OF_SEQUENCE = 4;
  // Returned when: 
  //       1. AddTri() is called before BeginModel().  The triangle will 
  //          be added anyway as if BeginModel() had been previously called.
  //       2. BeginModel() is called immediately after AddTri().  The
  //          model will be placed into an empty initial state.  
  // This error code is something like a warning: the invoked
  // operation takes place anyway, but the returned error code may tip
  // off the client that something out of the ordinary is happenning.


const int RAPID_ERR_BUILD_EMPTY_MODEL = 5; 
  // Returned when EndModel() is called on a model to which no
  // triangles have been added.  This is similar in spirit to the
  // OUT_OF_SEQUENCE return code, except that the requested operation
  // has FAILED -- the model remains "unprocessed".


/****************************************************************************/


class box;
struct tri;

class RAPID_model
{
public:

  // these are only for internal use

  box *b;
  int num_boxes_alloced;

  tri *tris;
  int num_tris;
  int num_tris_alloced;

  int build_state;
  
  int build_hierarchy();
  
  friend int Collide(double R1[3][3], double T1[3], 
		 double s1, RAPID_model *RAPID_model1,
		 double R2[3][3], double T2[3], 
		 double s2, RAPID_model *RAPID_model2,
		 int flag);
public:

  // these are for the client

  RAPID_model();
  ~RAPID_model();
  
  int BeginModel();
  int AddTri(const double *p1, const double *p2, const double *p3, int id);
  int EndModel();
  
};

/****************************************************************************/

// these are for the client

const int ALL_CONTACTS = 1;    // Find all pairwise intersecting triangles

const int FIRST_CONTACT = 2;   // Just report one intersecting triangle pair
                               //   if there are any.

// this is the collision query invocation.  It assumes that the 
// models are not being scaled up or down, but have their native
// dimensions.
int 
Collide(double R1[3][3], double T1[3], RAPID_model *o1,
	double R2[3][3], double T2[3], RAPID_model *o2,
	int flag = ALL_CONTACTS);

// this collision query permits the models to each be scaled by
// some nonnegative factor.
int 
Collide(double R1[3][3], double T1[3], double s1, RAPID_model *o1,
	double R2[3][3], double T2[3], double s2, RAPID_model *o2,
	int flag = ALL_CONTACTS);

// this is for the client
struct collision_pair
{
  int id1;
  int id2;
};

/****************************************************************************/

extern int RAPID_first_contact;
extern  int RAPID_num_box_tests;
extern  int RAPID_num_tri_tests;
extern  int RAPID_num_contacts;
extern  struct collision_pair *RAPID_contact;

#endif








