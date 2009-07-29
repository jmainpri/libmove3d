#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"


static int MUST_DRAW_ALL_OBBS  = FALSE;
static int MUST_DRAW_ROB_OBBS  = FALSE;
static int MUST_DRAW_AABB_TREE = FALSE;
static int MUST_DRAW_ALL_AABBS = FALSE;
static int KCD_MUST_SHOW_INFO = FALSE;



/* RGB Couleurs utilisees */
static double Whitev[4] = {1.0,1.0,1.0,0.5 };
static double Blackv[4] = {.0,.0,.0,0.5 };
static double Bluev[4] =  {.0,.0,1.0,0.5};
static double Redv[4]  =  {1.0,.0,.0,0.5};
static double Yellowv[4] =  {1.0,1.0,.0,0.5};
static double Greenv[4] =  {0.0,1.0,.0,0.5};
static double Greyv[4] = {0.7,0.7,0.7,0.5};
static double Brownv[4] =  {1.0,1.0,0.5,0.5};
static double Violetv[4] =  {1.0,0.0,1.0,0.5};
static double Blue2v[4] =  {0.0,1.0,1.0,0.5};
static double Skinv[4] =  {1.0,0.81,0.81,0.5};
static double DGreyv[4] = {0.2,0.2,0.2,0.5};
static double DSkinv[4] =  {1.0,0.5,0.5,0.5};
static double DBrownv[4] =  {0.5,0.5,0.25,0.5};
static double DGreenv[4] =  {0.0,0.25,0.0,0.5};

static double tWhitev[4] = {1.0,1.0,1.0,0.5 };
static double tBlackv[4] = {.0,.0,.0,0.5 };
static double tBluev[4] =  {.0,.0,1.0,0.5};
static double tRedv[4]  =  {1.0,.0,.0,0.5};
static double tYellowv[4] =  {1.0,1.0,.0,0.5};
static double tGreenv[4] =  {0.0,1.0,.0,0.5};
static double tGreyv[4] = {0.7,0.7,0.7,0.5};
static double tBrownv[4] =  {1.0,1.0,0.5,0.5};
static double tVioletv[4] =  {1.0,0.0,1.0,0.5};
static double tBlue2v[4] =  {0.0,1.0,1.0,0.5};
static double tSkinv[4] =  {1.0,0.81,0.81,0.5};
static double tDGreyv[4] = {0.2,0.2,0.2,0.5};
static double tDSkinv[4] =  {1.0,0.5,0.5,0.5};
static double tDBrownv[4] =  {0.5,0.5,0.25,0.5};
static double tDGreenv[4] =  {0.0,0.25,0.0,0.5};




int g3d_get_kcd_show_info()
{
	return (KCD_MUST_SHOW_INFO);
}
void g3d_set_kcd_show_info(int val)
{
	KCD_MUST_SHOW_INFO = val;
}
void g3d_set_kcd_draw_all_obbs(int val)
{
	MUST_DRAW_ALL_OBBS  = val;
}

void g3d_set_kcd_draw_robot_obbs(int val)
{
	MUST_DRAW_ROB_OBBS = val;
}

void g3d_set_kcd_draw_aabb_hier(int val)
{
	MUST_DRAW_AABB_TREE = val;
}

void g3d_set_kcd_draw_all_aabbs(int val)
{
	MUST_DRAW_ALL_AABBS = val;
}


/*****************************************************************************\
 @ p3d_CxformPoint()
 -----------------------------------------------------------------------------
 description : transform a point:  M * (p 1) => (p2 1)
 input       :
 output      :
 notes       :
\*****************************************************************************/
static void p3d_CxformPoint(p3d_matrix4 M, p3d_vector3 p, p3d_vector3 p2)
{
	int i;

	for (i = 0; i < 3; i++)
		p2[i] = M[i][0] * p[0] + M[i][1] * p[1] + M[i][2] * p[2] + M[i][3];
} /** End of p3d_CxformPoint() **/


/*******************************************/
/* Fonction tracant une boite  aabb        */
/*******************************************/
static
void g3d_kcd_draw_aabb(int colour,double x1,double x2,double y1,
		double y2,double z1,double z2)
{
	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	switch(colour){
	case Blue:
		glColor4dv(Bluev);
		break;
	case Yellow:
		glColor4dv(Yellowv);
		break;
	case Red:
		glColor4dv(Redv);
		break;
	case Green:
		glColor4dv(Greenv);
		break;
	case White:
		glColor4dv(Whitev);
		break;
	case Black:
		glColor4dv(Blackv);
		break;
	case Violet:
		glColor4dv(Violetv);
		break;
	case Grey:
		glColor4dv(Greyv);
		break;
	case Brown:
		glColor4dv(Brownv);
		break;
	case Skin:
		glColor4dv(Skinv);
		break;
	case Blue2:
		glColor4dv(Blue2v);
		break;
		/********* Carole : nouvelles couleurs **********/
	case DGrey:
		glColor4dv(DGreyv);
		break;
	case DSkin:
		glColor4dv(DSkinv);
		break;
	case DBrown:
		glColor4dv(DBrownv);
		break;
	case DGreen:
		glColor4dv(DGreenv);
		break;



	case tBlue:
		glColor4dv(tBluev);
		break;
	case tYellow:
		glColor4dv(tYellowv);
		break;
	case tRed:
		glColor4dv(tRedv);
		break;
	case tGreen:
		glColor4dv(tGreenv);
		break;
	case tWhite:
		glColor4dv(tWhitev);
		break;
	case tBlack:
		glColor4dv(tBlackv);
		break;
	case tViolet:
		glColor4dv(tVioletv);
		break;
	case tGrey:
		glColor4dv(tGreyv);
		break;
	case tBrown:
		glColor4dv(tBrownv);
		break;
	case tSkin:
		glColor4dv(tSkinv);
		break;
	case tBlue2:
		glColor4dv(tBlue2v);
		break;
		/********* Carole : nouvelles couleurs **********/
	case tDGrey:
		glColor4dv(tDGreyv);
		break;
	case tDSkin:
		glColor4dv(tDSkinv);
		break;
	case tDBrown:
		glColor4dv(tDBrownv);
		break;
	case tDGreen:
		glColor4dv(tDGreenv);
		break;
	}


	glBegin(GL_LINE_LOOP);
	{
		glVertex3d(x1,y1,z2);
		glVertex3d(x1,y2,z2);
		glVertex3d(x2,y2,z2);
		glVertex3d(x2,y1,z2);
	}
	glEnd();

	glBegin(GL_LINE_LOOP);
	{
		glVertex3d(x1,y1,z1);
		glVertex3d(x1,y2,z1);
		glVertex3d(x2,y2,z1);
		glVertex3d(x2,y1,z1);
	}
	glEnd();

	glBegin(GL_LINES);
	{
		glVertex3d(x1,y1,z1);
		glVertex3d(x1,y1,z2);

		glVertex3d(x2,y1,z1);
		glVertex3d(x2,y1,z2);

		glVertex3d(x2,y2,z1);
		glVertex3d(x2,y2,z2);

		glVertex3d(x1,y2,z1);
		glVertex3d(x1,y2,z2);
	}
	glEnd();

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glEndList();

}

static
void g3d_kcd_draw_aabb_level(int hierheight)
{
	int nof_boxes,i;
	kcd_bb *the_box;
	double x1,x2,y1,y2,z1,z2;

	nof_boxes = kcd_get_nofels_on_level(hierheight);
	PrintInfo(("nof_boxes = %i on level %i\n",nof_boxes,hierheight));
	for(i=0;i<nof_boxes;i++)
	{
		the_box = all_bbs[kcd_get_aabb_i_by_level(hierheight,i)];
		x1 = the_box->x1;
		x2 = the_box->x2;
		y1 = the_box->y1;
		y2 = the_box->y2;
		z1 = the_box->z1;
		z2 = the_box->z2;

		g3d_kcd_draw_aabb((hierheight+2)%4+1,x1,x2,y1,y2,z1,z2);
	}
}

void g3d_kcd_draw_aabb_hier()
{
	int i, total_height;

	if(MUST_DRAW_AABB_TREE)
	{
		total_height = kcd_get_total_height();
		PrintInfo(("aabb hierarchy: total height = %i\n",total_height));
		for(i=0;i<total_height;i++)
		{
			g3d_kcd_draw_aabb_level(i);
		}
	}
}

void g3d_kcd_draw_all_aabbs()
{
	if(MUST_DRAW_ALL_AABBS)
	{
		int i;
		int ext_o_id;
		int kcd_ext_o;

		int nb_drawn=0;

		int nb_of_poly = 0;

		std::map<p3d_obj*,int> nbOfPoly;
		std::map<p3d_obj*,int> idOfObj;

		p3d_rob* robot = XYZ_ENV->cur_robot;

		for(i=0;i<robot->no;i++) {

			int nb_of_non_graphic=0;

			for(int j=0;j<robot->o[i]->np;j++){

				if( robot->o[i]->pol[j]->TYPE !=  P3D_GRAPHIC ){
					nb_of_non_graphic++;
				}

			}

//			printf("-------------------------------------\n");
//			printf("Id of the object = %d\n",i);
//			printf("Object is pure graphic = %d\n",p3d_col_object_is_pure_graphic(robot->o[i]));
//			printf("nb_of_non_graphic = %d\n",  nb_of_non_graphic);
//			printf("nb_of_poly_tot = %d\n",  robot->o[i]->np);

			nb_of_poly += robot->o[i]->np;
			nbOfPoly[robot->o[i]] = nb_of_non_graphic;
			idOfObj[robot->o[i]] = i;
		}



//		printf("nb_of_poly = %d\n",nb_of_poly);
//		printf("-------------------------------------\n");

		for(i=0;i<nof_bbs;i++)
		{
			if((all_bbs[i]->is_aabb_tree_leaf) /*&& (all_bbs[i]->is_robot_prim)*/){

				bool draw=true;

				if(all_bbs[i]->is_robot_prim){

					kcd_ext_o = all_bbs[i]->ext_obj_id;
					ext_o_id = get_p3d_id_from_input_index(kcd_ext_o);

					p3d_obj* ptr_kcd_obj = get_obj_ptr_from_o_id(ext_o_id);
					p3d_obj* ptr_p3d_obj = ptr_kcd_obj->jnt->o;

					if(all_bbs[i]->pol){
						if(((p3d_poly*)all_bbs[i]->pol)->TYPE == P3D_GRAPHIC){
							draw = true;
						}
						else{
							draw = false;
						}
					}

				}

				if(draw){
					g3d_kcd_draw_aabb(Green,
							all_bbs[i]->x1,all_bbs[i]->x2,
							all_bbs[i]->y1,all_bbs[i]->y2,
							all_bbs[i]->z1,all_bbs[i]->z2);
				}
				else{
					g3d_kcd_draw_aabb(Red,
							all_bbs[i]->x1,all_bbs[i]->x2,
							all_bbs[i]->y1,all_bbs[i]->y2,
							all_bbs[i]->z1,all_bbs[i]->z2);
				}

				nb_drawn++;
			}
		}
//		printf("\n");
		printf("%d AA bounding boxes drawn\n",nb_drawn);
	}

	/*   a_robot = (XYZ_ENV->cur_robot); */
/*   p3d_get_BB_rob(a_robot,&x1,&x2,&y1,&y2,&z1,&z2); */
/*   g3d_kcd_draw_aabb(Red,x1,x2,y1,y2,z1,z2); */
}



/*******************************************/
/* Fonction tracant une boite  aabb        */
/*******************************************/
static
void g3d_kcd_draw_a_bb(int colour, p3d_vector3 center, p3d_vector3 v1,p3d_vector3 v2,p3d_vector3 v3,p3d_matrix4 place)
{
	p3d_vector3 sum,vertex1,vertex2,vertex3,vertex4,vertex5,vertex6,vertex7,vertex8;
	p3d_vector3 plvert1,plvert2,plvert3,plvert4,plvert5,plvert6,plvert7,plvert8;

	/* p3d_mat4Print(place,"place g3d_kcd_draw_a_bb"); */

	p3d_vectAdd(v1,v2,sum);
	p3d_vectAdd(sum,v3,plvert1);
	p3d_vectSub(sum,v3,plvert2);
	p3d_vectSub(v1,v2,sum);
	p3d_vectSub(sum,v3,plvert3);
	p3d_vectAdd(sum,v3,plvert4);
	p3d_vectScale(plvert1,plvert7,-1.0);
	p3d_vectScale(plvert2,plvert8,-1.0);
	p3d_vectScale(plvert3,plvert5,-1.0);
	p3d_vectScale(plvert4,plvert6,-1.0);

	p3d_vectAdd(plvert1,center,plvert1);
	p3d_vectAdd(plvert2,center,plvert2);
	p3d_vectAdd(plvert3,center,plvert3);
	p3d_vectAdd(plvert4,center,plvert4);
	p3d_vectAdd(plvert5,center,plvert5);
	p3d_vectAdd(plvert6,center,plvert6);
	p3d_vectAdd(plvert7,center,plvert7);
	p3d_vectAdd(plvert8,center,plvert8);

	/*   p3d_vec3Mat4Mult(plvert1,place,vertex1); */
	/*   p3d_vec3Mat4Mult(plvert2,place,vertex2); */
	/*   p3d_vec3Mat4Mult(plvert3,place,vertex3); */
	/*   p3d_vec3Mat4Mult(plvert4,place,vertex4); */
	/*   p3d_vec3Mat4Mult(plvert5,place,vertex5); */
	/*   p3d_vec3Mat4Mult(plvert6,place,vertex6); */
	/*   p3d_vec3Mat4Mult(plvert7,place,vertex7); */
	/*   p3d_vec3Mat4Mult(plvert8,place,vertex8); */

	p3d_CxformPoint(place,plvert1,vertex1);
	p3d_CxformPoint(place,plvert2,vertex2);
	p3d_CxformPoint(place,plvert3,vertex3);
	p3d_CxformPoint(place,plvert4,vertex4);
	p3d_CxformPoint(place,plvert5,vertex5);
	p3d_CxformPoint(place,plvert6,vertex6);
	p3d_CxformPoint(place,plvert7,vertex7);
	p3d_CxformPoint(place,plvert8,vertex8);

	/*   p3d_vectAdd(vertex1,center,vertex1); */
	/*   p3d_vectAdd(vertex2,center,vertex2); */
	/*   p3d_vectAdd(vertex3,center,vertex3); */
	/*   p3d_vectAdd(vertex4,center,vertex4); */
	/*   p3d_vectAdd(vertex5,center,vertex5); */
	/*   p3d_vectAdd(vertex6,center,vertex6); */
	/*   p3d_vectAdd(vertex7,center,vertex7); */
	/*   p3d_vectAdd(vertex8,center,vertex8); */

	/*   center[0]=place[0][3]; */
	/*   center[1]=place[1][3]; */
	/*   center[2]=place[2][3]; */



	/* p3d_CxformPoint(place,plcenter,center); */



	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	switch(colour){
	case Blue:
		glColor4dv(Bluev);
		break;
	case Yellow:
		glColor4dv(Yellowv);
		break;
	case Red:
		glColor4dv(Redv);
		break;
	case Green:
		glColor4dv(Greenv);
		break;
	case White:
		glColor4dv(Whitev);
		break;
	case Grey:
		glColor4dv(Greyv);
		break;
	case Brown:
		glColor4dv(Brownv);
		break;
	case Skin:
		glColor4dv(Skinv);
		break;
	case Blue2:
		glColor4dv(Blue2v);
		break;
		/********* Carole : nouvelles couleurs **********/
	case DGrey:
		glColor4dv(DGreyv);
		break;
	case DSkin:
		glColor4dv(DSkinv);
		break;
	case DBrown:
		glColor4dv(DBrownv);
		break;
	case DGreen:
		glColor4dv(DGreenv);
		break;

	case tBlue:
		glColor4dv(tBluev);
		break;
	case tYellow:
		glColor4dv(tYellowv);
		break;
	case tRed:
		glColor4dv(tRedv);
		break;
	case tGreen:
		glColor4dv(tGreenv);
		break;
	case tWhite:
		glColor4dv(tWhitev);
		break;
	case tGrey:
		glColor4dv(tGreyv);
		break;
	case tBrown:
		glColor4dv(tBrownv);
		break;
	case tSkin:
		glColor4dv(tSkinv);
		break;
	case tBlue2:
		glColor4dv(tBlue2v);
		break;
		/********* Carole : nouvelles couleurs **********/
	case tDGrey:
		glColor4dv(tDGreyv);
		break;
	case tDSkin:
		glColor4dv(tDSkinv);
		break;
	case tDBrown:
		glColor4dv(tDBrownv);
		break;
	case tDGreen:
		glColor4dv(tDGreenv);
		break;

	}

	glBegin(GL_LINE_LOOP);
	{
		glVertex3d(vertex1[0],vertex1[1],vertex1[2]);
		glVertex3d(vertex2[0],vertex2[1],vertex2[2]);
		glVertex3d(vertex3[0],vertex3[1],vertex3[2]);
		glVertex3d(vertex4[0],vertex4[1],vertex4[2]);
	}
	glEnd();

	glBegin(GL_LINE_LOOP);
	{
		glVertex3d(vertex5[0],vertex5[1],vertex5[2]);
		glVertex3d(vertex6[0],vertex6[1],vertex6[2]);
		glVertex3d(vertex7[0],vertex7[1],vertex7[2]);
		glVertex3d(vertex8[0],vertex8[1],vertex8[2]);
	}
	glEnd();

	glBegin(GL_LINES);
	{
		glVertex3d(vertex1[0],vertex1[1],vertex1[2]);
		glVertex3d(vertex5[0],vertex5[1],vertex5[2]);

		glVertex3d(vertex2[0],vertex2[1],vertex2[2]);
		glVertex3d(vertex6[0],vertex6[1],vertex6[2]);

		glVertex3d(vertex3[0],vertex3[1],vertex3[2]);
		glVertex3d(vertex7[0],vertex7[1],vertex7[2]);

		glVertex3d(vertex4[0],vertex4[1],vertex4[2]);
		glVertex3d(vertex8[0],vertex8[1],vertex8[2]);
	}
	glEnd();

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glEndList();

}

void g3d_kcd_draw_nearest_bbs()
{
	int i,nr,bb_id;
	p3d_vector3 cent,c1,c2,c3;
	kcd_bb *a_bb;

	nr = p3d_get_desc_number(P3D_ROBOT);
	for(i=0;i<nr;i++)
	{
		bb_id = kcd_get_nearest_obstacle_bb_id(i);
		PrintInfo(("bb_id=%i\n",bb_id));
		if(bb_id >= 0)
		{
			a_bb = all_bbs[bb_id];
			PrintInfo(("best_box_type = %i\n",a_bb->best_box_type));
			if(a_bb->best_box_type == OBB_BOXTYPE)
			{
				c1[0] = a_bb->d[0] * a_bb->eigenv[0][0];
				c1[1] = a_bb->d[0] * a_bb->eigenv[0][1];
				c1[2] = a_bb->d[0] * a_bb->eigenv[0][2];
				c2[0] = a_bb->d[1] * a_bb->eigenv[1][0];
				c2[1] = a_bb->d[1] * a_bb->eigenv[1][1];
				c2[2] = a_bb->d[1] * a_bb->eigenv[1][2];
				c3[0] = a_bb->d[2] * a_bb->eigenv[2][0];
				c3[1] = a_bb->d[2] * a_bb->eigenv[2][1];
				c3[2] = a_bb->d[2] * a_bb->eigenv[2][2];
				cent[0] = a_bb->center[0];
				cent[1] = a_bb->center[1];
				cent[2] = a_bb->center[2];
				g3d_kcd_draw_a_bb(Red,cent,c1,c2,c3,p3d_mat4IDENTITY);
			}
			else
			{
				g3d_kcd_draw_aabb(Red,a_bb->x1,a_bb->x2,a_bb->y1,a_bb->y2,
						a_bb->z1,a_bb->z2);
			}
		}
	}
}



/* static int tak = 0; */
static
void g3d_kcd_draw_obb_tree(int colour,kcd_bb *a_bb, int this_level, int *height, int *nof_leafs)
{
	int i, tot_children;
	p3d_vector3 cent,c1,c2,c3;

	this_level++;
	c1[0] = a_bb->d[0] * a_bb->eigenv[0][0];
	c1[1] = a_bb->d[0] * a_bb->eigenv[0][1];
	c1[2] = a_bb->d[0] * a_bb->eigenv[0][2];
	c2[0] = a_bb->d[1] * a_bb->eigenv[1][0];
	c2[1] = a_bb->d[1] * a_bb->eigenv[1][1];
	c2[2] = a_bb->d[1] * a_bb->eigenv[1][2];
	c3[0] = a_bb->d[2] * a_bb->eigenv[2][0];
	c3[1] = a_bb->d[2] * a_bb->eigenv[2][1];
	c3[2] = a_bb->d[2] * a_bb->eigenv[2][2];

	/*   PrintInfo(("c1:   %f,\t%f,\t%f\n",c1[0],c1[1],c1[2])); */
	/*   PrintInfo(("c2:   %f,\t%f,\t%f\n",c2[0],c2[1],c2[2])); */
	/*   PrintInfo(("c3:   %f,\t%f,\t%f\n",c3[0],c3[1],c3[2])); */

	cent[0] = a_bb->center[0];
	cent[1] = a_bb->center[1];
	cent[2] = a_bb->center[2];

	/*   PrintInfo(("cent: %f,\t%f,\t%f\n\n",cent[0],cent[1],cent[2])); */

	if(a_bb->is_robot_prim){
		int kcd_ext_o = a_bb->ext_obj_id;
		p3d_obj *mov_obj = get_obj_ptr_from_o_id(get_p3d_id_from_input_index(kcd_ext_o));
		if( mov_obj && mov_obj->jnt ){
			g3d_kcd_draw_a_bb(colour,cent,c1,c2,c3,mov_obj->jnt->abs_pos);
		}
		else{
			g3d_kcd_draw_a_bb(colour,cent,c1,c2,c3,p3d_mat4IDENTITY);
		}
	}
	else{
		g3d_kcd_draw_a_bb(colour,cent,c1,c2,c3,p3d_mat4IDENTITY);
	}

	tot_children = a_bb->nof_children_bb;
	/* PrintInfo(("a_bb->nof_children_bb=%i\n",tot_children)); */
	if(colour == Violet)
		colour=Red;
	else if(colour == White-1)
		colour=Brown;
	else if(colour == Filaire-1)
		colour = Blue2;
	else if(colour == Any-1)
		colour = Violet;
	else
		colour++;
	if(this_level > (*height))
		(*height) = this_level;
	if(tot_children == 0)
	{
		(*nof_leafs) += 1;
		/* PrintInfo(("leaf\n")); */
	}
	if(tot_children == 1)
	{
		/* PrintInfo(("DEG\n")); */
	}

	for(i=0;i<tot_children;i++)
	{
		/* PrintInfo(("kid %i\t",i)); */
		g3d_kcd_draw_obb_tree(colour,a_bb->children_bb[i],this_level,height,nof_leafs);
	}
	/* PrintInfo(("\n")); */
}

void g3d_kcd_draw_all_obbs()
{
	int i,height=0,nof_leafs=0;
	int colour = Red;
	int nof_simple_obbs=0;
	int deg=0;
	int n_aabb_leafs=0,box_not_leaf=0;

	if(MUST_DRAW_ALL_OBBS)
	{

		for(i=0;i<nof_bbs;i++)
		{
			/*       PrintInfo(("all_bbs[%i]->bb_id_in_all_bbs=%i\n",i,all_bbs[i]->bb_id_in_all_bbs)); */
			/*       if(i!=all_bbs[i]->bb_id_in_all_bbs) */
			/* 	PrintInfo(("%i VERGETEN\n",i)); */
			height=0;
			nof_leafs=0;
			if((all_bbs[i]->is_aabb_tree_leaf) /*&& (!(all_bbs[i]->is_robot_prim))*/)
			{
				g3d_kcd_draw_obb_tree(colour,all_bbs[i],0,&height,&nof_leafs);
				/* 	  if(height > 0) */
				/* 	    PrintInfo(("new tree: height=%i, nof_leafs=%i\n",height,nof_leafs)); */
				n_aabb_leafs++;
				if(height == 1)
					nof_simple_obbs++;
				else if(height == nof_leafs)
					deg++;
			}
			else
			{
				box_not_leaf++;
			}
		}
		PrintInfo(("n_aabb_leafs=%i,nof_simple_obbs=%i,deg=%i\n",n_aabb_leafs,nof_simple_obbs,deg));
		PrintInfo(("nof_bbs=%i,box_not_leaf=%i\n", nof_bbs,box_not_leaf));
	}
}

static
void g3d_kcd_draw_obb_tree_at_place(kcd_bb *a_bb, p3d_matrix4 place)
{
	int colour = Red;
	int i, tot_children;
	p3d_vector3 cent,c1,c2,c3;

	/* PrintInfo(("box size in draw: %f,%f,%f\n",a_bb->d[0],a_bb->d[1],a_bb->d[2])); */

	c1[0] = a_bb->d[0] * a_bb->eigenv[0][0];
	c1[1] = a_bb->d[0] * a_bb->eigenv[0][1];
	c1[2] = a_bb->d[0] * a_bb->eigenv[0][2];
	c2[0] = a_bb->d[1] * a_bb->eigenv[1][0];
	c2[1] = a_bb->d[1] * a_bb->eigenv[1][1];
	c2[2] = a_bb->d[1] * a_bb->eigenv[1][2];
	c3[0] = a_bb->d[2] * a_bb->eigenv[2][0];
	c3[1] = a_bb->d[2] * a_bb->eigenv[2][1];
	c3[2] = a_bb->d[2] * a_bb->eigenv[2][2];

	/*   PrintInfo(("c1:   %f,\t%f,\t%f\n",c1[0],c1[1],c1[2])); */
	/*   PrintInfo(("c2:   %f,\t%f,\t%f\n",c2[0],c2[1],c2[2])); */
	/*   PrintInfo(("c3:   %f,\t%f,\t%f\n",c3[0],c3[1],c3[2])); */

	cent[0] = a_bb->center[0];
	cent[1] = a_bb->center[1];
	cent[2] = a_bb->center[2];

	/*   PrintInfo(("cent: %f,\t%f,\t%f\n\n",cent[0],cent[1],cent[2])); */

	g3d_kcd_draw_a_bb(colour,cent,c1,c2,c3,place);
	tot_children = a_bb->nof_children_bb;
	for(i=0;i<tot_children;i++)
	{
		g3d_kcd_draw_obb_tree_at_place(a_bb->children_bb[i],place);
	}
}

void g3d_kcd_draw_robot_obbs()
{
	int i;
	int nof_bs = 0;
	int guess_of_poly_id = 0;
	p3d_matrix4 *mat;
	p3d_matrix4 inv_pos;

	if(MUST_DRAW_ROB_OBBS)
	{

		if(kcd_body_bb_table)
		{
			nof_bs = p3d_get_desc_number(P3D_BODY);
			for(i=0;i<nof_bs;i++)
			{
				guess_of_poly_id = kcd_body_bb_table[XYZ_ENV->cur_robot->num+1][i].bb_id;

				int kcd_ext_o = all_bbs[guess_of_poly_id]->ext_obj_id;

				int ext_o_id = get_p3d_id_from_input_index(kcd_ext_o);
				p3d_obj *mov_obj = get_obj_ptr_from_o_id(ext_o_id);

				mat = &mov_obj->jnt->abs_pos;

//				p3d_mat4Print(*mat,"mat");
//				p3d_mat4Print(XYZ_ENV->cur_robot->o[i]->jnt->pos0,">o[i]->jnt->pos0");
//				printf("----------------------\n");

				/* PrintInfo(("robot number: %i, body number: %i, box: %i, name:%s\n",XYZ_ENV->cur_robot->num,i,guess_of_poly_id,XYZ_ENV->cur_robot->o[i]->name)); */
				/* p3d_mat4Print(pol_it->pos_rel_jnt,"poly"); */
				/* PrintInfo(("nof_bbs=%i\n",nof_bbs)); */
				/* print_bb(all_bbs[guess_of_poly_id]); */
				/* 		  g3d_kcd_draw_obb_tree_at_place(all_bbs[guess_of_poly_id],p3d_mat4IDENTITY); */
				/* 		  g3d_kcd_draw_obb_tree_at_place(all_bbs[guess_of_poly_id],pol_it->pos_rel_jnt); */
				/*		  g3d_kcd_draw_obb_tree_at_place(all_bbs[guess_of_poly_id],pol_it->poly->pos);*/

				//	      p3d_matInvertXform( XYZ_ENV->cur_robot->o[i]->jnt->pos0, inv_pos );
				//	      p3d_matMultXform( XYZ_ENV->cur_robot->o[i]->jnt->abs_pos, inv_pos, mat );

				g3d_kcd_draw_obb_tree_at_place(all_bbs[guess_of_poly_id],*mat);
				/* PrintInfo(("we drew robot obbs \n")); */
			}
		}
	}
}


/* function to trace a line between the two closest points
   of each body
   NOTE: doesn't work for mo that don't belong to a group
 */

void g3d_kcd_draw_closest_points()
{
	int r, mo, nof_bodies;
	int nr_robots;
	double *distances;
	p3d_vector3 *body,*other;


	if(KCD_MUST_SHOW_INFO > 1)
	{
		PrintInfo(("*********** KCD EXACT DISTANCE COMPUTATION ***************\n"));
		nr_robots = XYZ_ENV->nr;
		for( r=0; r<nr_robots ; r++)
		{
			nof_bodies = XYZ_ENV->robot[r]->no;
			distances = NULL; body = NULL; other = NULL;
			distances = MY_ALLOC(double,nof_bodies);
			body = MY_ALLOC(p3d_vector3,nof_bodies);
			other = MY_ALLOC(p3d_vector3,nof_bodies);

			p3d_kcd_closest_points_between_bodies(XYZ_ENV->robot[r],body,other ,distances);
			PrintInfo(("distances between bodies for robot %d\n",r));
			for(mo = 0; mo < nof_bodies; mo++)
			{
				PrintInfo(("ro %d mo %d:info pa = %.0f %.0f %.0f  / pb =%.0f %.0f %.0f \n",r,mo,body[mo][0] ,body[mo][1],body[mo][2],other[mo][0], other[mo][1],other[mo][2]));
				g3d_drawOneLine(body[mo][0],body[mo][1],body[mo][2],other[mo][0],other[mo][1],other[mo][2],Violet,NULL);
				PrintInfo(("     mo %d:     dist = %f\n",mo,distances[mo]));
			}
			p3d_kcd_closest_points_robot_environment(XYZ_ENV->robot[r],body,other,distances);
			PrintInfo(("distances between robot %d and the environment\n",r));
			for(mo = 0; mo < nof_bodies; mo++)
			{
				PrintInfo(("ro %d mo %d:info pa = %.0f %.0f %.0f  / pb =%.0f %.0f %.0f \n",r,mo,body[mo][0] ,body[mo][1],body[mo][2],other[mo][0], other[mo][1],other[mo][2]));
				g3d_drawOneLine(body[mo][0],body[mo][1],body[mo][2],other[mo][0],other[mo][1],other[mo][2],Blue2,NULL);
				PrintInfo(("     mo %d:     dist = %f\n",mo,distances[mo]));
			}
			MY_FREE( distances, double,      nof_bodies);
			MY_FREE( body,      p3d_vector3, nof_bodies);
			MY_FREE( other,     p3d_vector3, nof_bodies);
		}
		PrintInfo(("************************** END **************************\n"));
	}
	else if(KCD_MUST_SHOW_INFO == 1)
	{
		nr_robots = XYZ_ENV->nr;
		for( r=0; r<nr_robots ; r++)
		{
			nof_bodies = XYZ_ENV->robot[r]->no;
			distances = NULL; body = NULL; other = NULL;
			distances = MY_ALLOC(double,nof_bodies);
			body = MY_ALLOC(p3d_vector3,nof_bodies);
			other = MY_ALLOC(p3d_vector3,nof_bodies);

			p3d_kcd_closest_points_between_bodies(XYZ_ENV->robot[r],body,other ,distances);
			for(mo = 0; mo < nof_bodies; mo++)
			{
				g3d_drawOneLine(body[mo][0],body[mo][1],body[mo][2],other[mo][0],other[mo][1],other[mo][2],Violet,NULL);
			}
			p3d_kcd_closest_points_robot_environment(XYZ_ENV->robot[r],body,other,distances);
			for(mo = 0; mo < nof_bodies; mo++)
			{
				g3d_drawOneLine(body[mo][0],body[mo][1],body[mo][2],other[mo][0],other[mo][1],other[mo][2],Blue2,NULL);
			}

			MY_FREE( distances, double,      nof_bodies);
			MY_FREE( body,      p3d_vector3, nof_bodies);
			MY_FREE( other,     p3d_vector3, nof_bodies);
		}
	}


}
