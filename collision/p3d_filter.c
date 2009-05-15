#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"


typedef double filterbox[6];
typedef filterbox *list_filterboxes;

static int FILTER_LINKS = TRUE;
static int FILTER_ACTIVE = FALSE; /* by default, don't filter */
static int current_active_link = 0;
static int filter_initialized = FALSE;  /* filterbox does not exist */
static int nr_shots_one_joint = 1000;
static int current_robot_nr = 0;
static filterbox *rbox = NULL; /* the filterbox */
static list_filterboxes *linkboxes = NULL;
static int DRAW_ROBOT_BOX = FALSE;  /* filterbox shouldn't be drawn */
static int robotboxlist = -1;       /* necessity to refresh screen */

static void p3d_filter_approx_global_robot_BB(filterbox *returnbox, 
					      list_filterboxes *linkreturnboxes, 
					      int nr_links);

int p3d_filter_mechanism_requested()
{
  return FILTER_ACTIVE;
}


void p3d_filter_switch_filter_mechanism(int waarde)
{
  FILTER_ACTIVE = waarde;
}

void p3d_filter_set_current_robot_nr(int waarde)
{
  current_robot_nr = waarde;
}

void p3d_filter_setoff_linkfilter()
{
  FILTER_LINKS = FALSE;
}

void p3d_filter_seton_linkfilter()
{
  FILTER_LINKS = TRUE;
}

void p3d_filter_set_draw_robot_box(int i)
{
  DRAW_ROBOT_BOX = i;
}

int must_draw_robot_box()
{
  return DRAW_ROBOT_BOX;
}
void p3d_filter_switch_draw_robot_box()
{
  if(DRAW_ROBOT_BOX)
    DRAW_ROBOT_BOX = FALSE;
  else
    DRAW_ROBOT_BOX = TRUE;
}

int p3d_get_robotboxlist()
{
  return robotboxlist;
}

void p3d_set_robotboxlist(int newvalue)
{
  robotboxlist = newvalue;
}

void p3d_reset_robotboxlist()
{
  robotboxlist = -1;
}

void p3d_set_filterbox(double x1,double x2,double y1,
		       double y2,double z1,double z2)
{
  p3d_reset_robotboxlist();
  rbox[current_robot_nr][0]=x1;rbox[current_robot_nr][1]=x2;
  rbox[current_robot_nr][2]=y1;rbox[current_robot_nr][3]=y2;
  rbox[current_robot_nr][4]=z1;rbox[current_robot_nr][5]=z2;
}

void p3d_get_filter_box(double *x1,double *x2,double *y1,
			double *y2,double *z1,double *z2)
{
  if(FILTER_ACTIVE)
    {
      *x1 = rbox[current_robot_nr][0];  *x2 = rbox[current_robot_nr][1];
      *y1 = rbox[current_robot_nr][2];  *y2 = rbox[current_robot_nr][3];
      *z1 = rbox[current_robot_nr][4];  *z2 = rbox[current_robot_nr][5];
    }
  else
    {
      PrintInfo(("WARNING (p3d_get_filter_box): filter by swept robot box not used\n"));
    }
}

/* void p3d_filter_approx_global_robot_BB(double *x1,double *x2,double *y1,
   double *y2,double *z1,double *z2) */
static void p3d_filter_approx_global_robot_BB(filterbox *returnbox, 
					      list_filterboxes *linkreturnboxes, 
					      int nr_links)
{
  p3d_rob *robbie;
  int nrjt;
  int i, nr_shots, linkn;
  configPt q, curq;
  double px1=0.0,px2=0.0,py1=0.0,py2=0.0,pz1=0.0,pz2=0.0; /* rp-box */
  double rx1=0.0,rx2=0.0,ry1=0.0,ry2=0.0,rz1=0.0,rz2=0.0; /* r-box */
  double *lpx1=NULL,*lpx2=NULL,*lpy1=NULL,
    *lpy2=NULL,*lpz1=NULL,*lpz2=NULL; /* rp-box for links */

  lpx1 = MY_ALLOC(double,nr_links);
  lpx2 = MY_ALLOC(double,nr_links);
  lpy1 = MY_ALLOC(double,nr_links);
  lpy2 = MY_ALLOC(double,nr_links);
  lpz1 = MY_ALLOC(double,nr_links);
  lpz2 = MY_ALLOC(double,nr_links);


  robbie = (p3d_rob *)p3d_get_desc_curid(P3D_ROBOT);
  nrjt = robbie->njoints;
  nr_shots=nr_shots_one_joint*(nrjt+4);
  /* keep in mind the current robot configuration */
  curq = p3d_get_robot_config(robbie);

  /* find box */
  q = p3d_alloc_config(robbie);
  /* PrintInfo(("shoot:%d\n",nr_shots)); */
  for(i=0;i<nr_shots;i++)
    {
      /* get a new configuration */
      p3d_shoot(robbie,q,1);
      /* put robot on new configuration and update its BB */
      p3d_set_and_update_robot_conf(q);
      /* get its BB, adapt the limits on the global BB */
      p3d_get_BB_rob(robbie,&rx1,&rx2,&ry1,&ry2,&rz1,&rz2);

      if(i==0)
	{
	  px1=rx1;px2=rx2;
	  py1=ry1;py2=ry2;
	  pz1=rz1;pz2=rz2;
	}
      else
	{
	  if(px1 > rx1)  px1 = rx1;
	  if(py1 > ry1)  py1 = ry1;
	  if(pz1 > rz1)  pz1 = rz1;
	  if(px2 < rx2)  px2 = rx2;
	  if(py2 < ry2)  py2 = ry2;
	  if(pz2 < rz2)  pz2 = rz2;
	}

      for(linkn=0;linkn<nr_links;linkn++)
      {
	/* PrintInfo(("linkn=%i,robbie->no=%i\n",linkn,robbie->no)); */
	/* PrintInfo(("robbie->name=%s\n",robbie->name)); */
	/* PrintInfo(("robbie->o[%i]->name=%s\n",linkn,robbie->o[linkn]->name)); */
	if(robbie->o[linkn])
	  {
	    p3d_get_BB_obj(robbie->o[linkn],&rx1,&rx2,&ry1,&ry2,&rz1,&rz2);
	    if(i==0)
	      {
		lpx1[linkn]=rx1;lpx2[linkn]=rx2;
		lpy1[linkn]=ry1;lpy2[linkn]=ry2;
		lpz1[linkn]=rz1;lpz2[linkn]=rz2;
	      }
	    else
	      {
		if(lpx1[linkn] > rx1)  lpx1[linkn] = rx1;
		if(lpy1[linkn] > ry1)  lpy1[linkn] = ry1;
		if(lpz1[linkn] > rz1)  lpz1[linkn] = rz1;
		if(lpx2[linkn] < rx2)  lpx2[linkn] = rx2;
		if(lpy2[linkn] < ry2)  lpy2[linkn] = ry2;
		if(lpz2[linkn] < rz2)  lpz2[linkn] = rz2;
	      }
	  }
      }
    }

  /*  
   *x1=px1;*x2=px2;
   *y1=py1;*y2=py2;
   *z1=pz1;*z2=pz2;
  */
  (*returnbox)[0]=px1;
  (*returnbox)[1]=px2;
  (*returnbox)[2]=py1;
  (*returnbox)[3]=py2;
  (*returnbox)[4]=pz1;
  (*returnbox)[5]=pz2;
  for(linkn=0;linkn<nr_links;linkn++)
  {
    (*linkreturnboxes)[linkn][0]=lpx1[linkn];
    (*linkreturnboxes)[linkn][1]=lpx2[linkn];
    (*linkreturnboxes)[linkn][2]=lpy1[linkn];
    (*linkreturnboxes)[linkn][3]=lpy2[linkn];
    (*linkreturnboxes)[linkn][4]=lpz1[linkn];
    (*linkreturnboxes)[linkn][5]=lpz2[linkn]; 
    /* PrintInfo(("link-box[%i]=(%f,%f,%f),(%f,%f,%f)\n",linkn, 
 	   (*linkreturnboxes)[linkn][0],(*linkreturnboxes)[linkn][2], 
 	   (*linkreturnboxes)[linkn][4],(*linkreturnboxes)[linkn][1],
 	   (*linkreturnboxes)[linkn][3],(*linkreturnboxes)[linkn][5])); */
  }
  /* restore old robot configuration */
    p3d_set_and_update_robot_conf(curq);

  if(curq)
    p3d_destroy_config(robbie, curq);
  if(q)
    p3d_destroy_config(robbie, q);
  MY_FREE(lpx1,double,nr_links);
  MY_FREE(lpx2,double,nr_links);
  MY_FREE(lpy1,double,nr_links);
  MY_FREE(lpy2,double,nr_links);
  MY_FREE(lpz1,double,nr_links);
  MY_FREE(lpz2,double,nr_links);
}

void p3d_filter_init_filter()
{
  int i,nof_robots = XYZ_ENV->nr;
  p3d_rob *cur_rob;

  if(FILTER_ACTIVE)
    {
      rbox = MY_ALLOC(filterbox,nof_robots);
      linkboxes = MY_ALLOC(list_filterboxes,nof_robots);
      /* keep track of current robot */
      cur_rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
      /* loop: */
      for(i=0;i<nof_robots;i++)
	{
	  /* activate robot j */
	  XYZ_ENV->cur_robot  = XYZ_ENV->robot[i];
	  p3d_filter_set_current_robot_nr(i);
	  /* init robot box for robot j */
	  p3d_filter_init_robot_box();
	}
      /* end loop */
      XYZ_ENV->cur_robot  = cur_rob;
    }
  else
    {
      PrintInfo(("filter by swept robot boxes not used\n"));
    }
}

void p3d_filter_init_robot_box()
{
  int current_col_mode;
  pp3d_rob robbie = (pp3d_rob)p3d_get_desc_curid(P3D_ROBOT);
  int nr_links = robbie->no;

  if(FILTER_ACTIVE)
    {
      linkboxes[current_robot_nr] = MY_ALLOC(filterbox,nr_links);

      /* don't test for collision */
      /* p3d_col_start(p3d_col_mode_none); */
      current_col_mode = p3d_col_get_mode();
      p3d_col_set_mode(p3d_col_mode_none);

      /* find a BB around an approximation of the swept volume of the robot */
      /* p3d_filter_approx_global_robot_BB(&wx1, &wx2, &wy1, &wy2, &wz1, &wz2); */
      p3d_filter_approx_global_robot_BB(&rbox[current_robot_nr], &linkboxes[current_robot_nr], nr_links);
      /* 
	 rbox[current_robot_nr][0]=wx1;
	 rbox[current_robot_nr][1]=wx2;
	 rbox[current_robot_nr][2]=wy1;
	 rbox[current_robot_nr][3]=wy2;
	 rbox[current_robot_nr][4]=wz1;
	 rbox[current_robot_nr][5]=wz2;
      */
      /* PrintInfo(("filterbox: %f,%f,%f,%f,%f,%f\n",rbox[current_robot_nr][0],rbox[current_robot_nr][1],rbox[current_robot_nr][2],rbox[current_robot_nr][3], 
 	 rbox[current_robot_nr][4],rbox[current_robot_nr][5]));  */

      p3d_reset_robotboxlist();
      filter_initialized = TRUE; 
      /* switch on collision detector again */
      /* p3d_col_start(p3d_col_mode_v_collide); */
      p3d_col_set_mode(current_col_mode); 
    }
}

static int boxes_overlap(double px1,double px2,double py1,
			 double py2,double pz1,double pz2,
			 double wx1,double wx2,double wy1,
			 double wy2,double wz1,double wz2)
{
  if((px1>px2)||(py1>py2)||(pz1>pz2))
    PrintInfo(("error in overlap (p): %f,%f,%f,%f,%f,%f\n",px1,px2,py1,py2,pz1,pz2));
  if((wx1>wx2)||(wy1>wy2)||(wz1>wz2))
    PrintInfo(("error in overlap (w): %f,%f,%f,%f,%f,%f\n",wx1,wx2,wy1,wy2,wz1,wz2));
  if(px1 < wx1)
    {
      if(wx1 > px2)
	return 0;
    }
  else
    {
      if(px1 > wx2)
	return 0;
    }
  if(py1 < wy1)
    {
      if(wy1 > py2)
	return 0;
    }
  else
    {
      if(py1 > wy2)
	return 0;
    }
  if(pz1 < wz1)
    {
      if(wz1 > pz2)
	return 0;
    }
  else
    {
      if(pz1 > wz2)
	return 0;
    }
  return 1;
}

void p3d_filter_set_current_active_link(int linknb)
{
  current_active_link = linknb;
}

int p3d_filter_relevant_pair(p3d_poly *objn)
{
  double pmin[3],pmax[3]; /* p-box */

  if(p3d_v_collide_is_non_active(objn)){return(FALSE);}
  if(!FILTER_ACTIVE){return TRUE;}

  if(FILTER_LINKS)
    {
      /* get Bbox around polyhedron *objn */
      p3d_BB_get_BB_poly(objn,&pmin[0],&pmax[0],&pmin[1],&pmax[1],
			 &pmin[2],&pmax[2]);

      if((rbox[current_robot_nr][0]==0.0)&&(rbox[current_robot_nr][1]==0.0)&&(rbox[current_robot_nr][2]==0.0)
	 &&(rbox[current_robot_nr][3]==0.0)&&(rbox[current_robot_nr][4]==0.0)&&(rbox[current_robot_nr][5]==0.0))
	{
	  PrintInfo(("no filter active in p3d_filter_relevant_poly\n"));
	  return TRUE;
	}
      else
	{
	  return boxes_overlap(pmin[0],pmax[0],pmin[1],pmax[1],pmin[2],pmax[2],
			       linkboxes[current_robot_nr][current_active_link][0],
			       linkboxes[current_robot_nr][current_active_link][1],
			       linkboxes[current_robot_nr][current_active_link][2],
			       linkboxes[current_robot_nr][current_active_link][3],
			       linkboxes[current_robot_nr][current_active_link][4],
			       linkboxes[current_robot_nr][current_active_link][5]);
	}
    }
  else
    {
      return TRUE;
    }
}

int p3d_filter_relevant_facet(p3d_poly *current_polyhedron, int facet_nr, int type)
{
  p3d_matrix4 *mat_ptr;
  double xmin[3],xmax[3],x[3],y[3];
  int nof_points = poly_get_nb_points_in_face(current_polyhedron->poly,
					      facet_nr);
  int i,j,i_nr,nr_robots,repons=TRUE;

  if(!FILTER_ACTIVE){return TRUE;}

  nr_robots = p3d_get_desc_number(P3D_ROBOT);
  for(i_nr=0;i_nr<nr_robots;i_nr++)
    {
      repons = repons && ((rbox[i_nr][0]==0.0)&&(rbox[i_nr][1]==0.0)&&(rbox[i_nr][2]==0.0)
			  &&(rbox[i_nr][3]==0.0)&&(rbox[i_nr][4]==0.0)&&(rbox[i_nr][5]==0.0));
    }
  if(repons)
    return TRUE;
  else
    {
      mat_ptr=p3d_get_poly_mat(current_polyhedron->poly);
      	
      /* init min and max */
      poly_get_point_in_pos_in_face(current_polyhedron->poly,
				    facet_nr,1,&x[0],&x[1],&x[2]);
      /* PrintInfo(("x:(%f,%f,%f)\n",x[0],x[1],x[2]));
	 for(i=0;i<4;i++)
	 for(j=0;j<4;j++)
	 PrintInfo(("[%i,%i]=%f, \n",i,j,*mat_ptr[i][j]));*/
      p3d_vec3Mat4Mult(x,*mat_ptr,xmin);
      /* PrintInfo(("xmin:(%f,%f,%f)\n",xmin[0],xmin[1],xmin[2])); */
      xmax[0]=xmin[0]; xmax[1]=xmin[1]; xmax[2]=xmin[2];
      /* look for smaller candidate for min, larger candidate for max */
      for(i=1;i<nof_points;i++)
	{
	  poly_get_point_in_pos_in_face(current_polyhedron->poly,
					facet_nr,i+1,&y[0],&y[1],&y[2]);
	  p3d_vec3Mat4Mult(y,*mat_ptr,x);
	  for(j=0;j<3;j++)
	    {
	      if(x[j] < xmin[j])
		xmin[j] = x[j];
	      else if (x[j] > xmax[j])
		xmax[j] = x[j];
	    }
	}
      repons = FALSE;
      for(i_nr=0;(i_nr<nr_robots)&&(!repons);i_nr++)
	{
	  repons = boxes_overlap(xmin[0],xmax[0],xmin[1],xmax[1],xmin[2],xmax[2],
				 rbox[i_nr][0],rbox[i_nr][1],rbox[i_nr][2],
				 rbox[i_nr][3],rbox[i_nr][4],rbox[i_nr][5]);
	}
      return repons;
    }
}

int p3d_filter_relevant_poly(p3d_poly *objn)
{
  double pmin[3],pmax[3]; /* p-box */
  
  if(!FILTER_ACTIVE){return TRUE;}

  /* get Bbox around polyhedron *objn */
  p3d_BB_get_BB_poly(objn,&pmin[0],&pmax[0],&pmin[1],&pmax[1],
		     &pmin[2],&pmax[2]);

  if((rbox[current_robot_nr][0]==0.0)&&(rbox[current_robot_nr][1]==0.0)&&(rbox[current_robot_nr][2]==0.0)
     &&(rbox[current_robot_nr][3]==0.0)&&(rbox[current_robot_nr][4]==0.0)&&(rbox[current_robot_nr][5]==0.0))
    {
      PrintInfo(("no filter active in p3d_filter_relevant_poly\n"));
      return TRUE;
    }
  else
    {
     return boxes_overlap(pmin[0],pmax[0],pmin[1],pmax[1],pmin[2],pmax[2],
			  rbox[current_robot_nr][0],rbox[current_robot_nr][1],rbox[current_robot_nr][2],rbox[current_robot_nr][3],rbox[current_robot_nr][4],rbox[current_robot_nr][5]);
    }
}

int p3d_filter_needs_init()
{
  return (!filter_initialized);
}

void p3d_filter_cleanup(void)
{
  int j,nr_robots=XYZ_ENV->nr;
  p3d_rob *cur_rob;
  int nr_links;

  if(FILTER_ACTIVE)
    {
      if(linkboxes)
	{  
	  /* keep track of active robot */
	  cur_rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
	  filter_initialized = FALSE;
	  /* loop */
	  for(j=0;j<nr_robots;j++)
	    {
	      /* activate robot j */
	      XYZ_ENV->cur_robot  = XYZ_ENV->robot[j];
	      nr_links = XYZ_ENV->cur_robot->no;
	      /* clean up for robot j */
	      if(linkboxes[j])
		MY_FREE(linkboxes[j],filterbox,nr_links);
	    }
	  /* end loop */
	  /* reset active robot */
	  XYZ_ENV->cur_robot  = cur_rob;
	  MY_FREE(linkboxes,list_filterboxes,nr_robots);
	}
      if(rbox)
	MY_FREE(rbox,filterbox,nr_robots);
      DRAW_ROBOT_BOX = FALSE;
      
      p3d_reset_robotboxlist();
    }
}
