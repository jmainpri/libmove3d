#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"


int G3D_DRAW_TRAJ = FALSE;
int G3D_DRAW_TRACE = FALSE;
int	G3D_DRAW_GRAPH;
int G3D_DRAW_OCUR_SPECIAL;
int G3D_SELECTED_JOINT = -999;
int G3D_SELECTED_ROBOT = -1;
int G3D_MOUSE_ROTATION = 0;
int boxlist; // liste opengl pour la boite materialisant l'environnment
int p3d_numcoll; // Variables externes pour le CC


/* VARIABLES EXPORTEES DANS g3d_draw.c POUR TRAITEMENT GRAPHIQUE */
GLfloat matrix_pos_absGL[16]; /* tableau (matrice GL) contenant 
la position du joint par rapport au repere global (cf. g3d_draw_object_moved)*/

static void g3d_draw_env(void);
static void g3d_draw_obstacle(G3D_Window *win);
static void g3d_draw_body(int coll,G3D_Window *win);

static void g3d_draw_object_moved(p3d_obj *o, int coll,G3D_Window* win);
static void g3d_draw_object(p3d_obj *o, int coll,G3D_Window *win);
#if 0
static void g3d_draw_obj_BB(p3d_obj *o);
#endif
/* Debut Modification Thibaut */
static void g3d_draw_ocur_special(G3D_Window *win);
/* Fin Modification Thibaut */
#if 0
static void g3d_draw_rob_BB(p3d_rob *r);
#endif
static void g3d_draw_robot_box(void);


/****************************************************************************************************/

void g3d_set_draw_coll(int n)
{
  p3d_numcoll = n;
}

void g3d_reinit_graphics(void)
{
	pp3d_env env;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	g3d_delete_all_poly(-1);

	if (boxlist != -1)
	{
		glDeleteLists(boxlist,1);
		boxlist = -1;
	}

	if (p3d_get_robotboxlist() != -1)
	{
		glDeleteLists(p3d_get_robotboxlist(),1);
		p3d_reset_robotboxlist();
	}

	env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	if (env != NULL)
	{
		env->INIT = 1;
	}
}


/**********************************************/
/* Fonction reglant les lumieres et affichant */
/* l'environnement                            */
/**********************************************/
void g3d_draw()
{
  g3d_set_light();
  g3d_draw_env();
   
}

/*******************************************************/
/* Fonction initialisant graphiquement l'environnement */
/* si besoin est et l'affichant                        */
/*******************************************************/
static 
void g3d_draw_env(void)
{pp3d_env e;
 pp3d_rob robotPt;
 G3D_Window *win;


  
  win = g3d_get_cur_win();
  e = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  if(e->INIT){  ChronoOn();
g3d_init_all_poly(); boxlist = -1; p3d_reset_robotboxlist(); e->INIT = 0;
ChronoPrint("INIT TIME");
  ChronoOff();}

    if(win->GOURAUD){
      g3d_init_all_poly_gouraud();
    } 



  
/*   printf("\n OpenGL Version %s \n",glGetString(GL_VERSION)); */

   /* glEnable(GL_CULL_FACE); */
/*    glCullFace(GL_BACK); */
/*    glFrontFace(GL_CCW); */
  
     g3d_draw_env_box();

  g3d_draw_robot_box();

  g3d_extract_frustum(win);
  g3d_draw_robots(win);
  g3d_draw_obstacles(win);
  
  
  g3d_kcd_draw_all_aabbs();     /* draw AABBs around static primitives */
  g3d_kcd_draw_aabb_hier();     /* draw AABB tree on static objects */
  g3d_kcd_draw_robot_obbs();    /* draw all obbs of current robot */
  g3d_kcd_draw_all_obbs();      /* draw all static obbs */

  g3d_kcd_draw_closest_points();
  /* Carl: just a test: KCD */
  /* kcd_init_movable_stuff(); */
  /* g3d_kcd_draw_nearest_bbs();   */ /* test nearest BB */
  /* Carl: end of test: KCD */

/* Debut Modification Thibaut */
  if(G3D_DRAW_OCUR_SPECIAL) g3d_draw_ocur_special(win);
/* Fin Modification Thibaut */

/*   p3d_get_robot_pos(&x,&y,&z,&t); */
/*   p3d_get_BB_rob(r,&x1,&x2,&y1,&y2,&z1,&z2); */
/*   ampl = sqrt(SQR(x2-x1)+SQR(y2-y1)+SQR(z2-z1)); */
  
/*   g3d_set_win_camera(win,x,y,z+0.5*ampl,ampl,180.0+t,20.0); */

  if(XYZ_GRAPH && G3D_DRAW_GRAPH){g3d_draw_graph();}
  if(G3D_DRAW_TRAJ){g3d_draw_all_tcur();}
  if(G3D_DRAW_TRACE){
    g3d_draw_trace_all_tcur();
    p3d_set_and_update_robot_conf(robotPt->ROBOT_POS);
     /* collision checking */
     p3d_numcoll = p3d_col_test_all();
    g3d_draw_robot(robotPt->num,win);
    p3d_set_and_update_robot_conf(robotPt->ROBOT_GOTO);
     /* collision checking */
     p3d_numcoll = p3d_col_test_all();
    g3d_draw_robot(robotPt->num,win);
  }
}


/**********************************************************/
/* Fonction tracant tous les obstacles d'un environnement */
/**********************************************************/
void g3d_draw_obstacles(G3D_Window* win)
{int   no,o,i; 
    
/** Initialisation de la matrice OpenGL unite **/
/** IMPORTANT pour le calcul de resolution de primitive **/
    for(i = 0 ; i< 16; i++){
      matrix_pos_absGL[i] = 0.;
    }
    
    matrix_pos_absGL[0] = 1.;
    matrix_pos_absGL[5] = 1.;
    matrix_pos_absGL[10] = 1.;
    matrix_pos_absGL[15] = 1.;

    o = p3d_get_desc_curnum(P3D_OBSTACLE);
    no= p3d_get_desc_number(P3D_OBSTACLE);
    
    
   
    if(no) {
      for(i=0;i<no;i++) { 
	p3d_sel_desc_num(P3D_OBSTACLE,i);
/* 	ChronoOn(); */
	g3d_draw_obstacle(win);
/* 	printf("obstacle %d",i); */
/* 	ChronoPrint(""); */
/* 	ChronoOff(); */
      }
      p3d_sel_desc_num(P3D_OBSTACLE,o);
    }
    
    
    
}

/*******************************************************/
/* Fonction tracant tous les robots d'un environnement */
/*******************************************************/
void g3d_draw_robots(G3D_Window *win)
{int   r,nr,ir;
 p3d_rob *rob; 

      
      r = p3d_get_desc_curnum(P3D_ROBOT);
      nr= p3d_get_desc_number(P3D_ROBOT);
      
      if(nr) {
	for(ir=0;ir<nr;ir++) {
	  p3d_sel_desc_num(P3D_ROBOT,ir);
	  rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	  /*g3d_draw_rob_BB(rob); */
	  g3d_draw_robot(ir,win);
	}
	p3d_sel_desc_num(P3D_ROBOT,r);

      }
    
}

/*******************************************/
/* Fonction tracant la boite materialisant */
/* les limites du robot                    */
/*******************************************/
static void g3d_draw_robot_box(void)
{double x1,x2,y1,y2,z1,z2;
/* double t1,t2,nampl,temp;*/
/* int i,n=10;*/
 int robot_box_list;

 if(must_draw_robot_box())
   {
     robot_box_list = p3d_get_robotboxlist();
     if(robot_box_list == -1){

       robot_box_list = glGenLists(1);
       p3d_set_robotboxlist(robot_box_list);
       glNewList(robot_box_list,GL_COMPILE_AND_EXECUTE);
       
       glDisable(GL_LIGHTING);
       glDisable(GL_LIGHT0);
       
       p3d_get_filter_box(&x1,&x2,&y1,&y2,&z1,&z2);
       PrintInfo(("-> filter: %f,%f, %f,%f, %f,%f\n",
	      x1,x2,y1,y2,z1,z2));
       g3d_set_color_vect(Red,NULL);
       
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
     else{
       glCallList(robot_box_list); 
     }
   } 
}

/*******************************************/
/* Fonction tracant la boite materialisant */
/* les limites de l'environnement          */
/*******************************************/
void g3d_draw_env_box(void)
{double x1,x2,y1,y2,z1,z2,nampl,temp;
 int i,n=10;

  if(boxlist == -1){

   boxlist = glGenLists(1);
   glNewList(boxlist,GL_COMPILE_AND_EXECUTE);

   p3d_get_env_box(&x1,&x2,&y1,&y2,&z1,&z2);

   glDisable(GL_LIGHTING);
   glDisable(GL_LIGHT0);
   g3d_set_color_vect(Black,NULL);

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

   glBegin(GL_LINES);
   {  
     nampl=(x2-x1)/n;
     for(i=1;i<=n-1;i++){
       temp=x1+i*nampl;
       glVertex3d(temp,y1,z1);  
       glVertex3d(temp,y2,z1);
     }
     nampl=(y2-y1)/n;
     for(i=1;i<=n-1;i++){
       temp=y1+i*nampl;
       glVertex3d(x1,temp,z1);  
       glVertex3d(x2,temp,z1);
     }
   }
   glEnd();
   
   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);

   glEndList();
 }
 else{
    glCallList(boxlist); 
 }
} 

/***************************************/
/* Fonction tracant l'obstacle courant */
/***************************************/
static
void g3d_draw_obstacle(G3D_Window *win)
{pp3d_obj o;


 o = (p3d_obj *) p3d_get_desc_curid(P3D_OBSTACLE);
 

/*  g3d_draw_obj_BB(o);  */
    

/* on teste si l'obstacle est dans le frustum avant de le dessiner */ 
 if (BoxInFrustum_obj(o,win)){
   g3d_draw_object(o,0,win);
 }

}
	



/***********************************************/
/* Fonction tracant le robot courant en tenant */
/* compte de s'il a percute un obstacle ou non */
/***********************************************/
void g3d_draw_robot(int ir,G3D_Window* win)
{int nb,b,ib,num;
 int coll=0;
 /* B Kineo Carl 22.02.2002 */
 /* test */
/*  p3d_poly *p1 = NULL; */
/*  p3d_poly *p2 = NULL; */
/*  p3d_obj *o1 = NULL; */
/*  p3d_obj *o2 = NULL; */
 /* E Kineo Carl 22.02.2002 */

  b = p3d_get_desc_curnum(P3D_BODY);
  nb= p3d_get_desc_number(P3D_BODY);

  num = p3d_get_desc_curnum(P3D_ROBOT);

  if(p3d_numcoll){
    coll = p3d_col_does_robot_collide(ir,p3d_numcoll);
  }

  /* B Kineo Carl 22.02.2002 */
  /* test */
/*   if(coll) */
/*     { */
/*       p3d_col_get_report(0,&p1,&p2); */
/*       printf("G3D: clash found %s, %s\n",p1->poly->name,p2->poly->name); */

/*       p3d_col_get_report_obj(&o1,&o2); */
/*       printf("G3D: clash found %s, %s\n",o1->name,o2->name); */
/*     } */
  /* E Kineo Carl 22.02.2002 */

  for(ib=0;ib<nb;ib++){
    p3d_sel_desc_num(P3D_BODY,ib);
    g3d_draw_body(coll,win);
  }
  p3d_sel_desc_num(P3D_BODY,b);
  p3d_drawRobotMoveMeshs();
}

void p3d_drawRobotMoveMeshs(void){
  if(G3D_SELECTED_JOINT != -999){
    p3d_rob* robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    if (robot->num != G3D_SELECTED_ROBOT){
      return;
    }
    p3d_jnt* jnt = robot->joints[G3D_SELECTED_JOINT];
    p3d_vector3 axis;
    double t = 0, size = 0;
    size = MAX(jnt->o->BB0.xmax - jnt->o->BB0.xmin, jnt->o->BB0.ymax - jnt->o->BB0.ymin);
    size = MAX(size, jnt->o->BB0.zmax - jnt->o->BB0.zmin);
    size /= 2;

    switch(jnt->type){
      case P3D_ROTATE:{
        glLoadName(-1);
        p3d_mat4ExtractRot(jnt->abs_pos, axis, &t);
        glPushMatrix();
        glTranslatef(jnt->abs_pos[0][3],jnt->abs_pos[1][3],jnt->abs_pos[2][3]);
        glRotatef((180/M_PI)*t, axis[0], axis[1], axis[2]);
        g3d_drawCircle(0,0, size + size*0.3, Green, NULL, 3);
        glPopMatrix();
        break;
      }
      case P3D_FREEFLYER:{
        p3d_matrix4 repPos;
        p3d_mat4Copy(jnt->pos0, repPos);
        repPos[0][3] = jnt->abs_pos[0][3];
        repPos[1][3] = jnt->abs_pos[1][3];
        repPos[2][3] = jnt->abs_pos[2][3];
        g3d_drawRepMoveObj(repPos, size + size*0.4, 7);
        g3d_drawSphMoveObj(repPos, size + size*0.1);
        break;
      }
      case P3D_KNEE:{
        p3d_matrix4 repPos;
        p3d_mat4Copy(jnt->pos0, repPos);
        repPos[0][3] = jnt->abs_pos[0][3];
        repPos[1][3] = jnt->abs_pos[1][3];
        repPos[2][3] = jnt->abs_pos[2][3];
        g3d_drawSphMoveObj(repPos, size + size*0.1);
        break;
      }
      case P3D_PLAN:{
        p3d_matrix4 repPos;
        p3d_mat4Copy(jnt->pos0, repPos);
        repPos[0][3] = jnt->abs_pos[0][3];
        repPos[1][3] = jnt->abs_pos[1][3];
        repPos[2][3] = jnt->abs_pos[2][3];
        g3d_drawRepMoveObj(repPos, size + size*0.4, 3);
        glLoadName(-3);
        p3d_mat4ExtractRot(repPos, axis, &t);
        glPushMatrix();
        glTranslatef(jnt->abs_pos[0][3],jnt->abs_pos[1][3],jnt->abs_pos[2][3]);
        glRotatef((180/M_PI)*t, axis[0], axis[1], axis[2]);
        g3d_drawCircle(0,0, size + size*0.1, Blue, NULL, 3);
        glPopMatrix();
        break;
      }
    }
  }
}
/******************************************************/
/* Fonction tracant le corps courant du robot courant */
/******************************************************/
static
void g3d_draw_body(int coll,G3D_Window* win)
{pp3d_obj o;

 o = (p3d_obj *) p3d_get_desc_curid(P3D_BODY);
 /* g3d_draw_obj_BB(o); */ /* Carl: KCD test */
 if (BoxInFrustum_obj(o,win)){
   g3d_draw_object_moved(o,coll,win);
 }
 
}

/*******************************************/
/* Fonction dessinant un objet en position */
/*******************************************/
static 
void g3d_draw_object_moved(p3d_obj *o, int coll, G3D_Window* win)
{
 int i,j; 

 /* On cree une matrice compatible avec opengl */
/* on y met la position du jnt par rapport au repere global*/
/* on s'en sert dans la fct 'g3d_calcule_resolution' (g3d_draw.c)*/
/* pour calculer la position de l'objet par rapport au repere global*/
/* de la scene */
  for(i=0 ; i<=3 ; i++){
    for(j=0 ; j<=3 ; j++){
      matrix_pos_absGL[4*j+i]=o->jnt->abs_pos[i][j];
    }
  }
  glPushMatrix();
  glMultMatrixf(matrix_pos_absGL);
  g3d_draw_object(o,coll,win);
  glPopMatrix();

}


/*******************************/
/* Fonction dessinant un objet */
/*******************************/
static 
void g3d_draw_object(p3d_obj *o, int coll, G3D_Window *win)
{int i;
  
  glLoadName(o->o_id_in_env);
  for(i=0;i<o->np;i++){
    if (o->pol[i]->TYPE!=P3D_GHOST){
      if((!win->FILAIRE)&&(!win->GOURAUD)){g3d_draw_poly(o->pol[i],win,coll,1);}
      if((!win->FILAIRE)&&(win->GOURAUD)){g3d_draw_poly(o->pol[i],win,coll,2);}
      if((win->FILAIRE || win->CONTOUR)){g3d_draw_poly(o->pol[i],win,coll,0);}
    }
    
  }
  
/*  for(i=0;i<o->np;i++){ */
/*    if (o->pol[i]->TYPE!=P3D_GHOST){ */
/*      if((win->FILAIRE || win->CONTOUR)){ */
/*        g3d_draw_poly(o->pol[i],win,coll,0); */
/*      } */
/*      if(!win->FILAIRE){ */
/*        if(!win->GOURAUD){ */
/* 	 g3d_draw_poly(o->pol[i],win,coll,1); */
/*        } */
/*        else{ */
/* 	 g3d_draw_poly(o->pol[i],win,coll,2); */
/*        } */
/*      } */
/*    } */
/*  } */
   

}

/***************************************************/
/* Fonction tracant la boite englobante d'un objet */
/***************************************************/
#if 0
static 
void g3d_draw_obj_BB(p3d_obj *o)
{double x1,x2,y1,y2,z1,z2;

 p3d_get_BB_obj(o,&x1,&x2,&y1,&y2,&z1,&z2); /* new Carl 23052001 */
/*  x1 = o->BB.xmin; */
/*  x2 = o->BB.xmax; */
/*  y1 = o->BB.ymin; */
/*  y2 = o->BB.ymax; */
/*  z1 = o->BB.zmin; */
/*  z2 = o->BB.zmax; */

 g3d_draw_a_box(x1,x2,y1,y2,z1,z2,Green,0);
} 
#endif

/* Debut Modification Thibaut */
/*************************************************************/
/* Fonction tracant la boite englobante de l'ostacle courant */
/*************************************************************/
static void g3d_draw_ocur_special(G3D_Window *win)
{
  double x1,x2,y1,y2,z1,z2;
  int i;
/*   G3D_Window *win; */

  pp3d_obj oc =  (pp3d_obj)p3d_get_desc_curid(P3D_OBSTACLE);

/*   win =  g3d_get_cur_win(); */

  if(G3D_DRAW_OCUR_SPECIAL == 1) {
    p3d_get_BB_obj(oc,&x1,&x2,&y1,&y2,&z1,&z2); /* new Carl 23052001 */
/*     x1 = oc->BB.xmin; */
/*     x2 = oc->BB.xmax; */
/*     y1 = oc->BB.ymin; */
/*     y2 = oc->BB.ymax; */
/*     z1 = oc->BB.zmin; */
/*     z2 = oc->BB.zmax; */
    g3d_draw_a_box(x1,x2,y1,y2,z1,z2,Red,0);
  } else {
    for(i=0;i<oc->np;i++)
      g3d_draw_poly_special(oc->pol[i],win,Red);
  }
}
/* Fin Modification Thibaut */

/***************************************************/
/* Fonction tracant la boite englobante d'un robot */
/***************************************************/
#if 0
static 
void g3d_draw_rob_BB(p3d_rob *r)
{double x1,x2,y1,y2,z1,z2;

 p3d_get_BB_rob(r,&x1,&x2,&y1,&y2,&z1,&z2); /* new Carl 23052001 */
/*  x1 = r->BB.xmin; */
/*  x2 = r->BB.xmax; */
/*  y1 = r->BB.ymin; */
/*  y2 = r->BB.ymax; */
/*  z1 = r->BB.zmin; */
/*  z2 = r->BB.zmax; */
 PrintInfo(("x1=%f,x2=%f,y1=%f,y2=%f,z1=%f,z2=%f\n",x1,x2,y1,y2,z1,z2);
 g3d_draw_a_box(x1,x2,y1,y2,z1,z2,Yellow,0)); 
} 
#endif
