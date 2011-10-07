#include "Graphic-pkg.h"
#include "P3d-pkg.h"

#ifdef P3D_COLLISION_CHECKING
#include "Collision-pkg.h"
#endif

#include "Rrt-pkg.h"


//int boxlist;
extern  int DRAW_TREE;

static void draw_tree (rrt_node *init, int couleur);
static void g3d_draw_env2(void);
void draw_rrt(void);

/******************************************************************************/
/******************************************************************************/

void draw_rrt(void)
{double x1,y1,x2,y2,z1,z2,xmil=0.,ymil=0.,zmil=0.,ampl=0.,xampl=0.,yampl=0.,zampl=0.;
 GLfloat light_position[] = { 20.0, -60.0, 100.0, 1.0 }; 
 GLfloat light_ambient[] = { 0.5, 0.5, 0.5, 1.0 };

  if(p3d_get_desc_number(P3D_ENV)) {
      p3d_get_env_box(&x1,&x2,&y1,&y2,&z1,&z2);
      xmil=(x2+x1)/2.; ymil=(y2+y1)/2.; zmil=(z2+z1)/2.;
      xampl=(x2-x1)/2.;yampl=(y2-y1)/2.;zampl=(z2-z1)/2.;
      ampl = 1.5*sqrt(xampl*xampl+yampl*yampl+zampl*zampl);
      light_position[0]=xmil; light_position[1]=ymil; light_position[2]=zmil+0.5*zampl;
  } 
  glLightfv(GL_LIGHT0, GL_POSITION, light_position); 
  glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 2./ampl); 
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  g3d_draw_env2();
}

/*******************************************************/
/* Fonction initialisant graphiquement l'environnement */
/* si besoin est et l'affichant                        */
/*******************************************************/
 
static void
g3d_draw_env2(void)
{
  pp3d_env e;
  pp3d_rob r;
  rrt_graph *g_rrt; 
  G3D_Window *win;
  g_rrt = get_rrt_graph();
  win = g3d_get_cur_win();
  e = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  if(e->INIT)
    {
      //g3d_init_poly(); db
      boxlist = -1; 
      p3d_reset_robotboxlist(); 
      e->INIT = 0;
    }
  g3d_draw_env_box();
  //  g3d_draw_robot_box(); db, cette fonction est statique dans
  //  g3d_draw_env.c...

  g3d_draw_obstacles(win,opengl_context);
  g3d_draw_robots(win,opengl_context);
  /* g3d_draw_MOVABLE_OBJECTS(); */
  g3d_draw();
  
  if (g_rrt != NULL)
    {
      draw_tree(g_rrt->init, 0);
      draw_tree(g_rrt->goal, 1);
    }    
}


/*********************************/
/* fonction dessinant les arbres */
/*********************************/

static void 
draw_tree (rrt_node *init, int couleur)
{
  rrt_edge *edge;
  rrt_node *node1;
  rrt_node *node2;
  rrt_node *list;

  int color;
  
  glDisable(GL_LIGHTING);
  glDisable(GL_LIGHT0);  
  
  if (couleur == 0)
    //    glColor3dv(Redv);
    color = Red;
  else
    color = Green;
    //    glColor3dv(Greenv);
  
  list = init;
  
  while (list != NULL)
    {
      if (list->fedge != NULL)
	{
	  edge = list->fedge;
	  while (edge != NULL)
	    {
	      node1 = edge->n1;
	      node2 = edge->n2;
	      g3d_drawOneLine(node1->q[0],node1->q[1],node1->q[2],
			      node2->q[0],node2->q[1],node2->q[2],
			      color, NULL);
	      edge = edge->next;
	    }
	}
      //PrintInfo(("q:%f",list->q[0]));
      list = list->next;
    }
  //PrintInfo(("\n")); 
  
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

}

