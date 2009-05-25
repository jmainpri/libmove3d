#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h" 
#include "Hri_planner-pkg.h"

#define HUMAN 111
#define CENTER 112

#define BTS_SIZE 10 /* maximum number of bitmaps allowed in a bitmapset */

hri_bitmapset* BTSET = NULL;
pp3d_graph BTGRAPH = NULL; 

static int insert2table(double value, int cx, int cy, int cz, double * Table,	int * x, int * y, int * z, int l);
static double Cellcost(hri_bitmap_cell* cell);
static int CalculateCellValue(hri_bitmapset * btset,hri_bitmap * bitmap,  hri_bitmap_cell* cell,hri_bitmap_cell* fomcell);
static int is_in_fow(double xh, double yh, double xt, double yt, double orient, double fowangle);

/****************************************************************/
/*!
 * \brief Creates a bitmap structure
 * 
 * \param x     xdimension of bitmap
 * \param y     ydimension
 * \param z     zdimension
 * \param pace  real distance equivalent of the dist between 2 cells 
 * !
 
 * \return NULL in case of a problem
 */
/****************************************************************/
hri_bitmap*  hri_bt_create_bitmap(int x, int y, int z, double pace, int type, double (*fct)(hri_bitmapset*,int, int, int))
{
  hri_bitmap* bitmap = hri_bt_create_empty_bitmap(x, y, z, pace, type, fct);
  
  if(bitmap == NULL)
    return NULL;  
  
  hri_bt_create_data(bitmap);
	
  
  return bitmap;
}

/****************************************************************/
/*!
 * \brief Creates necessary data field for an empty bitmap
 * 
 * \param bitmap    empty bitmap
 * 
 * \return NULL in case of a problem
 */
/****************************************************************/
int hri_bt_create_data(hri_bitmap* bitmap)
{
  int x,y,z;
  
  if(bitmap==NULL)
    return FALSE;
  
  bitmap->data = MY_ALLOC(hri_bitmap_cell**,bitmap->nx);
  for(x=0; x<bitmap->nx; x++) {
    bitmap->data[x] = MY_ALLOC(hri_bitmap_cell*,bitmap->ny);
    for(y=0; y<bitmap->ny; y++) {
      bitmap->data[x][y] = MY_ALLOC(hri_bitmap_cell,bitmap->nz);
      for(z=0; z<bitmap->nz; z++) {
	bitmap->data[x][y][z].val = 0;
	bitmap->data[x][y][z].h = -1;
	bitmap->data[x][y][z].g = 0;
	bitmap->data[x][y][z].parent = NULL;
	bitmap->data[x][y][z].closed = FALSE;
	bitmap->data[x][y][z].open   = FALSE;
	bitmap->data[x][y][z].x = x;
	bitmap->data[x][y][z].y = y;
	bitmap->data[x][y][z].z = z;
	bitmap->data[x][y][z].locked = FALSE; 
      }
    }
  }
  return TRUE;
}

/****************************************************************/
/*!
 * \brief Creates an empty bitmap 
 * 
 * \param x     xdimension of bitmap
 * \param y     ydimension
 * \param z     zdimension
 * \param pace  real distance equivalent of the dist between 2 cells 
 * \param type  type of the bitmap
 * 
 * \return NULL in case of a problem
 */
/****************************************************************/
hri_bitmap*  hri_bt_create_empty_bitmap(int x, int y, int z, double pace, int type, double (*fct)(hri_bitmapset*,int, int, int))
{
  hri_bitmap* bitmap = MY_ALLOC(hri_bitmap,1);
  
  if(x < 1 || y < 1 || z < 1){
    PrintWarning(("NHP - Be careful, you're creating a bitmap with x<1 or y<1"));
    return NULL;
  }
  
  bitmap->active = FALSE; /* the activation flag allows the bitmap be visible on screen */
  
  //bitmap->realx = env->box.x1;
  //bitmap->realy = env->box.y1;
  //bitmap->realz = env->box.z1;
  
  bitmap->nx = x;
  bitmap->ny = y;
  bitmap->nz = z;
  //btset->pace = pace;
  bitmap->search_start = NULL;
  bitmap->search_goal = NULL;
  bitmap->current_search_node = NULL;
  bitmap->searched = FALSE;
  bitmap->type = type;
  bitmap->data = NULL;
  
  bitmap->calculate_cell_value = fct;
	
  return bitmap;
}

int hri_bt_change_bitmap_position(hri_bitmapset * btset, double x, double y, double z)
{
  if(btset == NULL)
    return FALSE;
  
  btset->realx = x;
  btset->realy = y;
  btset->realz = z;
  
  return TRUE;
}


/****************************************************************/
/*!
 * \brief Destroys a bitmap structure
 *
 * \param bitmap  the bitmap
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_destroy_bitmap(hri_bitmap* bitmap)
{
  if(bitmap==NULL)
    return TRUE;
  
  if(bitmap->data != NULL)
    hri_bt_destroy_bitmap_data(bitmap);
  
  MY_FREE(bitmap,hri_bitmap,1);
  return TRUE;
}

/****************************************************************/
/*!
 * \brief Destroys a bitmap data
 *
 * \param bitmap  the bitmap
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_destroy_bitmap_data(hri_bitmap* bitmap)
{
  int x,y;
  
  for(x=0;x<bitmap->nx; x++) {
    for(y=0;y<bitmap->ny; y++)      
      MY_FREE(bitmap->data[x][y],hri_bitmap_cell,bitmap->nz);
    MY_FREE(bitmap->data[x],hri_bitmap_cell*,bitmap->ny);
  }
  MY_FREE(bitmap->data,hri_bitmap_cell**,bitmap->nx);
  
  return TRUE;
}

/****************************************************************/
/*!
 * \brief Destroys a bitmapset structure
 *
 * \param bitmapset  the bitmapset
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_destroy_bitmapset(hri_bitmapset* bitmapset)
{
  int i;
  
  if(bitmapset->bitmap != NULL){
    for(i=0; i<bitmapset->n; i++){
      hri_bt_destroy_bitmap(bitmapset->bitmap[i]);
    }  
    MY_FREE(bitmapset->bitmap,hri_bitmap*,BT_BITMAP_NO);
  }
  
  for(i=0; i<BT_HUMAN_NO; i++){
    if(bitmapset->human[i] != NULL)
      hri_bt_destroy_human(bitmapset->human[i]);
  }   
  MY_FREE(bitmapset->human,hri_human*,BT_HUMAN_NO);
	
  if(bitmapset->path)
    hri_bt_destroy_path(bitmapset);
  
  MY_FREE(bitmapset,hri_bitmapset,1);  

  bitmapset = NULL;
  
  return TRUE;
}

/****************************************************************/
/*!
 * \brief Destroys a human :-)
 *
 * \param human  the human
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_destroy_human(hri_human* human)
{
  int i;
  
  for(i=0; i<BT_STATE_NO; i++){
    hri_bt_destroy_state(human->state[i]);
  }
  MY_FREE(human->state,hri_human_state,BT_STATE_NO);
  MY_FREE(human,hri_human,1);  
  
  return TRUE;
}

/****************************************************************/
/*!
 * \brief Destroys a human state
 *
 * \param state  the state
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_destroy_state(hri_human_state state)
{
  /* it's empty for the moment */
  
  return TRUE;
  
}

/****************************************************************/
/*!
 * \brief Marks a bitmap as active
 * 
 * \param type  the bitmap
 * 
 * \return TRUE if successful 
 */
/****************************************************************/
int hri_bt_activate(int type, hri_bitmapset* bitmapset)
{
  int i;
	
  if(bitmapset==NULL || bitmapset->bitmap==NULL)
    return FALSE;
  
  for(i=0; i<bitmapset->n; i++){    
    if(bitmapset->bitmap[i] != NULL && bitmapset->bitmap[i]->type == type){
      if(bitmapset->bitmap[i]->data == NULL)
	hri_bt_create_data(bitmapset->bitmap[i]);
      if(!hri_bt_fill_bitmap(bitmapset,type)){
	PrintWarning(("NHP - Try to fill an unvalid typed bitmap"));
	return FALSE;
      }
      else{
	bitmapset->bitmap[i]->active = TRUE;
	return TRUE;
      }
    }
  } 
	
  return FALSE;
}

/****************************************************************/
/*!
 * \brief Fill the bitmap with right parameters
 * 
 * \param type  type of the bitmap
 * 
 * \return TRUE if successful 
 */
/****************************************************************/
int hri_bt_fill_bitmap(hri_bitmapset * btset, int type)
{
  int x,y,z;
  
  if(type == BT_OBSTACLES){
    if(btset->bitmap[BT_OBSTACLES] != NULL)
      return TRUE;
  } 
  if(type == BT_PATH){
    PrintWarning(("NHP - Trying to fill a BT_PATH bitmap"));
    return TRUE;
  }
  for(x=0; x<btset->bitmap[type]->nx; x++){
    for(y=0; y<btset->bitmap[type]->ny; y++){
      for(z=0; z<btset->bitmap[type]->nz; z++){
	btset->bitmap[type]->data[x][y][z].val = 
	  btset->bitmap[type]->calculate_cell_value(btset,x,y,z);
      }
    }
  }
  
  return TRUE;
}


/****************************************************************/
/*!
 * \brief Put all obstacles to the bitmaps, obs value is -1
 * 
 * \param G          a graph
 * \param bitmapset  bitmaps 
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_create_obstacles( hri_bitmapset* btset )
{
  int i;
  p3d_env* env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int expand_rate, minimum_expand_rate;
  configPt robotq;
	
  if(btset == NULL)
    return FALSE;
 	
  if(btset->robot == NULL)
    expand_rate = 0;
  else {
    robotq = p3d_get_robot_config(btset->robot) ;
    
    if(DISTANCE2D(btset->robot->BB.xmax,btset->robot->BB.ymax,robotq[ROBOTq_X],robotq[ROBOTq_Y]) >
       DISTANCE2D(btset->robot->BB.xmin,btset->robot->BB.ymin,robotq[ROBOTq_X],robotq[ROBOTq_Y]))
      
      expand_rate = (btset->robot->BB.xmax-robotq[ROBOTq_X] > btset->robot->BB.ymax-robotq[ROBOTq_Y])?
	((btset->robot->BB.xmax-robotq[ROBOTq_X])/btset->pace):
	((btset->robot->BB.ymax-robotq[ROBOTq_Y])/btset->pace);
    
    else
      expand_rate = (btset->robot->BB.xmin-robotq[ROBOTq_X] > btset->robot->BB.ymin-robotq[ROBOTq_Y])?
	((robotq[ROBOTq_X]-btset->robot->BB.xmin)/btset->pace):
	((robotq[ROBOTq_Y]-btset->robot->BB.ymin)/btset->pace);	 
    
    p3d_destroy_config(btset->robot,robotq);
    
  }
  /* expand_rate--; */
  
  minimum_expand_rate = (int) (0.40/btset->pace) - 1; /* THIS IS FOR JIDO  - NEEDS TO BE DONE PROPERLY*/
  
  
  /* expand_rate is always >= than minimum_expand_rate */	
  
  for(i=0; i<env->no ; i++){  
    hri_bt_insert_1obs2bitmap(btset,btset->bitmap[BT_OBSTACLES], env->o[i], env, expand_rate, -1)  ;
  }  
  for(i=0; i<env->no ; i++){                             
    hri_bt_insert_1obs2bitmap(btset,btset->bitmap[BT_OBSTACLES], env->o[i], env, minimum_expand_rate, -2);
  }  

  env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  for(i=0; i<env->nr; i++){
    if( strcmp("robot",env->robot[i]->name) ){
      hri_bt_insert_1obs2bitmaprobot(btset,btset->bitmap[BT_OBSTACLES],env->robot[i] , env, minimum_expand_rate, -2);
      /* printf("Obstacles updated for %s\n",env->robot[i]->name); */
    }
  }

  return TRUE;  
}

/****************************************************************/
/*!
 * \brief Put one obstacle to the bitmap, obs value is -1
 * 
 * \param G       a graph
 * \param bitmap  a bitmap
 * \param obj     obstacle object
 * \param robot  to whom the obstacle belongs 
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_insert_1obs2bitmap(hri_bitmapset * btset, hri_bitmap* bitmap, p3d_obj* obj, p3d_env* env, double expand, double value)
{
  int objxmin,objxmax,objymin,objymax,objzmin,objzmax;
  
  if(bitmap == NULL){
    PrintError(("NHP - Cannot create obstacles bitmap=NULL\n"));
    return FALSE;
  }
  
  if(obj == NULL){
    PrintError(("object bitmap insertion problem\n"));
    return FALSE;
  }
  
  /* if(obj->BB.xmax < env->box.x1 || obj->BB.xmin > env->box.x2 ||   */
  /*      obj->BB.ymax < env->box.y1 || obj->BB.ymin > env->box.y2 || */
  /*      obj->BB.zmax < env->box.z1 || obj->BB.zmin > env->box.z2) { */
  /*     /\*PrintError(("object out of range\n"));*\/ */
  /*     return FALSE;   */
  /*   } */
  
  if(obj->BB.xmax < btset->realx || obj->BB.xmin > bitmap->nx*btset->pace+btset->realx ||
     obj->BB.ymax < btset->realy || obj->BB.ymin > bitmap->ny*btset->pace+btset->realy ||
     obj->BB.zmax < btset->realz || obj->BB.zmin > bitmap->nz*btset->pace+btset->realz) {    
    /*PrintError(("object out of range\n"));*/
    return FALSE;  
  }
  
  /* if(obj->BB.xmin > env->box.x1) */
  /*     objxmin = (int)((obj->BB.xmin - btset->realx)/btset->pace); */
  /*   else */
  /*     objxmin = 0; */
	
  /*   if(obj->BB.xmax < env->box.x2) */
  /*     objxmax = (int)((obj->BB.xmax - btset->realx)/btset->pace); */
  /*   else */
  /*     objxmax = (int)(bitmap->nx-1); */
  
  /*   if(obj->BB.ymin > env->box.y1) */
  /*     objymin = (int)((obj->BB.ymin - btset->realy)/btset->pace); */
  /*   else */
  /*     objymin = 0; */
  
  /*   if(obj->BB.ymax < env->box.y2) */
  /*     objymax = (int)((obj->BB.ymax - btset->realy)/btset->pace); */
  /*   else */
  /*     objymax = (int)(bitmap->ny-1); */
  
  /*   if(obj->BB.zmin > env->box.z1) */
  /*     objzmin = (int)((obj->BB.zmin - btset->realz)/btset->pace); */
  /*   else */
  /*     objzmin = 0; */
  
  /*   if(obj->BB.zmax < env->box.z2) */
  /*     objzmax = (int)((obj->BB.zmax - btset->realz)/btset->pace); */
  /*   else */
  /*     objzmax = (int)(bitmap->nz-1); */
	
	
  objxmin = (int)((obj->BB.xmin - btset->realx)/btset->pace);
  objxmax = (int)((obj->BB.xmax - btset->realx)/btset->pace);
  objymin = (int)((obj->BB.ymin - btset->realy)/btset->pace);
  objymax = (int)((obj->BB.ymax - btset->realy)/btset->pace);
  objzmin = (int)((obj->BB.zmin - btset->realz)/btset->pace);
  objzmax = (int)((obj->BB.zmax - btset->realz)/btset->pace);
	
  hri_bt_fill_bitmap_zone(bitmap, objxmin-expand, objxmax+expand, objymin-expand, 
			  objymax+expand,objzmin-expand, objzmax+expand, value);
  
  return TRUE;
}

/****************************************************************/
/*!
 * \brief Put one obstacle to the bitmap, obs value is -1
 * 
 * \param G       a graph
 * \param bitmap  a bitmap
 * \param obj     obstacle object
 * \param robot  to whom the obstacle belongs 
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_insert_1obs2bitmaprobot(hri_bitmapset * btset, hri_bitmap* bitmap, p3d_rob* obj, p3d_env* env, double expand, double value)
{
  int objxmin,objxmax,objymin,objymax,objzmin,objzmax;
  
  if(bitmap == NULL){
    PrintError(("NHP - Cannot create obstacles bitmap=NULL\n"));
    return FALSE;
  }
  
  if(obj == NULL){
    PrintError(("object bitmap insertion problem\n"));
    return FALSE;
  }
  

  /* printf("Object limits %f %f, %f %f\n",obj->BB.xmin,obj->BB.xmax,obj->BB.ymin,obj->BB.ymax ); */


  if(obj->BB.xmax < btset->realx || obj->BB.xmin > bitmap->nx*btset->pace+btset->realx ||
     obj->BB.ymax < btset->realy || obj->BB.ymin > bitmap->ny*btset->pace+btset->realy ||
     obj->BB.zmax < btset->realz || obj->BB.zmin > bitmap->nz*btset->pace+btset->realz) {    
    /*PrintError(("object out of range\n"));*/
    return FALSE;  
  }
  
  objxmin = (int)((obj->BB.xmin - btset->realx)/btset->pace);
  objxmax = (int)((obj->BB.xmax - btset->realx)/btset->pace);
  objymin = (int)((obj->BB.ymin - btset->realy)/btset->pace);
  objymax = (int)((obj->BB.ymax - btset->realy)/btset->pace);
  objzmin = (int)((obj->BB.zmin - btset->realz)/btset->pace);
  objzmax = (int)((obj->BB.zmax - btset->realz)/btset->pace);
	
  hri_bt_fill_bitmap_zone(bitmap, objxmin-expand, objxmax+expand, objymin-expand, 
			  objymax+expand,objzmin-expand, objzmax+expand, value);
  
  return TRUE;
}


/****************************************************************/
/*!
 * \brief Set a value for a given rectangle on bitmap  
 * 
 * \param bitmap    a bitmap
 * \param objxmin   the minimum x coordinate of rectancle
 * \param objxmax
 * \param objymin
 * \param objymax
 * \param objzmin
 * \param objzmax
 * \param val       the value to be set 
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/  
int  hri_bt_fill_bitmap_zone(hri_bitmap* bitmap, int objxmin, int objxmax, int objymin,
			     int objymax, int objzmin, int objzmax, int val)
{
  int x,y,z,i;
	
  if(bitmap == NULL)
    return FALSE;
  
  if(objxmin<0)
    objxmin = 0;
  if(objymin<0)
    objymin = 0;
  if(objzmin<0)
    objzmin = 0;
  if(objxmax>bitmap->nx-1)
    objxmax = bitmap->nx-1;
  if(objymax>bitmap->ny-1)
    objymax = bitmap->ny-1;
  if(objzmax>bitmap->nz-1)
    objzmax = bitmap->nz-1;
  
  for(x=objxmin; x<objxmax+1; x++)
    for(y=objymin; y<objymax+1; y++)
      for(z=objzmin; z<objzmax+1; z++){
	bitmap->data[x][y][z].val = val; /* -2 hard obstacles, -1 soft */
	for(i=0; i<8; i++){
	  //  bitmap->data[x][y][z].obstacle[i] = val; 
	  bitmap->data[x][y][z].obstacle[i] = FALSE; // PRAGUE
	}
      }
  
  return TRUE;
}

/****************************************************************/
/*!
 * \brief Shows a presentation of a found path on bitmap 
 * 
 * \param bitmap    a bitmap
 * \param color    
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/  
void hri_bt_show_path(hri_bitmapset * btset, hri_bitmap* bitmap)
{
  hri_bitmap_cell* current;
 	
  if(bitmap == NULL)
    return;
  
  if(bitmap->searched){
    
    g3d_drawOneLine(bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0,  
		    bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0.5, Red, NULL);
    g3d_drawOneLine(bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0,  
		    bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0.5, Red, NULL);
    
    
    
    current = bitmap->search_goal;
    while(current != bitmap->search_start){
      g3d_drawOneLine( current->x*btset->pace+btset->realx,
		       current->y*btset->pace+btset->realy,
		       current->z*btset->pace+btset->realz, 
		       current->parent->x*btset->pace+btset->realx,
		       current->parent->y*btset->pace+btset->realy,
		       current->parent->z*btset->pace+btset->realz,
		       4, NULL);
      current = current->parent;      
    } 
  }
	
}

void hri_bt_show_cone(hri_bitmapset * btset, hri_bitmap* bitmap, int h, int r)
{
  int x,y,z;
  
  for(x=0; x<bitmap->nx; x++){
    for(y=0; y<bitmap->ny; y++){
      for(z=0; z<50; z++){
	if(sqrt(x*x+y*y)-z == 0)
	  g3d_drawOneLine(x*btset->pace,y*btset->pace,1-z*0.01,
			  x*btset->pace,y*btset->pace,1,1,NULL);
      }
    }
  }
}

/* REVISION */
/****************************************************************/
/*!
 * \brief Shows a presentation a bitmap 
 * 
 * \param bitmap    a bitmap
 * 
 * \return FALSE in case of a problem
 */
/***************************************************************/  
void  hri_bt_show_bitmap(hri_bitmapset * btset, hri_bitmap* bitmap) 
{
  int i,j;
  double colorvector[4];
  
  if( bitmap->type == BT_PATH){
    hri_bt_show_path(btset,bitmap);
    return;
  }
	
  colorvector[0] = 1;       //red
  colorvector[1] = 0;       //green
  colorvector[2] = 0;       //blue
  colorvector[3] = 0.75;    //transparency
	
  for(i=0; i<bitmap->nx; i++){
    for(j=0; j<bitmap->ny; j++){
      switch(bitmap->type){
					case BT_DISTANCE:
						g3d_drawOneLine(i*btset->pace+btset->realx, j*btset->pace+btset->realy, 1*bitmap->data[i][j][0].val*0.01,  
														i*btset->pace+btset->realx, j*btset->pace+btset->realy, 1*bitmap->data[i][j][0].val*0.01+0.1, 1, NULL); 
						break;
						
					case BT_VISIBILITY:
						
						g3d_drawOneLine(i*btset->pace+btset->realx, j*btset->pace+btset->realy, bitmap->data[i][j][0].val*0.01, 
														i*btset->pace+btset->realx, j*btset->pace+btset->realy, bitmap->data[i][j][0].val*0.01+0.1, Green, NULL);
						break;
						
					case BT_HIDZONES:	
						if(bitmap->data[i][j][0].val>0) 
							g3d_drawOneLine(i*btset->pace+btset->realx, j*btset->pace+btset->realy, 0, 
															i*btset->pace+btset->realx, j*btset->pace+btset->realy, bitmap->data[i][j][0].val*0.001, 1, NULL);
						break;
						
					case BT_OBSTACLES:
						if(bitmap->data[i][j][0].val == -2)
							g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy, 0,
															i*btset->pace+btset->realx,j*btset->pace+btset->realy, 0.1, Red, NULL);
						//g3d_draw_a_Box(btset->robot->BB.xmin,btset->robot->BB.ymin,btset->robot->BB.zmin,btset->robot->BB.xmax,btset->robot->BB.ymax,btset->robot->BB.zmax);
						if(bitmap->data[i][j][0].val == -1 ) // && bitmap->data[i][j][0].obstacle[0] == TRUE && bitmap->data[i][j][0].obstacle[1] == -1 &&
						  //	 bitmap->data[i][j][0].obstacle[2] == -1 && bitmap->data[i][j][0].obstacle[3] == -1 &&
						  // bitmap->data[i][j][0].obstacle[4] == -1 && bitmap->data[i][j][0].obstacle[5] == -1 &&
						  //	 bitmap->data[i][j][0].obstacle[6] == -1 && bitmap->data[i][j][0].obstacle[7] == -1)
							g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy, 0,
															i*btset->pace+btset->realx,j*btset->pace+btset->realy, 0.1, Blue, NULL);
						break; 
						
					case BT_COMBINED:
						if(bitmap->data[i][j][0].val != -1)
							g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy, bitmap->data[i][j][0].val*0.01,
															i*btset->pace+btset->realx,j*btset->pace+btset->realy, bitmap->data[i][j][0].val*0.01+0.1, 1, NULL);
						break; 
						
					case BT_VELOCITY:
						if(bitmap->data[i][j][0].val != -1)
							g3d_drawOneLine(i*btset->pace,j*btset->pace, bitmap->data[i][j][0].val, i*btset->pace,j*btset->pace,bitmap->data[i][j][0].val+50, 1, NULL);
						break; 
						
					
      }    
      
    }
  }
	
}

/* REVISION*/
int hri_bt_save_bitmap(hri_bitmapset* btset,hri_bitmap * B)
{
  FILE* f = NULL;
  int i, j, k;
	
  f = fopen("gridsample.txt","w");
  fprintf(f,"#grid sample: the first two int is the length and height followed by \\n \n");
  fprintf(f,"#grid sample: negative values are obstacles, no permitted to cross over \n");
  //fprintf(f,"%d %d\n ",(int)B->nx,(int)B->ny);
  for(i=0; i<B->nx; i++){
    for(j=0; j<B->ny; j++){
      for(k=0; k<B->nz; k++){ 
	fprintf(f,"%f %f %f %f\n ",i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz,B->data[i][j][k].val);
      }
      fprintf(f,"\n"); 
    }
    fprintf(f,"\n");  
  }  
  fclose(f);
	
  return 0;
}


/* REVISION*/
/****************************************************************/
/*!
 * \brief Shows a presentation all bitmaps 
 * 
 * \param bitmapset   an array of bitmap
 * \param step        the diffenrence of height between bitmaps 
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/  
void  hri_bt_show_bitmapset(hri_bitmapset* bitmapset) 
{
  int i;  
  
  for(i=0; i<BT_BITMAP_NO; i++){
    hri_bt_show_bitmap(bitmapset,bitmapset->bitmap[i]);   
  }
  
}

hri_bitmapset* hri_bt_create_bitmaps()
{
  hri_bitmapset* bitmapset = MY_ALLOC(hri_bitmapset,1);
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int i, hnumber=0;  
  
  bitmapset->human = MY_ALLOC(hri_human*,BT_HUMAN_NO);
  for(i=0; i<BT_HUMAN_NO; i++)
    bitmapset->human[i] = NULL;
  
  bitmapset->visball = NULL;
  bitmapset->robot = NULL;
  bitmapset->object = NULL;
  
  for(i=0; i<env->nr; i++){
    if( !strcmp("robot",env->robot[i]->name) )
      bitmapset->robot = env->robot[i];
    if( !strncmp("human",env->robot[i]->name,5) ){
      bitmapset->human[hnumber] = hri_bt_create_human(env->robot[i]);
      hnumber++;
    }
    if( !strcmp("visball",env->robot[i]->name) )
      bitmapset->visball = env->robot[i];  
    if( !strcmp("bottle",env->robot[i]->name) )
      bitmapset->object = env->robot[i];  
  }
  
  if(hnumber == 0)
    PrintWarning(("NHP - No humans in the environment"));
  if(bitmapset->visball == NULL)
    PrintWarning(("NHP - No visibility ball present, check the p3d file"));
	
  bitmapset->human_no = hnumber;
  bitmapset->actual_human = 0;
  bitmapset->bitmap = NULL;
  bitmapset->manip = 0;
	
  bitmapset->BT_target_available = FALSE;
	
	
  return bitmapset;
}

/****************************************************************/
/*!
 * \brief Creates a bitmapset structure with empty bitmaps
 * 
 * \param x     xdimension of bitmaps
 * \param y     ydimension
 * \param pace  real distance equivalent of the dist between 2 cells 
 * \param no    number of bitmaps u want to create
 * 
 * \return NULL in case of a problem
 */
/****************************************************************/
int hri_bt_init_bitmaps(hri_bitmapset * bitmapset,int x, int y, int z, double pace) 
{ 
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	
  if(x < 1 || y < 1 || z < 1){
    PrintWarning(("NHP - Creating empty bitmaps with x<1 or y<1 or z<1"));
    return FALSE;
  }
	
  bitmapset->realx = env->box.x1;
  bitmapset->realy = env->box.y1;
  bitmapset->realz = env->box.z1;
  bitmapset->pace = pace;
  
  bitmapset->bitmap = MY_ALLOC(hri_bitmap*,BT_BITMAP_NO);
  bitmapset->max_size = BT_BITMAP_NO;
  
  bitmapset->bitmap[BT_DISTANCE]   = hri_bt_create_empty_bitmap(x,y,z,pace,BT_DISTANCE, hri_bt_calc_dist_value);
  bitmapset->bitmap[BT_VISIBILITY] = hri_bt_create_empty_bitmap(x,y,z,pace,BT_VISIBILITY,hri_bt_calc_vis_value);
  bitmapset->bitmap[BT_HIDZONES]   = hri_bt_create_empty_bitmap(x,y,z,pace,BT_HIDZONES,hri_bt_calc_hz_value);
  bitmapset->bitmap[BT_OBSTACLES]  = hri_bt_create_bitmap(x,y,z,pace,BT_OBSTACLES,NULL);
  hri_bt_create_obstacles(bitmapset); 
  bitmapset->bitmap[BT_VELOCITY]   = hri_bt_create_empty_bitmap(x,y,z,pace,BT_VELOCITY,hri_bt_calc_vel_value);
  bitmapset->bitmap[BT_COMBINED]   = hri_bt_create_empty_bitmap(x,y,z,pace,BT_COMBINED,hri_bt_calc_combined_value);
  bitmapset->bitmap[BT_PATH]       = hri_bt_create_bitmap(x,y,z,pace,BT_PATH,hri_bt_calc_combined_value);
  
  bitmapset->n = 7;
	
  bitmapset->path = NULL;
  bitmapset->pathexist = FALSE;
  bitmapset->combine_type = BT_COMBINE_SUM; /* default value */
  bitmapset->changed = FALSE;
  
	
	
  return TRUE;
	
}


hri_human* hri_bt_create_human(p3d_rob * robot)
{
  hri_human * human = MY_ALLOC(hri_human,1);
  
  human->HumanPt = robot;
  human->state = MY_ALLOC(hri_human_state,BT_STATE_NO);
  human->states_no = 2;
  human->exists = FALSE; /* HUMAN EXIST */
  
  strcpy(human->state[BT_SITTING].name,"SITTING");
  human->state[BT_SITTING].dheight = 40;
  human->state[BT_SITTING].dradius = 17;
  human->state[BT_SITTING].vheight = 30;
  human->state[BT_SITTING].vback = 4;
  human->state[BT_SITTING].vsides = 8.2;
  human->state[BT_SITTING].hradius = 2.8;
  
  human->state[BT_SITTING].c1 =  DTOR(-14.08);
  human->state[BT_SITTING].c2 =  DTOR( 90);
  human->state[BT_SITTING].c3 =  DTOR( 14.08);
  human->state[BT_SITTING].c4 =  DTOR( 90);
  human->state[BT_SITTING].c5 =  DTOR(-90);
  human->state[BT_SITTING].c6 =  DTOR(-90);
  human->state[BT_SITTING].c7 =  0.46;
  
  
  strcpy(human->state[BT_STANDING].name,"STANDING");
  human->state[BT_STANDING].dheight = 30;
  human->state[BT_STANDING].dradius = 20;
  human->state[BT_STANDING].vheight = 30;
  human->state[BT_STANDING].vback = 4.5;
  human->state[BT_STANDING].vsides = 9;
  human->state[BT_STANDING].hradius = 1.5;
  
  human->state[BT_STANDING].c1 = 0;
  human->state[BT_STANDING].c2 = 0;
  human->state[BT_STANDING].c3 = 0;
  human->state[BT_STANDING].c4 = 0;
  human->state[BT_STANDING].c5 = 0;
  human->state[BT_STANDING].c6 = 0;
  human->state[BT_STANDING].c7 = 1;
  
  human->actual_state = BT_STANDING;
	
  return human;
	
} 

/* REVISION */
/****************************************************************/
/*!
 * \brief create the graph regarding to the path found on bitmap
 * 
 * \param G      the graph
 * \param bitmap the bitmap
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_bitmap_to_GRAPH(hri_bitmapset * btset, p3d_graph *G, hri_bitmap* bitmap) 
{
  configPt q; 
  p3d_node *prev_node;
  double dist = 0;
  int done = FALSE;
  p3d_node *NewNode = NULL;
  double prev_orient;
  
  if(bitmap->searched)
    bitmap->current_search_node = bitmap->search_goal;
  else
    return FALSE;  
  /* c'est ici qu'on doit gerer le bras */  
  
  prev_orient = atan2(G->last_node->N->q[ROBOTq_Y]-bitmap->current_search_node->y,
		      G->last_node->N->q[ROBOTq_X]-bitmap->current_search_node->x);
  
  prev_node =  G->last_node->N;
  q = p3d_copy_config(G->rob, G->search_goal->q);
  
  while(!done){
    if(bitmap->current_search_node == bitmap->search_start)
      done = TRUE; 
    /*  if( (bitmap->current_search_node != bitmap->search_goal) &&  */
    /* 	(bitmap->current_search_node != bitmap->search_start)){ */
    
    q[ROBOTq_X] = bitmap->current_search_node->x * btset->pace + btset->realx;
    q[ROBOTq_Y] = bitmap->current_search_node->y * btset->pace + btset->realy;
    
    /*  q[ROBOTq_RZ] = (prev_orient + atan2((bitmap->current_search_node->y-bitmap->current_search_node->parent->y), */
    /* 					(bitmap->current_search_node->x-bitmap->current_search_node->parent->x)))/2; */
    if(bitmap->current_search_node != bitmap->search_start)
      q[ROBOTq_RZ] = atan2((bitmap->current_search_node->parent->y-bitmap->current_search_node->y),
			   (bitmap->current_search_node->parent->x-bitmap->current_search_node->x))+M_PI;
    else
      q[ROBOTq_RZ] = G->search_start->q[ROBOTq_RZ];
    q[ROBOTq_PAN] = 0;
    
    if(!p3d_equal_config(G->rob, q, G->search_start->q)){
      NewNode = p3d_APInode_make(G, q);
      NewNode->type = LINKING;  
      p3d_insert_node(G, NewNode);
      dist = p3d_APInode_dist(G,prev_node,NewNode);
      p3d_create_edges(G,prev_node,NewNode,dist);
      p3d_add_node_compco(NewNode, prev_node->comp);    
      prev_node = NewNode;
    }
    //  }
    q = p3d_copy_config(G->rob, G->search_start->q);
    bitmap->current_search_node = bitmap->current_search_node->parent; 
    
  } 
  p3d_destroy_config(G->rob,q);
  dist = p3d_APInode_dist(G,prev_node,G->search_start);
  p3d_create_edges(G,prev_node ,G->search_start ,dist);
  p3d_merge_comp(G, G->search_start->comp, &(prev_node->comp));
	
  return TRUE;
	
}

/****************************************************************/
/*!
 * \brief resets all the path related item in bitmaps
 * 
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/ 
void  hri_bt_reset_path(hri_bitmapset * btset)
{  
  int i,j,k;
  hri_bitmap* bitmap;
  
  bitmap = btset->bitmap[BT_PATH];
  
  if(bitmap == NULL){
    PrintError(("Try to erase a path from non path bitmap\n"));
    return;
  }
  
  bitmap->search_start = NULL;
  bitmap->search_goal = NULL;
  bitmap->current_search_node = NULL;
  bitmap->searched = FALSE;
  
  for(i=0; i<bitmap->nx; i++) {
    for(j=0; j<bitmap->ny; j++) {
      for(k=0; k<bitmap->nz; k++) {
	bitmap->data[i][j][k].val = 1;
	bitmap->data[i][j][k].h = -1;
	bitmap->data[i][j][k].g = 0;
	bitmap->data[i][j][k].parent = NULL;
	bitmap->data[i][j][k].closed = FALSE;
	bitmap->data[i][j][k].open   = FALSE;
	bitmap->data[i][j][k].x = i;
	bitmap->data[i][j][k].y = j;
	bitmap->data[i][j][k].z = k;
      }
    }
  }
  
  btset->pathexist = FALSE;
  
}

/****************************************************************/
/*!
 * \brief get the cell of given coordinates 
 * 
 * \param bitmap the bitmap
 * \param x      x coord
 * \param y      y coord  
 * \param z      z coord  
 * 
 * \return NULL in case of a problem
 */
/****************************************************************/
hri_bitmap_cell* hri_bt_get_cell(hri_bitmap* bitmap, int x, int y, int z)
{
  if(bitmap==NULL)
    return NULL;
  
  if(x<0 || x>bitmap->nx-1)
    return NULL;
  if(y<0 || y>bitmap->ny-1)
    return NULL;
  if(z<0 || z>bitmap->nz-1)
    return NULL;
  
  return &bitmap->data[x][y][z];
}

/****************************************************************/
/*!
 * \brief Starts A* search to find a path the bitmap bitmap of a bitmapset with start finish nodes 
 * 
 * \param qs  Start config
 * \param qf  End config
 * \param bitmapset the bitmapset 
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
double hri_bt_start_search(double qs[3], double qf[3], hri_bitmapset* bitmapset, int manip)
{
  hri_bitmap* bitmap;
  int i;
  double enlargement,res;
  configPt qc;
	
  if(bitmapset==NULL || bitmapset->bitmap[BT_PATH]==NULL){
    PrintError(("Trying to find a path in a non existing bitmap or bitmapset\n"));    
    return FALSE;
  }
  bitmap = bitmapset->bitmap[BT_PATH]; 
  
  if(bitmapset->pathexist)
    hri_bt_reset_path(bitmapset);
	
  bitmap->search_start = hri_bt_get_cell(bitmap,(int)((qs[0]-bitmapset->realx)/bitmapset->pace),
					 (int)((qs[1]-bitmapset->realy)/bitmapset->pace),
					 (int)((qs[2]-bitmapset->realz)/bitmapset->pace));  
  bitmap->search_goal  = hri_bt_get_cell(bitmap,(int)((qf[0]-bitmapset->realx)/bitmapset->pace),
					 (int)((qf[1]-bitmapset->realy)/bitmapset->pace),
					 (int)((qf[2]-bitmapset->realz)/bitmapset->pace)); 
  
  if(bitmap->search_start == NULL) {
    PrintWarning(("Search start cell does not exist\n"));
    bitmapset->pathexist = FALSE;
    return -4;
  }
  if(bitmap->search_goal == NULL ){
    PrintWarning(("Search goal cell does not exist\n"));
    bitmapset->pathexist = FALSE;
    return -4;
  }
       	
  if(!manip){
    for(i=0; i<bitmapset->human_no; i++){
      if(bitmapset->human[i]->exists){
				
	if(p3d_col_test_robot_other(bitmapset->robot,bitmapset->human[i]->HumanPt, FALSE)){
	  PrintWarning(("Human too close to start position"));
	  return -3;
	}
      }
    }

    if(DISTANCE2D(bitmapset->robot->BB.xmax,bitmapset->robot->BB.ymax,qs[0],qs[1]) >
       DISTANCE2D(bitmapset->robot->BB.xmin,bitmapset->robot->BB.ymin,qs[0],qs[1]))
      
      enlargement = (bitmapset->robot->BB.xmax-qs[0] > bitmapset->robot->BB.ymax-qs[1])?
	((bitmapset->robot->BB.xmax-qs[0])):
	((bitmapset->robot->BB.ymax-qs[1]));
    
    else
      enlargement = (bitmapset->robot->BB.xmin-qs[0] > bitmapset->robot->BB.ymin-qs[1])?
	((qs[0]-bitmapset->robot->BB.xmin)):
	((qs[1]-bitmapset->robot->BB.ymin));
    
    /*if(qs[0] > bitmapset->human[i]->HumanPt->o[1]->BB.xmin-enlargement &&
      qs[0] < bitmapset->human[i]->HumanPt->o[1]->BB.xmax+enlargement &&
      qs[1] > bitmapset->human[i]->HumanPt->o[1]->BB.ymin-enlargement &&
      qs[1] < bitmapset->human[i]->HumanPt->o[1]->BB.ymax+enlargement ){
      PrintWarning(("Human too close to start position"));
      return FALSE;
      } */
		
    if(bitmapset->bitmap[BT_OBSTACLES]->data[bitmap->search_start->x][bitmap->search_start->y][bitmap->search_start->z].val <0 ||
       bitmapset->bitmap[BT_COMBINED]->calculate_cell_value(bitmapset,bitmap->search_start->x,bitmap->search_start->y,bitmap->search_start->z)<0){
			
      qc = p3d_get_robot_config(bitmapset->robot);
      qc[6]  = bitmap->search_start->x*bitmapset->pace+bitmapset->realx;
      qc[7]  = bitmap->search_start->y*bitmapset->pace+bitmapset->realy;
      qc[11] = bitmapset->robot->ROBOT_POS[11];
      p3d_set_and_update_this_robot_conf(bitmapset->robot, qc);
      if(!p3d_col_test_robot_statics(bitmapset->robot,FALSE)){
        bitmapset->bitmap[BT_OBSTACLES]->data[bitmap->search_start->x][bitmap->search_start->y][bitmap->search_start->z].val = 1;
        p3d_destroy_config(bitmapset->robot, qc);
      }
      else{
	p3d_destroy_config(bitmapset->robot, qc);
	PrintWarning(("Start Position is in an obstacle\n"));
	return -1;
      }
    }
    if(bitmapset->bitmap[BT_OBSTACLES]->data[bitmap->search_goal->x][bitmap->search_goal->y][bitmap->search_goal->z].val <0 ||
       bitmapset->bitmap[BT_COMBINED]->calculate_cell_value(bitmapset,bitmap->search_goal->x,bitmap->search_goal->y,bitmap->search_goal->z)<0){
			
      qc = p3d_get_robot_config(bitmapset->robot);
      qc[6]  = bitmap->search_goal->x*bitmapset->pace+bitmapset->realx;
      qc[7]  = bitmap->search_goal->y*bitmapset->pace+bitmapset->realy;
      qc[11] = bitmapset->robot->ROBOT_GOTO[11];
      p3d_set_and_update_this_robot_conf(bitmapset->robot, qc);
      if(!p3d_col_test_robot_statics(bitmapset->robot,FALSE)){
	bitmapset->bitmap[BT_OBSTACLES]->data[bitmap->search_goal->x][bitmap->search_goal->y][bitmap->search_goal->z].val = 1;
	p3d_destroy_config(bitmapset->robot, qc);
      }
      else{
	p3d_destroy_config(bitmapset->robot, qc);
	PrintWarning(("Goal Position is in an obstacle\n"));
	return -2;
      }
    }
  }

  res=hri_bt_astar_bh(bitmapset,bitmap);
  if( res > -1){
    bitmapset->pathexist = TRUE;
    return res;
  }
  else{
    bitmapset->pathexist = FALSE;
    return -5;
  }   
  
}

/****************************************************************/
/*!
 * \brief A* search: heuristic function 
 * 
 * \param bitmap the bitmap
 * \param x_s    x coord of current cell
 * \param x_s    y coord of current cell 
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/

double hri_bt_dist_heuristic(hri_bitmap* bitmap, int x_s, int y_s, int z_s)
{
  int x_f = bitmap->search_goal->x,
    y_f = bitmap->search_goal->y,
    z_f = bitmap->search_goal->z;
  double cost = 0;
  double h_2ddiag, h_2dmanh,h_diag;
  double D3 = sqrt(3.),D2 = sqrt(2.), D=1.;
  
  if(DISTANCE3D(x_s,y_s,z_s,x_f,y_f,z_f) == 0)
    return 0;
  
  h_diag = MIN( MIN(ABS(x_f-x_s), ABS(y_f-y_s)) , ABS(z_f-z_s) );  
  cost+= h_diag * D3;
  
  if( MIN(ABS(x_f-x_s), ABS(y_f-y_s)) >  ABS(z_f-z_s)){
    h_2ddiag = MIN(ABS(x_f-x_s)-h_diag, ABS(y_f-y_s)-h_diag);
    h_2dmanh = ABS(x_f-x_s)-h_diag + ABS(y_f-y_s)-h_diag;
    cost+= D2 * h_2ddiag + D * (h_2dmanh - 2*h_2ddiag);
  }
  else{
    if( ABS(x_f-x_s) > ABS(y_f-y_s)){
      h_2ddiag = MIN(ABS(x_f-x_s)-h_diag, ABS(z_f-z_s)-h_diag);
      h_2dmanh = ABS(x_f-x_s)-h_diag + ABS(z_f-z_s)-h_diag;
      cost+= D2 * h_2ddiag + D * (h_2dmanh - 2*h_2ddiag);
    }
    else{
      h_2ddiag = MIN(ABS(z_f-z_s)-h_diag, ABS(y_f-y_s)-h_diag);
      h_2dmanh = ABS(z_f-z_s)-h_diag + ABS(y_f-y_s)-h_diag;
      cost+= D2 * h_2ddiag + D * (h_2dmanh - 2*h_2ddiag); 
    }
  }
  cost+=cost;
  //cost*=(1+0.01);
	
  return cost;
  
} 

/****************************************************************/
/*!
 * \brief copy a bitmap
 * 
 * \param bitmap the bitmap to be copied
 * 
 * \return the new bitmap
 */
/****************************************************************/  
hri_bitmap* hri_bt_create_copy(hri_bitmap* bitmap)
{
  hri_bitmap* newbitmap = MY_ALLOC(hri_bitmap,1);
  int i,j,k;
	
  if(bitmap == NULL)
    return NULL;
  
  //  newbtset->realx = btset->realx;
  //newbtset->realy = btset->realy;
  //newbtset->realz = btset->realz;
	
  newbitmap->active = bitmap->active; 
  newbitmap->nx = bitmap->nx;
  newbitmap->ny = bitmap->ny;
  newbitmap->nz = bitmap->nz;
  //newbtset->pace = btset->pace;
  newbitmap->search_start =  bitmap->search_start;
  newbitmap->search_goal = bitmap->search_goal;
  newbitmap->current_search_node = bitmap->current_search_node;
  newbitmap->searched = bitmap->searched;
  
  
  newbitmap->data = MY_ALLOC(hri_bitmap_cell**,bitmap->nx);
  for(i=0; i<bitmap->nx; i++) {
    newbitmap->data[i] = MY_ALLOC(hri_bitmap_cell*,newbitmap->ny);
    for(j=0; j<bitmap->ny; j++) {
      newbitmap->data[i][j] = MY_ALLOC(hri_bitmap_cell,newbitmap->nz);
      for(k=0; k<bitmap->nz; k++) {
	newbitmap->data[i][j][k].val = bitmap->data[i][j][k].val;
	newbitmap->data[i][j][k].h = bitmap->data[i][j][k].h;
	newbitmap->data[i][j][k].g =  bitmap->data[i][j][k].g;
	newbitmap->data[i][j][k].parent = bitmap->data[i][j][k].parent;
	newbitmap->data[i][j][k].closed =  bitmap->data[i][j][k].closed;
	newbitmap->data[i][j][k].open   = bitmap->data[i][j][k].open;
	newbitmap->data[i][j][k].x = bitmap->data[i][j][k].x;
	newbitmap->data[i][j][k].y = bitmap->data[i][j][k].y;
	newbitmap->data[i][j][k].z = bitmap->data[i][j][k].z;
	newbitmap->data[i][j][k].locked = bitmap->data[i][j][k].locked;
      }
    }
  }
  
  return newbitmap;
}  


/****************************************************************/
/*!
 * \brief Creates an empty bitmapset
 * 
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/  
hri_bitmapset*  hri_bt_create_empty_bitmapset()
{
  int i;
  hri_bitmapset* bitmapset = MY_ALLOC(hri_bitmapset,1);
  
  if( bitmapset==NULL)
    return FALSE;
  
  bitmapset->bitmap = MY_ALLOC(hri_bitmap*,BTS_SIZE);
  if(bitmapset->bitmap==NULL)
    return FALSE;
  
  for(i=0; i<BTS_SIZE; i++){
    bitmapset->bitmap[i] = NULL;
  } 
  
  bitmapset->pathexist = FALSE;
  
  return bitmapset;
}

/****************************************************************/
/*!
 * \brief Draws active bitmaps 
 * 
 * \param type type of the bitmap
 * 
 * \return 
 */
/****************************************************************/  
void g3d_hri_bt_draw_active_bitmaps(hri_bitmapset* bitmapset)
{
  int i;
  if(bitmapset!=NULL && bitmapset->bitmap!=NULL){
    for(i=0; i<bitmapset->n; i++)
      if( bitmapset->bitmap[i]->active)
	hri_bt_show_bitmap(bitmapset,bitmapset->bitmap[i]);
  }
}

/****************************************************************/
/*!
 * \brief Draws the robot's target 
 * 
 * \param type type of the bitmap
 * 
 * \return 
 */
/****************************************************************/  
void g3d_hri_bt_draw_targets(hri_bitmapset* bitmapset)
{
  if(bitmapset == NULL)
    return;
	
  if(bitmapset->BT_target_available){
    g3d_drawDisc(bitmapset->BT_target[0],bitmapset->BT_target[1],0,0.5,1,NULL); 
  }
}


/****************************************************************/
/*!
 * \brief  desactivate bitmap of given type 
 * 
 * \param type type of the bitmap
 * 
 * \return 
 */
/****************************************************************/ 
void hri_bt_desactivate(int type, hri_bitmapset* bitmapset)
{
  int i;
	
  if(bitmapset==NULL)
    return;
  
  for(i=0; i<bitmapset->n; i++)
    if(bitmapset->bitmap[i]!=NULL && bitmapset->bitmap[i]->type == type)
      bitmapset->bitmap[i]->active = FALSE;
}

/****************************************************************/
/*!
 * \brief activate bitmap of given type 
 * 
 * \param type type of the bitmap
 * 
 * \return 
 */
/****************************************************************/ 
int hri_bt_is_active(int type, hri_bitmapset* bitmapset)
{
  int i;
	
  if(bitmapset == NULL)
    return FALSE;
  
  for(i=0; i<bitmapset->n; i++)
    if(bitmapset->bitmap[i]!=NULL && bitmapset->bitmap[i]->type == type)
      return bitmapset->bitmap[i]->active;
	
  return FALSE;
}

/* REVISION */
/****************************************************************/
/*!
 * \brief refill all the active bitmaps
 * 
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/ 
int hri_bt_refresh_all(hri_bitmapset * btset)
{
  int i;
  
  if(btset == NULL || btset->bitmap==NULL)
    return FALSE; 
  for(i=0; i<btset->n; i++){
    if(btset->bitmap[i]->active){
      if(btset->bitmap[i]->type == BT_DISTANCE)
	hri_bt_update_distance(btset,btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].dheight,
			       btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].dradius);
      if(btset->bitmap[i]->type == BT_VISIBILITY)
	hri_bt_update_visibility(btset,btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].vheight,
				 btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].vback,
				 btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].vsides);
      if(btset->bitmap[i]->type == BT_HIDZONES)
	hri_bt_update_hidzones(btset,btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].hradius);
      if(btset->bitmap[i]->type == BT_COMBINED)
	hri_bt_update_combined(btset);
			
    }
  }
  return TRUE;
}

/* REVISION */
/****************************************************************/
/*!
 * \brief refill all the active bitmaps
 * 
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/ 
int hri_bt_3drefresh_all(hri_bitmapset * btset)
{
  int l,i,j,k;
  configPt humanConf;
  double hx,hy,hz;
  double Ccoord[6];
  
  if(btset == NULL || btset->bitmap==NULL)
    return FALSE; 
  
  humanConf = p3d_get_robot_config(btset->human[btset->actual_human]->HumanPt);
  
  hx = humanConf[6] - (btset->bitmap[0]->nx * btset->pace /2);
  hy = humanConf[7] - (btset->bitmap[0]->ny * btset->pace /2);
  
  p3d_destroy_config(btset->human[btset->actual_human]->HumanPt,humanConf);
  
  p3d_mat4ExtractPosReverseOrder(btset->human[btset->actual_human]->HumanPt->joints[HUMANj_NECK_TILT]->abs_pos,
		     Ccoord, Ccoord+1, Ccoord+2, Ccoord+3, Ccoord+4, Ccoord+5); 
  
  hz = Ccoord[2] - (btset->bitmap[0]->nz * btset->pace /2);
  
  hri_bt_change_bitmap_position(btset,hx,hy,hz);
  
  for(l=0; l<btset->n; l++){
    if(btset->bitmap[l]->active){
      for(i=0;i<btset->bitmap[l]->nx;i++){
	for(j=0;j<btset->bitmap[l]->ny;j++){    
	  for(k=0;k<btset->bitmap[l]->nz;k++)   
	    btset->bitmap[l]->data[i][j][k].val = btset->bitmap[l]->calculate_cell_value(btset,i,j,k);
	}
      }
    }
  }
	
  hri_exp_fill_obstacles(btset);
	
  return TRUE;
}

/****************************************************************/
/*!
 * \brief get the bitmap of given type 
 * 
 * \param type type of the bitmap
 * 
 * \return the bitmap
 */
/****************************************************************/ 
hri_bitmap* hri_bt_get_bitmap(int type, hri_bitmapset* bitmapset)
{
  int i;
	
  if(bitmapset==NULL)
    return NULL;
  else{
    for(i=0; i<BT_BITMAP_NO; i++)
      if(type == bitmapset->bitmap[i]->type)
	return bitmapset->bitmap[i];
  }
  return NULL;
}


/****************************************************************/
/*!
 * \brief resets the given bitmap data
 * 
 * \param bitmap  the bitmap
 * 
 * \return 
 */
/****************************************************************/   
void hri_bt_reset_bitmap_data(hri_bitmap* B)
{
  int i,j,k;
  
  if(B == NULL)
    return;
  
  for(i=0; i<B->nx; i++)
    for(j=0; j<B->ny; j++)
      for(k=0; k<B->nz; k++)
	B->data[i][j][k].val = 0;
  
}

/* REVISION */
/****************************************************************/
/*!
 * \brief to link the goal position values of the form
 * 
 * \param bitmap  goal position
 * \param type    position type
 * 
 * \return 
 */
/****************************************************************/   
void hri_bt_init_TARGET(int place, int type)
{
  if(place!=0 && type!=0){
    PLACEMENT = place;
    PLCMT_TYPE = type;
  } 
}

/* REVISION */
/****************************************************************/
/*!
 * \brief Gives the configuration of the goal position
 * 
 * 
 * \return the configuration
 */
/****************************************************************/   
configPt hri_bt_set_TARGET()
{
  configPt q,q_f;
  int cost=0, places=0, min=0, mincost=9999;
  /*  int MAXX = BTSET->human[0]->joints[1]->dof_data[0].vmax , MAXY = BTSET->human[0]->joints[1]->dof_data[1].vmax; */
  double orient =   BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->dof_data->v +
    BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[5].v; 
  
  if(PLACEMENT==0 || PLCMT_TYPE==0) 
    return NULL; 
  
  q = p3d_copy_config(BTSET->robot, BTSET->robot->ROBOT_POS);   
  if(PLCMT_TYPE == BT_TRG_BODY){  
    /*if(BTSET->human[0]->joints[1]->dof_data[0].v-1>0 && ){ */
		
    switch(PLACEMENT){ 
      /*     case 1: */
      /*       if(BTSET->human[0]->joints[1]->dof_data[0].v-1000>0){ */
      /* 	q[6]=BTSET->human[0]->joints[1]->dof_data[0].v-1000; */
      /* 	q[7] = BTSET->human[0]->joints[1]->dof_data[1].v; */
      /*       } */
      /*       break; */
      /*     case 2: */
      /*       if(BTSET->human[0]->joints[1]->dof_data[0].v-1000>0 && BTSET->human[0]->joints[1]->dof_data[1].v-1000>0){ */
      /* 	q[6] = BTSET->human[0]->joints[1]->dof_data[0].v-1000; */
      /* 	q[7] = BTSET->human[0]->joints[1]->dof_data[1].v+1000; */
      /*       } */
      /*       break;  */
    case 3: 
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-1*sin(orient); 
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+1*cos(orient); 
      break; 
    case 4: 
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+0.7*cos(orient)-0.7*sin(orient); 
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+0.7*sin(orient)+0.7*cos(orient); 
      break;  
    case 5: 
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+1*cos(orient); 
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+1*sin(orient); 
      break;  
    case 6: 
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+0.7*cos(orient)+0.7*sin(orient);
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+0.7*sin(orient)-0.7*cos(orient); 
      break; 
    case 7: 
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+1*sin(orient); 
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-1*cos(orient); 
      break;  
      /*     case 8: */
      /*       if(BTSET->human[0]->joints[1]->dof_data[0].v-1000>0 && BTSET->human[0]->joints[1]->dof_data[1].v-1000>0){ */
      /* 	q[6] = BTSET->human[0]->joints[1]->dof_data[0].v-1000; */
      /* 	q[7] = BTSET->human[0]->joints[1]->dof_data[1].v-1000; */
      /*       } */
      /*       break;  */
    } 
  } 
  if(PLCMT_TYPE == BT_TRG_LOOK){ 
    q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-1*cos(orient); 
    q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-1*sin(orient); 
  } 
  if(PLCMT_TYPE == BT_TRG_APPROACH){
    q_f = p3d_copy_config(BTSET->robot, q);
    while(places < 5){
      if(places == 0){
	q_f[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-1*sin(orient); 
	q_f[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+1*cos(orient); 
      }
      if(places == 1){
	q_f[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+0.7*cos(orient)-0.7*sin(orient); 
	q_f[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+0.7*sin(orient)+0.7*cos(orient); 
      } 
      if(places == 2){
	q_f[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+1*cos(orient); 
	q_f[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+1*sin(orient); 
      } 
      if(places == 3){
	q_f[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+0.7*cos(orient)+0.7*sin(orient);
	q_f[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+0.7*sin(orient)-0.7*cos(orient); 
      } 
      if(places == 4){
	q_f[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+1*sin(orient); 
	q_f[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-1*cos(orient); 
      }
      hri_bt_reset_path(BTSET);
      cost = hri_bt_start_search(q,q_f,BTSET,FALSE);
      if(cost!=-1 && cost<mincost){
	mincost = cost;
	min = places;
      }
      places++;
    }  
		
    if(min == 0){
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-1*sin(orient); 
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+1*cos(orient); 
    }
    if(min == 1){
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+0.7*cos(orient)-0.7*sin(orient); 
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+0.7*sin(orient)+0.7*cos(orient); 
    } 
    if(min == 2){
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+1*cos(orient); 
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+1*sin(orient); 
    } 
    if(min == 3){
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+0.7*cos(orient)+0.7*sin(orient);
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+0.7*sin(orient)-0.7*cos(orient); 
    } 
    if(min == 4){
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+1*sin(orient); 
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-1*cos(orient); 
    } 
		
    q[ROBOTq_RZ] = atan2((BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-q[ROBOTq_Y]),
			 (BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-q[ROBOTq_X]));
    if(q[ROBOTq_RZ]<0)
      q[ROBOTq_RZ] = q[ROBOTq_RZ]+M_PI;
    else
      q[ROBOTq_RZ] = q[ROBOTq_RZ]-M_PI;
    /*q[12] = M_PI;
      q[18] = 0; Only for blue humanoid robot */
		
  }
  /*printf("Target is %d and orientation is base: %f top:%f\n",min,q[11]*180/M_PI,q[12]*180/M_PI);*/
	
  BTSET->BT_target[0] = q[ROBOTq_X];
  BTSET->BT_target[1] = q[ROBOTq_Y];
  BTSET->BT_target_available = TRUE;
  return q;
	
}

/****************************************************************/
/*!
 * \brief Calculate hidden zones cost of the given coordinate
 * 
 * \param x x coordinate
 * \param y y coordinate
 * \param z  coordinate
 * 
 * \return the cost
 */
/****************************************************************/
double hri_bt_calc_hz_value(hri_bitmapset * btset,int x, int y, int z)
{
  double humanx, humany;
  int hx, hy;
  configPt qhuman, qtarget;
  double dist;
  p3d_localpath *path=NULL;
  double treshhold;
  double v,val,res=-3;

  int i,ntest;
  
  for(i=0; i<btset->human_no; i++){  
    if(!btset->human[i]->exists)
      continue;
    treshhold = btset->human[i]->state[BTSET->human[i]->actual_state].hradius;
    hx = (btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-btset->realx)/btset->pace;
    hy = (btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-btset->realy)/btset->pace;
    
    dist = DISTANCE2D(hx,hy,x,y) * btset->pace;
    
    if(dist>treshhold){
      val = 0;
      continue;
    }
    
    humanx = btset->human[i]->HumanPt->joints[1]->dof_data[0].v;
    humany = btset->human[i]->HumanPt->joints[1]->dof_data[1].v;
    
    qhuman = p3d_copy_config(btset->visball, btset->visball->ROBOT_POS);  
    qhuman[6] = humanx;
    qhuman[7] = humany;
    qhuman[8] = btset->human[i]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[2][3];
		
    qtarget = p3d_copy_config(btset->visball,qhuman);
    qtarget[6] = btset->pace*x+btset->realx;
    qtarget[7] = btset->pace*y+btset->realy;
    
    p3d_col_deactivate_rob_rob(btset->visball,btset->human[i]->HumanPt);
    if(is_in_fow(humanx, humany, qtarget[6], qtarget[7],
		 btset->human[i]->HumanPt->joints[HUMANj_NECK_PAN]->dof_data->v+
		 btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[5].v, 80)){
      v = p3d_get_env_dmax();
      p3d_set_env_dmax(0);
      if( (path = p3d_local_planner(btset->visball, qtarget, qhuman)) ){ 
	if(p3d_unvalid_localpath_test(btset->visball, path, &ntest)){
	  p3d_set_env_dmax(v);
	  destroy_list_localpath(btset->visball, path);
	  p3d_destroy_config(btset->visball, qhuman);
	  p3d_destroy_config(btset->visball, qtarget);
	  val = (double)( (((double)-1000)/treshhold)*dist+1000);
	}	
	else{
	  p3d_set_env_dmax(v);
	  destroy_list_localpath(btset->visball, path);
	  p3d_destroy_config(btset->visball, qhuman);
	  p3d_destroy_config(btset->visball, qtarget);
	  val = -1;
	}
      }
      else{
	p3d_set_env_dmax(v);
	destroy_list_localpath(btset->visball, path);
	p3d_destroy_config(btset->visball, qhuman);
	p3d_destroy_config(btset->visball, qtarget);
	val= -1;
      }
    }  
    else{
      p3d_destroy_config(btset->visball, qhuman);
      p3d_destroy_config(btset->visball, qtarget);
      val= -2;
    }
    if(res<val){
      res = val;
    }       
  }
  return res;
}  

/****************************************************************/
/*!
 * \brief Test if the point is in the field of view of the man
 * 
 * \param xh x coordinate of human
 * \param yh y coordinate of human
 * \param xt x coordinate of the point
 * \param yt y coordinate of the point
 * \param orient humans looking direction
 * \param fowangle humans fow angle
 * 
 * \return the cost
 */
/****************************************************************/
static int is_in_fow(double xh, double yh, double xt, double yt, double orient, double fowangle)
{
  double angle2human = atan2(yt-yh, xt-xh);
	
  if( abs(angle2human-orient)<fowangle )
    return TRUE;
  else{
    if( (angle2human>M_PI_2) && (M_PI>angle2human) && (orient<-M_PI_2) && (orient>-M_PI-EPS5))
      if(orient+M_2PI-angle2human<fowangle )
	return TRUE;
    if( (angle2human<-M_PI_2) && (-M_PI-EPS5<angle2human) && (orient>M_PI_2) && (orient<M_PI+EPS5))
      if(angle2human-orient+M_2PI<fowangle )
	return TRUE;
    
  }  
  return FALSE;
  
}

/****************************************************************/
/*!
 * \brief Calculate safety cost of the given coordinate
 * 
 * \param x x coordinate
 * \param y y coordinate
 * 
 * \return the cost
 */
/****************************************************************/    
double hri_bt_calc_dist_value(hri_bitmapset * btset, int x, int y, int z)
{
  int hx,hy,i;
  double radius,height;
  double val,res =0;
  
  if(btset==NULL){
    PrintError(("btset is null, cant get distance value\n"));
    return -1;
  }
  else{
    for(i=0; i<btset->human_no; i++){ 
      if(!btset->human[i]->exists)
	continue;
      height = btset->human[i]->state[btset->human[i]->actual_state].dheight;
      radius = btset->human[i]->state[btset->human[i]->actual_state].dradius;
      
      hx = (int)((btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-btset->realx)/btset->pace);
      hy = (int)((btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-btset->realy)/btset->pace);
      if(radius*(y-hy)<180 && radius*(y-hy)>-180 && radius*(x-hx)<180 && radius*(x-hx)>-180)	
	val =  height/2 * (cos((double)(DTOR(radius*(x-hx))))+1)*(cos((double)(DTOR(radius*(y-hy))))+1);
      else 
	val = 0;
      /* if(radius*3*(y-hy)<180 && radius*3*(y-hy)>-180 && radius*3*(x-hx-10)<180 && radius*3*(x-hx-10)>-180) */
      /* 	res =  height/2 * (cos((double)(radius*3*(x-hx-10)*3.14/180))+1)*(cos((double)(radius*3*(y-hy)*3.14/180))+1);  */
      if(res<val){
	res = val;
      }      
    }
  }
  return res;
  /*  if(radius*(y-hy)<180 && radius*(y-hy)>-180 && radius*(x-hx)<180 && radius*(x-hx)>-180)	 */
  /*     return  height * (cos((double)(radius*(x-hx)*3.14/180))+1)*(cos((double)(radius*(y-hy)*3.14/180))+1); */
  /*   else  */
  /*     return 0; */ 
}




/****************************************************************/
/*!
 * \brief Calculate velocity cost of the given coordinate
 * 
 * \param x x coordinate
 * \param y y coordinate
 * 
 * \return the cost
 */
/****************************************************************/    
double hri_bt_calc_vel_value(hri_bitmapset * btset,int x, int y, int z)
{
	
	
	
  return 0;
}



/****************************************************************/
/*!
 * \brief Calculate visibility cost of the given coordinate
 * 
 * \param x x coordinate
 * \param y y coordinate
 * 
 * \return the cost
 */
/****************************************************************/
double hri_bt_calc_vis_value(hri_bitmapset * btset,int x, int y, int z)
{
  int xp,yp,hx,hy,i;
  double angle,angle0,deltax,deltay,distance,orient;
  double p1,p2,p3;
  double val=0,res=0;
  
  if(btset==NULL){
    PrintError(("btset is null, cant get visibility value\n"));
    return -1;
  }
	
  for(i=0; i<btset->human_no; i++){  
    if(!btset->human[i]->exists)
      continue;
    p1 = btset->human[i]->state[btset->human[i]->actual_state].vheight;
    p2 = btset->human[i]->state[btset->human[i]->actual_state].vback;
    p3 = btset->human[i]->state[btset->human[i]->actual_state].vsides;
    
    hx = (int)((btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-btset->realx)/btset->pace);
    hy = (int)((btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-btset->realy)/btset->pace);
    orient = btset->human[i]->HumanPt->joints[HUMANj_NECK_PAN]->dof_data->v + btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[5].v;
    
    deltax = x-hx;
    deltay = y-hy;
    
    if(deltax>0)
      angle = atan(deltay/deltax);
    else
      if(deltax<0)
	angle = M_PI+atan(deltay/deltax);
      else
	if(deltay>0)
	  angle = M_PI_2;
	else 
	  angle = M_PI*3/2;
    
    angle0 = angle - orient;
    
    distance = sqrt(deltax*deltax+deltay*deltay);
    
    xp = (int)(hx + distance * cos(angle0));
    yp = (int)(hy + distance * sin(angle0));
		
    /* if(xp<0 || yp<0 || xp>bitmap->nx-1 || yp>bitmap->ny-1){ */
    /*     printf("%d %d \n",xp,yp); */
    /*     PrintError(("cant calc vis value:unvalid coordinate")); */
    /*     return -1; */
    /*   } */
    if(p3*(yp-hy)<90 && p3*(yp-hy)>-90 &&  p2*(xp-hx)<90 && p2*(xp-hx)>-90){      
      if(yp==hy ){
	if(xp<hx)
	  val = p1*M_PI*cos((double)(DTOR(p2*(xp-hx))))*
	    cos((double)DTOR((p3*(yp-hy))));
	else
	  val = 0;
      }
      if(xp==hx){   	
	//val = p1*abs(3.1416/2)*10*cos((double)(p3*(y-hy)*3.14/180));
	val = p1*abs(M_PI_2)*cos((double)(DTOR(p3*(yp-hy))));
      }
      else {
	if(xp>hx && yp>hy)	 
	  //    val = 0;
	  val = p1*abs(atan2(abs(yp-hy),abs(xp-hx)))*cos((double)(DTOR(p3*(yp-hy))));
	if(xp<hx && yp>hy) 
	  val = p1*abs((M_PI-atan2(abs(yp-hy),abs(xp-hx))))*
	    cos((double)(DTOR(p2*(xp-hx))))*cos((double)(DTOR(p3*(yp-hy))));
	if(xp>hx && yp<hy)	
	  //val = 0;
	  val = p1*abs(-1*atan2(abs(yp-hy),abs(xp-hx)))*cos((double)(DTOR(p3*(yp-hy))));
	if(xp<hx && yp<hy)
	  val = p1*abs((atan2(abs(yp-hy),abs(xp-hx))-M_PI))*
	    cos((double)(DTOR(p2*(xp-hx))))*cos((double)(DTOR(p3*(yp-hy))));
      }
    }
    else    
      val = 0;
    if(res<val){
      res = val;
    }      
  }
  
  return res;
}
/****************************************************************/
/*!
 * \brief Calculate the combined cost of the given coordinate
 * 
 * \param x x coordinate
 * \param y y coordinate
 * 
 * \return the cost
 */
/****************************************************************/
double hri_bt_calc_combined_value(hri_bitmapset * btset, int x, int y, int z)
{
  double dist,vis,hz;
  int i;
  double realx,realy;
  double enlargement;
  configPt robotq;
   
  /*  double enlargement = (btset->robot->BB.xmax-btset->robot->BB.xmin > btset->robot->BB.ymax-btset->robot->BB.ymin)? */
  /*     ((btset->robot->BB.xmax-btset->robot->BB.xmin)/2): */
  /*     ((btset->robot->BB.ymax-btset->robot->BB.ymin)/2); */
  
  if(btset==NULL || btset->bitmap==NULL){
    PrintError(("Try to calculate an unexisting bitmap\n"));
    return -2;
  }  
  
  // if( btset->bitmap[BT_OBSTACLES]!= NULL &&  btset->bitmap[BT_OBSTACLES]->data != NULL)
  //   if(btset->bitmap[BT_OBSTACLES]->data[x][y][z].val < 0)
  //     return -1;      COMMENT TO CHECK OBSTACLES OUTSIDE OF THIS FUNCTION ACCORDING TO WHERE WE CAME FROM
  
  robotq = p3d_get_robot_config(btset->robot);
  
  if(DISTANCE2D(btset->robot->BB.xmax,btset->robot->BB.ymax,robotq[ROBOTq_X],robotq[ROBOTq_Y]) >
     DISTANCE2D(btset->robot->BB.xmin,btset->robot->BB.ymin,robotq[ROBOTq_X],robotq[ROBOTq_Y]))
    
    enlargement = (btset->robot->BB.xmax-robotq[ROBOTq_X] > btset->robot->BB.ymax-robotq[ROBOTq_Y])?
      ((btset->robot->BB.xmax-robotq[ROBOTq_X])):
      ((btset->robot->BB.ymax-robotq[ROBOTq_Y]));
  
  else
    enlargement = (btset->robot->BB.xmin-robotq[ROBOTq_X] > btset->robot->BB.ymin-robotq[ROBOTq_Y])?
      ((robotq[ROBOTq_X]-btset->robot->BB.xmin)):
      ((robotq[ROBOTq_Y]-btset->robot->BB.ymin));
  

  p3d_destroy_config(btset->robot,robotq);

  realx = (x*btset->pace)+btset->realx;
  realy = (y*btset->pace)+btset->realy;
  
  for(i=0; i<btset->human_no; i++)
    if(btset->human[i]->exists)
      if(realx > btset->human[i]->HumanPt->o[1]->BB.xmin-enlargement &&
	 realx < btset->human[i]->HumanPt->o[1]->BB.xmax+enlargement &&
	 realy > btset->human[i]->HumanPt->o[1]->BB.ymin-enlargement &&
	 realy < btset->human[i]->HumanPt->o[1]->BB.ymax+enlargement )
	return -2;
  
  
  /* res = p3d_col_test_robot(btset->robot,TRUE); */
  
  /*   if(res){ */
  /*     printf("Robot in colision!!!\n"); */
  /*     return 0; */
  /*   } */
  
  hz =  btset->bitmap[BT_HIDZONES]->calculate_cell_value(btset,x,y,z);
  // hri_bt_calc_hz_value(x,y,z,0,NULL);
  if(hz > -1) 
    return hz;
  vis  =  btset->bitmap[BT_VISIBILITY]->calculate_cell_value(btset,x,y,z); 
  // hri_bt_calc_vis_value(x,y,z,0,NULL);
  dist =  btset->bitmap[BT_DISTANCE]->calculate_cell_value(btset,x,y,z);
  // dist = hri_bt_calc_dist_VALUE(x,y,z,0,NULL);
  //printf("Values: %f %f\n",dist,vis);
  if(btset->combine_type == BT_COMBINE_SUM)  return dist + vis;
  if(btset->combine_type == BT_COMBINE_MAX){
    if(dist>vis) return dist;
    else         return vis;
  }
  
  PrintError(("Can't combine bitmaps\n"));
  return 0;
	
}

/**************************ASTAR********************************/
/*!
 * \brief A* search: close the cell 
 * 
 * \param bitmap       the bitmap
 * \param current_cell the cell to be closed
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_close_cell(hri_bitmap* bitmap, hri_bitmap_cell* current_cell)
{
  if(current_cell == NULL)
    return FALSE;
  if( (bitmap->nx-1<current_cell->x) || (bitmap->ny-1<current_cell->y) || (bitmap->nz-1<current_cell->z) ||
      (0>current_cell->x) || (0>current_cell->y) || (0>current_cell->z))
    return FALSE;
  
  bitmap->data[current_cell->x][current_cell->y][current_cell->z].closed = TRUE;
  return TRUE;
}

/***************************ASTAR********************************/
/*!
 * \brief Starts A* search to find a path on bitmap 
 * 
 * \param bitmap the bitmap
 * \param start_cell start cell
 * \param final_cell goal cell
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
hri_bitmap_cell ** OPENLIST;
int OL_cellnbr;

double hri_bt_astar_bh(hri_bitmapset * btset, hri_bitmap* bitmap)
{
	
  hri_bitmap_cell * current_cell;;                 
  int reached = FALSE;  
	
  if(bitmap->search_start == NULL || bitmap->search_goal == NULL){
    PrintError(("hri_bt_astar_bh: start/final cell is NULL\n"));
    return -1;
  }
  current_cell = bitmap->search_start;
	
  OPENLIST = MY_ALLOC(hri_bitmap_cell*,bitmap->nx * bitmap->ny * bitmap->nz); /* first element of this array is not used */
  OL_cellnbr = 0;
	
  if(!hri_bt_close_cell(bitmap,current_cell)){
    PrintError(("cant close start cell!\n"));
    return -1;
  }
  hri_bt_A_neigh_costs(btset,bitmap,current_cell,bitmap->search_goal,&reached);
	
  while(!reached) {
    current_cell = hri_bt_A_remove_OL();    
    hri_bt_close_cell(bitmap,current_cell); 
    hri_bt_A_neigh_costs(btset,bitmap,current_cell,bitmap->search_goal,&reached);
    if( OL_cellnbr==0 && !reached ){
      PrintError(("A*:no path found!"));
      MY_FREE(OPENLIST, hri_bitmap_cell*, bitmap->nx*bitmap->ny*bitmap->nz);
      return -1;
    }
  } 
  bitmap->searched = TRUE;
	
  printf("\ncost: %f \n",bitmap->search_goal->g + bitmap->search_goal->h);
  bitmap->type = BT_PATH;
  MY_FREE(OPENLIST, hri_bitmap_cell*, bitmap->nx*bitmap->ny*bitmap->nz);
	
  return bitmap->search_goal->g + bitmap->search_goal->h;
}

/***************************ASTAR********************************/
/*!
 * \brief A* insert to the OPENLIST, using binary heaps
 * 
 * \param cell  cell to insert
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_A_insert_OL(hri_bitmap_cell *cell)
{
  int index;
  hri_bitmap_cell* temp;
	
  if(OL_cellnbr == 0){
    OPENLIST[1] = cell;
    OL_cellnbr++;
    return TRUE;
  }
	
  OPENLIST[OL_cellnbr+1] = cell;
  OL_cellnbr++;
  index = OL_cellnbr;
	
  while(index != 1){
    if( Cellcost(OPENLIST[index]) <= Cellcost(OPENLIST[index/2]) ){
      temp = OPENLIST[index/2];
      OPENLIST[index/2] = OPENLIST[index];
      OPENLIST[index] = temp;
      index = index/2;
    }
    else
      break;
  } 
  return TRUE;
}

/***************************ASTAR********************************/
/*!
 * \brief A* remove from the OPENLIST, using binary heaps
 * 
 * 
 * \return gives the removed cell
 */
/****************************************************************/  
hri_bitmap_cell* hri_bt_A_remove_OL()
{
  int parent, child;
  hri_bitmap_cell* temp, * result;
  
  if(OL_cellnbr == 0)
    return NULL;
  
  result = OPENLIST[1];
  
  OPENLIST[1] = OPENLIST[OL_cellnbr];
  OPENLIST[OL_cellnbr] = NULL;
  OL_cellnbr--;
  child = 1;
  
  while(TRUE){
    parent = child;
    if( 2*parent+1 <= OL_cellnbr ){
      if( Cellcost(OPENLIST[parent]) >= Cellcost(OPENLIST[2*parent])  ) child = 2*parent;
      if( Cellcost(OPENLIST[child]) >= Cellcost(OPENLIST[2*parent+1]) ) child = 2*parent+1;
    }
    else{
      if( 2*parent <= OL_cellnbr ){
	if( Cellcost(OPENLIST[parent]) >= Cellcost(OPENLIST[2*parent]) ) child = 2*parent;
      }
    }
    if(parent!=child){
      temp = OPENLIST[parent];
      OPENLIST[parent] = OPENLIST[child];
      OPENLIST[child] = temp;
    }
    else
      break;
  }
  return result;
  
}

/***************************ASTAR********************************/
/*!
 * \brief A* Update the OPENLIST, using binary heaps
 * 
 * \param cell  cell to update
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/  
int hri_bt_A_update_cell_OL(hri_bitmap_cell *cell)
{
  int i;
  hri_bitmap_cell *temp;
  
  if(!cell->open){
    PrintWarning(("A*:Accessing a nonexistent cell in OL"));
    return FALSE;
  }
  
  for(i=1; i<OL_cellnbr+1; i++){
    if(OPENLIST[i] == cell)
      break;
  }
  
  if(i==OL_cellnbr+1)
    return FALSE;
  
  while(i != 1){
    if( Cellcost(OPENLIST[i]) <= Cellcost(OPENLIST[i/2]) ){
      temp = OPENLIST[i];
      OPENLIST[i] = OPENLIST[i/2];
      OPENLIST[i/2] = temp;
      i = i/2;
    }
    else
      break;
  } 
  return TRUE;
  
} 

/*********************ASTAR***************************************/
/*!
 * \brief A* search: calculate neighbours
 * 
 * \param bitmap the bitmap
 * \param center_cell whose neighbours
 * \param final_cell  goal cell
 * \param reached TRUE if we reach to the goal
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int  hri_bt_A_neigh_costs(hri_bitmapset* btset, hri_bitmap* bitmap, hri_bitmap_cell* center_cell, hri_bitmap_cell* final_cell, int* reached)
{
  int i,j,k;
  int x, y,z;
  int xdiff,ydiff,fromcellno;
  hri_bitmap_cell* current_cell;
  double pas3diagonal = sqrt(3),pas2diagonal=sqrt(2), pasnormal=1;
  if(center_cell == NULL)
    return FALSE;
  else{
    x=center_cell->x; y=center_cell->y; z=center_cell->z;
  }
  
  if( (bitmap->nx-1<center_cell->x) || (bitmap->ny-1<center_cell->y) || (bitmap->nz-1<center_cell->z) ||
      (0>center_cell->x) ||  (0>center_cell->y)  ||  (0>center_cell->z)){
    PrintError(("cant get cell\n"));
    return FALSE;
  }
  
  for(i=-1; i<2; i++){
    for(j=-1; j<2; j++){
      for(k=-1; k<2; k++){
	if(i==0 && j==0 && k==0) continue;
	if(x==0 && i==-1) continue;
	if(y==0 && j==-1) continue;
	if(z==0 && k==-1) continue;
	if(x==bitmap->nx-1 && i==1) continue;
	if(y==bitmap->ny-1 && j==1) continue;
	if(z==bitmap->nz-1 && k==1) continue;
	
	if( !(current_cell = hri_bt_get_cell(bitmap,x+i,y+j,z+k)) ){
	  PrintError(("Can't get cell\n"));
	  return FALSE;
	}      
	
	if(btset->bitmap[BT_OBSTACLES]->data[x+i][y+j][z+k].val == -2) continue; /* Is the cell in obstacle? */
	
	if(current_cell->closed) continue; /* is it already closed? */
	
	/* if(current_cell->open){  is it in open list?   */
	/* 	  if( ((i+j+k)!=-1) && ((i+j+k)!=1) ){ */
	/* 	    if(current_cell->g > center_cell->g + current_cell->val + pasdiagonal){ */
	/* 	      current_cell->g =  center_cell->g + current_cell->val + pasdiagonal; */
	/* 	      current_cell->parent = center_cell; */
	/* 	      hri_bt_A_update_cell_OL(current_cell); */
	/* 	    } */
	/* 	    else */
	/* 	      continue; */
	/* 	  } */
	/* 	  else { */
	/* 	    if(current_cell->g > center_cell->g + current_cell->val + pasnormal){ */
	/* 	      current_cell->g =  center_cell->g + current_cell->val + pasnormal; */
	/* 	      current_cell->parent = center_cell;	   */
	/* 	      hri_bt_A_update_cell_OL(current_cell); */
	/* 	    } */
	/* 	    else */
	/* 	      continue; */
	/* 	  }	 */
	/* 	} */
	if(current_cell->open){  /* is it in open list? */ 
	  if(ABS(i)+ABS(j)+ABS(k)==1){
	    if(current_cell->g > center_cell->g + current_cell->val + pasnormal){
	      current_cell->g =  center_cell->g + current_cell->val + pasnormal;
	      current_cell->parent = center_cell;
	      hri_bt_A_update_cell_OL(current_cell);
	    }
	    else
	      continue;
	  }
	  if(ABS(i)+ABS(j)+ABS(k)==2){
	    if(current_cell->g > center_cell->g + current_cell->val + pas2diagonal){
	      current_cell->g =  center_cell->g + current_cell->val + pas2diagonal;
	      current_cell->parent = center_cell;
	      hri_bt_A_update_cell_OL(current_cell);
	    }
	    else
	      continue;
	  }
	  if(ABS(i)+ABS(j)+ABS(k)==3){
	    if(current_cell->g > center_cell->g + current_cell->val + pas3diagonal){
	      current_cell->g =  center_cell->g + current_cell->val + pas3diagonal;
	      current_cell->parent = center_cell;
	      hri_bt_A_update_cell_OL(current_cell);
	    }
	    else
	      continue;
	  }
	}
	else{	
	  if(!CalculateCellValue(btset,bitmap,current_cell,center_cell)) continue;
	  current_cell->h = hri_bt_dist_heuristic(bitmap,current_cell->x,current_cell->y,current_cell->z); 
	  if(btset->bitmap[BT_OBSTACLES]->data[current_cell->x][current_cell->y][current_cell->z].val == -1 && !btset->manip){
	    xdiff = current_cell->x - center_cell->x;
	    ydiff = current_cell->y - center_cell->y;
	    if(xdiff==-1 && ydiff==-1) fromcellno = 0;
	    if(xdiff==-1 && ydiff== 0) fromcellno = 1;
	    if(xdiff==-1 && ydiff== 1) fromcellno = 2;
	    if(xdiff== 0 && ydiff==-1) fromcellno = 3;
	    if(xdiff== 0 && ydiff== 1) fromcellno = 4;
	    if(xdiff== 1 && ydiff==-1) fromcellno = 5;
	    if(xdiff== 1 && ydiff== 0) fromcellno = 6;
	    if(xdiff== 1 && ydiff== 1) fromcellno = 7;
	    if(btset->bitmap[BT_OBSTACLES]->data[current_cell->x][current_cell->y][current_cell->z].obstacle[fromcellno]==TRUE) // it was !=, PRAGUE
	      continue;
	    current_cell->g = center_cell->g +  current_cell->val ; //current_cell->obstacle[fromcellno];
	  }
	  else{
	    current_cell->g = center_cell->g + current_cell->val;
	  }
	  /* printf("It is g=%f val=%f\n",current_cell->g,current_cell->val); */
	  /*   if( (i+j+k)!=-1 && (i+j+k)!=1 ) */
	  /* 	     current_cell->g += pasdiagonal; */
	  /* 	   else */
	  /* 	     current_cell->g += pasnormal; */
	  
	  if(ABS(i)+ABS(j)+ABS(k)==1)
	    current_cell->g += pasnormal;
	  if(ABS(i)+ABS(j)+ABS(k)==2)
	    current_cell->g += pas2diagonal;
	  if(ABS(i)+ABS(j)+ABS(k)==3)
	    current_cell->g += pas3diagonal;
	  
	  current_cell->parent = center_cell;
	  if(current_cell == final_cell){
	    *reached = TRUE;
	    return TRUE;
	  }
	  /*  printf("It is g=%f now\n",current_cell->g); */
	  hri_bt_A_insert_OL(current_cell);	
	  current_cell->open = TRUE;
	}        
      }
    }
  }
  return TRUE;
}

/*********************ASTAR**************************************/
/*!
 * \brief A* Gives the search cost of a cell
 * 
 * \param cell the cell
 * 
 * \return the cost
 */
/****************************************************************/
static double Cellcost(hri_bitmap_cell* cell)
{
  return (cell->h + cell->g);
}

/*********************ASTAR**************************************/
/*!
 * \brief Calculate the cost of a cell
 * 
 * \param cell the cell
 * 
 * \return FALSE in case of a collision
 */
/****************************************************************/  
static int CalculateCellValue(hri_bitmapset * btset, hri_bitmap * bitmap,  hri_bitmap_cell* cell, hri_bitmap_cell* fromcell )
{
  int xdiff, ydiff, fromcellno;
  configPt qc,q_o;
  double saved[3];
	
  qc = p3d_get_robot_config(btset->robot); /* ALLOC */
  
  if(btset->manip == 2){
    q_o = p3d_get_robot_config(btset->object);  
    saved[0] = q_o[6]; saved[1] = q_o[7]; saved[2] = q_o[8];
    
    q_o[6] = cell->x*btset->pace+btset->realx;
    q_o[7] = cell->y*btset->pace+btset->realy;
    q_o[8] = cell->z*btset->pace+btset->realz;
     
    p3d_set_and_update_this_robot_conf(btset->object,q_o);
    
    if(!hri_compute_R6IK(btset->robot,btset->object,qc)){
      btset->bitmap[BT_3D_OBSTACLES]->data[cell->x][cell->y][cell->z].val = -2;
      cell->val = -2;
      p3d_destroy_config(btset->robot, qc);
    }
    else{
      cell->val = bitmap->calculate_cell_value(btset,cell->x,cell->y,cell->z);
      cell->q = qc;
    }
    
    q_o[6] = saved[0];  q_o[7] = saved[1];  q_o[8] = saved[2];
    p3d_set_and_update_this_robot_conf(btset->object,q_o);
    p3d_destroy_config(btset->object, q_o);
    
    if(cell->val < 0)
      return FALSE;
    else
      return TRUE;
    
  }
  
  if(btset->bitmap[BT_OBSTACLES]->data[cell->x][cell->y][cell->z].val == -1 && !btset->manip){ /* soft obstacles */
    qc[6]  = cell->x*btset->pace+btset->realx;
    qc[7]  = cell->y*btset->pace+btset->realy;
    qc[11] = atan2(cell->y-fromcell->y,cell->x-fromcell->x);
    
    p3d_set_and_update_this_robot_conf(btset->robot, qc); 
    p3d_destroy_config(btset->robot, qc); /*  FREE */ 
    if(!p3d_col_test_robot_statics(btset->robot,FALSE)){
      xdiff = fromcell->x - cell->x;
      ydiff = fromcell->y - cell->y;

      if(xdiff==-1 && ydiff==-1) fromcellno = 0;
      if(xdiff==-1 && ydiff== 0) fromcellno = 1;
      if(xdiff==-1 && ydiff== 1) fromcellno = 2;
      if(xdiff== 0 && ydiff==-1) fromcellno = 3;
      if(xdiff== 0 && ydiff== 1) fromcellno = 4;
      if(xdiff== 1 && ydiff==-1) fromcellno = 5;
      if(xdiff== 1 && ydiff== 0) fromcellno = 6;
      if(xdiff== 1 && ydiff== 1) fromcellno = 7;
      
      btset->bitmap[BT_OBSTACLES]->data[cell->x][cell->y][cell->z].obstacle[fromcellno] = TRUE; /* collision when u move from fromcell to cell */
      cell->obstacle[fromcellno] = bitmap->calculate_cell_value(btset,cell->x,cell->y,cell->z); 
      
      return TRUE;
    }
    else{
      return FALSE;
    }
  } 
  
  cell->val = bitmap->calculate_cell_value(btset,cell->x,cell->y,cell->z);
  if(cell->val<0)
    return FALSE;
  
  return TRUE;
  
}  

/****************************************************************/
/*!
 * \brief reconstructs the distance bitmap
 * 
 * \param radius radius
 * \param heigth height
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_update_distance(hri_bitmapset * btset, double height, double radius)
{
  hri_bitmap* bitmap;
  int i,j,k;
  
  if(btset==NULL)
    return FALSE;
  else
    bitmap = btset->bitmap[BT_DISTANCE];  
  
  btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].dheight = height;
  btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].dradius = radius;
  if(!bitmap->active){
    return TRUE;
  }
  for(i=0;i<bitmap->nx;i++){
    for(j=0;j<bitmap->ny;j++){    
      for(k=0;k<bitmap->nz;k++)   
	btset->bitmap[BT_DISTANCE]->data[i][j][k].val = btset->bitmap[BT_DISTANCE]->calculate_cell_value(btset,i,j,k);
      //hri_bt_calc_dist_VALUE(i,j,k,0,NULL);
    }
  } 
  
  return TRUE;
}

/****************************************************************/
/*!
 * \brief reconstructs the visibility bitmap
 * 
 * \param height height
 * \param p2     back radius
 * \param p3     sides radius
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_update_visibility(hri_bitmapset * btset,double height, double p2, double p3)
{
  hri_bitmap* bitmap;
  int i,j,k;
	
  if(btset==NULL)
    return FALSE;
  else
    bitmap = btset->bitmap[BT_VISIBILITY];
	
  btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].vheight = height;
  btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].vback = p2;
  btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].vsides = p3;
  
  if(!bitmap->active){
    return TRUE;
  }
  hri_bt_reset_bitmap_data(btset->bitmap[BT_VISIBILITY]);
  for(i=0; i<bitmap->nx; i++){
    for(j=0; j<bitmap->ny; j++){
      for(k=0; k<bitmap->nz; k++)
	btset->bitmap[BT_VISIBILITY]->data[i][j][k].val = hri_bt_calc_vis_value(btset,i,j,k);
    }  
  }
  
  return TRUE;  
}

/****************************************************************/
/*!
 * \brief reconstructs the hidden zones bitmap
 * 
 * \param radius vision radius
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_update_hidzones(hri_bitmapset * btset,double radius)
{
  hri_bitmap* bitmap;
  int i,j,k;
  double val;
  
  if(btset==NULL)
    return FALSE;
  else
    bitmap = btset->bitmap[BT_HIDZONES];
	
  /*  bitmap->p4 = radius; */
	
  btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].hradius = radius;
  
  if(!bitmap->active){
    return TRUE;
  }
  
  for(i=0; i<bitmap->nx; i++){
    for(j=0; j<bitmap->ny; j++){
      for(k=0; k<bitmap->nz; k++){ 
	val = hri_bt_calc_hz_value(btset,i,j,k);
	if(val>0)
	  btset->bitmap[BT_HIDZONES]->data[i][j][k].val = val;
	else
	  btset->bitmap[BT_HIDZONES]->data[i][j][k].val = 0;
      }  
    }
  }
	
  return TRUE;
}

/****************************************************************/
/*!
 * \brief reconstructs the distance bitmap
 * 
 * \param radius radius
 * \param heigth height
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_update_combined(hri_bitmapset * btset)
{
  hri_bitmap* bitmap;
  int i,j,k;
  
  if(btset==NULL)
    return FALSE;
  else
    bitmap = btset->bitmap[BT_COMBINED];  
  
  if(!bitmap->active){
    return TRUE;
  }
  for(i=0;i<bitmap->nx;i++){
    for(j=0;j<bitmap->ny;j++){   
      for(k=0;j<bitmap->nz;k++){   
	if(btset->bitmap[BT_OBSTACLES]->data[i][j][k].val < 0)
	  btset->bitmap[BT_COMBINED]->data[i][j][k].val = -1;
	else
	  btset->bitmap[BT_COMBINED]->data[i][j][k].val = hri_bt_calc_combined_value(btset,i,j,k);
      }
    } 
  } 
  return TRUE;
}

int hri_bt_sit_stand(p3d_rob* human)
{
  return 0;
}


int hri_bt_write_TRAJ(hri_bitmapset * btset, p3d_jnt * joint)
{
  hri_bitmap * bitmap;
  int i=0;
  int length =0;
  
  if( (btset == NULL) || (btset->bitmap[BT_PATH] == NULL) || (!btset->pathexist) ){
    PrintError(("Cant write path structure\n"));
    return FALSE;
  }
  bitmap = btset->bitmap[BT_PATH];
  
  printf("\n*****Creating the path structure*****\n"); 
  
  btset->path = MY_ALLOC(hri_bt_path,1);

  bitmap->current_search_node = bitmap->search_goal;
  while(bitmap->current_search_node != NULL){
    length++;
    bitmap->current_search_node = bitmap->current_search_node->parent;
  }
  length = length+2; /* ADDINDG STAR AND GOAL CONFIGS */
  
  btset->path->xcoord = MY_ALLOC(double,length);
  btset->path->ycoord = MY_ALLOC(double,length);
  btset->path->zcoord = MY_ALLOC(double,length);
  btset->path->theta  = MY_ALLOC(double,length);
  btset->path->length = length;
  
  btset->path->xcoord[length-1] = btset->robot->ROBOT_GOTO[ROBOTq_X];
  btset->path->ycoord[length-1] = btset->robot->ROBOT_GOTO[ROBOTq_Y];
  btset->path->zcoord[length-1] = btset->robot->ROBOT_GOTO[ROBOTq_Z];
  btset->path->theta[length-1]  = btset->robot->ROBOT_GOTO[ROBOTq_RZ];
  
  bitmap->current_search_node = bitmap->search_goal;
  i = length-2;
  while(bitmap->current_search_node->parent->parent != NULL){
    btset->path->xcoord[i] = (bitmap->current_search_node->x*btset->pace)+btset->realx;
    btset->path->ycoord[i] = (bitmap->current_search_node->y*btset->pace)+btset->realy;
    btset->path->zcoord[i] = (bitmap->current_search_node->z*btset->pace)+btset->realz;
    //  btset->path->theta[i]  = btset->path->theta[i+1];
   
    btset->path->theta[i]  = (atan2(btset->path->ycoord[i-1]-btset->path->ycoord[i-2],btset->path->xcoord[i-1]-btset->path->xcoord[i-2])+
			      atan2(btset->path->ycoord[i]-btset->path->ycoord[i-1],btset->path->xcoord[i]-btset->path->xcoord[i]))/2 ;
    i--;
    bitmap->current_search_node = bitmap->current_search_node->parent;
  }

  btset->path->xcoord[i] = (bitmap->current_search_node->x*btset->pace)+btset->realx;
  btset->path->ycoord[i] = (bitmap->current_search_node->y*btset->pace)+btset->realy;
  btset->path->zcoord[i] = (bitmap->current_search_node->z*btset->pace)+btset->realz;
  btset->path->theta[i]  = (atan2(btset->path->ycoord[i-1]-btset->robot->ROBOT_POS[7],btset->path->xcoord[i-1]-btset->robot->ROBOT_POS[6])+
			    atan2(btset->path->ycoord[i]-btset->path->ycoord[i-1],btset->path->xcoord[i]-btset->path->xcoord[i]))/2 ;
  i--;
  bitmap->current_search_node = bitmap->current_search_node->parent;
  
  btset->path->xcoord[i] = (bitmap->current_search_node->x*btset->pace)+btset->realx;
  btset->path->ycoord[i] = (bitmap->current_search_node->y*btset->pace)+btset->realy;
  btset->path->zcoord[i] = (bitmap->current_search_node->z*btset->pace)+btset->realz;
  btset->path->theta[i]  = atan2(btset->path->ycoord[i]-btset->robot->ROBOT_POS[7],btset->path->xcoord[i]-btset->robot->ROBOT_POS[6]);
  
  i--;
  
  if(i!=0)
    printf("\nWrite Traj: there is a problem: i=%d\n",i);
  
  btset->path->xcoord[0] = btset->robot->ROBOT_POS[ROBOTq_X];
  btset->path->ycoord[0] = btset->robot->ROBOT_POS[ROBOTq_Y];
  btset->path->zcoord[0] = btset->robot->ROBOT_POS[ROBOTq_Z];
  btset->path->theta[0]  = btset->robot->ROBOT_POS[ROBOTq_RZ];
	
  printf("\n*****Path structure created*****\n");
  return TRUE;
}


int hri_bt_verify_path(hri_bitmapset * btset)
{
  int i,ntest;
  configPt qc = NULL, qg = NULL;
  int x,y,z;
  p3d_localpath * localpath;
	
  if(btset == NULL){
    printf("BTSET is null, cannot verify the path\n");
    return FALSE;
  }
  
  if(btset->path == NULL){
    printf("BTSET->path is null, cannot verify the path\n");
    return FALSE;
  }
	
  qc = p3d_get_robot_config(btset->robot); /* ALLOC */
  qg = p3d_get_robot_config(btset->robot); /* ALLOC */
	
	
  for(i=0; i<btset->path->length; i++){
		
    /** ROTATION **/
    /* coming configuration */
		
    qc[6]  = btset->path->xcoord[i];
    qc[7]  = btset->path->ycoord[i];
    if(i==0)
      qc[11] =  btset->path->theta[i];
    else
      qc[11] = atan2(btset->path->ycoord[i]-btset->path->ycoord[i-1],btset->path->xcoord[i]-btset->path->xcoord[i-1]);
		
    /* going configuration */
		
    qg[6]  = btset->path->xcoord[i];
    qg[7]  = btset->path->ycoord[i];
    if(i==btset->path->length-1)
      qg[11] =  btset->path->theta[i];
    else
      qg[11] = atan2(btset->path->ycoord[i+1]-btset->path->ycoord[i],btset->path->xcoord[i+1]-btset->path->xcoord[i]);
		
    /* test */
    if(!p3d_equal_config(btset->robot,qc, qg)){
      if( (localpath = p3d_local_planner(btset->robot, qc, qg)) ){ 
	if(p3d_unvalid_localpath_test(btset->robot, localpath, &ntest)){
	  destroy_list_localpath(btset->robot, localpath);
	  x = (int)((btset->path->xcoord[i]-btset->realx)/btset->pace);
	  y = (int)((btset->path->ycoord[i]-btset->realy)/btset->pace);
	  z = (int)((btset->path->zcoord[i]-btset->realz)/btset->pace);
	  btset->bitmap[BT_OBSTACLES]->data[x][y][z].val = -1;
	  p3d_destroy_config(btset->robot, qc);
	  p3d_destroy_config(btset->robot, qg);
	  printf("Path collision at x:%d y:%d z:%d\n",x,y,z);
	  return FALSE;
	}
      }
      destroy_list_localpath(btset->robot, localpath);
    }
    if(i!=btset->path->length-1){
      /** TRANSLATION **/
      /* coming configuration */
			
      qc[11] = qg[11];
			
      /* going configuration */
			
      qg[6]  = btset->path->xcoord[i+1];
      qg[7]  = btset->path->ycoord[i+1];
			
      /* test */
			
      if( (localpath = p3d_local_planner(btset->robot, qc, qg)) ){ 
	if(p3d_unvalid_localpath_test(btset->robot, localpath, &ntest)){
	  destroy_list_localpath(btset->robot, localpath);
	  x = (int)((btset->path->xcoord[i]-btset->realx)/btset->pace);
	  y = (int)((btset->path->ycoord[i]-btset->realy)/btset->pace);
	  z = (int)((btset->path->zcoord[i]-btset->realz)/btset->pace);
	  btset->bitmap[BT_OBSTACLES]->data[x][y][z].val = -1;
	  p3d_destroy_config(btset->robot, qc);
	  p3d_destroy_config(btset->robot, qg);
	  printf("Path collision at x:%d y:%d z:%d\n",x,y,z);
	  return FALSE;
	}
      }
      destroy_list_localpath(btset->robot, localpath);
    }
		
		
    //		g3d_draw_allwin_active();
  }
	
	
  p3d_destroy_config(btset->robot, qc);
  p3d_destroy_config(btset->robot, qg);
	
	
  printf("Path verified\n");
  return TRUE;
	
}



int hri_bt_verify_pathold(hri_bitmapset * btset)
{
  int ntest;
  p3d_localpath * localpath;
  configPt qc, qn;
  hri_bitmap * bitmap;
  
  if(btset == NULL){
    printf("BTSET is null, cannot verify the path\n");
    return FALSE;
  }
  
  if(btset->path == NULL){
    printf("BTSET->path is null, cannot verify the path\n");
    return FALSE;
  }
	
  bitmap = btset->bitmap[BT_PATH];
  
  qc = p3d_get_robot_config(btset->robot); /* ALLOC */
  qn = p3d_get_robot_config(btset->robot); /* ALLOC */
	
  bitmap->current_search_node = bitmap->search_goal->parent;
	
  qn[6]  = (bitmap->search_goal->x*btset->pace)+btset->realx;
  qn[7]  = (bitmap->search_goal->y*btset->pace)+btset->realy;
  qn[11] = atan2(bitmap->search_goal->y-bitmap->current_search_node->y,bitmap->search_goal->x-bitmap->current_search_node->x);
	
  while(bitmap->current_search_node != NULL){
		
    /* Translation */
    qc[6]  = (bitmap->current_search_node->x*btset->pace)+btset->realx;
    qc[7]  = (bitmap->current_search_node->y*btset->pace)+btset->realy;
    qc[11] = qn[11];
		
    if( (localpath = p3d_local_planner(btset->robot, qc, qn)) ){ 
      if(p3d_unvalid_localpath_test(btset->robot, localpath, &ntest)){
	destroy_list_localpath(btset->robot, localpath);
	btset->bitmap[BT_OBSTACLES]->data[bitmap->current_search_node->x][bitmap->current_search_node->y][bitmap->current_search_node->z].val = -1;
	return FALSE;
      }
    }
    destroy_list_localpath(btset->robot, localpath);
		
    /* Rotation */
		
    qn[6] = qc[6];
    qn[7] = qc[7];
    qn[11] = qc[11];
		
    qc[11] = atan2(bitmap->current_search_node->y-bitmap->current_search_node->parent->y,bitmap->current_search_node->x-bitmap->current_search_node->parent->x);    
    
    if( (localpath = p3d_local_planner(btset->robot, qc, qn)) ){ 
      if(p3d_unvalid_localpath_test(btset->robot, localpath, &ntest)){
	destroy_list_localpath(btset->robot, localpath);
	return FALSE;
      }
    }
    destroy_list_localpath(btset->robot, localpath);      
 	
		
    qc[6] = (bitmap->current_search_node->x*btset->pace)+btset->realx;
    qc[7] = (bitmap->current_search_node->y*btset->pace)+btset->realy;
    qc[11] = atan2(btset->robot->ROBOT_GOTO[7]-bitmap->search_goal->y,btset->robot->ROBOT_GOTO[6]-bitmap->search_goal->x); 
    
    qn[6] = (bitmap->current_search_node->x*btset->pace)+btset->realx;
    qn[7] = (bitmap->current_search_node->y*btset->pace)+btset->realy;
    qn[11] = atan2(btset->robot->ROBOT_GOTO[7]-bitmap->search_goal->y,btset->robot->ROBOT_GOTO[6]-bitmap->search_goal->x); 
    
		
    //  if( (localpath = p3d_local_planner(btset->visball, qtarget, qhuman)) ){ 
    // if(p3d_unvalid_localpath_test(btset->robot, localpath, &ntest)){
    
  }
  
  return TRUE;
  
}




int hri_bt_destroy_path(hri_bitmapset* bitmapset)
{
  if(bitmapset->path == NULL){
    PrintWarning(("Trying to destroy a non existing path"));
    return FALSE;
  }
  
  MY_FREE(bitmapset->path->xcoord,double,bitmapset->path->length);
  MY_FREE(bitmapset->path->ycoord,double,bitmapset->path->length);
  MY_FREE(bitmapset->path->zcoord,double,bitmapset->path->length);
  MY_FREE(bitmapset->path->theta,double,bitmapset->path->length);
  
  MY_FREE(bitmapset->path,hri_bt_path,1);
  
  return TRUE;
}

/* REVISION */
void hri_bt_print_PATH(hri_bitmapset* bitmapset)
{
  int i=0;
  
  if(bitmapset->path == NULL)
    return;
  
  printf("\nPath length = %d\n",bitmapset->path->length);
  
  for(i=0; i<bitmapset->path->length; i++){
    printf("no %d, %f, %f, %f, %f\n",i,bitmapset->path->xcoord[i],bitmapset->path->ycoord[i],bitmapset->path->zcoord[i],bitmapset->path->theta[i]);
  }
  
}


int hri_bt_opt_path(hri_bitmap* bitmap)
{
  hri_bitmap_cell * current, * prev;
  double p_ratio, c_ratio;
  
  if(bitmap->search_goal != NULL &&
     bitmap->search_goal->parent !=NULL &&
     bitmap->search_goal->parent->parent != NULL &&
     bitmap->search_goal->parent->parent->parent != NULL){
    
    prev = bitmap->search_goal;
    current = prev->parent->parent;
    
   
    while(current->parent != NULL){

      if(current->parent->y - current->y)
	c_ratio = (current->parent->x - current->x)/(current->parent->y - current->y);
      else
	c_ratio = 9999;
      
      if(prev->parent->y - prev->y)
	p_ratio = (prev->parent->x - prev->x)/(prev->parent->y - prev->y);
      else
	p_ratio = 9999;
      
      if(c_ratio == p_ratio)
	prev->parent = current->parent;
      else{
	prev = current;
	p_ratio = c_ratio;
      }
      
      if(prev->parent==NULL || prev->parent->parent==NULL ||
	 prev->parent->parent->parent == NULL)
	break;
      else	 
	current = prev->parent->parent;
    }
    
  }
  

  prev = bitmap->search_goal;
  current = prev->parent;
  
  if(current->y - prev->y)
    p_ratio = (current->x - prev->x)/(current->y - prev->y);
  else
    p_ratio = 9999;
  
  while(current->parent != NULL){
    if(current->parent->y - current->y)
      c_ratio = (current->parent->x - current->x)/(current->parent->y - current->y);
    else
      c_ratio = 9999;
    if(c_ratio == p_ratio)
      prev->parent = current->parent;
    else{
      prev = current;
      p_ratio = c_ratio;
    }
    current = current->parent;
  }
  
  return TRUE;
  
}

int hri_bt_write_path2file()
{
  FILE* f = NULL;
  int i;
  
  if( hri_bt_write_TRAJ(BTSET,BTSET->robot->joints[1]) == FALSE ){
    PrintError(("Cant write path structure\n"));
    return FALSE;
  }
  
  f = fopen("path1.dat","w");
  
  for(i=0; i<BTSET->path->length; i++){
    fprintf(f,"%f %f\n",BTSET->path->xcoord[i], BTSET->path->ycoord[i]);
  }
  
  fclose(f);
  
  return TRUE;
  
}

int hri_bt_calculate_bitmap_path(hri_bitmapset * btset, p3d_rob *robotPt, configPt qs, configPt qg, int manip)
{
  p3d_graph *G;
  p3d_node  *Ns=NULL,*Ng=NULL;
  p3d_list_node *graph_node;
  configPt  q_s=NULL, q_g=NULL; 
  double searchgoal[3], searchstart[3];
 	
  if(qs == NULL){
    PrintInfo(("hri_hri_planner : ERROR : no initial configuration\n"));
    return(FALSE);
  }
  if(qg == NULL){
    PrintInfo(("hri_hri_planner : ERROR : no final configuration\n"));
    return(FALSE);
  }
  if(p3d_equal_config(robotPt, qs, qg)) {
    PrintInfo(("hri_hri_planner : ERROR : qs = qg\n"));
    return(FALSE);
  }  
  
  searchstart[0] = qs[ROBOTq_X]; searchstart[1] = qs[ROBOTq_Y]; searchstart[2] = qs[ROBOTq_Z];
  searchgoal[0]  = qg[ROBOTq_X];  searchgoal[1] = qg[ROBOTq_Y];  searchgoal[2] = qg[ROBOTq_Z];
	
  if(!btset->pathexist){ 
    //	while(!pathisOK){
    ChronoOn();
    if(hri_bt_start_search(searchstart, searchgoal, btset, manip) < 0 ){ /* here we find the path */
      PrintInfo(("hri_planner : ERROR : A*: no path found\n"));
      return(FALSE);
    }
		
    ChronoPrint("A* STAR - SEARCH TIME");
    ChronoOff();
		
    //		hri_bt_write_TRAJ(btset, btset->robot->joints[1]);
		
    //			pathisOK = hri_bt_verify_path(btset);
		
    //		}
  }
	
  if(!BTGRAPH)     G = hri_bt_create_graph(robotPt);
  else             G = BTGRAPH;
  
  robotPt->GRAPH = G;
  graph_node = G->nodes;
  
  if(Ns == NULL) {
    q_s = p3d_copy_config(robotPt, qs);
    Ns = p3d_APInode_make(G,q_s);
    p3d_insert_node(G,Ns);
    p3d_create_compco(G,Ns);
    Ns->type = ISOLATED; 
  }
  
  if(Ng == NULL) {
    q_g = p3d_copy_config(robotPt, qg);
    Ng = p3d_APInode_make(G,q_g);
    p3d_insert_node(G,Ng);
    p3d_create_compco(G,Ng);
    Ng->type = ISOLATED;
  }
  
  /* Initialize some data in the graph for the A* graph search */
  G->search_start = Ns;
  G->search_goal = Ng; 
  G->search_done = FALSE;
  
  btset->changed = FALSE;
  
  //hri_bt_write_path2file();
  
  //hri_bt_opt_path(BTSET->bitmap[BT_PATH]);
  //hri_bt_write_path2file();
  hri_bt_bitmap_to_GRAPH(btset,G,btset->bitmap[BT_PATH]);   
  
  /* hri_print_info_graph(G);*/
  MY_ALLOC_INFO("After hri_hri_planner");
  
  return(TRUE);
}


/*********************************************************/
/* Cree une graphe pour le robot dans le contexte bitmap */
/* courant                                               */
/* In :                                                  */
/* Out : le graphe                                       */
/*********************************************************/
p3d_graph * hri_bt_create_graph(p3d_rob* Robot)
{
  p3d_graph * Graph;
  
  Graph      = p3d_allocinit_graph();
  Graph->env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  Graph->rob = Robot;
  
  if (Robot != NULL) {
    if (Robot->GRAPH != NULL) {
      p3d_del_graph(Robot->GRAPH); 
    }
    Robot->GRAPH = Graph;     
  }
  
  return Graph;
}


void g3d_hri_bt_draw_active_3dbitmaps(hri_bitmapset* bitmapset)
{
  int i;
  
  if(bitmapset==NULL)
    return ;
  
  for(i=0; i<bitmapset->n; i++)
    if(bitmapset->bitmap[i]!=NULL && bitmapset->bitmap[i]->active)
      hri_bt_show_3dbitmap(bitmapset,bitmapset->bitmap[i]);
	
}

void  hri_bt_show_3dbitmap(hri_bitmapset* btset,hri_bitmap* bitmap) 
{
  int i,j,k;
  double colorvector[4];
  
  colorvector[0] = 1;       //red
  colorvector[1] = 0;       //green
  colorvector[2] = 0;       //blue
  colorvector[3] = 0.75;    //transparency
  
  if(bitmap->type == BT_3D_PATH){
    hri_bt_show_path(btset,bitmap);
    return;
  }
  
  for(i=0; i<bitmap->nx; i++){
    for(j=0; j<bitmap->ny; j++){
      for(k=0; k<bitmap->nz; k++){
				
	colorvector[1] = bitmap->data[i][j][k].val;
				
	switch(bitmap->type){
	case BT_3D_DISTANCE:
						
	  /* g3d_drawSphere(i*btset->pace+btset->realx,j*btset->pace+btset->realy, */
	  /* 			 k*btset->pace+btset->realz,0.01, Any,colorvector); */
	  //g3d_drawDisc(i*bitmap->pace+bitmap->realx,j*bitmap->pace+bitmap->realy,
	  //	       k*bitmap->pace+bitmap->realz,bitmap->pace/2, Red,NULL);
						
	  //g3d_draw_a_box2(i*btset->pace+btset->realx,(i+1)*btset->pace+btset->realx,
	  //		  j*btset->pace+btset->realy,(j+1)*btset->pace+btset->realy,
	  //		  k*btset->pace+btset->realz,(k+1)*btset->pace+btset->realz,
	  //		  Any,colorvector);
	  g3d_drawOneLine(i*btset->pace+btset->realx-0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
			  i*btset->pace+btset->realx+0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
			  Any,colorvector);
	  g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy-0.01,k*btset->pace+btset->realz,
			  i*btset->pace+btset->realx,j*btset->pace+btset->realy+0.01,k*btset->pace+btset->realz,
			  Any,colorvector);
	  g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz-0.01,
			  i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz+0.01,
			  Any,colorvector);
	  break;
						
	case BT_3D_VISIBILITY:
	  /* g3d_draw_a_box2(i*btset->pace+btset->realx,(i+1)*btset->pace+btset->realx, */
	  /* 	  		  j*btset->pace+btset->realy,(j+1)*btset->pace+btset->realy, */
	  /* 	  		  k*btset->pace+btset->realz,(k+1)*btset->pace+btset->realz, */
	  /* 	  		  Any,colorvector); */
	  g3d_drawOneLine(i*btset->pace+btset->realx-0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
			  i*btset->pace+btset->realx+0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
			  Any,colorvector);
	  g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy-0.01,k*btset->pace+btset->realz,
			  i*btset->pace+btset->realx,j*btset->pace+btset->realy+0.01,k*btset->pace+btset->realz,
			  Any,colorvector);
	  g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz-0.01,
			  i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz+0.01,
			  Any,colorvector);
						
						
	  break;
						
/* 	case BT_3D_HCOMFORT: */
/* 	  g3d_draw_a_box2(i*btset->pace+btset->realx,(i+1)*btset->pace+btset->realx, */
/* 			  j*btset->pace+btset->realy,(j+1)*btset->pace+btset->realy, */
/* 			  k*btset->pace+btset->realz,(k+1)*btset->pace+btset->realz, */
/* 			  Any,colorvector); */
/* 	  break;  */
						
/* 	case BT_3D_RREACH: */
/* 	  g3d_draw_a_box2(i*btset->pace+btset->realx,(i+1)*btset->pace+btset->realx, */
/* 			  j*btset->pace+btset->realy,(j+1)*btset->pace+btset->realy, */
/* 			  k*btset->pace+btset->realz,(k+1)*btset->pace+btset->realz, */
/* 			  Any,colorvector); */
/* 	  break;  */
						
/* 	case BT_3D_PATH: */
/* 	  if(bitmap->data[i][j][k].open) */
/* 	    g3d_draw_a_box2(i*btset->pace+btset->realx,(i+1)*btset->pace+btset->realx, */
/* 			    j*btset->pace+btset->realy,(j+1)*btset->pace+btset->realy, */
/* 			    k*btset->pace+btset->realz,(k+1)*btset->pace+btset->realz, */
/* 			    Any,colorvector); */
/* 	  break;  */
						
	case BT_3D_OBSTACLES:
	  if(bitmap->data[i][j][k].val < -1){
	    g3d_drawOneLine(i*btset->pace+btset->realx-0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
			    i*btset->pace+btset->realx+0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
			    Black,NULL);
	    g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy-0.01,k*btset->pace+btset->realz,
			    i*btset->pace+btset->realx,j*btset->pace+btset->realy+0.01,k*btset->pace+btset->realz,
			    Black,NULL);
	    g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz-0.01,
			    i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz+0.01,
			    Black,NULL);
	  }
          else{
	    if(bitmap->data[i][j][k].val > 0){
	      g3d_drawOneLine(i*btset->pace+btset->realx-0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
			      i*btset->pace+btset->realx+0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
			      Red,NULL);
	      g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy-0.01,k*btset->pace+btset->realz,
			      i*btset->pace+btset->realx,j*btset->pace+btset->realy+0.01,k*btset->pace+btset->realz,
			      Red,NULL);
	      g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz-0.01,
			      i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz+0.01,
			      Red,NULL);
	    } 
	  }
	  break; 
						
	}	
      }
    }
  }
}

double hri_bt_min_cell_limited(hri_bitmapset * btset, hri_bitmap * bitmap, int *x, int *y, int *z, double xlimit, double ylimit, double zlimit )
{
  int i,j,k;
  double mincost=100;
  double cost;
  int xb, xe, yb, ye, zb, ze;
  
  xb = (int)( (bitmap->nx - xlimit/btset->pace)/2 );
  xe = (int)( (bitmap->nx + xlimit/btset->pace)/2 );
  yb = (int)( (bitmap->ny - ylimit/btset->pace)/2 );
  ye = (int)( (bitmap->ny + ylimit/btset->pace)/2 );
  zb = (int)( (bitmap->nz - zlimit/btset->pace)/2 );
  ze = (int)( (bitmap->nz + zlimit/btset->pace)/2 );
  
  for(i=xb; i<xe; i+=2){
    for(j=yb; j<ye; j+=2){
      for(k=zb; k<ze; k+=2){
	if(btset->bitmap[BT_3D_OBSTACLES]->data[i][j][k].val == -1)
	  continue;
	cost = bitmap->calculate_cell_value(btset,i,j,k);
	if(cost < 0)
	  continue;
	if(mincost > cost){
	  mincost =  cost;
	  *x=i;  *y=j;  *z=k;	  
	}
      }
    }
  } 
  
  return mincost;
}



double hri_bt_min_cell(hri_bitmapset * btset,hri_bitmap * bitmap, int *x, int *y, int *z)
{
  int i,j,k;
  double mincost=100;
  double cost;
  
  for(i=0; i<bitmap->nx; i+=2){
    for(j=0; j<bitmap->ny; j+=2){
      for(k=0; k<bitmap->nz; k+=2){
	if(btset->bitmap[BT_3D_OBSTACLES]->data[i][j][k].val == -1)
	  continue;
	cost = bitmap->calculate_cell_value(btset,i,j,k);
	if(cost < 0)
	  continue;
	if(mincost > cost){
	  mincost =  cost;
	  *x=i;  *y=j;  *z=k;	  
	}
      }
    }
  } 
  
  return mincost;
}

double hri_bt_max_cell(hri_bitmapset * btset,hri_bitmap * bitmap, int *x, int *y, int *z)
{
  int i,j,k;
  double maxcost=0;
  double cost;
	
  for(i=0; i<bitmap->nx; i+=2){
    for(j=0; j<bitmap->ny; j+=2){
      for(k=0; k<bitmap->nz; k+=2){
	cost = bitmap->calculate_cell_value(btset,i,j,k);
	if(cost < 0)
	  continue;
	if(maxcost < cost){
	  maxcost =  cost;
	  *x=i;  *y=j;  *z=k;	  
	}
      }
    }
  }  
  return maxcost;
}

int hri_bt_calculate_bitmap_pathwGIK(hri_bitmapset * btset, p3d_vector3 start, p3d_vector3 goal, int manip)
{
  p3d_graph *G;
  p3d_list_node *graph_node;
  configPt q_s=NULL, q_g=NULL, qsaved; 
  p3d_vector3 startt[3], goall[3];
  int i;
  double coord[3], orient[3];
   
  if(start== NULL || goal==NULL){
    PrintError((" start/goal points are null\n"));
    return FALSE;
  }
  if(start[0] == goal[0] && start[1] == goal[1] && start[2] == goal[2]){
    PrintError((" start/goal points are equal\n"));
    return FALSE;
  }
	
  for(i=0; i<3; i++){
    startt[0][i] = start[i];
    startt[1][i] = start[i];
    startt[2][i] = start[i];
  }
  for(i=0; i<3; i++){
    goall[0][i] = goal[i];
    goall[1][i] = goal[i];
    goall[2][i] = goal[i];
  }
	
  if(!BTGRAPH)     G = hri_bt_create_graph(btset->robot);
  else             G = BTGRAPH;
  
  btset->robot->GRAPH = G;
  graph_node = G->nodes;
  
  qsaved = p3d_get_robot_config(btset->robot);
  
  /* Start node in the graph */
  q_s = p3d_get_robot_config(btset->robot);
  hri_gik_compute(btset->robot, HRI_GIK, 500, 0.01, 1, 0, startt,NULL,&q_s, NULL);
  //q_s = p3d_get_robot_config(btset->robot);
  
  G->search_start  = p3d_APInode_make(G,q_s);    
  p3d_insert_node(G,G->search_start);
  p3d_create_compco(G,G->search_start);
  G->search_start->type = ISOLATED; 
  
  /* Goal node in the graph */
  
  p3d_set_and_update_this_robot_conf(btset->robot,qsaved);
  
  /* for 2nd task if it exists */
  p3d_mat4ExtractPosReverseOrder(btset->human[btset->actual_human]->HumanPt->joints[HUMANj_NECK_TILT]->abs_pos, 
		     coord, coord+1, coord+2, orient, orient+1, orient+2); 
  goall[1][0] = coord[0];
  goall[1][1] = coord[1];
  goall[1][2] = coord[2]+0.1;
  /*   */
  
  q_g = p3d_get_robot_config(G->rob);
  hri_gik_compute(btset->robot, HRI_GIK, 500, 0.01, 1, 0, goall,NULL,&q_g, NULL);
  //q_g = p3d_get_robot_config(G->rob);
  
  // Should copy q_g to ROBOT_GOTO
  p3d_copy_config_into(G->rob, q_g, &G->rob->ROBOT_GOTO);
	
  G->search_goal  = p3d_APInode_make(G,q_g);  
  p3d_insert_node(G,G->search_goal);
  p3d_create_compco(G,G->search_goal);
  G->search_goal->type = ISOLATED;
  
  
  if(!btset->pathexist){ 
    ChronoOn();
    if(hri_bt_start_search(start, goal, btset, manip) > 0){ /* here we find the path */
      PrintInfo(("hri_planner : ERROR : A*: no path found\n"));
      return(FALSE);
    }
    
    ChronoPrint("A STAR - TIME");
    ChronoOff();
  } 
	
#ifdef JIDO  
  hri_bt_write_TRAJ(btset,btset->robot->joints[ROBOTj_GRIP]);
  hri_bt_print_PATH(btset);
#endif
	
  G->search_done = FALSE;
  
  p3d_set_and_update_this_robot_conf(btset->robot,qsaved);
  
  
  hri_bt_bitmap_to_graphwGIK(btset,G,btset->bitmap[BT_PATH]);   
  
  p3d_destroy_config(btset->robot,qsaved);
  
  return(TRUE);
}

/****************************************************************/
/*!
 * \brief create the graph regarding to the path found on bitmap
 * 
 * \param G      the graph
 * \param bitmap the bitmap
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_bitmap_to_graphwGIK(hri_bitmapset * btset, p3d_graph *G, hri_bitmap* bitmap) 
{
  configPt q,qsaved; 
  p3d_node *prev_node;
  double dist = 0;
  int done = FALSE;
  p3d_node *NewNode = NULL;
  int count=0;
  p3d_vector3 next_target[3];
  int i, res = FALSE;
  
  if(bitmap->searched)
    bitmap->current_search_node = bitmap->search_goal;
  else
    return FALSE;  
  
  
  qsaved = p3d_get_robot_config(btset->robot);
  
  prev_node =  G->last_node->N;
  q = p3d_copy_config(G->rob, G->search_goal->q);
  while(!done){
    
    /* if(!HRI_GIK_CONTINUOUS) */
    /*       p3d_set_and_update_this_robot_conf(btset->robot,qsaved); */
    
    if(bitmap->current_search_node == bitmap->search_start)
      done = TRUE;    
    
    for(i=0; i<3; i++){ /* 3 tasks */
      next_target[i][0] = bitmap->current_search_node->x * btset->pace + btset->realx;
      next_target[i][1] = bitmap->current_search_node->y * btset->pace + btset->realy;
      next_target[i][2] = bitmap->current_search_node->z * btset->pace + btset->realz;
    }
    
    /* if(giktaskcounter == 0){ */
    /*       p3d_mat4ExtractPos(btset->human[btset->actual_human]->HumanPt->joints[HUMANj_LHAND]->abs_pos, */
    /* 			 coord ,coord+1 ,coord+2 ,orient, orient+1, orient+2);  */
    
    /*       next_target[1][0] = coord[0]; */
    /*       next_target[1][1] = coord[1]; */
    /*       next_target[1][2] = coord[2]; */
    /*       giktaskcounter=0; */
    /*     } */
    /*     else{ */
    /*       next_target[1][0] = bitmap->current_search_node->x * btset->pace + btset->realx; */
    /*       next_target[1][1] = bitmap->current_search_node->y * btset->pace + btset->realy; */
    /*       next_target[1][2] = bitmap->current_search_node->z * btset->pace + btset->realz; */
    /*     } */
    /*     giktaskcounter++; */
    
    res = hri_gik_compute(INTERPOINT->robot, HRI_GIK, 200, 0.01, 1, 0, next_target,NULL,&q, NULL);
    printf("added a node %d\n",res);
    if(HRI_GIK_CONTINUOUS){
      p3d_set_and_update_this_robot_conf(INTERPOINT->robot,q);
    }
    
    q = p3d_get_robot_config(G->rob);
    
    if( /* (count==5) && */ (bitmap->current_search_node != bitmap->search_goal) && 
	(bitmap->current_search_node != bitmap->search_start)){
      count = 0;
      
      if(!p3d_equal_config(G->rob, q, G->search_start->q)){
	NewNode = p3d_APInode_make(G, q);
	NewNode->type = LINKING;  
	p3d_insert_node(G, NewNode);
	dist = p3d_APInode_dist(G,prev_node,NewNode);
	p3d_create_edges(G,prev_node,NewNode,dist);
	p3d_add_node_compco(NewNode, prev_node->comp);    
	prev_node = NewNode;
      }
    }
    bitmap->current_search_node = bitmap->current_search_node->parent; 
    count++;
  }  
  dist = p3d_APInode_dist(G,prev_node,G->search_start);
  p3d_create_edges(G,prev_node ,G->search_start ,dist);
  p3d_merge_comp(G, G->search_start->comp, &(prev_node->comp));
  
  
  p3d_set_and_update_this_robot_conf(INTERPOINT->robot,qsaved);
  p3d_destroy_config(btset->robot,qsaved);
  
  return TRUE;
  
}

void hri_bt_min_cell_n(hri_bitmapset * btset, hri_bitmap * bitmap, int *x, int *y, int *z, double * cost, int n)
{
  int i,j,k;
  int index ;
  
  for(i=0; i<n; i++)
    cost[i] = 1000;
  
  for(i=0; i<bitmap->nx; i++){
    for(j=0; j<bitmap->ny; j++){
      for(k=0; k<bitmap->nz; k++){
				
	index = insert2table(bitmap->calculate_cell_value(btset,i,j,k),i,j,k,
			     cost,x,y,z, n);
				
      }  
    }
  }
}

static int insert2table(double value, int cx, int cy, int cz, double * Table,
			int * x, int * y, int * z, int l)
{
  int i,j;
  
  if(value != -1){	
    for(i=0; i<l; i++){
      if(value < Table[i]){
	for(j=l-1; j>i; j--){
	  Table[j] = Table[j-1];
	  x[j] = x[j-1];
	  y[j] = y[j-1];
	  z[j] = z[j-1];
	}
	Table[i] = value;
	x[i] = cx;
	y[i] = cy;
	z[i] = cz;
	return i;
      }
    }
  }
  return -1;
}

int hri_change_human_state(hri_human * human, int state, configPt  config )
{
  if(config == NULL || human == NULL)
    return FALSE;
  
  if(human->actual_state == BT_STANDING &&
     state == BT_SITTING){
    human->actual_state = state;
    config[8] = human->state[state].c7;
    config[6] = config[6]-0.33*cos(config[11]);
    config[7] = config[7]-0.33*cos(config[11]);
    config[43] = human->state[state].c1;
    config[44] = human->state[state].c2;
    config[46] = human->state[state].c3;
    config[47] = human->state[state].c4;
    config[50] = human->state[state].c5;
    config[53] = human->state[state].c6;
    /* Right Hand */
    config[66] = config[6] + cos(config[11]-0.4)*0.5;
    config[67] = config[7] + sin(config[11]-0.4)*0.5;
    config[68] = config[68]-0.34;
    /* Left Hand */
    config[72] = config[6] + cos(config[11]+0.4)*0.5;
    config[73] = config[7] + sin(config[11]+0.4)*0.5;
    config[74] = config[74]-0.34;
  }
  else
    if(human->actual_state == BT_SITTING &&
       state == BT_STANDING){
      human->actual_state = state;
      config[8] = human->state[state].c7;
      config[43] = human->state[state].c1;
      config[44] = human->state[state].c2;
      config[46] = human->state[state].c3;
      config[47] = human->state[state].c4;
      config[50] = human->state[state].c5;
      config[53] = human->state[state].c6;

      config[66] = config[6] + cos(config[11]-0.4)*0.5;
      config[67] = config[7] + sin(config[11]-0.4)*0.5;
      config[68] = 1.1;
      /* Left Hand */
      config[72] = config[6] + cos(config[11]+0.4)*0.5;
      config[73] = config[7] + sin(config[11]+0.4)*0.5;
      config[74] = 1.1;
    }
    else
      return FALSE;
  
  return TRUE;

}

int hri_bt_create_precise_obstacles(hri_bitmapset * bitmapset)
{
  configPt visq;
  hri_bitmap * bitmap;
  int x,y,z;

  bitmap = bitmapset->bitmap[BT_3D_OBSTACLES];
  
  visq = p3d_get_robot_config(bitmapset->visball);

  for(x=0; x<bitmap->nx; x++){
     for(y=0; y<bitmap->ny; y++){
       for(z=0; z<bitmap->nz; z++){ 
	 visq[6]  = x*bitmapset->pace+bitmapset->realx;
	 visq[7]  = y*bitmapset->pace+bitmapset->realy;
	 visq[8]  = z*bitmapset->pace+bitmapset->realz;
	 p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
	 if(p3d_col_test_robot_statics(bitmapset->visball,FALSE))
	   bitmapset->bitmap[BT_3D_OBSTACLES]->data[x][y][z].val = -2;
	 else
	   bitmapset->bitmap[BT_3D_OBSTACLES]->data[x][y][z].val = 0;
       }
     }
  }
  p3d_destroy_config(bitmapset->visball, visq);
  

  return TRUE;
}


int hri_bt_calculate_bitmap_pathwR6IK(hri_bitmapset * btset, p3d_vector3 start, p3d_vector3 goal, int manip)
{
  p3d_graph *G;
  p3d_list_node *graph_node;
  configPt q_s=NULL, q_g=NULL, qsaved; 
  
  if(start== NULL || goal==NULL){
    PrintError((" start/goal points are null\n"));
    return FALSE;
  }
  if(start[0] == goal[0] && start[1] == goal[1] && start[2] == goal[2]){
    PrintError((" start/goal points are equal\n"));
    return FALSE;
  }
       	
  if(!BTGRAPH)     G = hri_bt_create_graph(btset->robot);
  else             G = BTGRAPH;
  
  btset->robot->GRAPH = G;
  graph_node = G->nodes;
  
  qsaved = p3d_get_robot_config(btset->robot);
  
  /* Start node in the graph */
  q_s = p3d_get_robot_config(btset->robot);
  p3d_copy_config_into(G->rob, q_s, &G->rob->ROBOT_POS);

  //hri_compute_R6IK(btset->robot,btset->object,q_s);
  
  G->search_start  = p3d_APInode_make(G,q_s);    
  p3d_insert_node(G,G->search_start);
  p3d_create_compco(G,G->search_start);
  G->search_start->type = ISOLATED; 
  
  /* Goal node in the graph */
  
  q_g = p3d_get_robot_config(G->rob);
  
  if(!hri_compute_R6IK(btset->robot,btset->object,q_g)){
    printf("Goal position is in collision\n");
    return FALSE;
  }
  /*  configPt q_deg; */
  /*   q_deg =  p3d_get_robot_config(G->rob); */
  /*   p3d_convert_config_rad_to_deg(btset->robot,q_g,&q_deg); */
  /*   // Should copy q_g to ROBOT_GOTO */
     p3d_copy_config_into(G->rob, q_g, &G->rob->ROBOT_GOTO); 
  /*   return FALSE;      */
  G->search_goal  = p3d_APInode_make(G,q_g);  
  p3d_insert_node(G,G->search_goal);
  p3d_create_compco(G,G->search_goal);
  G->search_goal->type = ISOLATED;
  
  if(!btset->pathexist){ 
    ChronoOn();
    if(hri_bt_start_search(start, goal, btset, manip) < 0){ /* here we find the path */
      PrintInfo(("hri_planner : ERROR : A*: no path found\n"));
      return(FALSE);
    }
    
    ChronoPrint("A STAR - TIME");
    ChronoOff();
  } 
  
#ifdef JIDO  
  hri_bt_write_TRAJ(btset,btset->robot->joints[ROBOTj_GRIP]);
  hri_bt_print_PATH(btset);
#endif
  
  G->search_done = FALSE;
  
  p3d_set_and_update_this_robot_conf(btset->robot,qsaved);
  
  hri_bt_bitmap_to_graphwR6IK(btset,G,btset->bitmap[BT_PATH]);   
  
  p3d_destroy_config(btset->robot,qsaved);
  
  return(TRUE);
}

/****************************************************************/
/*!
 * \brief create the graph regarding to the path found on bitmap
 * 
 * \param G      the graph
 * \param bitmap the bitmap
 * 
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_bitmap_to_graphwR6IK(hri_bitmapset * btset, p3d_graph *G, hri_bitmap* bitmap) 
{
  configPt q,qsaved; 
  p3d_node *prev_node;
  double dist = 0;
  int done = FALSE;
  p3d_node *NewNode = NULL;
  int count=0;
   
  if(bitmap->searched)
    bitmap->current_search_node = bitmap->search_goal;
  else
    return FALSE;  
  
  //q_o = p3d_get_robot_config(btset->object); /* ALLOC */
  qsaved = p3d_get_robot_config(btset->robot);

  //qtemp = p3d_copy_config(btset->robot,btset->robot->ROBOT_POS);
  
  prev_node =  G->last_node->N;
  
  while(!done){
    
    if(bitmap->current_search_node == bitmap->search_start)
      done = TRUE;    
    
    //q_o[6] = bitmap->current_search_node->x * btset->pace + btset->realx;
    //q_o[7] = bitmap->current_search_node->y * btset->pace + btset->realy;
    //q_o[8] = bitmap->current_search_node->z * btset->pace + btset->realz;
    /*     q_o[9] = -90*M_PI/180; */
    /*     q_o[10] =-90*M_PI/180; */
    /*     q_o[11] = 113*M_PI/180; */
    
    //p3d_set_and_update_this_robot_conf(btset->object,q_o);
    //   p3d_set_and_update_this_robot_conf(btset->robot,qtemp);
    //q = p3d_copy_config(G->rob, G->search_goal->q);
    //hri_compute_R6IK(btset->robot,btset->object,q);
    //p3d_set_and_update_this_robot_conf(btset->robot,q);
    //p3d_destroy_config(btset->object, q_o);

    q = bitmap->current_search_node->q;
    
    if( (bitmap->current_search_node != bitmap->search_goal) && 
	(bitmap->current_search_node != bitmap->search_start)){
      count = 0;
      
      if(!p3d_equal_config(G->rob, q, G->search_start->q)){
	NewNode = p3d_APInode_make(G, q);
	NewNode->type = LINKING;  
	p3d_insert_node(G, NewNode);
	dist = p3d_APInode_dist(G,prev_node,NewNode);
	p3d_create_edges(G,prev_node,NewNode,dist);
	p3d_add_node_compco(NewNode, prev_node->comp);    
	prev_node = NewNode;
      }
    }
    bitmap->current_search_node = bitmap->current_search_node->parent; 
    count++;
  }  
  dist = p3d_APInode_dist(G,prev_node,G->search_start);
  p3d_create_edges(G,prev_node ,G->search_start ,dist);
  p3d_merge_comp(G, G->search_start->comp, &(prev_node->comp));
  
  
  p3d_set_and_update_this_robot_conf(btset->robot,qsaved);
  p3d_destroy_config(btset->robot,qsaved);
  
  return TRUE;
  
}

gnuplot_ctrl * hri_bt_init_gnuplot(double xmin, double xmax, double ymin, double ymax,double zmin, double zmax)
{
  gnuplot_ctrl * h; 
  
  h = gnuplot_init();
  
  if(h == NULL){
    printf("Gnuplot Init problem");
    return h;
  }

  gnuplot_cmd(h,"set term x11");
  gnuplot_cmd(h,"set xrange [%f:%f]",xmin,xmax);
  gnuplot_cmd(h,"set yrange [%f:%f]",ymin,ymax);
  gnuplot_cmd(h,"set zrange [%f:%f]",zmin,zmax);
  
  return h;
}


int hri_bt_gnuplot_bitmap(gnuplot_ctrl * h,hri_bitmapset * btset, int btno, double exclude)
{
  
  char myfile[] = "temp.dat" ;
  
  if(h==NULL){
    printf("Plot not initialized\n");
    return FALSE;
  }
  
  hri_exp_save(btset,btset->bitmap[btno],myfile,exclude);
  
  gnuplot_cmd(h, "splot '%s' using 1:2:3:4 with points palette " , myfile);
  
  return TRUE;
}
  
hri_bitmapset* hri_bt_create_bitmapsworobots()
{
  hri_bitmapset* bitmapset = MY_ALLOC(hri_bitmapset,1);
  
  bitmapset->human = NULL;  
  bitmapset->human_no = 0;
  bitmapset->visball = NULL;
  bitmapset->robot = NULL;
  bitmapset->actual_human = 0;
  bitmapset->bitmap = NULL;
	
  bitmapset->BT_target_available = FALSE;
	
	
  return bitmapset;
}


int hri_set_human_state(hri_human * human,int state, configPt  config )
{
  if(config == NULL)
    return FALSE;
  
  if(state == BT_SITTING){
    config[8] = human->state[state].c7;
    config[6] = config[6]-0.33*cos(config[11]);
    config[7] = config[7]-0.33*cos(config[11]);
    config[43] = human->state[state].c1;
    config[44] = human->state[state].c2;
    config[46] = human->state[state].c3;
    config[47] = human->state[state].c4;
    config[50] = human->state[state].c5;
    config[53] = human->state[state].c6;
    /* Right Hand */
    config[66] = config[6] + cos(config[11]-0.4)*0.5; /* REVIEW 0.4 --> 0.2 */
    config[67] = config[7] + sin(config[11]-0.4)*0.5;
    config[68] = config[68]-0.34+0.1;
    /* Left Hand */
    config[72] = config[6] + cos(config[11]+0.4)*0.5;
    config[73] = config[7] + sin(config[11]+0.4)*0.5;
    config[74] = config[74]-0.34+0.1;
  }
  else
    if(state == BT_STANDING){
      config[8] = human->state[state].c7;
      config[43] = human->state[state].c1;
      config[44] = human->state[state].c2;
      config[46] = human->state[state].c3;
      config[47] = human->state[state].c4;
      config[50] = human->state[state].c5;
      config[53] = human->state[state].c6;

      config[66] = config[6] + cos(config[11]-0.4)*0.5;
      config[67] = config[7] + sin(config[11]-0.4)*0.5;
      config[68] = 1.1;
      /* Left Hand */
      config[72] = config[6] + cos(config[11]+0.4)*0.5;
      config[73] = config[7] + sin(config[11]+0.4)*0.5;
      config[74] = 1.1;
    }
    else
      return FALSE;
  
  return TRUE;

}
