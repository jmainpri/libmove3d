#ifndef _MIGHTABILITY_MAP_H
#define _MIGHTABILITY_MAP_H
////#include "hri_bitmap.h"

// **** AKP : Structures for storing different surfaces in the environment
//typedef struct bitmap_set hri_bitmapset;

////////#define MM_FOR_VIRTUALLY_STANDING_HUMAN
////////#define MM_SHOW_DEBUG_MODE_BUTTONS



extern p3d_vector3 to_reach_target;
extern struct grid_3D grid_around_HRP2;
extern int HRP2_GIK_MANIP;// Just to set the type of the bitmap
extern int HRP2_GIK_path_calculated;
extern int Affordances_Found; 

extern int grid_3d_affordance_calculated;
extern int HRP2_CURRENT_STATE;//1 for sitting, 2 for half sitting
extern int SHOW_OBSTACLE_CELLS;
extern int CANDIDATE_POINTS_FOR_TASK_FOUND;

extern p3d_vector3 point_to_look;


// AKP : structure to store x-y co ordinates of point
typedef struct point_co_ordi
{
 double x;
 double y;
 double z;
 double theta;
}point_co_ordi;

typedef struct Mightability_Map_set{


// For Human

int reachable_by_human_LHand; 
int reachable_by_human_RHand; 
int visible_by_human_straight_head_orientation;
int visible_by_human;
int visible_by_human_neck_turn;
int visible_by_human_torso_neack_turn;
int visible_by_standing_human;
int visible_by_standing_human_neck_turn;
int visible_by_standing_human_torso_neack_turn;
int visible_by_human_straight_head_orientation_standing;
int inside; // -1 means grid cell is outside the surface boundary, 1 means inside the surface boundary, because grid will be constructed based on the bounding box so even it may be inside box but not necessarily inside the actual boundary

int reachable_by_LHand_by_bending; 
int reachable_by_RHand_by_bending; 

int reachable_by_LHand_by_turning_around; 
int reachable_by_RHand_by_turning_around;

int reachable_by_LHand_by_turning_around_bending; 
int reachable_by_RHand_by_turning_around_bending;  


int reachable_by_LHand_by_standing; 
int reachable_by_RHand_by_standing;

int reachable_by_LHand_by_standing_bending; 
int reachable_by_RHand_by_standing_bending;

int reachable_by_LHand_by_standing_turning_around; 
int reachable_by_RHand_by_standing_turning_around;

int reachable_by_LHand_by_standing_turning_around_bending; 
int reachable_by_RHand_by_standing_turning_around_bending;  




//For HRP2
int reachable_by_HRP2_LHand; 
int reachable_by_HRP2_RHand; 
int visible_by_HRP2_straight_head_orientation;
int visible_by_HRP2;
int visible_by_HRP2_neck_turn;
int visible_by_HRP2_torso_neck_turn;
int visible_by_standing_HRP2;
int visible_by_standing_HRP2_neck_turn;
int visible_by_standing_HRP2_torso_neck_turn;
int visible_by_HRP2_straight_head_orientation_standing;

//For JIDO
int reachable_by_JIDO_Hand; 
int visible_by_JIDO;
int visible_by_JIDO_straight_head_orientation;
int visible_by_JIDO_neck_turn;



 


}Mightability_Map_set;

typedef struct surface_grid_cell{

// For Human
int reachable_by_LHand; 
int reachable_by_RHand; 
int visible;
int inside; // -1 means grid cell is outside the surface boundary, 1 means inside the surface boundary, because grid will be constructed based on the bounding box so even it may be inside box but not necessarily inside the actual boundary

int reachable_by_LHand_by_bending; 
int reachable_by_RHand_by_bending; 

int reachable_by_LHand_by_turning_around; 
int reachable_by_RHand_by_turning_around;

int reachable_by_LHand_by_turning_around_bending; 
int reachable_by_RHand_by_turning_around_bending;  

int reachable_by_LHand_by_standing; 
int reachable_by_RHand_by_standing;

int reachable_by_LHand_by_standing_bending; 
int reachable_by_RHand_by_standing_bending;


//For HRP2
int reachable_by_HRP2_LHand; 
int reachable_by_HRP2_RHand; 
int visible_by_HRP2;
 

}surface_grid_cell;

typedef struct flat_surface{
int surface_ID;
int no_vertices;
int surface_shape; //Polygon=1, circle=2, rectangle=3 etc
point_co_ordi vertices[10]; // for the case of polygon or rectangle or non circle NOTE : The vertices should be stored either in clockwise or in anticlockwise order for the purpose of detecting a point is inside or not
double BR_x_min; // min and max values of x and y of the bounding rectangle
double BR_x_max;
double BR_y_min;
double BR_y_max;
double BR_z;
point_co_ordi Bounding_Rectangle[4];  // It should be populated in the sequence of (x_min, y_min), (x_max,y_min), (x_max, y_max), (x_min, y_max)

double circle_centre; // for the case of circle
double radius; // for the case of circle

surface_grid_cell surf_grid[100][100]; // Max 100x100 cells
int grid_i_max;//The maximum valid 1st index of surf_grid[i][]
int grid_j_max;// The maximum valid 2nd index of surf_grid[][j]

}flat_surface;



typedef struct env_surfaces{
flat_surface flat_surf[100]; // Total no. of surfaces in the environment 
int total_no_of_surfaces; 

}env_surfaces;

typedef struct grid_Bounding_Box{
double min_x;
double max_x;
double min_y;
double max_y;
double min_z;
double max_z;
}grid_BB;

typedef struct grid_3D{
grid_Bounding_Box grid_BB;
////hri_bitmapset *GRID_SET;
struct bitmap_set *GRID_SET;
grid_3D()
 {
 GRID_SET = NULL;
 }
}grid_3D;

typedef struct candidate_poins_for_task{
point_co_ordi point[1000];
double weight[1000];
int status[1000]; //0 is not tested for validity, 1 is accepted, 2 is rejected 
int no_points; 
}candidate_poins_for_task;

typedef struct robots_status{
char rob_name[50];//This index should be synchronized with the indices of robots in env
//int is_robot;
////int rob_index;
configPt rob_prev_config;
////p3d_rob *rob_prev_config;
////p3d_rob *rob_prev_pos;

/*
double prev_x;
double prev_y;
double prev_z;

double curr_x;
double curr_y;
double curr_z;
*/
////p3d_BB prev_BB;
int has_moved;

}object_status;

typedef struct Mightability_map_cell_object_info{

int belongs_to_objects_indx[50];//This index should be synchronized with the indices of robots in env, as it gives object having index i in env belongs to the cell or not
int objects_belonging_to[50];//It stores the indices of the objects belonging to this cell. This is redundant information, but required for fast access depending upon the case.//NOTE AKP Warning:: Don't use it. It is not fully implemented yet.

int no_belongs_to_objects;

int close_to_objects_indx[50];//This index should be synchronized with the indices of robots in env
int no_close_to_objects;

int belongs_to_object_BB_centre_indx[50];//This index should be synchronized with the indices of robots in env
int no_belongs_to_object_BB_centre;

//For belonging to flat surfaces
int is_horizontal_surface;
int near_horizontal_surface;
int surface_id;

//For belonging to the first non visible cell of different type for human
int first_non_visible_by_human_straight_head_orientation;
int first_non_visible_by_human;
int first_non_visible_by_human_neck_turn;
int first_non_visible_by_human_torso_neack_turn;
int first_non_visible_by_standing_human;
int first_non_visible_by_standing_human_neck_turn;
int first_non_visible_by_standing_human_torso_neack_turn;
int first_non_visible_by_human_straight_head_orientation_standing; 

//For belonging to the first non visible cell of different type for HRP2
int first_non_visible_by_HRP2_straight_head_orientation;
int first_non_visible_by_HRP2;
int first_non_visible_by_HRP2_neck_turn;
int first_non_visible_by_HRP2_torso_neck_turn;
int first_non_visible_by_standing_HRP2;
int first_non_visible_by_standing_HRP2_neck_turn;
int first_non_visible_by_standing_HRP2_torso_neck_turn;
int first_non_visible_by_HRP2_straight_head_orientation_standing;

//For belonging to the first non visible cell of different type for JIDO
int first_non_visible_by_JIDO;
int first_non_visible_by_JIDO_straight_head_orientation;
int first_non_visible_by_JIDO_neck_turn;

}Mightability_map_cell_object_info;

typedef struct MM_based_task
{
 //Mightability_Map_set for_task;
 int is_true;
 int no_points;
 point_co_ordi point[200];

}MM_based_task;

typedef struct Symbolic_Mightability_Maps_Relations
{
 
 int reachable_for_grasping_to_carry_by_human;//For able to grasp, hold and carry
 int reachable_for_grasping_to_carry_by_HRP2;//For able to grasp, hold and carry
 int reachable_for_grasping_to_carry_by_JIDO;//For able to grasp, hold and carry

//If at least one cell at the z_max of the bounding box of the object is reachable by agent, it is assumed that the agent could put somting inside that object if it is a container. Of course it should be filtered at higher level based on the size of the object to be put
 ////int reachable_for_putting_inside_by_human;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside
 ////int reachable_for_putting_inside_by_JIDO;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside
 ////int reachable_for_putting_inside_by_HRP2;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside

//If at least one cell at the z_max of the bounding box of the object is reachable by agent, it is assumed that the agent could put somting inside that object if it is a container. Of course it should be filtered at higher level based on the size of the object to be put
 MM_based_task reachable_for_putting_inside_by_human;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside
 MM_based_task reachable_for_putting_inside_by_JIDO;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside
 MM_based_task reachable_for_putting_inside_by_HRP2;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside
 
//If Any cell which is occupied AND which belongs to that object is reachable by agent, then we assume agent can at least touch the object
 int reachable_for_touching_by_human;//For able to just touch the bounding box of the object
 int reachable_for_touching_by_HRP2;//For able to just touch the bounding box of the object
 int reachable_for_touching_by_JIDO;//For able to just touch the bounding box of the object

}Symbolic_Mightability_Maps_Relations;
 

typedef struct object_mightability_set
{
 Mightability_Map_set geo_MM;
 Symbolic_Mightability_Maps_Relations sym_MM;
}object_mightability_set;

typedef struct object_Symbolic_Mightability_Maps_Relation
{
 //int object_indx[50]; //This index should be synchronized with the indices of robots in env
 char object_name[50];
 ////Symbolic_Mightability_Maps_Relations MM_Relations;
 ////////Mightability_Map_set object[50];//This index should be synchronized with the indices of robots in env
 object_mightability_set object[50];//This index should be synchronized with the indices of robots in env

 
 
} object_Symbolic_Mightability_Maps_Relation;

typedef struct robots_indices
{
int HRP2;
int JIDO_ROBOT;
int YELLOW_BOTTLE;
int HUMAN;
} robots_indices;



////**** structures for exporting the object level mightability information to the external modules
typedef struct object_mightabilities
{
int is_reachable_by_human;
int is_visible_by_human;
int is_reachable_by_robot;
int is_visible_by_robot;
}object_mightabilities;

typedef struct object_mightabilities_info
{
char object_name[50];//Name of robot(movable objects)
int object_index;//Index of robot (movable objects) in env
object_mightabilities obj_mightability;
int total_no_obj;// Total number of robot(movable objects)

}object_mightabilities_info;

typedef struct object_mightabilities_info_set
{
object_mightabilities_info object[100];//all the robot(movable objects)
int total_no_obj;// Total number of robot(movable objects)

}object_mightabilities_info_set;


#endif
