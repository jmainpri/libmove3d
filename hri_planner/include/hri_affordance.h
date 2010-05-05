#ifndef _HRI_AFFORDANCE_H
#define _HRI_AFFORDANCE_H
////#include "hri_bitmap.h"

// **** AKP : Structures for storing different surfaces in the environment
//typedef struct bitmap_set hri_bitmapset;

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
point_co_ordi point[500];
double weight[500];
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

int belongs_to_objects_indx[50];//This index should be synchronized with the indices of robots in env
int no_belongs_to_objects;

int close_to_objects_indx[50];//This index should be synchronized with the indices of robots in env
int no_close_to_objects;

//AKP:For belonging to flat surfaces
int is_horizontal_surface;
int near_horizontal_surface;
int surface_id;
}Mightability_map_cell_object_info;

//FOR HATP
typedef struct HATP_constraint_relation
{
 char constraint_name[50];
 int contraint_ID;
 char for_agent_name[50];
 int agent_index;
 char for_object_name[50];
 int object_index;
 int validity_type; //True or False for now
}HATP_constraint_relation;


typedef struct HATP_atomic_actions_mapping
{
 char action_name[50];
 int action_ID;
 int node_ID;
 int no_arguments;//Fixed for 1 action
 char for_agent_name[50];
 int agent_index;
 
 HATP_constraint_relation HATP_constraint[10];
 
}HATP_atomic_actions_mapping;
 


#endif
