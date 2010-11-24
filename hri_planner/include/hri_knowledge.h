#ifndef _HRI_KNOWLEDGE_H
#define _HRI_KNOWLEDGE_H

typedef enum ENUM_HRI_ENTITY_TYPE {
  HRI_OBJECT = 0,
  HRI_ISAGENT = 1,
  HRI_AGENT_PART = 2,
  HRI_OBJECT_PART = 3,
  HRI_VIRTUAL_OBJECT = 4
} HRI_ENTITY_TYPE;

typedef struct STRUCT_HRI_ENTITY {
  
  HRI_ENTITY_TYPE type;
  int can_disappear;
  
  p3d_rob * robotPt;
  p3d_obj * partPt;
  int agent_idx;
  
} HRI_ENTITY;

typedef struct STRUCT_HRI_ENTITIES {
  HRI_ENTITY * entities;
  int entities_nb;
} HRI_ENTITIES;

typedef enum ENUM_HRI_VISIBILITY {
  HRI_VISIBLE = 0,
  HRI_INVISIBLE = 1,
  HRI_UK_VIS = 2
} HRI_VISIBILITY;

typedef enum ENUM_HRI_VISIBILITY_PLACEMENT {
  HRI_FOA = 1,
  HRI_FOV = 2,
  HRI_OOF = 3,
  HRI_UK_VIS_PLACE = 4
} HRI_VISIBILITY_PLACEMENT;

typedef struct STRUCT_HRI_VISIBILITY_LIST {
  HRI_VISIBILITY *vis; /* The index is the same as env->nb */
  HRI_VISIBILITY_PLACEMENT *vispl; /* The index is the same as env->nb */
  int vis_nb; /* Normally this number should be = to the number of robots in the env */
} HRI_VISIBILITY_LIST;

typedef enum ENUM_HRI_REACHABILITY {
  HRI_UNREACHABLE = 0,
  HRI_REACHABLE = 1,
  HRI_HARDLY_REACHABLE = 2,
  HRI_UK_REACHABILITY = 3
} HRI_REACHABILITY;

typedef enum ENUM_HRI_MOTION {
  HRI_STATIC = 1,
  HRI_MOVING = 2,
  HRI_UK_MOTION = 0
} HRI_MOTION;

typedef enum ENUM_HRI_PLACEMENT_RELATION {
  HRI_ISIN     = 0,
  HRI_ISON     = 1,
  HRI_ISNEXTTO = 2,
  HRI_NOPLR    = 3,
  HRI_UK_PLR   = 4
} HRI_PLACEMENT_RELATION;

typedef enum ENUM_HRI_SPATIAL_RELATION {
  HRI_NO_RELATION = 0,
  HRI_NEAR_FRONT = 1,
  HRI_NEAR_FRONT_LEFT =  2,
  HRI_NEAR_LEFT = 3,
  HRI_NEAR_BACK_LEFT = 4,
  HRI_NEAR_BACK = 5,
  HRI_NEAR_BACK_RIGHT = 6,
  HRI_NEAR_RIGHT = 7,
  HRI_NEAR_FRONT_RIGHT = 8,
  HRI_FAR_FRONT = 9,
  HRI_FAR_FRONT_LEFT =  10,
  HRI_FAR_LEFT = 11,
  HRI_FAR_BACK_LEFT = 12,
  HRI_FAR_BACK = 13,
  HRI_FAR_BACK_RIGHT = 14,
  HRI_FAR_RIGHT = 15,
  HRI_FAR_FRONT_RIGHT = 16
} HRI_SPATIAL_RELATION;

typedef struct STRUCT_HRI_KNOWLEDGE_ON_ENTITY {
  char name[64];
  HRI_ENTITY_TYPE type;
  long update_date;
  int is_present;
  
  HRI_MOTION motion;
  
  HRI_VISIBILITY_PLACEMENT is_placed_from_visibility;
  
  HRI_SPATIAL_RELATION * is_located_from_agent; 
  int is_located_from_agent_nb;    /* length is HRI_AGENTS->all_agents_nb */
  
  HRI_PLACEMENT_RELATION * is_placed;  
  int is_placed_nb;  /* length is HRI_ENTITIES->all_entities_nb */
  
} HRI_KNOWLEDGE_ON_ENTITY;

typedef struct STRUCT_HRI_KNOWLEDGE {
  /* The spatial knowledge on the state of things from the perspective of the agent */
  
  HRI_VISIBILITY * sees;
  int sees_nb;
  HRI_REACHABILITY * reaches;
  int reaches_nb;
  
  int * looks_at;
  int looks_at_nb;
  int * points_at;
  int points_at_nb;
  
  HRI_KNOWLEDGE_ON_ENTITY * entities;
  int entities_nb;
} HRI_KNOWLEDGE;


#endif