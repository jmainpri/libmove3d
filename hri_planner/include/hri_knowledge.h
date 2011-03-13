#ifndef _HRI_KNOWLEDGE_H
#define _HRI_KNOWLEDGE_H

typedef enum ENUM_HRI_ENTITY_TYPE {
  HRI_OBJECT = 0,
  HRI_ISAGENT = 1, /* There is already a type called HRI_AGENT */
  HRI_AGENT_PART = 2,
  HRI_OBJECT_PART = 3,
  HRI_VIRTUAL_OBJECT = 4
} HRI_ENTITY_TYPE;

// We had some more sementic to adapt the level of processing and reasoning to the
// 
typedef enum ENUM_HRI_ENTITY_SUBTYPE {
  HRI_MOVABLE_OBJECT = 0, // Tape, Bottles
  HRI_OBJECT_SUPPORT = 1,  // Tables, Shelf , ...
  HRI_OBJECT_CONTAINER = 2, // Dust bin, Box
  HRI_AGENT_HEAD = 3,
  HRI_AGENT_HAND = 4,
  HRI_AGENT_AGENT = 5, 
  HRI_OBJECT_PLACEMAT = 6,  // planar objects to materialize some areas on furnitures 
  HRI_UK_ENTITY_SUBTYPE = 7
} HRI_ENTITY_SUBTYPE;

typedef enum ENUM_HRI_MOTION {
  HRI_STATIC = 1,
  HRI_MOVING = 2,
  HRI_UK_MOTION = 0 /* Unknown motion. meaning we don't know */
} HRI_MOTION;

typedef enum ENUM_HRI_DETECTION { 
  HRI_NEVER_DETECTED = 0,
  HRI_DETECTED = 1,
  HRI_EXPLAINED_UNDETECTION = 2,
  HRI_UNEXPLAINED_UNDETECTION_ITER = 3, 
  HRI_UNEXPLAINED_UNDETECTION_MAX = 4
} HRI_DETECTION;

typedef enum ENUM_HRI_PLACEMENT_STATE_TRANSITION { 
  HRI_DISAPPEAR = 0,
  HRI_APPEAR = 1,
  HRI_START_MOVING = 2,
  HRI_STOP_MOVING = 3, 
  HRI_UK_PL_STATE_TRANSITION = 4
} HRI_PLACEMENT_STATE_TRANSITION;

typedef struct STRUCT_HRI_ENTITY {
  char name[64];
  
  HRI_ENTITY_TYPE type;
  ENUM_HRI_ENTITY_SUBTYPE subtype;
  int is_present; /* Is present in the env, i.e. at least seen once by the robot */
  
  int is_detected; // has robot just perceived the object
  unsigned long detection_time;
  unsigned long last_detection_time;
  int undetection_iter;
  HRI_DETECTION undetection_status; /* why and how much was the object undetected */
  double visibility_percentage; /** visibility percentage score for robot for disappear management */

  int can_disappear_and_move; /* Can this entity disappear? For example a furniture can be considered a not movable */
  int disappeared; /* Is present but it is not at the place where it is supposed to be */

  int last_ismoving_iter; /* how many*/
  HRI_MOTION filtered_motion; /* */

  int is_pl_state_transition_new;
  HRI_PLACEMENT_STATE_TRANSITION pl_state_transition;
  

  p3d_rob * robotPt;
  p3d_obj * partPt;
  int agent_idx; /* if it's a part of an agent, then its index */

} HRI_ENTITY;

typedef struct STRUCT_HRI_ENTITIES {
  HRI_ENTITY ** entities;
  int entities_nb;
  int eventsInTheWorld;
  int lastEventsInTheWorldStep;
  int isWorldStatic;
  int needSituationAssessmentUpdate;
} HRI_ENTITIES;

typedef enum ENUM_HRI_VISIBILITY {
  HRI_VISIBLE = 0,
  HRI_INVISIBLE = 1,
  HRI_UK_VIS = 2 /* Unknown visibility. meaning we don't know */
} HRI_VISIBILITY;

/* For the boolean facts we always have possible values : true, false and unknown */
typedef enum ENUM_HRI_TRUE_FALSE_UK_V {
  HRI_FALSE_V = 0,
  HRI_TRUE_V = 1,
  HRI_UK_V = 2 
} HRI_TRUE_FALSE_UK_V;

typedef enum ENUM_HRI_VISIBILITY_PLACEMENT {
  HRI_FOA = 1,
  HRI_FOV = 2,
  HRI_OOF = 3,
  HRI_UK_VIS_PLACE = 4 /* Unknown visibility placement. meaning we don't know */
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
  HRI_UK_REACHABILITY = 3 /* Unknown reachability. meaning we don't know */
} HRI_REACHABILITY;

typedef enum ENUM_HRI_PLACEMENT_RELATION {
  HRI_ISIN     = 0,
  HRI_ISON     = 1,
  HRI_ISNEXTTO = 2,
  HRI_NOPLR    = 3,
  HRI_UK_PLR   = 4 /* Unknown placement. meaning we don't know */
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
  HRI_FAR_FRONT_RIGHT = 16,
  HRI_UK_RELATION = 17, /* Unknown relation. meaning we don't know */
} HRI_SPATIAL_RELATION;

typedef struct STRUCT_HRI_KNOWLEDGE_ON_ENTITY {
  HRI_ENTITY * entPt;

  long update_date;

  int disappeared_isexported; /* bool to know wether disappear information was exported for this agent */  

  HRI_MOTION motion;
  int motion_ischanged;
  int motion_isexported;

  HRI_VISIBILITY visibility;
  int visibility_ischanged;
  int visibility_isexported;

  HRI_REACHABILITY reachability;
  int reachability_ischanged;
  int reachability_isexported;

  HRI_TRUE_FALSE_UK_V is_looked_at;
  int is_looked_at_ischanged;
  int is_looked_at_isexported;

  HRI_TRUE_FALSE_UK_V is_pointed_at;
  int is_pointed_at_ischanged;
  int is_pointed_at_isexported;

  HRI_VISIBILITY_PLACEMENT is_placed_from_visibility; /* oof, fov, ... */
  int visibility_placement_ischanged;
  int visibility_placement_isexported;

  HRI_SPATIAL_RELATION is_located_from_agent; /* Front, right, .... */
  int spatial_relation_ischanged;
  int spatial_relation_isexported;

  HRI_PLACEMENT_RELATION * is_placed; /* on, in, ... */
  HRI_PLACEMENT_RELATION * is_placed_old; /* To limit number of messages send to ontology */
  int * placement_relation_ischanged;
  int * placement_relation_isexported;
  int is_placed_nb;  /* length is HRI_ENTITIES->all_entities_nb */

} HRI_KNOWLEDGE_ON_ENTITY;


/* Each agent has a knowledge structure. It presents the agent's knowledge on the geometry of the world */
typedef struct STRUCT_HRI_KNOWLEDGE {
  /* The spatial knowledge on the state of things from the perspective of the agent */
  /* Normally all indexes should be synchronized with entities structure */

  /* int * looks_at; /\* indexes of entities in entity structure *\/ */
  /* int looks_at_nb; */
  /* int * points_at; /\* indexes of entities in entity structure *\/ */
  /* int points_at_nb; */

  HRI_KNOWLEDGE_ON_ENTITY * entities;
  int entities_nb;
} HRI_KNOWLEDGE;

#endif
