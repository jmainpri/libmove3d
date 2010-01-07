
#define UNSET  0                     /* for AABB and OBB */
#define MOVED_BACKWARD   1           /* for AABB */
#define MOVED_FORWARD   2            /* for AABB */
#define UNCHANGED  3                 /* for AABB and OBB */
#define MOVED  4                     /* for OBB */

#define WORK  1                      /* OBB defined w.r.t. workframe */
#define LOCAL  2                     /* OBB defined w.r.t. local frame */

/* what is inside */
#define WAS_POLYHEDRON  1            /* (O/AA)BB sits around polyhedron */
#define WAS_SOLID  2                 /* (O/AA)BB sits around a solid */
#define WAS_BBS   3                  /* (O/AA)BB sits around at least 1 other BB */


#define GO_DOWN_ROBOT_TREE     1
#define GO_DOWN_BOTH_TREES     2
#define GO_DOWN_2_ROBOT_TREES  3


/* entity attributes */
/* Begin modif Pepijn june 2001 */
/* 
 * KCD_SMALL_VOLUME_BOX is used as an entity type attribute
 * and not as the whats_inside attribute
 */
#define KCD_SMALL_VOLUME_BOX 8
#define KCD_OBB 9 /* used for exact distance between OBB's*/

#define KCD_AABB 10


/* end modif Pepijn june 2001 */

#define AABB_BOXTYPE  1              /* box type is AABB */
#define OBB_BOXTYPE  2               /* box type is OBB  */

#define OBB_FLAT  0
#define OBB_SPHERICAL  1
#define OBB_TALL  2



