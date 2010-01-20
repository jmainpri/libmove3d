#ifndef EBT_INCLUDE
#define EBT_INCLUDE 1

#ifndef EBT_FREE
#define EBT_FREE(s) ((s)?free(s):(void)NULL)
#endif

#ifndef TALLOC
#define TALLOC(type,n)  (type *) calloc(n,sizeof(type))
#endif

#ifndef RETALLOC
#define RETALLOC(type,n,ptr) (type *)realloc(ptr,(n)*sizeof(type))
#endif



/**********************************************************************/

typedef struct ebtnode EBTNode , *pEBTNode;

struct ebtnode {
  pEBTNode         left;
  pEBTNode         right;
  char             *key;
  short            bal;
};

/**********************************************************************/

#define L_BAL -1
#define E_BAL 0
#define R_BAL 1

#define HEIGHT_NO_MODIFIED 0
#define HEIGHT_MODIFIED    1

/**********************************************************************/

#define EBT_SET_LEFT(n,nl)  ((n)->left  = nl)
#define EBT_SET_RIGHT(n,nr) ((n)->right = (nr))
#define EBT_SET_KEY(n,k)    ((n)->key   = (k))
#define EBT_SET_BAL(n,b)    ((n)->bal   = (b))

#define EBT_GET_LEFT(n)     ((n)->left)
#define EBT_GET_RIGHT(n)    ((n)->right)
#define EBT_GET_BAL(n)      ((n)->bal)
#define EBT_GET_KEY(n)      ((n)->key)

#define EBT_ADD_LEFT(n,key)  (((n)->bal)--,(n)->left=MakeNode(key))
#define EBT_ADD_RIGHT(n,key) (((n)->bal)++,(n)->right=MakeNode(key))
#define EBT_ADD_BAL(n,val)   (((n)->bal) += val)
#define EBT_DEL_LEFT(n)      (((n)->bal)++,MY_FREE((n)->left,EBTNode,1),(n)->left=NULL)
#define EBT_DEL_RIGHT(n)     (((n)->bal)--,MY_FREE((n)->right,EBTNode,1),(n)->right=NULL)

#define EBT_BAL_EQUILIBRATED(bal) (abs(bal) > 1 ? FALSE : TRUE)

#define EBT_GET_SON(n,sens)     ((sens)==L_BAL ? (n)->left : (n)->right)
#define EBT_GET_SON_ADDR(n,sens)     ((sens)==L_BAL ? &(n)->left : &(n)->right)
#define EBT_GET_OTHERSON(n,sens)((sens)==L_BAL ? (n)->right : (n)->left)
#define EBT_ADD_SON(n,sens,key) ((sens)==L_BAL ? EBT_ADD_LEFT(n,key) :        \
                                                 EBT_ADD_RIGHT(n,key))


/**********************************************************************/



#define EBT_EMPTY(open) ((open) == NULL ? TRUE : FALSE)

#define EBT_INSERT(node,open) (               \
 (void)EBTInsertNode((pEBTNode *)(open),(char *)(node),ebtBestNode))
 // need to set (node)->opened = TRUE, (node)->closed = TRUE after

#define EBT_GET_BEST(node,open) (                                       \
 (node) = (p3d_node *)EBTFindDeleteFirstNode((pEBTNode *)(open)))
// need to set (node)->opened = FALSE after

#define EBT_DELETE(node,open) (                                         \
  (void)EBTDeleteNode((pEBTNode *)(open),(char *)(node),ebtBestNode),  \
  (node)->opened = FALSE)

#define EBT_CLOSED(node) ((node)->closed == TRUE ? TRUE : FALSE)
#define EBT_OPENED(node) ((node)->opened == TRUE ? TRUE : FALSE)

//start path deform
#define EBT_INSERT_PATH(node,open) (                                         \
 (void)EBTInsertNode((pEBTNode *)(open),(char *)(node),BestPath),   \
 (node)->opened = TRUE, (node)->closed = TRUE)

#define EBT_GET_BEST_PATH(node,open) (                                       \
 (node) = (p3d_path_nodes *)EBTFindDeleteFirstNode((pEBTNode *)(open)))
// need to set (node)->opened = FALSE after

//end path deform

#endif


