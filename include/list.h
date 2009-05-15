#ifndef LIST_INCLUDE
#define LIST_INCLUDE 1

#ifndef SYS_INCLUDE
#include "p3d_sys.h"
#endif

/************************************************************************/

typedef struct listnode listnode;
typedef listnode *List;

struct listnode {
	int	*data;
	struct listnode *prev, *next;
}LIST_NODE;

#define L_FIRST(L) ((L) ? L_NEXT(L) : NULL)
#define L_NEXT(L) ((L)->next)
#define L_PREV(L) ((L)->prev)
#define LISTDATA(type,L) ((type *)((L)->data))


/* extern int  CLTraverse(); */
/* extern void CLAppend(); */
/* extern void CLInsert(); */
/* extern void CLDeleteNode(); */
/* extern int  CLDeleteData(); */
/* extern void CLKill(); */
/* extern void CLAddData(); */
/* extern List CLFindNode(); */
/* extern PtrI CLFindData(); */
/* extern List CLIterator(); */
/* extern void CLSort(); */
/* extern int  CLLength(); */

#include "list_proto.h"
/* Profiler */
/* #include "Init.prof.h" */
#endif

