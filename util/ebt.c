#include "Util-pkg.h"

//#include <stdlib.h>
//#include "ebt.h"

/***********************************************************************
 
       Written by T. Simeon 
       nic@grasp.cis.upenn.edu
       U.P.E.N.N july 89
 
  This package provides a set of functions for manipulating EBT trees.
 
  These trees are binary and equilibrated search trees for which 
  the insertion, deletion or search operations can be performed 
  in O(log n).
  
  The main routines are:
 
     (int)EBTInsertNode((char **)ebt , (char *)key , fct_compar)
     (int)EBTDeleteNode((char **)ebt , (char *)key , fct_compar)
  (char *)EBTFindNode  ((char *) ebt , (char *)key , fct_compar)
  
  They are generalized from Knuth (6.2.3). All comparisons are done with
  a user-supplied routine. This routine is called with two arguments.
  The first one is a pointer to the new key, the other one is a pointer 
  to the current key. The routine must return -1, 0 or 1 according to
  whether the first argument is to be considered less than, equal to or
  greater than the second argument. The comparaison function need not
  compare every byte, so arbitrary data may be contained in the elements
  in addition to the value being compared.
 
  In addition, the following routines are available:
 
  (int)EBTCheckBalance((char *)ebt)
       returns TRUE if the tree is well balanced (only for debugging purpose)
 
  (int)EBTFindDeleteFirstNode((char *)ebt)              
       returns the best node of the tree
 
  (int)EBTHeight((char *)ebt)
  (int)EBTWidth((char *)ebt)
  (int)EBTLength((char *)ebt)
      return the height,the number of leaves and the total number of
      nodes in the tree.
 
 *********************************************************************/

static pEBTNode MakeNode(char *key) {
  /* pEBTNode n = TALLOC(EBTNode,1); */

  pEBTNode n = MY_ALLOC(EBTNode,1);
  n->left = NULL;
  n->right = NULL;
  n->key = NULL;
  n->bal = 0;

  if(!n) {
    fprintf(stderr,"Can't create a new Ebt node\n");
    return(NULL);
  }

  n->key    = key;
  return(n);

}

/**********************************************************************/

static int RotateLeft(pEBTNode *p_na) {
  register pEBTNode na = *p_na;
  register pEBTNode nb = EBT_GET_LEFT(na);
  short    bb = EBT_GET_BAL(nb);
  pEBTNode nc;
  short    bc;

  return
    (EBT_GET_BAL(nb) == L_BAL) ?
    (EBT_SET_BAL(na,E_BAL),
     EBT_SET_BAL(nb,E_BAL),
     EBT_SET_LEFT (na , EBT_GET_RIGHT(nb)),
     EBT_SET_RIGHT(nb , na),
     *p_na = nb,
     TRUE):

    (bb == E_BAL) ?
    (EBT_SET_BAL(na,L_BAL),
     EBT_SET_BAL(nb,R_BAL),
     EBT_SET_LEFT (na , EBT_GET_RIGHT(nb)),
     EBT_SET_RIGHT(nb , na),
     *p_na = nb,
     FALSE):

    (nc = EBT_GET_RIGHT(nb),
     bc = EBT_GET_BAL(nc),
     EBT_SET_BAL(nc,E_BAL),
     (bc == L_BAL) ?
     (EBT_SET_BAL(na,R_BAL),
      EBT_SET_BAL(nb,E_BAL)):
     (bc == E_BAL) ?
     (EBT_SET_BAL(na,E_BAL),
      EBT_SET_BAL(nb,E_BAL)):
     (EBT_SET_BAL(na,E_BAL),
      EBT_SET_BAL(nb,L_BAL)),
     EBT_SET_RIGHT(nb,EBT_GET_LEFT(nc)),
     EBT_SET_LEFT(nc,nb),
     EBT_SET_LEFT(na,EBT_GET_RIGHT(nc)),
     EBT_SET_RIGHT(nc,na),
     *p_na = nc,
     TRUE);
}

/**********************************************************************/
static int RotateRight(pEBTNode *p_na) {
  register pEBTNode na = *p_na;
  register pEBTNode nb = EBT_GET_RIGHT(na);
  short    bb = EBT_GET_BAL(nb);
  pEBTNode nc;
  short    bc;

  return
    (EBT_GET_BAL(nb) == R_BAL) ?
    (EBT_SET_BAL(na,E_BAL),
     EBT_SET_BAL(nb,E_BAL),
     EBT_SET_RIGHT(na , EBT_GET_LEFT(nb)),
     EBT_SET_LEFT(nb , na),
     *p_na = nb,
     TRUE):

    (bb == E_BAL) ?
    (EBT_SET_BAL(na,R_BAL),
     EBT_SET_BAL(nb,L_BAL),
     EBT_SET_RIGHT(na , EBT_GET_LEFT(nb)),
     EBT_SET_LEFT(nb , na),
     *p_na = nb,
     FALSE):

    (nc = EBT_GET_LEFT(nb),
     bc = EBT_GET_BAL(nc),
     EBT_SET_BAL(nc,E_BAL),
     (bc == L_BAL) ?
     (EBT_SET_BAL(na,E_BAL),
      EBT_SET_BAL(nb,R_BAL)):
     (bc == E_BAL) ?
     (EBT_SET_BAL(na,E_BAL),
      EBT_SET_BAL(nb,E_BAL)):
     (EBT_SET_BAL(na,L_BAL),
      EBT_SET_BAL(nb,E_BAL)),
     EBT_SET_LEFT(nb,EBT_GET_RIGHT(nc)),
     EBT_SET_RIGHT(nc,nb),
     EBT_SET_RIGHT(na,EBT_GET_LEFT(nc)),
     EBT_SET_LEFT(nc,na),
     *p_na = nc,
     TRUE);
}

/**********************************************************************/
static int Rotate(pEBTNode *p_ebt, int bal) {
  return (bal < L_BAL) ? RotateLeft(p_ebt):
         (bal > R_BAL) ? RotateRight(p_ebt):
         !bal;
}

/**********************************************************************/
static int
ChangeLeftSubtree(pEBTNode *p_ebt, char **key) {
  pEBTNode node       = *p_ebt;
  pEBTNode left_son   = EBT_GET_LEFT(node);
  pEBTNode *right_son = &EBT_GET_RIGHT(node);
  short    newbal;

  if(*right_son) {
    if(ChangeLeftSubtree(right_son,key)) {
      if((newbal = EBT_ADD_BAL(node,L_BAL)) < L_BAL)
        return(RotateLeft(p_ebt));
      else
        return(!newbal);
    }
    else
      return(HEIGHT_NO_MODIFIED);
  }
  else {
    *key = EBT_GET_KEY(node);

    if(left_son) {
      EBT_SET_KEY(node,EBT_GET_KEY(left_son));
      EBT_DEL_LEFT(node);
    }
    else {
      MY_FREE(node,EBTNode,1);
      *p_ebt = NULL;
    }
    return(HEIGHT_MODIFIED);
  }
}
/**********************************************************************/
static int DeleteThisNode(pEBTNode *p_ebt) {
  pEBTNode node       = *p_ebt;
  pEBTNode *left_son  = &EBT_GET_LEFT(node);
  pEBTNode right_son  = EBT_GET_RIGHT(node);
  short    newbal;

  if(*left_son) {
    if(ChangeLeftSubtree(left_son,&EBT_GET_KEY(node))) {

      if((newbal = EBT_ADD_BAL(node,R_BAL)) > R_BAL)
        return(RotateRight(p_ebt));

      else
        return(!newbal);
    }
    else
      return(HEIGHT_NO_MODIFIED);
  }
  else {
    if(right_son) {
      EBT_SET_KEY(node,EBT_GET_KEY(right_son));
      EBT_DEL_RIGHT(node);
    }
    else {
      MY_FREE(node,EBTNode,1);
      *p_ebt = NULL;
    }

    return(HEIGHT_MODIFIED);
  }
}
/**********************************************************************/
static int InsertNode(pEBTNode *p_ebt, char *key,
           int (*fct_order)(void *, void *), int *res) {
  pEBTNode node = *p_ebt;
  pEBTNode *son;
  short    sens;
  short    newbal;

  if((sens = (*fct_order)(key,EBT_GET_KEY(node)))) {
    son = EBT_GET_SON_ADDR(node,sens);
    if(*son) {
      if(InsertNode(son,key,fct_order,res)) {
        newbal = EBT_ADD_BAL(node,sens);
        if(EBT_BAL_EQUILIBRATED(newbal))
          return(abs(newbal));
        else {
          Rotate(p_ebt,newbal);
          return(HEIGHT_NO_MODIFIED);
        }
      }
      else
        return(HEIGHT_NO_MODIFIED);
    }
    else {
      EBT_ADD_SON(node,sens,key);

      if(EBT_GET_OTHERSON(node,sens))
        return(HEIGHT_NO_MODIFIED);
      else
        return(HEIGHT_MODIFIED);
    }
  }
  else {
    *res = FALSE;
    return(HEIGHT_NO_MODIFIED);
  }
}
/**********************************************************************/
int EBTInsertNode(pEBTNode *p_ebt,char *key,int (*fct_order)(void *, void *)) {
  if(*p_ebt) {
    int res = TRUE;
    InsertNode(p_ebt,key,fct_order,&res);
    return(res);
  } else {
    *p_ebt = MakeNode(key);
    return(TRUE);
  }
}
/**********************************************************************/
char * EBTFindNode(pEBTNode ebt,char *key,
            int (*fct_order)(void *, void *)) {
  char  *keyebt;
  short sens;

  if(!ebt)
    return(NULL);

  keyebt = EBT_GET_KEY(ebt);

  if((sens = (*fct_order)(key,keyebt)))
    return(EBTFindNode(EBT_GET_SON(ebt,sens),key,fct_order));
  else
    return(keyebt);
}
/**********************************************************************/
static int DeleteNode(pEBTNode *p_ebt, char *key,
           int (*fct_order)(void *, void *), int *res) {
  pEBTNode node = *p_ebt;
  pEBTNode *son;
  short    sens;

  if((sens = (*fct_order)(key,EBT_GET_KEY(node)))) {
    son = EBT_GET_SON_ADDR(node,sens);

    if(*son) {
      if(DeleteNode(son,key,fct_order,res))
        return(Rotate(p_ebt,EBT_ADD_BAL(node,-sens)));

      else
        return(HEIGHT_NO_MODIFIED);
    }
    else {
      *res = FALSE;
      return(HEIGHT_NO_MODIFIED);
    }
  }
  else
    return(DeleteThisNode(p_ebt));
}
/**********************************************************************/
int EBTDeleteNode(pEBTNode *p_ebt, char *key,int (*fct_order)(void *, void *)) {

  if(*p_ebt) {
    int res = TRUE;
    DeleteNode(p_ebt,key,fct_order,&res);
    return(res);
  }
  else
    return(FALSE);
}
/**********************************************************************/
static int FindDeleteFirstNode(pEBTNode *p_ebt,char **key) {
  pEBTNode node  = *p_ebt;
  pEBTNode *left = &EBT_GET_LEFT(node);
  pEBTNode right;

  if(*left) {
    if(FindDeleteFirstNode(left,key))
      return(Rotate(p_ebt,EBT_ADD_BAL(node,R_BAL)));
    else
      return(HEIGHT_NO_MODIFIED);
  } else {
    *key = EBT_GET_KEY(node);

    if((right = EBT_GET_RIGHT(node))) {
      EBT_SET_KEY(node,EBT_GET_KEY(right));
      EBT_DEL_RIGHT(node);
    }
    else {
      MY_FREE(node,EBTNode,1);
      *p_ebt = NULL;
    }

    return(HEIGHT_MODIFIED);
  }
}

/**********************************************************************/
char * EBTFindDeleteFirstNode(pEBTNode *p_ebt) {
  if(*p_ebt) {
    char *res;
    FindDeleteFirstNode(p_ebt,&res);
    return(res);
  }
  else
    return(NULL);
}
/**********************************************************************/
int EBTCheckBalance(pEBTNode ebt) {
  int h_l = 0;
  int h_r = 0;

  if(!ebt)
    return(0);

  if(EBT_GET_LEFT(ebt))
    h_l = EBTCheckBalance(EBT_GET_LEFT(ebt));

  if(EBT_GET_RIGHT(ebt))
    h_r = EBTCheckBalance(EBT_GET_RIGHT(ebt));

  if(h_r - h_l != ebt->bal)
    fprintf(stderr,"Warning: Error in the balance computation\n");

  if(!EBT_BAL_EQUILIBRATED(ebt->bal))
    fprintf(stderr,"Warning: EBT non well balanced\n");

  return(1 + MAX(h_l,h_r));
}

/**********************************************************************/

int EBTHeight(pEBTNode ebt) {
  if(!ebt)
    return(-1);

  return(1 + MAX(EBTHeight(ebt->left),EBTHeight(ebt->right)));
}

/**********************************************************************/

int EBTWidth(pEBTNode ebt) {
  int w;

  if(!ebt)
    return(0);

  w = EBTWidth(EBT_GET_LEFT(ebt)) + EBTWidth(EBT_GET_RIGHT(ebt));

  return(MAX(1,w));
}

/**********************************************************************/

int EBTLength(pEBTNode ebt) {
  int w;

  if(!ebt)
    return(0);

  w = 1 + EBTLength(EBT_GET_LEFT(ebt)) + EBTLength(EBT_GET_RIGHT(ebt));
  return(w);
}
/**********************************************************************/
int EBTShowTree (pEBTNode ebt, void (*print_func)(void *)) {
  if(!ebt)
    return(0);

  (*print_func) ((void*)EBT_GET_KEY(ebt));

  if(EBT_GET_LEFT(ebt))
    EBTShowTree (EBT_GET_LEFT(ebt), print_func);

  if(EBT_GET_RIGHT(ebt))
    EBTShowTree (EBT_GET_RIGHT(ebt), print_func);

  return(1);
}
