#include "atoms.h"
#include "protein.h"

/**********************************************************************/
// FUNCTIONS TO GET BKB ATOMS

atom *get_N(residue *resPt)
{
  switch(resPt->resType) {
  case GLY: 
    return resPt->bkbAlist[GLY_N];
    break;
  case GLYH: 
    return resPt->bkbAlist[GLYH_N];
    break;
  case PRO: 
    return resPt->bkbAlist[PRO_N];
    break;
  case PROH: 
    return resPt->bkbAlist[PROH_N];
    break;
  default:
    if(resPt->flagH == 0)
      return resPt->bkbAlist[gen_N];
    else
      return resPt->bkbAlist[genH_N];
  }
}


atom *get_CA(residue *resPt)
{
  switch(resPt->resType) {
  case GLY: 
    return resPt->bkbAlist[GLY_CA];
    break;
  case GLYH: 
    return resPt->bkbAlist[GLYH_CA];
    break;
  case PRO: 
    return resPt->bkbAlist[PRO_CA];
    break;
  case PROH: 
    return resPt->bkbAlist[PROH_CA];
    break;
  default:
    if(resPt->flagH == 0)
      return resPt->bkbAlist[gen_CA];
    else
      return resPt->bkbAlist[genH_CA];
  }
}


atom *get_C(residue *resPt)
{
  switch(resPt->resType) {
  case GLY: 
    return resPt->bkbAlist[GLY_C];
    break;
  case GLYH: 
    return resPt->bkbAlist[GLYH_C];
    break;
  case PRO: 
    return resPt->bkbAlist[PRO_C];
    break;
  case PROH: 
    return resPt->bkbAlist[PROH_C];
    break;
  default:
    if(resPt->flagH == 0)
      return resPt->bkbAlist[gen_C];
    else
      return resPt->bkbAlist[genH_C];
  }
}

/**********************************************************************/
// FUNCTIONS EXTRACTING GEOMETRIC INFORMATION FORM PROTEIN STRUCTURE

void get_N_pos(residue *resPt, double **apos)
{
  switch(resPt->resType) {
  case GLY: 
    *apos = resPt->bkbAlist[GLY_N]->pos;
    break;
  case GLYH: 
    *apos = resPt->bkbAlist[GLYH_N]->pos;
    break;
  case PRO: 
    *apos = resPt->bkbAlist[PRO_N]->pos;
    break;
  case PROH: 
    *apos = resPt->bkbAlist[PROH_N]->pos;
    break;
  default:
    if(resPt->flagH == 0)
      *apos = resPt->bkbAlist[gen_N]->pos;
    else
      *apos = resPt->bkbAlist[genH_N]->pos;
  }
}

void get_CA_pos(residue *resPt, double **apos)
{
  switch(resPt->resType) {
  case GLY: 
    *apos = resPt->bkbAlist[GLY_CA]->pos;
    break;
  case GLYH: 
    *apos = resPt->bkbAlist[GLYH_CA]->pos;
    break;
  case PRO: 
    *apos = resPt->bkbAlist[PRO_CA]->pos;
    break;
  case PROH: 
    *apos = resPt->bkbAlist[PROH_CA]->pos;
    break;
  default:
    if(resPt->flagH == 0)
      *apos = resPt->bkbAlist[gen_CA]->pos;
    else
      *apos = resPt->bkbAlist[genH_CA]->pos;
  }
}


void get_C_pos(residue *resPt, double **apos)
{
  switch(resPt->resType) {
  case GLY: 
    *apos = resPt->bkbAlist[GLY_C]->pos;
    break;
  case GLYH: 
    *apos = resPt->bkbAlist[GLYH_C]->pos;
    break;
  case PRO: 
    *apos = resPt->bkbAlist[PRO_C]->pos;
    break;
  case PROH: 
    *apos = resPt->bkbAlist[PROH_C]->pos;
    break;
  default:
    if(resPt->flagH == 0)
      *apos = resPt->bkbAlist[gen_C]->pos;
    else
      *apos = resPt->bkbAlist[genH_C]->pos;
  }
}


