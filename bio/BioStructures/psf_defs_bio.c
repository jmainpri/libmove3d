
#include <string.h>

#include "P3d-pkg.h"
#include "Bio-pkg.h"


int resName_to_resType(char *resName, residueTypes *resTypePt)
{
  // consider by default that there are H
  // (the type is modified later if there are no H)
  if(strcmp(resName,"ALA") == 0) { *resTypePt = psf_ALAH; return TRUE; }
  if(strcmp(resName,"ARG") == 0) { *resTypePt = psf_ARGH; return TRUE; }
  if(strcmp(resName,"ASN") == 0) { *resTypePt = psf_ASNH; return TRUE; }
  if(strcmp(resName,"ASP") == 0) { *resTypePt = psf_ASPH; return TRUE; }
  if(strcmp(resName,"CYS") == 0) { *resTypePt = psf_CYSH; return TRUE; }
  if(strcmp(resName,"GLN") == 0) { *resTypePt = psf_GLNH; return TRUE; }
  if(strcmp(resName,"GLU") == 0) { *resTypePt = psf_GLUH; return TRUE; }
  if(strcmp(resName,"GLY") == 0) { *resTypePt = psf_GLYH; return TRUE; }
  if(strcmp(resName,"HIS") == 0) { *resTypePt = psf_HISH; return TRUE; }
  if(strcmp(resName,"ILE") == 0) { *resTypePt = psf_ILEH; return TRUE; }
  if(strcmp(resName,"LEU") == 0) { *resTypePt = psf_LEUH; return TRUE; }
  if(strcmp(resName,"LYS") == 0) { *resTypePt = psf_LYSH; return TRUE; }
  if(strcmp(resName,"MET") == 0) { *resTypePt = psf_METH; return TRUE; }
  if(strcmp(resName,"PHE") == 0) { *resTypePt = psf_PHEH; return TRUE; }
  if(strcmp(resName,"PRO") == 0) { *resTypePt = psf_PROH; return TRUE; }
  if(strcmp(resName,"SER") == 0) { *resTypePt = psf_SERH; return TRUE; }
  if(strcmp(resName,"THR") == 0) { *resTypePt = psf_THRH; return TRUE; }
  if(strcmp(resName,"TRP") == 0) { *resTypePt = psf_TRPH; return TRUE; }
  if(strcmp(resName,"TYR") == 0) { *resTypePt = psf_TYRH; return TRUE; }
  if(strcmp(resName,"VAL") == 0) { *resTypePt = psf_VALH; return TRUE; }
  return FALSE;
}




/**********************************************************************/

int atomName_to_bkbatomindex(char *atomName, atomTypes *aType, residueTypes resType)
{
  switch(resType) {
  case psf_GLYH:
    if(strcmp(atomName,"C") == 0) {*aType = psf_CARBON; return ((int) psf_GLYH_C);}
    if(strcmp(atomName,"O") == 0) {*aType = psf_OXYGEN; return ((int) psf_GLYH_O);}
    if(strcmp(atomName,"CA") == 0) {*aType = psf_CARBON; return ((int) psf_GLYH_CA);}
    if(strcmp(atomName,"1HA") == 0) {*aType = psf_HYDROGEN; return ((int) psf_GLYH_1HA);}
    if(strcmp(atomName,"2HA") == 0) {*aType = psf_HYDROGEN; return ((int) psf_GLYH_2HA);}
    if(strcmp(atomName,"N") == 0) {*aType = psf_NITROGEN_H; return ((int) psf_GLYH_N);}
    if(strcmp(atomName,"1H") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_GLYH_H);}   
    if(strcmp(atomName,"2H") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_GLYH_H_Nterm);}   
    if(strcmp(atomName,"H") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_GLYH_H);}
    if(strcmp(atomName,"OXT") == 0) {*aType = psf_OXYGEN; return ((int) psf_GLYH_OXT);}
    return -1;
  case psf_PROH:
    if(strcmp(atomName,"C") == 0) {*aType = psf_CARBON; return ((int) psf_PROH_C);}
    if(strcmp(atomName,"O") == 0) {*aType = psf_OXYGEN; return ((int) psf_PROH_O);}
    if(strcmp(atomName,"CA") == 0) {*aType = psf_CARBON; return ((int) psf_PROH_CA);}
    if(strcmp(atomName,"HA") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PROH_HA);}   
    if(strcmp(atomName,"CB") == 0) {*aType = psf_CARBON; return ((int) psf_PROH_CB);}   
    if(strcmp(atomName,"N") == 0) {*aType = psf_NITROGEN_FULL; return ((int) psf_PROH_N);}   
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PROH_1HB);}   
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PROH_2HB);}   
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_PROH_CG);}   
    if(strcmp(atomName,"1HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PROH_1HG);}   
    if(strcmp(atomName,"2HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PROH_2HG);}   
    if(strcmp(atomName,"CD") == 0) {*aType = psf_CARBON; return ((int) psf_PROH_CD);}   
    if(strcmp(atomName,"1HD") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PROH_1HD);}   
    if(strcmp(atomName,"2HD") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PROH_2HD);}   
    if(strcmp(atomName,"OXT") == 0) {*aType = psf_OXYGEN; return ((int) psf_PROH_OXT);}   
    return -1;
  default:   // genH
    if(strcmp(atomName,"C") == 0) {*aType = psf_CARBON; return ((int) psf_genH_C);}
    if(strcmp(atomName,"O") == 0) {*aType = psf_OXYGEN; return ((int) psf_genH_O);}
    if(strcmp(atomName,"CA") == 0) {*aType = psf_CARBON; return ((int) psf_genH_CA);}
    if(strcmp(atomName,"HA") == 0) {*aType = psf_HYDROGEN; return ((int) psf_genH_HA);}   
    if(strcmp(atomName,"CB") == 0) {*aType = psf_CARBON; return ((int) psf_genH_CB);}   
    if(strcmp(atomName,"N") == 0) {*aType = psf_NITROGEN_H; return ((int) psf_genH_N);}       
    if(strcmp(atomName,"OXT") == 0) {*aType = psf_OXYGEN; return ((int) psf_genH_OXT);}      
    if(strcmp(atomName,"1H") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_genH_H);}   
    if(strcmp(atomName,"2H") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_genH_H_Nterm);}   
    if(strcmp(atomName,"H") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_genH_H);}   
    // some pdb files indicate a 3H atom. It is not considered here
    // (warning message).
    return -1;
  }
}

/**********************************************************************/

int atomName_to_schatomindex(char *atomName, atomTypes *aType, residueTypes resType)
{
  switch(resType) {
  case psf_ALAH:
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ALAH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ALAH_2HB);}
    if(strcmp(atomName,"3HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ALAH_3HB);}
    return -1;
  case psf_ARGH:
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_ARGH_CG);}
    if(strcmp(atomName,"CD") == 0) {*aType = psf_CARBON; return ((int) psf_ARGH_CD);}
    if(strcmp(atomName,"NE") == 0) {*aType = psf_NITROGEN_H; return ((int) psf_ARGH_NE);}
    if(strcmp(atomName,"CZ") == 0) {*aType = psf_CARBON; return ((int) psf_ARGH_CZ);}
    if(strcmp(atomName,"NH1") == 0) {*aType = psf_NITROGEN_H; return ((int) psf_ARGH_NH1);}
    if(strcmp(atomName,"NH2") == 0) {*aType = psf_NITROGEN_H; return ((int) psf_ARGH_NH2);}
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ARGH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ARGH_2HB);}
    if(strcmp(atomName,"1HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ARGH_1HG);}
    if(strcmp(atomName,"2HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ARGH_2HG);}
    if(strcmp(atomName,"1HD") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ARGH_1HD);}
    if(strcmp(atomName,"2HD") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ARGH_2HD);}
    if(strcmp(atomName,"1HH1") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_ARGH_1HH1);}
    if(strcmp(atomName,"2HH1") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_ARGH_2HH1);}
    if(strcmp(atomName,"1HH2") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_ARGH_1HH2);}
    if(strcmp(atomName,"2HH2") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_ARGH_2HH2);} 
    if(strcmp(atomName,"HE") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_ARGH_HE);} 
    return -1;
  case psf_ASNH:
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_ASNH_CG);}
    if(strcmp(atomName,"OD1") == 0) {*aType = psf_OXYGEN; return ((int) psf_ASNH_OD1);}
    if(strcmp(atomName,"ND2") == 0) {*aType = psf_NITROGEN_H; return ((int) psf_ASNH_ND2);}
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ASNH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ASNH_2HB);}
    if(strcmp(atomName,"1HD2") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_ASNH_1HD2);}
    if(strcmp(atomName,"2HD2") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_ASNH_2HD2);}
    return -1;
  case psf_ASPH:
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_ASPH_CG);}
    if(strcmp(atomName,"OD1") == 0) {*aType = psf_OXYGEN; return ((int) psf_ASPH_OD1);}
    if(strcmp(atomName,"OD2") == 0) {*aType = psf_OXYGEN_H; return ((int) psf_ASPH_OD2);} 
    /* OD1 or OD2 is associated to an hydrogen. I assume is OD2 */
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ASPH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ASPH_2HB);}
    return -1;
  case psf_CYSH:
    if(strcmp(atomName,"SG") == 0) {*aType = psf_SULPHUR; return ((int) psf_CYSH_SG);}
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_CYSH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_CYSH_2HB);}
    if(strcmp(atomName,"HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_CYSH_HG);}
    return -1;
  case psf_GLNH:
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_GLNH_CG);}
    if(strcmp(atomName,"CD") == 0) {*aType = psf_CARBON; return ((int) psf_GLNH_CD);}
    if(strcmp(atomName,"OE1") == 0) {*aType = psf_OXYGEN; return ((int) psf_GLNH_OE1);}
    if(strcmp(atomName,"NE2") == 0) {*aType = psf_NITROGEN_H; return ((int) psf_GLNH_NE2);}
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_GLNH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_GLNH_2HB);}
    if(strcmp(atomName,"1HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_GLNH_1HG);}
    if(strcmp(atomName,"2HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_GLNH_2HG);}
    if(strcmp(atomName,"1HE2") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_GLNH_1HE2);}
    if(strcmp(atomName,"2HE2") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_GLNH_2HE2);}
    return -1;
  case psf_GLUH:
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_GLUH_CG);}
    if(strcmp(atomName,"CD") == 0) {*aType = psf_CARBON; return ((int) psf_GLUH_CD);}
    if(strcmp(atomName,"OE1") == 0) {*aType = psf_OXYGEN; return ((int) psf_GLUH_OE1);}
    if(strcmp(atomName,"OE2") == 0) {*aType = psf_OXYGEN_H; return ((int) psf_GLUH_OE2);}
    /* OE1 or OE2 is associated to an hydrogen. I assume it is OE2 */
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_GLUH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_GLUH_2HB);}
    if(strcmp(atomName,"1HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_GLUH_1HG);}
    if(strcmp(atomName,"2HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_GLUH_2HG);}
    return -1;
  //case GLYH:
  // no sch atoms
  case psf_HISH:
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_HISH_CG);}
    if(strcmp(atomName,"ND1") == 0) {*aType = psf_NITROGEN_H; return ((int) psf_HISH_ND1);}
    if(strcmp(atomName,"CD2") == 0) {*aType = psf_CARBON; return ((int) psf_HISH_CD2);}
    if(strcmp(atomName,"CE1") == 0) {*aType = psf_CARBON; return ((int) psf_HISH_CE1);}
    if(strcmp(atomName,"NE2") == 0) {*aType = psf_NITROGEN_H; return ((int) psf_HISH_NE2);}
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_HISH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_HISH_2HB);}
    if(strcmp(atomName,"HD2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_HISH_HD2);}
    if(strcmp(atomName,"HE1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_HISH_HE1);}
    if(strcmp(atomName,"HE2") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_HISH_HE2);}
    if(strcmp(atomName,"HD1") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_HISH_HD1);}
    return -1;
  case psf_ILEH:
    if(strcmp(atomName,"CG1") == 0) {*aType = psf_CARBON; return ((int) psf_ILEH_CG1);}
    if(strcmp(atomName,"CG2") == 0) {*aType = psf_CARBON; return ((int) psf_ILEH_CG2);}
    if(strcmp(atomName,"CD1") == 0) {*aType = psf_CARBON; return ((int) psf_ILEH_CD1);}
    if(strcmp(atomName,"HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ILEH_HB);}
    if(strcmp(atomName,"1HG2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ILEH_1HG2);}
    if(strcmp(atomName,"2HG2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ILEH_2HG2);}
    if(strcmp(atomName,"3HG2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ILEH_3HG2);}
    if(strcmp(atomName,"1HG1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ILEH_1HG1);}
    if(strcmp(atomName,"2HG1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ILEH_2HG1);}
    if(strcmp(atomName,"1HD1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ILEH_1HD1);}
    if(strcmp(atomName,"2HD1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ILEH_2HD1);}
    if(strcmp(atomName,"3HD1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_ILEH_3HD1);}
    return -1;
  case psf_LEUH:
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_LEUH_CG);}
    if(strcmp(atomName,"CD1") == 0) {*aType = psf_CARBON; return ((int) psf_LEUH_CD1);}
    if(strcmp(atomName,"CD2") == 0) {*aType = psf_CARBON; return ((int) psf_LEUH_CD2);}
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LEUH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LEUH_2HB);}
    if(strcmp(atomName,"HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LEUH_HG);}
    if(strcmp(atomName,"1HD1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LEUH_1HD1);}
    if(strcmp(atomName,"2HD1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LEUH_2HD1);}
    if(strcmp(atomName,"3HD1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LEUH_3HD1);}
    if(strcmp(atomName,"1HD2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LEUH_1HD2);}
    if(strcmp(atomName,"2HD2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LEUH_2HD2);}
    if(strcmp(atomName,"3HD2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LEUH_3HD2);}
    return -1;
  case psf_LYSH:
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_LYSH_CG);} 
    if(strcmp(atomName,"CD") == 0) {*aType = psf_CARBON; return ((int) psf_LYSH_CD);}
    if(strcmp(atomName,"CE") == 0) {*aType = psf_CARBON; return ((int) psf_LYSH_CE);}
    if(strcmp(atomName,"NZ") == 0) {*aType = psf_NITROGEN_H; return ((int) psf_LYSH_NZ);}
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LYSH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LYSH_2HB);}
    if(strcmp(atomName,"1HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LYSH_1HG);}
    if(strcmp(atomName,"2HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LYSH_2HG);}
    if(strcmp(atomName,"1HD") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LYSH_1HD);}
    if(strcmp(atomName,"2HD") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LYSH_2HD);}
    if(strcmp(atomName,"1HE") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LYSH_1HE);}
    if(strcmp(atomName,"2HE") == 0) {*aType = psf_HYDROGEN; return ((int) psf_LYSH_2HE);}
    if(strcmp(atomName,"1HZ") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_LYSH_1HZ);}
    if(strcmp(atomName,"2HZ") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_LYSH_2HZ);}
    if(strcmp(atomName,"3HZ") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_LYSH_3HZ);}
    return -1;
  case psf_METH:
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_METH_CG);}
    if(strcmp(atomName,"SD") == 0) {*aType = psf_SULPHUR; return ((int) psf_METH_SD);}
    if(strcmp(atomName,"CE") == 0) {*aType = psf_CARBON; return ((int) psf_METH_CE);}
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_METH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_METH_2HB);}
    if(strcmp(atomName,"1HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_METH_1HG);}
    if(strcmp(atomName,"2HG") == 0) {*aType = psf_HYDROGEN; return ((int) psf_METH_2HG);}
    if(strcmp(atomName,"1HE") == 0) {*aType = psf_HYDROGEN; return ((int) psf_METH_1HE);}
    if(strcmp(atomName,"2HE") == 0) {*aType = psf_HYDROGEN; return ((int) psf_METH_2HE);}
    if(strcmp(atomName,"3HE") == 0) {*aType = psf_HYDROGEN; return ((int) psf_METH_3HE);}
    return -1;
  case psf_PHEH:
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_PHEH_CG);}
    if(strcmp(atomName,"CD1") == 0) {*aType = psf_CARBON; return ((int) psf_PHEH_CD1);}
    if(strcmp(atomName,"CD2") == 0) {*aType = psf_CARBON; return ((int) psf_PHEH_CD2);}
    if(strcmp(atomName,"CE1") == 0) {*aType = psf_CARBON; return ((int) psf_PHEH_CE1);}
    if(strcmp(atomName,"CE2") == 0) {*aType = psf_CARBON; return ((int) psf_PHEH_CE2);}
    if(strcmp(atomName,"CZ") == 0) {*aType = psf_CARBON; return ((int) psf_PHEH_CZ);}
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PHEH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PHEH_2HB);}
    if(strcmp(atomName,"HD1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PHEH_HD1);}
    if(strcmp(atomName,"HD2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PHEH_HD2);}
    if(strcmp(atomName,"HE1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PHEH_HE1);}
    if(strcmp(atomName,"HE2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PHEH_HE2);}
    if(strcmp(atomName,"HZ") == 0) {*aType = psf_HYDROGEN; return ((int) psf_PHEH_HZ);}
    return -1;
  //case PROH:
  // no sch atoms
  case psf_SERH:
    if(strcmp(atomName,"OG") == 0) {*aType = psf_OXYGEN_H; return ((int) psf_SERH_OG);}
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_SERH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_SERH_2HB);}
    if(strcmp(atomName,"HG") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_SERH_HG);}
    return -1;
  case psf_THRH:
    if(strcmp(atomName,"OG1") == 0) {*aType = psf_OXYGEN_H; return ((int) psf_THRH_OG1);}
    if(strcmp(atomName,"CG2") == 0) {*aType = psf_CARBON; return ((int) psf_THRH_CG2);}
    if(strcmp(atomName,"HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_THRH_HB);}
    if(strcmp(atomName,"HG1") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_THRH_HG1);}
    if(strcmp(atomName,"1HG2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_THRH_1HG2);}
    if(strcmp(atomName,"2HG2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_THRH_2HG2);}
    if(strcmp(atomName,"3HG2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_THRH_3HG2);}
    return -1;
  case psf_TRPH:
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_TRPH_CG);}
    if(strcmp(atomName,"CD1") == 0) {*aType = psf_CARBON; return ((int) psf_TRPH_CD1);}
    if(strcmp(atomName,"CD2") == 0) {*aType = psf_CARBON; return ((int) psf_TRPH_CD2);}
    if(strcmp(atomName,"NE1") == 0) {*aType = psf_NITROGEN_H; return ((int) psf_TRPH_NE1);}
    if(strcmp(atomName,"CE2") == 0) {*aType = psf_CARBON; return ((int) psf_TRPH_CE2);}
    if(strcmp(atomName,"CE3") == 0) {*aType = psf_CARBON; return ((int) psf_TRPH_CE3);}
    if(strcmp(atomName,"CZ2") == 0) {*aType = psf_CARBON; return ((int) psf_TRPH_CZ2);}
    if(strcmp(atomName,"CZ3") == 0) {*aType = psf_CARBON; return ((int) psf_TRPH_CZ3);}
    if(strcmp(atomName,"CH2") == 0) {*aType = psf_CARBON; return ((int) psf_TRPH_CH2);}
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TRPH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TRPH_2HB);}
    if(strcmp(atomName,"HD1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TRPH_HD1);}
    if(strcmp(atomName,"HE1") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_TRPH_HE1);}
    if(strcmp(atomName,"HE3") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TRPH_HE3);}
    if(strcmp(atomName,"HZ2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TRPH_HZ2);}
    if(strcmp(atomName,"HZ3") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TRPH_HZ3);}
    if(strcmp(atomName,"HH2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TRPH_HH2);}
    return -1;
  case psf_TYRH:
    if(strcmp(atomName,"CG") == 0) {*aType = psf_CARBON; return ((int) psf_TYRH_CG);}
    if(strcmp(atomName,"CD1") == 0) {*aType = psf_CARBON; return ((int) psf_TYRH_CD1);}
    if(strcmp(atomName,"CD2") == 0) {*aType = psf_CARBON; return ((int) psf_TYRH_CD2);}
    if(strcmp(atomName,"CE1") == 0) {*aType = psf_CARBON; return ((int) psf_TYRH_CE1);}
    if(strcmp(atomName,"CE2") == 0) {*aType = psf_CARBON; return ((int) psf_TYRH_CE2);}
    if(strcmp(atomName,"CZ") == 0) {*aType = psf_CARBON; return ((int) psf_TYRH_CZ);}
    if(strcmp(atomName,"OH") == 0) {*aType = psf_OXYGEN_H; return ((int) psf_TYRH_OH);}
    if(strcmp(atomName,"1HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TYRH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TYRH_2HB);}
    if(strcmp(atomName,"HD1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TYRH_HD1);}
    if(strcmp(atomName,"HD2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TYRH_HD2);}
    if(strcmp(atomName,"HE1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TYRH_HE1);}
    if(strcmp(atomName,"HE2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_TYRH_HE2);}
    if(strcmp(atomName,"HH") == 0) {*aType = psf_HYDROGEN_P; return ((int) psf_TYRH_HH);}
    return -1;
  case psf_VALH:
    if(strcmp(atomName,"CG1") == 0) {*aType = psf_CARBON; return ((int) psf_VALH_CG1);}
    if(strcmp(atomName,"CG2") == 0) {*aType = psf_CARBON; return ((int) psf_VALH_CG2);}
    if(strcmp(atomName,"HB") == 0) {*aType = psf_HYDROGEN; return ((int) psf_VALH_HB);}
    if(strcmp(atomName,"1HG1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_VALH_1HG1);}
    if(strcmp(atomName,"2HG1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_VALH_2HG1);}
    if(strcmp(atomName,"3HG1") == 0) {*aType = psf_HYDROGEN; return ((int) psf_VALH_3HG1);}
    if(strcmp(atomName,"1HG2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_VALH_1HG2);}
    if(strcmp(atomName,"2HG2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_VALH_2HG2);}
    if(strcmp(atomName,"3HG2") == 0) {*aType = psf_HYDROGEN; return ((int) psf_VALH_3HG2);}
    return -1;
  default:
    return -1;    
  }

  return -1;
}

/**********************************************************************/


int number_bkb_atoms(residueTypes resType)
{
  switch(resType) {
  case psf_GLY:
    return N_GLY_BKB_ATOMS;
  case psf_GLYH:
    return N_GLYH_BKB_ATOMS;
  case psf_PRO:
    return N_PRO_BKB_ATOMS;
  case psf_PROH:
    return N_PROH_BKB_ATOMS;      
  default:
    if(((int) resType) < 20)
      return N_GEN_BKB_ATOMS;      
    else
      return N_GENH_BKB_ATOMS;      
  }
}

/**********************************************************************/

int number_sch_atoms(residueTypes resType)
{
  switch(resType) {
  case psf_ALA:
    return N_ALA_SCH_ATOMS;
  case psf_ARG:
    return N_ARG_SCH_ATOMS;
  case psf_ASN:
    return N_ASN_SCH_ATOMS;
  case psf_ASP:
    return N_ASP_SCH_ATOMS;
  case psf_CYS:
    return N_CYS_SCH_ATOMS;
  case psf_GLN:
    return N_GLN_SCH_ATOMS;
  case psf_GLU:
    return N_GLU_SCH_ATOMS;
  case psf_GLY:
    return N_GLY_SCH_ATOMS;
  case psf_HIS:
    return N_HIS_SCH_ATOMS;
  case psf_ILE:
    return N_ILE_SCH_ATOMS;
  case psf_LEU:
    return N_LEU_SCH_ATOMS;
  case psf_LYS:
    return N_LYS_SCH_ATOMS;
  case psf_MET:
    return N_MET_SCH_ATOMS;
  case psf_PHE:
    return N_PHE_SCH_ATOMS;
  case psf_PRO:
    return N_PRO_SCH_ATOMS;
  case psf_SER:
    return N_SER_SCH_ATOMS;
  case psf_THR:
    return N_THR_SCH_ATOMS;
  case psf_TRP:
    return N_TRP_SCH_ATOMS;
  case psf_TYR:
    return N_TYR_SCH_ATOMS;
  case psf_VAL:
    return N_VAL_SCH_ATOMS;
  case psf_ALAH:
    return N_ALAH_SCH_ATOMS;
  case psf_ARGH:
    return N_ARGH_SCH_ATOMS;
  case psf_ASNH:
    return N_ASNH_SCH_ATOMS;
  case psf_ASPH:
    return N_ASPH_SCH_ATOMS;
  case psf_CYSH:
    return N_CYSH_SCH_ATOMS;
  case psf_GLNH:
    return N_GLNH_SCH_ATOMS;
  case psf_GLUH:
    return N_GLUH_SCH_ATOMS;
  case psf_GLYH:
    return N_GLYH_SCH_ATOMS;
  case psf_HISH:
    return N_HISH_SCH_ATOMS;
  case psf_ILEH:
    return N_ILEH_SCH_ATOMS;
  case psf_LEUH:
    return N_LEUH_SCH_ATOMS;
  case psf_LYSH:
    return N_LYSH_SCH_ATOMS;
  case psf_METH:
    return N_METH_SCH_ATOMS;
  case psf_PHEH:
    return N_PHEH_SCH_ATOMS;
  case psf_PROH:
    return N_PROH_SCH_ATOMS;
  case psf_SERH:
    return N_SERH_SCH_ATOMS;
  case psf_THRH:
    return N_THRH_SCH_ATOMS;
  case psf_TRPH:
    return N_TRPH_SCH_ATOMS;
  case psf_TYRH:
    return N_TYRH_SCH_ATOMS;
  case psf_VALH:
    return N_VALH_SCH_ATOMS;
  }
  return -1;
}

/**********************************************************************/

int elementName_to_lig_AtomType(char *atomName, atomTypes *atomTypePt)
{
  if(strcmp(atomName,"C") == 0) { *atomTypePt = psf_CARBON; return 1; }
  if(strcmp(atomName,"O") == 0) { *atomTypePt = psf_OXYGEN; return 1; }
  if(strcmp(atomName,"N") == 0) { *atomTypePt = psf_NITROGEN; return 1; }
  if(strcmp(atomName,"S") == 0) { *atomTypePt = psf_SULPHUR; return 1; }
  if(strcmp(atomName,"H") == 0) { *atomTypePt = psf_HYDROGEN; return 1; }
  if(strcmp(atomName,"F") == 0) { *atomTypePt = psf_FLUORINE; return 1; }
  if(strcmp(atomName,"Br") == 0) { *atomTypePt = psf_BROMINE; return 1; }
  if(strcmp(atomName,"Cl") == 0) { *atomTypePt = psf_CHLORINE; return 1; }
  if(strcmp(atomName,"P") == 0) { *atomTypePt = psf_PHOSPHORUS; return 1; }
  if(strcmp(atomName,"I") == 0) { *atomTypePt = psf_IODINE; return 1; }
  printf("ERROR : atom type %s is not defined",atomName);
  return -1;
}

/**********************************************************************/

int get_vdwR_by_type(atomTypes atomType, double* vdwR)
{
  switch(atomType) {
  case psf_CARBON:
    *vdwR = C_VDWR;
    return TRUE;

  case psf_NITROGEN:
  case psf_NITROGEN_H:
  case psf_NITROGEN_FULL:
   *vdwR = N_VDWR;
     return TRUE;
 
  case psf_OXYGEN:
  case psf_OXYGEN_H:
    *vdwR = O_VDWR;
    return TRUE;
  
  case psf_SULPHUR:
  case psf_SULPHUR_H:
    *vdwR = S_VDWR;
     return TRUE;
 
  case psf_HYDROGEN:
    *vdwR = H_VDWR;
    return TRUE;
  
  case psf_FLUORINE: 
    *vdwR = F_VDWR;    
     return TRUE;
 
  case psf_BROMINE:
    *vdwR = Br_VDWR;
    return TRUE;
  
  case psf_CHLORINE:
    *vdwR = Cl_VDWR;   
    return TRUE;
  
  case psf_PHOSPHORUS:
    *vdwR = P_VDWR;   
    return TRUE;
  
  case psf_IODINE:
    *vdwR = I_VDWR;   
    return TRUE;

  default:
    return FALSE;
  }  
}
