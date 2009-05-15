// INCLUDES


#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "defs.h"
#include "atoms.h"
#include "protein.h"
#include "atomhandling.h"
#include "mathfcts.h"
#include "pdbFormat.h"

#define TRUE 1
#define FALSE 0

/**********************************************************************/
// ATOM DATA IN PDB

typedef struct s_pdb_atom_data {
  int serial;
  char name[5];
  atomTypes type;
  int resSeq;
  char resName[4];
  residueTypes resType;    // <- index corresponding to resName
  char chainID[2];
  double pos[3];
} pdb_atom_data;

// INTERMEDIATE AMINO-ACID STRUCTURE

typedef struct s_unknown_atom {
  int serial;
  char name[5];
  double pos[3];
} unknown_atom;

typedef struct s_res_atoms {
  char chainID[2];
  char resName[4];  
  residueTypes resType;
  int resSeq;
  int nbkbatoms;
  int nschatoms;
  int n_unknownAtoms;
  struct s_atom **bkbatoms;
  struct s_atom **schatoms; 
  struct s_unknown_atom **unknownAtoms;
} res_atoms;


/**********************************************************************/
// GLOBAL VARIABLES

// number of read lines in PDF file 
static int npdfline = 0;

// flag indicating first time reading pdb file
static int first_time_reading = 1;

// flag indicating that OXT has ben read
static int read_OXT = 0;

// first atom read in a residue
static pdb_atom_data firstatom;

/**********************************************************************/
// LIST MANAGEMENT

static void insert_pointer_in_list(void *thePt, void ***listPt, int *nelems)
{
  if(*nelems == 0) {
    *listPt = (void **) malloc(sizeof(void *));
  }
  else {
    *listPt = (void **) realloc(*listPt,sizeof(void *) * (*nelems + 1));
  }
  (*listPt)[*nelems] = thePt;
  (*nelems)++;
}


/**********************************************************************/
/**********************************************************************/
// PDB TO PROTEIN STRUCTURE
/**********************************************************************/
/**********************************************************************/

// FUNCTIONS IDENTIFYING ATOMS AND RESIDUES

static int resName_to_resType(char *resName, residueTypes *resTypePt)
{
  // consider by default that there are H
  // (the type is modified later if there are no H)
  if(strcmp(resName,"ALA") == 0) { *resTypePt = ALAH; return 1; }
  if(strcmp(resName,"ARG") == 0) { *resTypePt = ARGH; return 1; }
  if(strcmp(resName,"ASN") == 0) { *resTypePt = ASNH; return 1; }
  if(strcmp(resName,"ASP") == 0) { *resTypePt = ASPH; return 1; }
  if(strcmp(resName,"CYS") == 0) { *resTypePt = CYSH; return 1; }
  if(strcmp(resName,"GLN") == 0) { *resTypePt = GLNH; return 1; }
  if(strcmp(resName,"GLU") == 0) { *resTypePt = GLUH; return 1; }
  if(strcmp(resName,"GLY") == 0) { *resTypePt = GLYH; return 1; }
  if(strcmp(resName,"HIS") == 0) { *resTypePt = HISH; return 1; }
  if(strcmp(resName,"ILE") == 0) { *resTypePt = ILEH; return 1; }
  if(strcmp(resName,"LEU") == 0) { *resTypePt = LEUH; return 1; }
  if(strcmp(resName,"LYS") == 0) { *resTypePt = LYSH; return 1; }
  if(strcmp(resName,"MET") == 0) { *resTypePt = METH; return 1; }
  if(strcmp(resName,"PHE") == 0) { *resTypePt = PHEH; return 1; }
  if(strcmp(resName,"PRO") == 0) { *resTypePt = PROH; return 1; }
  if(strcmp(resName,"SER") == 0) { *resTypePt = SERH; return 1; }
  if(strcmp(resName,"THR") == 0) { *resTypePt = THRH; return 1; }
  if(strcmp(resName,"TRP") == 0) { *resTypePt = TRPH; return 1; }
  if(strcmp(resName,"TYR") == 0) { *resTypePt = TYRH; return 1; }
  if(strcmp(resName,"VAL") == 0) { *resTypePt = VALH; return 1; }
  return -1;
}

/**********************************************************************/

static void noHresidueType(residue *resPt)
{
  switch(resPt->resType) {
  case ALAH: 
    resPt->resType = ALA;
    break;
  case ARGH: 
    resPt->resType = ARG;
    break;    
  case ASNH: 
    resPt->resType = ASN;
    break;
  case ASPH: 
    resPt->resType = ASP;
    break;
  case CYSH: 
    resPt->resType = CYS;
    break;
  case GLNH: 
    resPt->resType = GLN;
    break;
  case GLUH: 
    resPt->resType = GLU;
    break;
  case GLYH: 
    resPt->resType = GLY;
    break;
  case HISH: 
    resPt->resType = HIS;
    break;
  case ILEH: 
    resPt->resType = ILE;
    break;
  case LEUH: 
    resPt->resType = LEU;
    break;
  case LYSH: 
    resPt->resType = LYS;
    break;
  case METH: 
    resPt->resType = MET;
    break;
  case PHEH: 
    resPt->resType = PHE;
    break;
  case PROH: 
    resPt->resType = PRO;
    break;
  case SERH: 
    resPt->resType = SER;
    break;
  case THRH: 
    resPt->resType = THR;
    break;
  case TRPH: 
    resPt->resType = TRP;
    break;
  case TYRH: 
    resPt->resType = TYR;
    break;
  case VALH: 
    resPt->resType = VAL;
    break;
  default:
    break;
  }
}

/**********************************************************************/

static int is_H_OXT_index(int index, residueTypes resType) {
  switch (resType) {
  case GLYH: return (index==(int)GLYH_OXT);
  case PROH: return (index==(int)PROH_OXT);
  default :
    return (index==(int)genH_OXT);
  }
}

/**********************************************************************/

static int is_OXT_index(int index, residueTypes resType) {
  switch (resType) {
  case GLY: return (index==(int)GLY_OXT);
  case GLYH: return (index==(int)GLYH_OXT);
  case PRO: return (index==(int)PRO_OXT);
  case PROH: return (index==(int)PROH_OXT);
  default :
    if(((int) resType) < 20)
      return (index==(int)gen_OXT);
    else
      return (index==(int)genH_OXT);
  }
}

/**********************************************************************/

static int atomName_to_bkbatomindex(char *atomName, atomTypes *aType, residueTypes resType)
{
  switch(resType) {
  case GLYH:
    if(strcmp(atomName,"C") == 0) {*aType = CARBON; return ((int) GLYH_C);}
    if(strcmp(atomName,"O") == 0) {*aType = OXYGEN; return ((int) GLYH_O);}
    if(strcmp(atomName,"CA") == 0) {*aType = CARBON; return ((int) GLYH_CA);}
    if(strcmp(atomName,"1HA") == 0) {*aType = HYDROGEN; return ((int) GLYH_1HA);}        // 2HA
    if(strcmp(atomName,"2HA") == 0) {*aType = HYDROGEN; return ((int) GLYH_2HA);}        // 3HA
    if(strcmp(atomName,"N") == 0) {*aType = NITROGEN_H; return ((int) GLYH_N);}
    if(strcmp(atomName,"OXT") == 0) {*aType = OXYGEN; return ((int) GLYH_OXT);}
    return -1;
  case PROH:
    if(strcmp(atomName,"C") == 0) {*aType = CARBON; return ((int) PROH_C);}
    if(strcmp(atomName,"O") == 0) {*aType = OXYGEN; return ((int) PROH_O);}
    if(strcmp(atomName,"CA") == 0) {*aType = CARBON; return ((int) PROH_CA);}
    if(strcmp(atomName,"HA") == 0) {*aType = HYDROGEN; return ((int) PROH_HA);}   
    if(strcmp(atomName,"CB") == 0) {*aType = CARBON; return ((int) PROH_CB);}   
    if(strcmp(atomName,"N") == 0) {*aType = NITROGEN_FULL; return ((int) PROH_N);}   
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) PROH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) PROH_2HB);}        // 3HB
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) PROH_CG);}   
    if(strcmp(atomName,"1HG") == 0) {*aType = HYDROGEN; return ((int) PROH_1HG);}        // 2HG   
    if(strcmp(atomName,"2HG") == 0) {*aType = HYDROGEN; return ((int) PROH_2HG);}        // 3HG   
    if(strcmp(atomName,"CD") == 0) {*aType = CARBON; return ((int) PROH_CD);}   
    if(strcmp(atomName,"1HD") == 0) {*aType = HYDROGEN; return ((int) PROH_1HD);}        // 2HD   
    if(strcmp(atomName,"2HD") == 0) {*aType = HYDROGEN; return ((int) PROH_2HD);}        // 3HD   
    if(strcmp(atomName,"OXT") == 0) {*aType = OXYGEN; return ((int) PROH_OXT);}          // H N-TERM ???
    return -1;
  default:   // genH
    if(strcmp(atomName,"C") == 0) {*aType = CARBON; return ((int) genH_C);}
    if(strcmp(atomName,"O") == 0) {*aType = OXYGEN; return ((int) genH_O);}
    if(strcmp(atomName,"CA") == 0) {*aType = CARBON; return ((int) genH_CA);}
    if(strcmp(atomName,"HA") == 0) {*aType = HYDROGEN; return ((int) genH_HA);}   
    if(strcmp(atomName,"CB") == 0) {*aType = CARBON; return ((int) genH_CB);}   
    if(strcmp(atomName,"N") == 0) {*aType = NITROGEN_H; return ((int) genH_N);}       
    if(strcmp(atomName,"OXT") == 0) {*aType = OXYGEN; return ((int) genH_OXT);}      
    // some pdb files indicate a 3H atom. It is not considered here
    // (warning message).
    return -1;
  }
}

/**********************************************************************/

static int atomName_to_schatomindex(char *atomName, atomTypes *aType, residueTypes resType)
{
  switch(resType) {
  case ALAH:
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) ALAH_1HB);}
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) ALAH_2HB);}
    if(strcmp(atomName,"3HB") == 0) {*aType = HYDROGEN; return ((int) ALAH_3HB);}
    return -1;
  case ARGH:
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) ARGH_CG);}
    if(strcmp(atomName,"CD") == 0) {*aType = CARBON; return ((int) ARGH_CD);}
    if(strcmp(atomName,"NE") == 0) {*aType = NITROGEN_H; return ((int) ARGH_NE);}
    if(strcmp(atomName,"CZ") == 0) {*aType = CARBON; return ((int) ARGH_CZ);}
    if(strcmp(atomName,"NH1") == 0) {*aType = NITROGEN_H; return ((int) ARGH_NH1);}
    if(strcmp(atomName,"NH2") == 0) {*aType = NITROGEN_H; return ((int) ARGH_NH2);}
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) ARGH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) ARGH_2HB);}        // 3HB
    if(strcmp(atomName,"1HG") == 0) {*aType = HYDROGEN; return ((int) ARGH_1HG);}        // 2HG
    if(strcmp(atomName,"2HG") == 0) {*aType = HYDROGEN; return ((int) ARGH_2HG);}        // 3HG
    if(strcmp(atomName,"1HD") == 0) {*aType = HYDROGEN; return ((int) ARGH_1HD);}        // 2HD
    if(strcmp(atomName,"2HD") == 0) {*aType = HYDROGEN; return ((int) ARGH_2HD);}        // 3HD
    return -1;
  case ASNH:
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) ASNH_CG);}
    if(strcmp(atomName,"OD1") == 0) {*aType = OXYGEN; return ((int) ASNH_OD1);}
    if(strcmp(atomName,"ND2") == 0) {*aType = NITROGEN_H; return ((int) ASNH_ND2);}
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) ASNH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) ASNH_2HB);}        // 3HB
    return -1;
  case ASPH:
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) ASPH_CG);}
    if(strcmp(atomName,"OD1") == 0) {*aType = OXYGEN; return ((int) ASPH_OD1);}
    if(strcmp(atomName,"OD2") == 0) {*aType = OXYGEN_H; return ((int) ASPH_OD2);} 
    /* OD1 or OD2 is associated to an hydrogen. I assume is OD2 */
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) ASPH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) ASPH_2HB);}        // 3HB
    return -1;
  case CYSH:
    if(strcmp(atomName,"SG") == 0) {*aType = SULPHUR; return ((int) CYSH_SG);}
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) CYSH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) CYSH_2HB);}        // 3HB
    return -1;
  case GLNH:
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) GLNH_CG);}
    if(strcmp(atomName,"CD") == 0) {*aType = CARBON; return ((int) GLNH_CD);}
    if(strcmp(atomName,"OE1") == 0) {*aType = OXYGEN; return ((int) GLNH_OE1);}
    if(strcmp(atomName,"NE2") == 0) {*aType = NITROGEN_H; return ((int) GLNH_NE2);}
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) GLNH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) GLNH_2HB);}        // 3HB
    if(strcmp(atomName,"1HG") == 0) {*aType = HYDROGEN; return ((int) GLNH_1HG);}        // 2HG
    if(strcmp(atomName,"2HG") == 0) {*aType = HYDROGEN; return ((int) GLNH_2HG);}        // 3HG
    return -1;
  case GLUH:
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) GLUH_CG);}
    if(strcmp(atomName,"CD") == 0) {*aType = CARBON; return ((int) GLUH_CD);}
    if(strcmp(atomName,"OE1") == 0) {*aType = OXYGEN; return ((int) GLUH_OE1);}
    if(strcmp(atomName,"OE2") == 0) {*aType = OXYGEN_H; return ((int) GLUH_OE2);}
    /* OE1 or OE2 is associated to an hydrogen. I assume it is OE2 */
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) GLUH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) GLUH_2HB);}        // 3HB
    if(strcmp(atomName,"1HG") == 0) {*aType = HYDROGEN; return ((int) GLUH_1HG);}        // 2HG
    if(strcmp(atomName,"2HG") == 0) {*aType = HYDROGEN; return ((int) GLUH_2HG);}        // 3HG
    return -1;
  //case GLYH:
  // no sch atoms
  case HISH:
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) HISH_CG);}
    if(strcmp(atomName,"ND1") == 0) {*aType = NITROGEN_H; return ((int) HISH_ND1);}
    if(strcmp(atomName,"CD2") == 0) {*aType = CARBON; return ((int) HISH_CD2);}
    if(strcmp(atomName,"CE1") == 0) {*aType = CARBON; return ((int) HISH_CE1);}
    if(strcmp(atomName,"NE2") == 0) {*aType = NITROGEN_H; return ((int) HISH_NE2);}
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) HISH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) HISH_2HB);}        // 3HB
    if(strcmp(atomName,"HD2") == 0) {*aType = HYDROGEN; return ((int) HISH_HD2);}
    if(strcmp(atomName,"HE1") == 0) {*aType = HYDROGEN; return ((int) HISH_HE1);}
    return -1;
  case ILEH:
    if(strcmp(atomName,"CG1") == 0) {*aType = CARBON; return ((int) ILEH_CG1);}
    if(strcmp(atomName,"CG2") == 0) {*aType = CARBON; return ((int) ILEH_CG2);}
    if(strcmp(atomName,"CD1") == 0) {*aType = CARBON; return ((int) ILEH_CD1);}
    if(strcmp(atomName,"HB") == 0) {*aType = HYDROGEN; return ((int) ILEH_HB);}
    if(strcmp(atomName,"1HG2") == 0) {*aType = HYDROGEN; return ((int) ILEH_1HG2);}
    if(strcmp(atomName,"2HG2") == 0) {*aType = HYDROGEN; return ((int) ILEH_2HG2);}
    if(strcmp(atomName,"3HG2") == 0) {*aType = HYDROGEN; return ((int) ILEH_3HG2);}
    if(strcmp(atomName,"1HG1") == 0) {*aType = HYDROGEN; return ((int) ILEH_1HG1);}      // 2HG1
    if(strcmp(atomName,"2HG1") == 0) {*aType = HYDROGEN; return ((int) ILEH_2HG1);}      // 3HG1
    if(strcmp(atomName,"1HD1") == 0) {*aType = HYDROGEN; return ((int) ILEH_1HD1);}
    if(strcmp(atomName,"2HD1") == 0) {*aType = HYDROGEN; return ((int) ILEH_2HD1);}
    if(strcmp(atomName,"3HD1") == 0) {*aType = HYDROGEN; return ((int) ILEH_3HD1);}
    return -1;
  case LEUH:
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) LEUH_CG);}
    if(strcmp(atomName,"CD1") == 0) {*aType = CARBON; return ((int) LEUH_CD1);}
    if(strcmp(atomName,"CD2") == 0) {*aType = CARBON; return ((int) LEUH_CD2);}
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) LEUH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) LEUH_2HB);}        // 3HB
    if(strcmp(atomName,"HG") == 0) {*aType = HYDROGEN; return ((int) LEUH_HG);}
    if(strcmp(atomName,"1HD1") == 0) {*aType = HYDROGEN; return ((int) LEUH_1HD1);}
    if(strcmp(atomName,"2HD1") == 0) {*aType = HYDROGEN; return ((int) LEUH_2HD1);}
    if(strcmp(atomName,"3HD1") == 0) {*aType = HYDROGEN; return ((int) LEUH_3HD1);}
    if(strcmp(atomName,"1HD2") == 0) {*aType = HYDROGEN; return ((int) LEUH_1HD2);}
    if(strcmp(atomName,"2HD2") == 0) {*aType = HYDROGEN; return ((int) LEUH_2HD2);}
    if(strcmp(atomName,"3HD2") == 0) {*aType = HYDROGEN; return ((int) LEUH_3HD2);}
    return -1;
  case LYSH:
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) LYSH_CG);} 
    if(strcmp(atomName,"CD") == 0) {*aType = CARBON; return ((int) LYSH_CD);}
    if(strcmp(atomName,"CE") == 0) {*aType = CARBON; return ((int) LYSH_CE);}
    if(strcmp(atomName,"NZ") == 0) {*aType = NITROGEN_H; return ((int) LYSH_NZ);}
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) LYSH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) LYSH_2HB);}        // 3HB
    if(strcmp(atomName,"1HG") == 0) {*aType = HYDROGEN; return ((int) LYSH_1HG);}        // 2HG
    if(strcmp(atomName,"2HG") == 0) {*aType = HYDROGEN; return ((int) LYSH_2HG);}        // 3HG
    if(strcmp(atomName,"1HD") == 0) {*aType = HYDROGEN; return ((int) LYSH_1HD);}        // 2HD
    if(strcmp(atomName,"2HD") == 0) {*aType = HYDROGEN; return ((int) LYSH_2HD);}        // 3HD
    if(strcmp(atomName,"1HE") == 0) {*aType = HYDROGEN; return ((int) LYSH_1HE);}        // 2HE
    if(strcmp(atomName,"2HE") == 0) {*aType = HYDROGEN; return ((int) LYSH_2HE);}        // 3HE
    return -1;
  case METH:
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) METH_CG);}
    if(strcmp(atomName,"SD") == 0) {*aType = SULPHUR; return ((int) METH_SD);}
    if(strcmp(atomName,"CE") == 0) {*aType = CARBON; return ((int) METH_CE);}
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) METH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) METH_2HB);}        // 3HB
    if(strcmp(atomName,"1HG") == 0) {*aType = HYDROGEN; return ((int) METH_1HG);}        // 2HG
    if(strcmp(atomName,"2HG") == 0) {*aType = HYDROGEN; return ((int) METH_2HG);}        // 3HG
    if(strcmp(atomName,"1HE") == 0) {*aType = HYDROGEN; return ((int) METH_1HE);}
    if(strcmp(atomName,"2HE") == 0) {*aType = HYDROGEN; return ((int) METH_2HE);}
    if(strcmp(atomName,"3HE") == 0) {*aType = HYDROGEN; return ((int) METH_3HE);}
    return -1;
  case PHEH:
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) PHEH_CG);}
    if(strcmp(atomName,"CD1") == 0) {*aType = CARBON; return ((int) PHEH_CD1);}
    if(strcmp(atomName,"CD2") == 0) {*aType = CARBON; return ((int) PHEH_CD2);}
    if(strcmp(atomName,"CE1") == 0) {*aType = CARBON; return ((int) PHEH_CE1);}
    if(strcmp(atomName,"CE2") == 0) {*aType = CARBON; return ((int) PHEH_CE2);}
    if(strcmp(atomName,"CZ") == 0) {*aType = CARBON; return ((int) PHEH_CZ);}
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) PHEH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) PHEH_2HB);}        // 3HB
    if(strcmp(atomName,"HD1") == 0) {*aType = HYDROGEN; return ((int) PHEH_HD1);}
    if(strcmp(atomName,"HD2") == 0) {*aType = HYDROGEN; return ((int) PHEH_HD2);}
    if(strcmp(atomName,"HE1") == 0) {*aType = HYDROGEN; return ((int) PHEH_HE1);}
    if(strcmp(atomName,"HE2") == 0) {*aType = HYDROGEN; return ((int) PHEH_HE2);}
    if(strcmp(atomName,"HZ") == 0) {*aType = HYDROGEN; return ((int) PHEH_HZ);}
    return -1;
  //case PROH:
  // no sch atoms
  case SERH:
    if(strcmp(atomName,"OG") == 0) {*aType = OXYGEN_H; return ((int) SERH_OG);}
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) SERH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) SERH_2HB);}        // 3HB
    return -1;
  case THRH:
    if(strcmp(atomName,"OG1") == 0) {*aType = OXYGEN_H; return ((int) THRH_OG1);}
    if(strcmp(atomName,"CG2") == 0) {*aType = CARBON; return ((int) THRH_CG2);}
    if(strcmp(atomName,"HB") == 0) {*aType = HYDROGEN; return ((int) THRH_HB);}
    if(strcmp(atomName,"1HG2") == 0) {*aType = HYDROGEN; return ((int) THRH_1HG2);}
    if(strcmp(atomName,"2HG2") == 0) {*aType = HYDROGEN; return ((int) THRH_2HG2);}
    if(strcmp(atomName,"3HG2") == 0) {*aType = HYDROGEN; return ((int) THRH_3HG2);}
    return -1;
  case TRPH:
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) TRPH_CG);}
    if(strcmp(atomName,"CD1") == 0) {*aType = CARBON; return ((int) TRPH_CD1);}
    if(strcmp(atomName,"CD2") == 0) {*aType = CARBON; return ((int) TRPH_CD2);}
    if(strcmp(atomName,"NE1") == 0) {*aType = NITROGEN_H; return ((int) TRPH_NE1);}
    if(strcmp(atomName,"CE2") == 0) {*aType = CARBON; return ((int) TRPH_CE2);}
    if(strcmp(atomName,"CE3") == 0) {*aType = CARBON; return ((int) TRPH_CE3);}
    if(strcmp(atomName,"CZ2") == 0) {*aType = CARBON; return ((int) TRPH_CZ2);}
    if(strcmp(atomName,"CZ3") == 0) {*aType = CARBON; return ((int) TRPH_CZ3);}
    if(strcmp(atomName,"CH2") == 0) {*aType = CARBON; return ((int) TRPH_CH2);}
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) TRPH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) TRPH_2HB);}        // 3HB
    if(strcmp(atomName,"HD1") == 0) {*aType = HYDROGEN; return ((int) TRPH_HD1);}
    if(strcmp(atomName,"HE3") == 0) {*aType = HYDROGEN; return ((int) TRPH_HE3);}
    if(strcmp(atomName,"HZ2") == 0) {*aType = HYDROGEN; return ((int) TRPH_HZ2);}
    if(strcmp(atomName,"HZ3") == 0) {*aType = HYDROGEN; return ((int) TRPH_HZ3);}
    if(strcmp(atomName,"HH2") == 0) {*aType = HYDROGEN; return ((int) TRPH_HH2);}
    return -1;
  case TYRH:
    if(strcmp(atomName,"CG") == 0) {*aType = CARBON; return ((int) TYRH_CG);}
    if(strcmp(atomName,"CD1") == 0) {*aType = CARBON; return ((int) TYRH_CD1);}
    if(strcmp(atomName,"CD2") == 0) {*aType = CARBON; return ((int) TYRH_CD2);}
    if(strcmp(atomName,"CE1") == 0) {*aType = CARBON; return ((int) TYRH_CE1);}
    if(strcmp(atomName,"CE2") == 0) {*aType = CARBON; return ((int) TYRH_CE2);}
    if(strcmp(atomName,"CZ") == 0) {*aType = CARBON; return ((int) TYRH_CZ);}
    if(strcmp(atomName,"OH") == 0) {*aType = OXYGEN_H; return ((int) TYRH_OH);}
    if(strcmp(atomName,"1HB") == 0) {*aType = HYDROGEN; return ((int) TYRH_1HB);}        // 2HB
    if(strcmp(atomName,"2HB") == 0) {*aType = HYDROGEN; return ((int) TYRH_2HB);}        // 3HB
    if(strcmp(atomName,"HD1") == 0) {*aType = HYDROGEN; return ((int) TYRH_HD1);}
    if(strcmp(atomName,"HD2") == 0) {*aType = HYDROGEN; return ((int) TYRH_HD2);}
    if(strcmp(atomName,"HE1") == 0) {*aType = HYDROGEN; return ((int) TYRH_HE1);}
    if(strcmp(atomName,"HE2") == 0) {*aType = HYDROGEN; return ((int) TYRH_HE2);}
    return -1;
  case VALH:
    if(strcmp(atomName,"CG1") == 0) {*aType = CARBON; return ((int) VALH_CG1);}
    if(strcmp(atomName,"CG2") == 0) {*aType = CARBON; return ((int) VALH_CG2);}
    if(strcmp(atomName,"HB") == 0) {*aType = HYDROGEN; return ((int) VALH_HB);}
    if(strcmp(atomName,"1HG1") == 0) {*aType = HYDROGEN; return ((int) VALH_1HG1);}
    if(strcmp(atomName,"2HG1") == 0) {*aType = HYDROGEN; return ((int) VALH_2HG1);}
    if(strcmp(atomName,"3HG1") == 0) {*aType = HYDROGEN; return ((int) VALH_3HG1);}
    if(strcmp(atomName,"1HG2") == 0) {*aType = HYDROGEN; return ((int) VALH_1HG2);}
    if(strcmp(atomName,"2HG2") == 0) {*aType = HYDROGEN; return ((int) VALH_2HG2);}
    if(strcmp(atomName,"3HG2") == 0) {*aType = HYDROGEN; return ((int) VALH_3HG2);}
    return -1;
  default:
    return -1;    
  }

  return -1;
}

/**********************************************************************/


static int number_H_bkb_atoms(residueTypes resType)
{
  switch(resType) {
  case GLY:
  case GLYH:
    return N_GLYH_BKB_ATOMS;
  case PRO:
  case PROH:
    return N_PROH_BKB_ATOMS;      
  default:
    return N_GENH_BKB_ATOMS;      
  }
}

/**********************************************************************/


static int number_noH_bkb_atoms(residueTypes resType)
{
  switch(resType) {
  case GLY:
  case GLYH:
    return N_GLY_BKB_ATOMS;
  case PRO:
  case PROH:
    return N_PRO_BKB_ATOMS;      
  default:
    return N_GEN_BKB_ATOMS;      
  }
}

/**********************************************************************/


static int number_bkb_atoms(residueTypes resType)
{
  switch(resType) {
  case GLY:
    return N_GLY_BKB_ATOMS;
  case GLYH:
    return N_GLYH_BKB_ATOMS;
  case PRO:
    return N_PRO_BKB_ATOMS;
  case PROH:
    return N_PROH_BKB_ATOMS;      
  default:
    if(((int) resType) < 20)
      return N_GEN_BKB_ATOMS;      
    else
      return N_GENH_BKB_ATOMS;      
  }
}

/**********************************************************************/

static int number_H_sch_atoms(residueTypes resType)
{
  switch(resType) {
  case ALA:
  case ALAH:
    return N_ALAH_SCH_ATOMS;
  case ARG:
  case ARGH:
    return N_ARGH_SCH_ATOMS;
  case ASN:
  case ASNH:
    return N_ASNH_SCH_ATOMS;
  case ASP:
  case ASPH:
    return N_ASPH_SCH_ATOMS;
  case CYS:
  case CYSH:
    return N_CYSH_SCH_ATOMS;
  case GLN:
  case GLNH:
    return N_GLNH_SCH_ATOMS;
  case GLU:
  case GLUH:
    return N_GLUH_SCH_ATOMS;
  case GLY:
  case GLYH:
    return N_GLYH_SCH_ATOMS;
  case HIS:
  case HISH:
    return N_HISH_SCH_ATOMS;
  case ILE:
  case ILEH:
    return N_ILEH_SCH_ATOMS;
  case LEU:
  case LEUH:
    return N_LEUH_SCH_ATOMS;
  case LYS:
  case LYSH:
    return N_LYSH_SCH_ATOMS;
  case MET:
  case METH:
    return N_METH_SCH_ATOMS;
  case PHE:
  case PHEH:
    return N_PHEH_SCH_ATOMS;
  case PRO:
  case PROH:
    return N_PROH_SCH_ATOMS;
  case SER:
  case SERH:
    return N_SERH_SCH_ATOMS;
  case THR:
  case THRH:
    return N_THRH_SCH_ATOMS;
  case TRP:
  case TRPH:
    return N_TRPH_SCH_ATOMS;
  case TYR:
  case TYRH:
    return N_TYRH_SCH_ATOMS;
  case VAL:
  case VALH:
    return N_VALH_SCH_ATOMS;
  }
  return -1;
}

/**********************************************************************/

static int number_noH_sch_atoms(residueTypes resType)
{
  switch(resType) {
  case ALA:
  case ALAH:
    return N_ALA_SCH_ATOMS;
  case ARG:
  case ARGH:
    return N_ARG_SCH_ATOMS;
  case ASN:
  case ASNH:
    return N_ASN_SCH_ATOMS;
  case ASP:
  case ASPH:
    return N_ASP_SCH_ATOMS;
  case CYS:
  case CYSH:
    return N_CYS_SCH_ATOMS;
  case GLN:
  case GLNH:
    return N_GLN_SCH_ATOMS;
  case GLU:
  case GLUH:
    return N_GLU_SCH_ATOMS;
  case GLY:
  case GLYH:
    return N_GLY_SCH_ATOMS;
  case HIS:
  case HISH:
    return N_HIS_SCH_ATOMS;
  case ILE:
  case ILEH:
    return N_ILE_SCH_ATOMS;
  case LEU:
  case LEUH:
    return N_LEU_SCH_ATOMS;
  case LYS:
  case LYSH:
    return N_LYS_SCH_ATOMS;
  case MET:
  case METH:
    return N_MET_SCH_ATOMS;
  case PHE:
  case PHEH:
    return N_PHE_SCH_ATOMS;
  case PRO:
  case PROH:
    return N_PRO_SCH_ATOMS;
  case SER:
  case SERH:
    return N_SER_SCH_ATOMS;
  case THR:
  case THRH:
    return N_THR_SCH_ATOMS;
  case TRP:
  case TRPH:
    return N_TRP_SCH_ATOMS;
  case TYR:
  case TYRH:
    return N_TYR_SCH_ATOMS;
  case VAL:
  case VALH:
    return N_VAL_SCH_ATOMS;
  }
  return -1;
}

/**********************************************************************/

static int number_sch_atoms(residueTypes resType)
{
  switch(resType) {
  case ALA:
    return N_ALA_SCH_ATOMS;
  case ARG:
    return N_ARG_SCH_ATOMS;
  case ASN:
    return N_ASN_SCH_ATOMS;
  case ASP:
    return N_ASP_SCH_ATOMS;
  case CYS:
    return N_CYS_SCH_ATOMS;
  case GLN:
    return N_GLN_SCH_ATOMS;
  case GLU:
    return N_GLU_SCH_ATOMS;
  case GLY:
    return N_GLY_SCH_ATOMS;
  case HIS:
    return N_HIS_SCH_ATOMS;
  case ILE:
    return N_ILE_SCH_ATOMS;
  case LEU:
    return N_LEU_SCH_ATOMS;
  case LYS:
    return N_LYS_SCH_ATOMS;
  case MET:
    return N_MET_SCH_ATOMS;
  case PHE:
    return N_PHE_SCH_ATOMS;
  case PRO:
    return N_PRO_SCH_ATOMS;
  case SER:
    return N_SER_SCH_ATOMS;
  case THR:
    return N_THR_SCH_ATOMS;
  case TRP:
    return N_TRP_SCH_ATOMS;
  case TYR:
    return N_TYR_SCH_ATOMS;
  case VAL:
    return N_VAL_SCH_ATOMS;
  case ALAH:
    return N_ALAH_SCH_ATOMS;
  case ARGH:
    return N_ARGH_SCH_ATOMS;
  case ASNH:
    return N_ASNH_SCH_ATOMS;
  case ASPH:
    return N_ASPH_SCH_ATOMS;
  case CYSH:
    return N_CYSH_SCH_ATOMS;
  case GLNH:
    return N_GLNH_SCH_ATOMS;
  case GLUH:
    return N_GLUH_SCH_ATOMS;
  case GLYH:
    return N_GLYH_SCH_ATOMS;
  case HISH:
    return N_HISH_SCH_ATOMS;
  case ILEH:
    return N_ILEH_SCH_ATOMS;
  case LEUH:
    return N_LEUH_SCH_ATOMS;
  case LYSH:
    return N_LYSH_SCH_ATOMS;
  case METH:
    return N_METH_SCH_ATOMS;
  case PHEH:
    return N_PHEH_SCH_ATOMS;
  case PROH:
    return N_PROH_SCH_ATOMS;
  case SERH:
    return N_SERH_SCH_ATOMS;
  case THRH:
    return N_THRH_SCH_ATOMS;
  case TRPH:
    return N_TRPH_SCH_ATOMS;
  case TYRH:
    return N_TYRH_SCH_ATOMS;
  case VALH:
    return N_VALH_SCH_ATOMS;
  }
  return -1;
}

/**********************************************************************/
// BONDS SETTING

void set_bond(atom *a1Pt, atom *a2Pt)
{
  insert_pointer_in_list(a1Pt,(void ***)&(a2Pt->bondedAlist),&(a2Pt->nbondedA));
  insert_pointer_in_list(a2Pt,(void ***)&(a1Pt->bondedAlist),&(a1Pt->nbondedA));
}


void set_peptide_bond(residue *resPt, residue *prevresPt)
{
  set_bond(get_N(resPt),get_C(prevresPt));
}


void set_residue_bonds(residue *resPt)
{

  // set bonds between backbone atoms
  switch(resPt->resType) {
  case GLY:
    set_bond(resPt->bkbAlist[GLY_N],resPt->bkbAlist[GLY_CA]);
    set_bond(resPt->bkbAlist[GLY_CA],resPt->bkbAlist[GLY_C]);
    set_bond(resPt->bkbAlist[GLY_C],resPt->bkbAlist[GLY_O]);
    if(resPt->flagCterm) {
      set_bond(resPt->bkbAlist[GLY_C],resPt->bkbAlist[GLY_OXT]);
    }
    break;
  case GLYH:
    set_bond(resPt->bkbAlist[GLYH_N],resPt->bkbAlist[GLYH_CA]);
    set_bond(resPt->bkbAlist[GLYH_CA],resPt->bkbAlist[GLYH_C]);
    set_bond(resPt->bkbAlist[GLYH_C],resPt->bkbAlist[GLYH_O]);
    set_bond(resPt->bkbAlist[GLYH_CA],resPt->bkbAlist[GLYH_1HA]);
    set_bond(resPt->bkbAlist[GLYH_CA],resPt->bkbAlist[GLYH_2HA]);
    if(resPt->flagCterm) {
      set_bond(resPt->bkbAlist[GLYH_C],resPt->bkbAlist[GLYH_OXT]);
    }
    break;
  case PRO:
    set_bond(resPt->bkbAlist[PRO_N],resPt->bkbAlist[PRO_CA]);
    set_bond(resPt->bkbAlist[PRO_CA],resPt->bkbAlist[PRO_C]);
    set_bond(resPt->bkbAlist[PRO_C],resPt->bkbAlist[PRO_O]);
    set_bond(resPt->bkbAlist[PRO_CA],resPt->bkbAlist[PRO_CB]);
    set_bond(resPt->bkbAlist[PRO_CB],resPt->bkbAlist[PRO_CG]);
    set_bond(resPt->bkbAlist[PRO_CG],resPt->bkbAlist[PRO_CD]);
    set_bond(resPt->bkbAlist[PRO_CD],resPt->bkbAlist[PRO_N]);
    if(resPt->flagCterm) {
      set_bond(resPt->bkbAlist[PRO_C],resPt->bkbAlist[PRO_OXT]);
    }
    break;
  case PROH:
    set_bond(resPt->bkbAlist[PROH_N],resPt->bkbAlist[PROH_CA]);
    set_bond(resPt->bkbAlist[PROH_CA],resPt->bkbAlist[PROH_C]);
    set_bond(resPt->bkbAlist[PROH_C],resPt->bkbAlist[PROH_O]);
    set_bond(resPt->bkbAlist[PROH_CA],resPt->bkbAlist[PROH_CB]);
    set_bond(resPt->bkbAlist[PROH_CB],resPt->bkbAlist[PROH_CG]);
    set_bond(resPt->bkbAlist[PROH_CG],resPt->bkbAlist[PROH_CD]);
    set_bond(resPt->bkbAlist[PROH_CD],resPt->bkbAlist[PROH_N]);
    set_bond(resPt->bkbAlist[PROH_CA],resPt->bkbAlist[PROH_HA]);
    set_bond(resPt->bkbAlist[PROH_CB],resPt->bkbAlist[PROH_1HB]);
    set_bond(resPt->bkbAlist[PROH_CB],resPt->bkbAlist[PROH_2HB]);
    set_bond(resPt->bkbAlist[PROH_CG],resPt->bkbAlist[PROH_1HG]);
    set_bond(resPt->bkbAlist[PROH_CG],resPt->bkbAlist[PROH_2HG]);
    set_bond(resPt->bkbAlist[PROH_CD],resPt->bkbAlist[PROH_1HD]);
    set_bond(resPt->bkbAlist[PROH_CD],resPt->bkbAlist[PROH_2HD]);
    if(resPt->flagCterm) {
      set_bond(resPt->bkbAlist[PROH_C],resPt->bkbAlist[PROH_OXT]);
    }
    break;
  default:

    if(resPt->flagH == 0) {
      set_bond(resPt->bkbAlist[gen_N],resPt->bkbAlist[gen_CA]);
      set_bond(resPt->bkbAlist[gen_CA],resPt->bkbAlist[gen_C]);
      set_bond(resPt->bkbAlist[gen_C],resPt->bkbAlist[gen_O]);
      set_bond(resPt->bkbAlist[gen_CA],resPt->bkbAlist[gen_CB]);
      if(resPt->flagCterm) {
	set_bond(resPt->bkbAlist[gen_C],resPt->bkbAlist[gen_OXT]);
      }
    }
    else {
      set_bond(resPt->bkbAlist[genH_N],resPt->bkbAlist[genH_CA]);
      set_bond(resPt->bkbAlist[genH_CA],resPt->bkbAlist[genH_C]);
      set_bond(resPt->bkbAlist[genH_C],resPt->bkbAlist[genH_O]);
      set_bond(resPt->bkbAlist[genH_CA],resPt->bkbAlist[genH_CB]);
      set_bond(resPt->bkbAlist[genH_CA],resPt->bkbAlist[genH_HA]);
      if(resPt->flagCterm) {
	set_bond(resPt->bkbAlist[genH_C],resPt->bkbAlist[genH_OXT]);
      }
    }
  }

  // set bonds between side-chain atoms
  switch(resPt->resType) {
  case ALA:
    break;
  case ARG:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[ARG_CG]);
    set_bond(resPt->schAlist[ARG_CG],resPt->schAlist[ARG_CD]);
    set_bond(resPt->schAlist[ARG_CD],resPt->schAlist[ARG_NE]);
    set_bond(resPt->schAlist[ARG_NE],resPt->schAlist[ARG_CZ]);
    set_bond(resPt->schAlist[ARG_CZ],resPt->schAlist[ARG_NH1]);
    set_bond(resPt->schAlist[ARG_CZ],resPt->schAlist[ARG_NH2]);
    break;
  case ASN:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[ASN_CG]);
    set_bond(resPt->schAlist[ASN_CG],resPt->schAlist[ASN_OD1]);
    set_bond(resPt->schAlist[ASN_CG],resPt->schAlist[ASN_ND2]);
    break;
  case ASP:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[ASP_CG]);
    set_bond(resPt->schAlist[ASP_CG],resPt->schAlist[ASP_OD1]);
    set_bond(resPt->schAlist[ASP_CG],resPt->schAlist[ASP_OD2]);
     break;
  case CYS:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[CYS_SG]);
    break;
  case GLN:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[GLN_CG]);
    set_bond(resPt->schAlist[GLN_CG],resPt->schAlist[GLN_CD]);
    set_bond(resPt->schAlist[GLN_CD],resPt->schAlist[GLN_OE1]);
    set_bond(resPt->schAlist[GLN_CD],resPt->schAlist[GLN_NE2]);
    break;
  case GLU:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[GLU_CG]);
    set_bond(resPt->schAlist[GLU_CG],resPt->schAlist[GLU_CD]);
    set_bond(resPt->schAlist[GLU_CD],resPt->schAlist[GLU_OE1]);
    set_bond(resPt->schAlist[GLU_CD],resPt->schAlist[GLU_OE2]);
    break;
  case GLY:
    break;
  case HIS:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[HIS_CG]);
    set_bond(resPt->schAlist[HIS_CG],resPt->schAlist[HIS_ND1]);
    set_bond(resPt->schAlist[HIS_CG],resPt->schAlist[HIS_CD2]);
    set_bond(resPt->schAlist[HIS_ND1],resPt->schAlist[HIS_CE1]);
    set_bond(resPt->schAlist[HIS_CD2],resPt->schAlist[HIS_NE2]);
    set_bond(resPt->schAlist[HIS_CE1],resPt->schAlist[HIS_NE2]);
   break;
  case ILE:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[ILE_CG1]);
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[ILE_CG2]);
    set_bond(resPt->schAlist[ILE_CG1],resPt->schAlist[ILE_CD1]);
    break;
  case LEU:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[LEU_CG]);
    set_bond(resPt->schAlist[LEU_CG],resPt->schAlist[LEU_CD1]);
    set_bond(resPt->schAlist[LEU_CG],resPt->schAlist[LEU_CD2]);
    break;
  case LYS:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[LYS_CG]);
    set_bond(resPt->schAlist[LYS_CG],resPt->schAlist[LYS_CD]);
    set_bond(resPt->schAlist[LYS_CD],resPt->schAlist[LYS_CE]);
    set_bond(resPt->schAlist[LYS_CE],resPt->schAlist[LYS_NZ]);
    break;
  case MET:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[MET_CG]);
    set_bond(resPt->schAlist[MET_CG],resPt->schAlist[MET_SD]);
    set_bond(resPt->schAlist[MET_SD],resPt->schAlist[MET_CE]);
    break;
  case PHE:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[PHE_CG]);
    set_bond(resPt->schAlist[PHE_CG],resPt->schAlist[PHE_CD1]);
    set_bond(resPt->schAlist[PHE_CG],resPt->schAlist[PHE_CD2]);
    set_bond(resPt->schAlist[PHE_CD1],resPt->schAlist[PHE_CE1]);
    set_bond(resPt->schAlist[PHE_CD2],resPt->schAlist[PHE_CE2]);
    set_bond(resPt->schAlist[PHE_CE1],resPt->schAlist[PHE_CZ]);
    set_bond(resPt->schAlist[PHE_CE2],resPt->schAlist[PHE_CZ]);
    break;
  case PRO:
    break;
  case SER:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[SER_OG]);
    break;
  case THR:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[THR_OG1]);
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[THR_CG2]);
    break;
  case TRP:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[TRP_CG]);
    set_bond(resPt->schAlist[TRP_CG],resPt->schAlist[TRP_CD1]);
    set_bond(resPt->schAlist[TRP_CG],resPt->schAlist[TRP_CD2]);
    set_bond(resPt->schAlist[TRP_CD1],resPt->schAlist[TRP_NE1]);
    set_bond(resPt->schAlist[TRP_NE1],resPt->schAlist[TRP_CE2]);
    set_bond(resPt->schAlist[TRP_CD2],resPt->schAlist[TRP_CE2]);
    set_bond(resPt->schAlist[TRP_CD2],resPt->schAlist[TRP_CE3]);
    set_bond(resPt->schAlist[TRP_CE2],resPt->schAlist[TRP_CZ2]);
    set_bond(resPt->schAlist[TRP_CE3],resPt->schAlist[TRP_CZ3]);
    set_bond(resPt->schAlist[TRP_CZ2],resPt->schAlist[TRP_CH2]);
    set_bond(resPt->schAlist[TRP_CZ3],resPt->schAlist[TRP_CH2]);
   break;
  case TYR:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[TYR_CG]);
    set_bond(resPt->schAlist[TYR_CG],resPt->schAlist[TYR_CD1]);
    set_bond(resPt->schAlist[TYR_CG],resPt->schAlist[TYR_CD2]);
    set_bond(resPt->schAlist[TYR_CD1],resPt->schAlist[TYR_CE1]);
    set_bond(resPt->schAlist[TYR_CD2],resPt->schAlist[TYR_CE2]);
    set_bond(resPt->schAlist[TYR_CE1],resPt->schAlist[TYR_CZ]);
    set_bond(resPt->schAlist[TYR_CE2],resPt->schAlist[TYR_CZ]);
    set_bond(resPt->schAlist[TYR_CZ],resPt->schAlist[TYR_OH]);
    break;
  case VAL:
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[VAL_CG1]);
    set_bond(resPt->bkbAlist[gen_CB],resPt->schAlist[VAL_CG2]);
    break;
  //- with H --------------------------------------------------
  case ALAH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ALAH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ALAH_2HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ALAH_3HB]);
    break;
  case ARGH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ARGH_CG]);
    set_bond(resPt->schAlist[ARGH_CG],resPt->schAlist[ARGH_CD]);
    set_bond(resPt->schAlist[ARGH_CD],resPt->schAlist[ARGH_NE]);
    set_bond(resPt->schAlist[ARGH_NE],resPt->schAlist[ARGH_CZ]);
    set_bond(resPt->schAlist[ARGH_CZ],resPt->schAlist[ARGH_NH1]);
    set_bond(resPt->schAlist[ARGH_CZ],resPt->schAlist[ARGH_NH2]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ARGH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ARGH_2HB]);
    set_bond(resPt->schAlist[ARGH_CG],resPt->schAlist[ARGH_1HG]);
    set_bond(resPt->schAlist[ARGH_CG],resPt->schAlist[ARGH_2HG]);
    set_bond(resPt->schAlist[ARGH_CD],resPt->schAlist[ARGH_1HD]);
    set_bond(resPt->schAlist[ARGH_CD],resPt->schAlist[ARGH_2HD]);
    break;
  case ASNH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ASNH_CG]);
    set_bond(resPt->schAlist[ASNH_CG],resPt->schAlist[ASNH_OD1]);
    set_bond(resPt->schAlist[ASNH_CG],resPt->schAlist[ASNH_ND2]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ASNH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ASNH_2HB]);
    break;
  case ASPH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ASPH_CG]);
    set_bond(resPt->schAlist[ASPH_CG],resPt->schAlist[ASPH_OD1]);
    set_bond(resPt->schAlist[ASPH_CG],resPt->schAlist[ASPH_OD2]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ASPH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ASPH_2HB]);
    break;
  case CYSH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[CYSH_SG]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[CYSH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[CYSH_2HB]);
    break;
  case GLNH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[GLNH_CG]);
    set_bond(resPt->schAlist[GLNH_CG],resPt->schAlist[GLNH_CD]);
    set_bond(resPt->schAlist[GLNH_CD],resPt->schAlist[GLNH_OE1]);
    set_bond(resPt->schAlist[GLNH_CD],resPt->schAlist[GLNH_NE2]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[GLNH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[GLNH_2HB]);
    set_bond(resPt->schAlist[GLNH_CG],resPt->schAlist[GLNH_1HG]);
    set_bond(resPt->schAlist[GLNH_CG],resPt->schAlist[GLNH_2HG]);
    break;
  case GLUH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[GLUH_CG]);
    set_bond(resPt->schAlist[GLUH_CG],resPt->schAlist[GLUH_CD]);
    set_bond(resPt->schAlist[GLUH_CD],resPt->schAlist[GLUH_OE1]);
    set_bond(resPt->schAlist[GLUH_CD],resPt->schAlist[GLUH_OE2]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[GLUH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[GLUH_2HB]);
    set_bond(resPt->schAlist[GLUH_CG],resPt->schAlist[GLUH_1HG]);
    set_bond(resPt->schAlist[GLUH_CG],resPt->schAlist[GLUH_2HG]);
    break;
  case GLYH:
    break;
  case HISH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[HISH_CG]);
    set_bond(resPt->schAlist[HISH_CG],resPt->schAlist[HISH_ND1]);
    set_bond(resPt->schAlist[HISH_CG],resPt->schAlist[HISH_CD2]);
    set_bond(resPt->schAlist[HISH_ND1],resPt->schAlist[HISH_CE1]);
    set_bond(resPt->schAlist[HISH_CD2],resPt->schAlist[HISH_NE2]);
    set_bond(resPt->schAlist[HISH_CE1],resPt->schAlist[HISH_NE2]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[HISH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[HISH_2HB]);
    set_bond(resPt->schAlist[HISH_CD2],resPt->schAlist[HISH_HD2]);
    set_bond(resPt->schAlist[HISH_CE1],resPt->schAlist[HISH_HE1]);
    break;
  case ILEH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ILEH_CG1]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ILEH_CG2]);
    set_bond(resPt->schAlist[ILEH_CG1],resPt->schAlist[ILEH_CD1]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[ILEH_HB]);
    set_bond(resPt->schAlist[ILEH_CG1],resPt->schAlist[ILEH_1HG1]);
    set_bond(resPt->schAlist[ILEH_CG1],resPt->schAlist[ILEH_2HG1]);
    set_bond(resPt->schAlist[ILEH_CG2],resPt->schAlist[ILEH_1HG2]);
    set_bond(resPt->schAlist[ILEH_CG2],resPt->schAlist[ILEH_2HG2]);
    set_bond(resPt->schAlist[ILEH_CG2],resPt->schAlist[ILEH_3HG2]);
    set_bond(resPt->schAlist[ILEH_CD1],resPt->schAlist[ILEH_1HD1]);
    set_bond(resPt->schAlist[ILEH_CD1],resPt->schAlist[ILEH_2HD1]);
    set_bond(resPt->schAlist[ILEH_CD1],resPt->schAlist[ILEH_3HD1]);
    break;
  case LEUH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[LEUH_CG]);
    set_bond(resPt->schAlist[LEUH_CG],resPt->schAlist[LEUH_CD1]);
    set_bond(resPt->schAlist[LEUH_CG],resPt->schAlist[LEUH_CD2]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[LEUH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[LEUH_2HB]);
    set_bond(resPt->schAlist[LEUH_CG],resPt->schAlist[LEUH_HG]);
    set_bond(resPt->schAlist[LEUH_CD1],resPt->schAlist[LEUH_1HD1]);
    set_bond(resPt->schAlist[LEUH_CD1],resPt->schAlist[LEUH_2HD1]);
    set_bond(resPt->schAlist[LEUH_CD1],resPt->schAlist[LEUH_3HD1]);
    set_bond(resPt->schAlist[LEUH_CD2],resPt->schAlist[LEUH_1HD2]);
    set_bond(resPt->schAlist[LEUH_CD2],resPt->schAlist[LEUH_2HD2]);
    set_bond(resPt->schAlist[LEUH_CD2],resPt->schAlist[LEUH_3HD2]);
    break;
  case LYSH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[LYSH_CG]);
    set_bond(resPt->schAlist[LYSH_CG],resPt->schAlist[LYSH_CD]);
    set_bond(resPt->schAlist[LYSH_CD],resPt->schAlist[LYSH_CE]);
    set_bond(resPt->schAlist[LYSH_CE],resPt->schAlist[LYSH_NZ]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[LYSH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[LYSH_2HB]);
    set_bond(resPt->schAlist[LYSH_CG],resPt->schAlist[LYSH_1HG]);
    set_bond(resPt->schAlist[LYSH_CG],resPt->schAlist[LYSH_2HG]);
    set_bond(resPt->schAlist[LYSH_CD],resPt->schAlist[LYSH_1HD]);
    set_bond(resPt->schAlist[LYSH_CD],resPt->schAlist[LYSH_2HD]);
    set_bond(resPt->schAlist[LYSH_CE],resPt->schAlist[LYSH_1HE]);
    set_bond(resPt->schAlist[LYSH_CE],resPt->schAlist[LYSH_2HE]);
    break;
   
  case METH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[METH_CG]);
    set_bond(resPt->schAlist[METH_CG],resPt->schAlist[METH_SD]);
    set_bond(resPt->schAlist[METH_SD],resPt->schAlist[METH_CE]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[METH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[METH_2HB]);
    set_bond(resPt->schAlist[METH_CG],resPt->schAlist[METH_1HG]);
    set_bond(resPt->schAlist[METH_CG],resPt->schAlist[METH_2HG]);
    set_bond(resPt->schAlist[METH_CE],resPt->schAlist[METH_1HE]);
    set_bond(resPt->schAlist[METH_CE],resPt->schAlist[METH_2HE]);
    set_bond(resPt->schAlist[METH_CE],resPt->schAlist[METH_3HE]);
    break;
  case PHEH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[PHEH_CG]);
    set_bond(resPt->schAlist[PHEH_CG],resPt->schAlist[PHEH_CD1]);
    set_bond(resPt->schAlist[PHEH_CG],resPt->schAlist[PHEH_CD2]);
    set_bond(resPt->schAlist[PHEH_CD1],resPt->schAlist[PHEH_CE1]);
    set_bond(resPt->schAlist[PHEH_CD2],resPt->schAlist[PHEH_CE2]);
    set_bond(resPt->schAlist[PHEH_CE1],resPt->schAlist[PHEH_CZ]);
    set_bond(resPt->schAlist[PHEH_CE2],resPt->schAlist[PHEH_CZ]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[PHEH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[PHEH_2HB]);
    set_bond(resPt->schAlist[PHEH_CD1],resPt->schAlist[PHEH_HD1]);
    set_bond(resPt->schAlist[PHEH_CD2],resPt->schAlist[PHEH_HD2]);
    set_bond(resPt->schAlist[PHEH_CE1],resPt->schAlist[PHEH_HE1]);
    set_bond(resPt->schAlist[PHEH_CE2],resPt->schAlist[PHEH_HE2]);
    set_bond(resPt->schAlist[PHEH_CZ],resPt->schAlist[PHEH_HZ]);
    break;
  case PROH:
    break;
  case SERH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[SERH_OG]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[SERH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[SERH_2HB]);
    break;
  case THRH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[THRH_OG1]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[THRH_CG2]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[THRH_HB]);
    set_bond(resPt->schAlist[THRH_CG2],resPt->schAlist[THRH_1HG2]);
    set_bond(resPt->schAlist[THRH_CG2],resPt->schAlist[THRH_2HG2]);
    set_bond(resPt->schAlist[THRH_CG2],resPt->schAlist[THRH_3HG2]);
    break;
  case TRPH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[TRPH_CG]);
    set_bond(resPt->schAlist[TRPH_CG],resPt->schAlist[TRPH_CD1]);
    set_bond(resPt->schAlist[TRPH_CG],resPt->schAlist[TRPH_CD2]);
    set_bond(resPt->schAlist[TRPH_CD1],resPt->schAlist[TRPH_NE1]);
    set_bond(resPt->schAlist[TRPH_NE1],resPt->schAlist[TRPH_CE2]);
    set_bond(resPt->schAlist[TRPH_CD2],resPt->schAlist[TRPH_CE2]);
    set_bond(resPt->schAlist[TRPH_CD2],resPt->schAlist[TRPH_CE3]);
    set_bond(resPt->schAlist[TRPH_CE2],resPt->schAlist[TRPH_CZ2]);
    set_bond(resPt->schAlist[TRPH_CE3],resPt->schAlist[TRPH_CZ3]);
    set_bond(resPt->schAlist[TRPH_CZ2],resPt->schAlist[TRPH_CH2]);
    set_bond(resPt->schAlist[TRPH_CZ3],resPt->schAlist[TRPH_CH2]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[TRPH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[TRPH_2HB]);
    set_bond(resPt->schAlist[TRPH_CD1],resPt->schAlist[TRPH_HD1]);
    set_bond(resPt->schAlist[TRPH_CE3],resPt->schAlist[TRPH_HE3]);
    set_bond(resPt->schAlist[TRPH_CZ2],resPt->schAlist[TRPH_HZ2]);
    set_bond(resPt->schAlist[TRPH_CZ3],resPt->schAlist[TRPH_HZ3]);
    set_bond(resPt->schAlist[TRPH_CH2],resPt->schAlist[TRPH_HH2]);
    break;
  case TYRH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[TYRH_CG]);
    set_bond(resPt->schAlist[TYRH_CG],resPt->schAlist[TYRH_CD1]);
    set_bond(resPt->schAlist[TYRH_CG],resPt->schAlist[TYRH_CD2]);
    set_bond(resPt->schAlist[TYRH_CD1],resPt->schAlist[TYRH_CE1]);
    set_bond(resPt->schAlist[TYRH_CD2],resPt->schAlist[TYRH_CE2]);
    set_bond(resPt->schAlist[TYRH_CE1],resPt->schAlist[TYRH_CZ]);
    set_bond(resPt->schAlist[TYRH_CE2],resPt->schAlist[TYRH_CZ]);
    set_bond(resPt->schAlist[TYRH_CZ],resPt->schAlist[TYRH_OH]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[TYRH_1HB]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[TYRH_2HB]);
    set_bond(resPt->schAlist[TYRH_CD1],resPt->schAlist[TYRH_HD1]);
    set_bond(resPt->schAlist[TYRH_CD2],resPt->schAlist[TYRH_HD2]);
    set_bond(resPt->schAlist[TYRH_CE1],resPt->schAlist[TYRH_HE1]);
    set_bond(resPt->schAlist[TYRH_CE2],resPt->schAlist[TYRH_HE2]);
    break;
  case VALH:
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[VALH_CG1]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[VALH_CG2]);
    set_bond(resPt->bkbAlist[genH_CB],resPt->schAlist[VALH_HB]);
    set_bond(resPt->schAlist[VALH_CG1],resPt->schAlist[VALH_1HG1]);
    set_bond(resPt->schAlist[VALH_CG1],resPt->schAlist[VALH_2HG1]);
    set_bond(resPt->schAlist[VALH_CG1],resPt->schAlist[VALH_3HG1]);
    set_bond(resPt->schAlist[VALH_CG2],resPt->schAlist[VALH_1HG2]);
    set_bond(resPt->schAlist[VALH_CG2],resPt->schAlist[VALH_2HG2]);
    set_bond(resPt->schAlist[VALH_CG2],resPt->schAlist[VALH_3HG2]);
    break;
  }
}

/**********************************************************************/
// INTERMEDIATE FUNCTIONS

static res_atoms* create_res_atoms(residueTypes resType,
				   int resSeq,
				   char* chainID,
				   char* resName) {

  int i;
  int nmax_bkb_atoms = number_H_bkb_atoms(resType);
  int nmax_sch_atoms = number_H_sch_atoms(resType);

  res_atoms* resatomsPt=(res_atoms*)malloc(sizeof(res_atoms));

  resatomsPt->resType = resType;
  resatomsPt->resSeq = resSeq;
  strcpy(resatomsPt->chainID,chainID);
  strcpy(resatomsPt->resName,resName);

  resatomsPt->nbkbatoms = nmax_bkb_atoms;
  resatomsPt->nschatoms = nmax_sch_atoms;
  resatomsPt->bkbatoms = (atom**)malloc(nmax_bkb_atoms * sizeof(atom));
  resatomsPt->schatoms = (atom**)malloc(nmax_sch_atoms * sizeof(atom));

  for(i=0; i<nmax_bkb_atoms; i++) 
    resatomsPt->bkbatoms[i] = NULL;
  for(i=0; i<nmax_sch_atoms; i++) 
    resatomsPt->schatoms[i] = NULL;

  resatomsPt->n_unknownAtoms = 0;
  resatomsPt->unknownAtoms = NULL;
  
  return resatomsPt;
}


/**********************************************************************/

static void free_res_atoms(res_atoms* res_atomsPt) {
  free(res_atomsPt->bkbatoms);
  free(res_atomsPt->schatoms);
  free(res_atomsPt);
}

/**********************************************************************/

static unknown_atom* create_unknown_atom(pdb_atom_data *adataPt) {
  
  int i;
  unknown_atom *unknown_atomPt;

  unknown_atomPt = (unknown_atom*)malloc(sizeof(unknown_atom));
  unknown_atomPt->serial = adataPt->serial;
  strcpy(unknown_atomPt->name,adataPt->name);
  for(i=0; i<3; i++) {
    unknown_atomPt->pos[i] = adataPt->pos[i];
  }
  
  return unknown_atomPt;
}


/**********************************************************************/

atom *copy_atom_data(pdb_atom_data *adataPt) 
{
  atom *aPt;
  int i;

  aPt = (atom *) malloc(sizeof(atom));
  
  aPt->serial = adataPt->serial;
  strcpy(aPt->name, adataPt->name);
  aPt->atomType = adataPt->type;
  for(i=0; i<3; i++) {
    aPt->pos[i] = adataPt->pos[i];
  }

  if(aPt->atomType == CARBON) {
    aPt->vdwR = C_VDWR;
  }
  else if((aPt->atomType == OXYGEN) || (aPt->atomType == OXYGEN_H)) {
    aPt->vdwR = O_VDWR;
  }
  else if((aPt->atomType == NITROGEN) || (aPt->atomType == NITROGEN_H)|| (aPt->atomType == NITROGEN_FULL)) {
    aPt->vdwR = N_VDWR;
  }
  else if((aPt->atomType == SULPHUR) || (aPt->atomType == SULPHUR_H)) {
    aPt->vdwR = S_VDWR;
  }
  else {
    aPt->vdwR = H_VDWR;    
  }

  //aPt->residuePt : LATER

  aPt->nbondedA = 0;

  return aPt;
}

/**********************************************************************/

static int insert_bkb_adata_in_res_atoms(pdb_atom_data *adataPt, res_atoms *resatomsPt, residueTypes resType)
{
  int bkbatomindex;

  bkbatomindex = atomName_to_bkbatomindex(adataPt->name,&(adataPt->type),resType);
  if(bkbatomindex < 0) {
    return 0;
  }

  if(resatomsPt->bkbatoms[bkbatomindex] != NULL) {
    printf("ERROR : atom already exists : atom serial %d\n",adataPt->serial);
    return -1;
  }
  resatomsPt->bkbatoms[bkbatomindex] = copy_atom_data(adataPt);

  return 1;
}


static int insert_sch_adata_in_res_atoms(pdb_atom_data *adataPt, res_atoms *resatomsPt, residueTypes resType)
{
  int schatomindex;

  schatomindex = atomName_to_schatomindex(adataPt->name,&(adataPt->type),resType);
  if(schatomindex < 0) {
    return 0;
  }

  if(resatomsPt->schatoms[schatomindex] != NULL) {
    printf("ERROR : atom already exists : atom serial %d\n",adataPt->serial);
    return -1;
  }
  resatomsPt->schatoms[schatomindex] = copy_atom_data(adataPt);

  return 1;  
}


static int insert_adata_in_res_atoms(pdb_atom_data *adataPt, res_atoms *resatomsPt, residueTypes resType)
{
  int state;
  unknown_atom* unknown_atomPt;
  
  state = insert_bkb_adata_in_res_atoms(adataPt,resatomsPt,resType);

  if(state < 0) {
    return -1;
  }
  else if(state == 1) {
    return 1;
  }

  state = insert_sch_adata_in_res_atoms(adataPt,resatomsPt,resType);
 
  if(state == 0) {
    unknown_atomPt = create_unknown_atom(adataPt);
    insert_pointer_in_list((void*)unknown_atomPt,(void***)&(resatomsPt->unknownAtoms), &(resatomsPt->n_unknownAtoms));
    //printf("WARNING (res %d): added unknown atom : %s - %d\n",resatomsPt->resSeq,adataPt->name,adataPt->serial);
  }

  return 1;
}

/**********************************************************************/

static int check_bkb_resatoms(res_atoms* resatomsPt, int* flagH, int* flagCterm) {

  int i=0;
  int error = FALSE;
  int nb_atoms = 0;
  residueTypes resType = resatomsPt->resType;
  int nmax_H_bkb_atoms = number_H_bkb_atoms(resType);
  int nmax_noH_bkb_atoms = number_noH_bkb_atoms(resType);

  *flagCterm = 1;
  *flagH = 1;

  for (i=0; i<nmax_H_bkb_atoms; i++){
    
    if (is_H_OXT_index(i, resType)) {
      *flagCterm = (resatomsPt->bkbatoms[i]!=NULL);
    }
    
    if (resatomsPt->bkbatoms[i]!=NULL) {
      nb_atoms++;
    }
    
  }

  if (*flagCterm) {
    
    if (nb_atoms == nmax_noH_bkb_atoms) {
      *flagH = 0;
    }
    else if (nb_atoms == nmax_H_bkb_atoms) {
      *flagH = 1;
    }
    else {
      error = TRUE;
    }
  }
  else {

    if (nb_atoms == nmax_noH_bkb_atoms-1) {
      *flagH = 0;
    }
    else if (nb_atoms == nmax_H_bkb_atoms-1) {
      *flagH = 1;
    }
    else {
      error = TRUE;
    }

  }
  if (error)
    printf("Error : unable to choose between H or no H version\n");      
  
  return (!error);

}

/**********************************************************************/

static int check_sch_resatoms(res_atoms* resatomsPt, int* flagH) {

  int i=0;
  int error = FALSE;
  int nb_atoms = 0;
  residueTypes resType = resatomsPt->resType;
  int nmax_H_sch_atoms = number_H_sch_atoms(resType); 
  int nmax_noH_sch_atoms = number_noH_sch_atoms(resType); 

  *flagH = 1;

  for (i=0; i<nmax_H_sch_atoms; i++){
    
    if (resatomsPt->schatoms[i]!=NULL) {
      nb_atoms++;
    }
    
  }

  if (nb_atoms == nmax_noH_sch_atoms) {
    *flagH = 0;
  }
  else if (nb_atoms == nmax_H_sch_atoms) {
    *flagH = 1;
  }
  else {
    error = TRUE;
  }

  if (error)
    printf("Error : unable to choose between H or no H version\n");      
  
  return (!error);

}

/**********************************************************************/

static int check_resatoms(res_atoms* resatomsPt, int* flagH, int* flagCterm) {

  int success = FALSE;
  int flagH_bkb = 1;
  int flagH_sch = 1;
  int term = 1;
  
  if (resatomsPt->resType == PROH) {
    success = check_bkb_resatoms(resatomsPt, &flagH_bkb, &term);
  }
  else {
    success = check_bkb_resatoms(resatomsPt, &flagH_bkb, &term);
    if (success && resatomsPt->resType!=GLYH) {
      success = (check_sch_resatoms(resatomsPt, &flagH_sch) && (flagH_bkb == flagH_sch));
      if (!success) {
	printf("ERROR : in residue %d, no coherent bkb and sch\n", resatomsPt->resSeq);
      }
    }
  }
  
  if (success) {
    *flagH = flagH_bkb;
    *flagCterm = term;
  }

  return success;
}

/**********************************************************************/

// ONLY PARTICULAR CASE !!!
static int extract_PRO_atoms(residue *resPt, res_atoms *resatomsPt)
{
  if (!resPt->flagH) {
    // no-H version
    
    resPt->nbkbatoms = N_PRO_BKB_ATOMS;

    if(resatomsPt->bkbatoms[(int) PROH_C] != NULL)
      resPt->bkbAlist[(int) PRO_C] = resatomsPt->bkbatoms[(int) PROH_C];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_O] != NULL)
      resPt->bkbAlist[(int) PRO_O] = resatomsPt->bkbatoms[(int) PROH_O];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_CA] != NULL)
      resPt->bkbAlist[(int) PRO_CA] = resatomsPt->bkbatoms[(int) PROH_CA];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_CB] != NULL)
      resPt->bkbAlist[(int) PRO_CB] = resatomsPt->bkbatoms[(int) PROH_CB];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_CG] != NULL)
      resPt->bkbAlist[(int) PRO_CG] = resatomsPt->bkbatoms[(int) PROH_CG];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_CD] != NULL)
      resPt->bkbAlist[(int) PRO_CD] = resatomsPt->bkbatoms[(int) PROH_CD];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_N] != NULL)
      resPt->bkbAlist[(int) PRO_N] = resatomsPt->bkbatoms[(int) PROH_N];
    else
      return -1;
    if(resPt->flagCterm) {
      resPt->bkbAlist[(int) PRO_OXT] = resatomsPt->bkbatoms[(int) PROH_OXT];
    }
  }
  else {
    // H version
    
    resPt->nbkbatoms = N_PROH_BKB_ATOMS;

    if(resatomsPt->bkbatoms[(int) PROH_C] != NULL)
      resPt->bkbAlist[(int) PROH_C] = resatomsPt->bkbatoms[(int) PROH_C];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_O] != NULL)
      resPt->bkbAlist[(int) PROH_O] = resatomsPt->bkbatoms[(int) PROH_O];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_CA] != NULL)
      resPt->bkbAlist[(int) PROH_CA] = resatomsPt->bkbatoms[(int) PROH_CA];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_HA] != NULL)
      resPt->bkbAlist[(int) PROH_HA] = resatomsPt->bkbatoms[(int) PROH_HA];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_CB] != NULL)
      resPt->bkbAlist[(int) PROH_CB] = resatomsPt->bkbatoms[(int) PROH_CB];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_N] != NULL)
      resPt->bkbAlist[(int) PROH_N] = resatomsPt->bkbatoms[(int) PROH_N];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_1HB] != NULL)
      resPt->bkbAlist[(int) PROH_1HB] = resatomsPt->bkbatoms[(int) PROH_1HB];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_2HB] != NULL)
      resPt->bkbAlist[(int) PROH_2HB] = resatomsPt->bkbatoms[(int) PROH_2HB];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_CG] != NULL)
      resPt->bkbAlist[(int) PROH_CG] = resatomsPt->bkbatoms[(int) PROH_CG];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_1HG] != NULL)
      resPt->bkbAlist[(int) PROH_1HG] = resatomsPt->bkbatoms[(int) PROH_1HG];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_2HG] != NULL)
      resPt->bkbAlist[(int) PROH_2HG] = resatomsPt->bkbatoms[(int) PROH_2HG];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_CD] != NULL)
      resPt->bkbAlist[(int) PROH_CD] = resatomsPt->bkbatoms[(int) PROH_CD];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_1HD] != NULL)
      resPt->bkbAlist[(int) PROH_1HD] = resatomsPt->bkbatoms[(int) PROH_1HD];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) PROH_2HD] != NULL)
      resPt->bkbAlist[(int) PROH_2HD] = resatomsPt->bkbatoms[(int) PROH_2HD];
    else
      return -1;
    if(resPt->flagCterm){
      resPt->bkbAlist[(int) PROH_OXT] = resatomsPt->bkbatoms[(int) PROH_OXT];
    }
  }  
  return 1;
}

/**********************************************************************/

static int extract_bkb_atoms(residue *resPt, res_atoms *resatomsPt)
{
  int i;
  int nmax_bkb_atoms = number_H_bkb_atoms(resPt->resType);
  int n = number_bkb_atoms(resPt->resType);

  if (resPt->nbkbatoms!=0)
    return FALSE;

  if(resatomsPt->resType == PROH) {
    // particular case : different relative order of heavy atoms between H and no-H versions    
    extract_PRO_atoms(resPt,resatomsPt);
  }
  else {
    for (i=0; i<nmax_bkb_atoms; i++) {
      if (is_H_OXT_index(i, resPt->resType) || resatomsPt->bkbatoms[i]!=NULL) {
	resPt->bkbAlist[resPt->nbkbatoms]=resatomsPt->bkbatoms[i];
	resPt->nbkbatoms++;
      }
    }
  }
  
  if (n!=resPt->nbkbatoms)
    return FALSE;

  return TRUE;
}

/**********************************************************************/

static int extract_sch_atoms(residue *resPt, res_atoms *resatomsPt)
{
  int i;
  int nmax_sch_atoms = number_H_sch_atoms(resPt->resType);
  int n = number_sch_atoms(resPt->resType);

  if (resPt->nschatoms!=0)
    return FALSE;
  
  for (i=0; i<nmax_sch_atoms; i++) {
    if (resatomsPt->schatoms[i]!=NULL) {
      resPt->schAlist[resPt->nschatoms]=resatomsPt->schatoms[i];
      resPt->nschatoms++;
    }
  }
  
  if (n!=resPt->nschatoms)
    return FALSE;
  
  return TRUE;
}

/**********************************************************************/

static int extract_and_bond_atoms(residue *resPt, res_atoms *resatomsPt)
{
  if (extract_bkb_atoms(resPt,resatomsPt)
      && extract_sch_atoms(resPt,resatomsPt)) {
    set_residue_bonds(resPt);
    return TRUE;
  }
  else {
    return FALSE;
  }
}

/**********************************************************************/

static int get_HPrime_neighboor(residue* resPt, double* pos,
				atom** neighboor, int* in_bkb) {
  
  atom* aPt = NULL;
  atom *bestatomPt = NULL;
  atomTypes atomType;
  double dist, bestdist;
  int found_in_bkb = TRUE;
  int nmax_bkb_atoms = number_bkb_atoms(resPt->resType);
  int nmax_sch_atoms = number_sch_atoms(resPt->resType);
  int i;
  
  bestdist = HUGE_VAL;
  bestatomPt = NULL;
  
  for(i=0; i<nmax_bkb_atoms; i++) {
    aPt = resPt->bkbAlist[i];
    if (aPt!=NULL) {
      atomType = aPt->atomType;
      if (atomType==OXYGEN || atomType==OXYGEN_H 
	  || atomType==NITROGEN || atomType==NITROGEN_H || atomType==NITROGEN_FULL) {
	compute_distance(aPt->pos,pos,&dist);
	if(dist < bestdist) {
	  bestdist = dist;
	  bestatomPt = aPt;
	}
      }
    }
  }

  for(i=0; i<nmax_sch_atoms; i++) {
    aPt = resPt->schAlist[i];
    if (aPt!=NULL) {
      atomType = aPt->atomType;
      if (atomType==OXYGEN || atomType==OXYGEN_H 
	  || atomType==NITROGEN || atomType==NITROGEN_H || atomType==NITROGEN_FULL) {
	compute_distance(aPt->pos,pos,&dist);
	if(dist < bestdist) {
	  bestdist = dist;
	  bestatomPt = aPt;
	  if (found_in_bkb)
	    found_in_bkb = FALSE;
	}
      }
    }
  }
  
  if (bestatomPt!=NULL) {
    *neighboor = bestatomPt;
    *in_bkb = found_in_bkb;
  }

  return (bestatomPt!=NULL);
}

/**********************************************************************/

static atom* create_HPrime_atom(int serial, char* name, double* pos){
  
  int i;

  atom* newAtomPt = (atom*)malloc(sizeof(atom));
  newAtomPt->serial = serial;
  strcpy(newAtomPt->name, name);
  newAtomPt->atomType = HYDROGEN_P;
  for (i=0; i<3; i++)
    newAtomPt->pos[i]=pos[i];
  newAtomPt->vdwR = H_VDWR;
  newAtomPt->nbondedA = 0;
  newAtomPt->bondedAlist = NULL;

  return newAtomPt;
}


/**********************************************************************/

static int extract_and_bond_unknown_atoms(residue* resPt, res_atoms* resatomsPt) {

  int extracted = TRUE;
  int in_bkb = TRUE;
  atom* closest_aPt = NULL;
  atom* newAtomPt = NULL;
  unknown_atom* unknown_atomPt = NULL;
  int i;
  int n_term_H = 0;
  int is_PROH_term = FALSE;

  if (resatomsPt->n_unknownAtoms!=0) {
 
    if (!(resPt->flagH)) {
      printf("ERROR : too many atoms in residue %d\n", resPt->resSeq);      
      extracted = FALSE;
    }
    else {
      
      i=0;
      while (i<resatomsPt->n_unknownAtoms && extracted){
	
	unknown_atomPt = resatomsPt->unknownAtoms[i];
	
	if (get_HPrime_neighboor(resPt, unknown_atomPt->pos, &closest_aPt, &in_bkb)) {
	    newAtomPt = create_HPrime_atom(unknown_atomPt->serial,
					    unknown_atomPt->name,
					    unknown_atomPt->pos);


	    if (in_bkb) {
	      insert_pointer_in_list((void*)newAtomPt,
				     (void***)&(resPt->bkbAlist), &(resPt->nbkbatoms));
	      
	      // To determmine if the residue is a N-terminal one
	      if (resPt->resType!=PROH) {
		if ((n_term_H<2) && (closest_aPt->atomType==NITROGEN
				     || closest_aPt->atomType==NITROGEN_FULL
				     || closest_aPt->atomType==NITROGEN_H)) {
		  // in that case, given that there is only one N atom in a residue bkb,
		  // if there is more than 1 H atoms bonded to it, the associated residue
		  // is terminal
		  n_term_H++;
		}
	      }
	      else {
		is_PROH_term = closest_aPt->atomType==NITROGEN
		  || closest_aPt->atomType==NITROGEN_FULL
		  || closest_aPt->atomType==NITROGEN_H;
	      }
	      
	    }
	    else {
	      insert_pointer_in_list((void*)newAtomPt, 
				     (void***)&(resPt->schAlist), &(resPt->nschatoms));	      
	    }
	    set_bond(closest_aPt, newAtomPt);
	    i++;
	}
	else {
	  printf("ERROR : unknown atom %d not recognized as H Prime\n", unknown_atomPt->serial);
	  extracted = FALSE;
	}

	
      }
    }
  
  }
  if (resPt->resType!=PROH) {
    resPt->flagNterm = (extracted && (n_term_H==2));
  }
  else {
    resPt->flagNterm = (extracted && (is_PROH_term));
  }
  return extracted;
}

/**********************************************************************/

static int fill_residue_struct(residue **resPtPt, res_atoms *resatomsPt, formatTypes pdb_format)
{
  int i;
  residue *resPt;
  int flagH = 1;
  int flagCterm = 1;
  int nmax_bkb_atoms, nmax_sch_atoms;

  if (!check_resatoms(resatomsPt, &flagH, &flagCterm)) {
    printf("ERROR : incomplete residue %d\n", resatomsPt->resSeq);
    return -1;
  }

  resPt = (residue *) malloc(sizeof(residue)); 
  *resPtPt = resPt;

  strcpy(resPt->chainID,resatomsPt->chainID);
  resPt->resSeq = resatomsPt->resSeq;
  strcpy(resPt->resName,resatomsPt->resName);
  resPt->resType = resatomsPt->resType;
  resPt->flagH = flagH;
  resPt->flagCterm = flagCterm;
  resPt->flagNterm = 0;

  if (!flagH) {
    noHresidueType(resPt);
  }
  nmax_bkb_atoms = number_bkb_atoms(resPt->resType);
  nmax_sch_atoms = number_sch_atoms(resPt->resType);

  // Init lists
  resPt->nbkbatoms = 0; // will be updated in extract_bkb_atoms
  resPt->nschatoms = 0; // will be updated in extract_sch_atoms
  resPt->bkbAlist = (atom**) malloc(sizeof(atom *) * nmax_bkb_atoms);
  resPt->schAlist = (atom**) malloc(sizeof(atom *) * nmax_sch_atoms);
  for(i=0; i<nmax_bkb_atoms; i++) 
    resPt->bkbAlist[i] = NULL;
  for(i=0; i<nmax_sch_atoms; i++) 
    resPt->schAlist[i] = NULL;

  // NOTE : residues must be complete (i.e. no missing atoms) 
  //        in both H and no-H versions 
  //      * required by BCD and protein-to-p3d translator

  // set bonded atoms in residue (not including unknown atoms) 
  if (!extract_and_bond_atoms(resPt, resatomsPt)) {
    printf("ERROR while extracting atom datas to create residue %d\n", resPt->resSeq);
    free(resPt);
    return -1;
  }

  // extract no translated atoms
  if (!extract_and_bond_unknown_atoms(resPt, resatomsPt)) {
    free(resPt);
    return -1;
  }

  // put in every atom a pointer to the residue
  for(i=0; i<resPt->nbkbatoms; i++)
    if (flagCterm || (!is_OXT_index(i, resPt->resType)))
      resPt->bkbAlist[i]->residuePt = resPt;
  for(i=0; i<resPt->nschatoms; i++) 
      resPt->schAlist[i]->residuePt = resPt;

  // next (default)
  resPt->next = NULL;
  if(resPt->flagCterm)
    return 2;

  return 1;
}

/**********************************************************************/

static int consecutive_resudiues(residue *res1Pt, residue *res2Pt)
{   
  double *a1pos,*a2pos;
  double posdiff[3],dist;

  if(res1Pt->resSeq != (res2Pt->resSeq - 1)) {
    get_C_pos(res1Pt,&a1pos);
    get_N_pos(res2Pt,&a2pos);
    vectSub(a2pos,a1pos,posdiff);
    dist = vectNorm(posdiff);
    if(dist > 3.0)
      return 0;
    else
      return 1;
  }
  else
    return 1;
}


/**********************************************************************/
// PDB READING FUNCTIONS

/**********************************************************************/

static int read_atom(FILE *pdbfile, pdb_atom_data *adataPt, formatTypes pdb_format)
{
  char *pdbline;
  char piece[10];
  char rec[10];
  char returned_s[100];
  char pdb_atom_name[5];
  char pdb_res_name[4];
  

  pdbline = fgets(returned_s,100,pdbfile);
  if(pdbline == NULL) {
     return 0;
  }
  npdfline++;
    
  strcpy(piece,"         ");
  strncpy(piece,pdbline,6);
  if(sscanf(piece,"%s",rec) < 0) return -2;
  if(strcmp(rec,"ATOM") != 0) return -2;

  // extract only used data
  strcpy(piece,"         ");
  strncpy(piece,pdbline+6,5);
  if(sscanf(piece,"%d",&adataPt->serial) < 0) return -1;
  
  strcpy(piece,"         ");
  strncpy(piece,pdbline+12,4);
  if(sscanf(piece,"%s",pdb_atom_name) < 0) return -1;

  strcpy(piece,"         ");
  strncpy(piece,pdbline+21,1);
  sscanf(piece,"%s",adataPt->chainID);
  // when no chainID
  if(strcmp(adataPt->chainID,"") == 0)
    strcpy(adataPt->chainID,"0");
  
  strcpy(piece,"         ");
  strncpy(piece,pdbline+17,3);
  if(sscanf(piece,"%s",pdb_res_name) < 0) return -1;
  // read_res_name (with format)
  
  strcpy(piece,"         ");
  strncpy(piece,pdbline+22,4);
  if(sscanf(piece,"%d",&adataPt->resSeq) < 0) return -1;
  strcpy(piece,"         ");
  strncpy(piece,pdbline+30,8);
  if(sscanf(piece,"%lf",&adataPt->pos[0]) < 0) return -1;
  strcpy(piece,"         ");
  strncpy(piece,pdbline+38,8);
  if(sscanf(piece,"%lf",&adataPt->pos[1]) < 0) return -1;
  strcpy(piece,"         ");
  strncpy(piece,pdbline+46,8);
  if(sscanf(piece,"%lf",&adataPt->pos[2]) < 0) return -1; 


  translate_pdb_res_name(pdb_res_name,adataPt->resName,pdb_format);
  // identify residue type (index)
  if(resName_to_resType(adataPt->resName, &adataPt->resType) < 0) {
    return -1;
  }

  translate_pdb_atom_name(pdb_atom_name,adataPt->name,adataPt->resType,pdb_format);

  // write flag read_OXT
  if(strcmp(adataPt->name,"OXT") == 0) 
    read_OXT = 1;
  else
    read_OXT = 0;

  return 1;
}

/**********************************************************************/
    
static int read_residue(FILE *pdbfile, residue **resPtPt, formatTypes pdb_format)
{
  int state, state2;
  res_atoms* theatomsPt;
  pdb_atom_data *adataPt;    

  // new residu
  read_OXT = 0;

  if(first_time_reading) {
    // read first atom
    state = -2;
    while(state == -2) {
      state = read_atom(pdbfile,&firstatom, pdb_format);
    }
    if(state <= 0) {
      if(state == -1) {
	printf("ERROR : wrong PDB ATOM line : %d\n",npdfline);
      }
      return state;
    }
    first_time_reading = 0;
  }

  theatomsPt = create_res_atoms(firstatom.resType,
				firstatom.resSeq,
				firstatom.chainID,
				firstatom.resName);

  adataPt = &firstatom;
  state = 1;
  // REMEMBER !!!:
  // state == -1 -> failure
  // state == 0  -> eof
  while((adataPt->resSeq == theatomsPt->resSeq) && (state > 0)) {
    if(insert_adata_in_res_atoms(adataPt, theatomsPt, theatomsPt->resType) < 0) {
      free_res_atoms(theatomsPt);
      return -1;
    }
    state = -2;    
    while(state == -2) {
      state = read_atom(pdbfile,adataPt, pdb_format);
    }    
    if(state == -1) {
      printf("ERROR : wrong PDB ATOM line : %d\n",npdfline);
      free_res_atoms(theatomsPt);
      return -1;
    }
  }

  // put res_atoms in residue structure (protein.h)
  state2 = fill_residue_struct(resPtPt, theatomsPt, pdb_format);


  if((state2 > 0) && (state == 0)) {
    if(state2 != 2) {
      printf("WARNING : no terminal Oxigen in chain %s\n",theatomsPt->chainID);
    }     
    free_res_atoms(theatomsPt);
    return 0;
  }
 
  free_res_atoms(theatomsPt);
  return state2;
}



/**********************************************************************/
// PROTEIN STRUCTURE WRITING FUNCTIONS

int fill_protein_struct(FILE *pdbfile, char *pdbfilename, protein *protPt)
{
  int state, prevstate;
  residue *theresPt=NULL,*prevresPt=NULL;
  AAchain *curAAchainPt;
  formatTypes pdb_format;

  // get pdb_format
  if (!scanFile(pdbfile, &pdb_format)) {
    printf("Unable to define pdb format (AMBER or INSIGHT)\n");
    return -1;
  }
  if (pdb_format==AMBER)
    printf("AMBER source file\n");
  if (pdb_format==INSIGHT)
    printf("INSIGHT source file\n");

  // protein name
  strcpy(protPt->name,pdbfilename);

  // alloc first chain
  protPt->nchains = 0;
  curAAchainPt = (AAchain *) malloc(sizeof(AAchain)); 
  insert_pointer_in_list(curAAchainPt,(void ***)&(protPt->chainlist),&(protPt->nchains));
  curAAchainPt->proteinPt = protPt;
  curAAchainPt->nresidues = 0;
  state = 1;

  // read residues
  while(state > 0){  // state == 0 -> last residue 
    state = read_residue(pdbfile,&theresPt,pdb_format);
    if(state < 0)
      return -1;

    // if new chain
    if(prevresPt != NULL) {
      if(strcmp(prevresPt->chainID,theresPt->chainID) != 0) {
	if(prevstate != 2) {
	  printf("WARNING : no terminal Oxigen in chain %s\n",prevresPt->chainID);
	}
	curAAchainPt = (AAchain *) malloc(sizeof(AAchain)); 
	insert_pointer_in_list(curAAchainPt,(void ***)&(protPt->chainlist),&(protPt->nchains));
	curAAchainPt->nresidues = 0;
	theresPt->subchainind = 0;
	// pointer to prev (if new chain) 
	theresPt->prev = NULL;
	// pointer to next in prev
	prevresPt->next = NULL;
      }
      // residues must be consecutive in the same chain
      // WARNING : they can be consecutive must can have non-consecutive resSeq
      // otherwise a new chain is created
      // (other possibility is to return ERROR)
      else if(!consecutive_resudiues(prevresPt,theresPt)) {
	// printf("ERROR : no consecutive residues : %d - %d\n",
	//         prevresPt->resSeq,theresPt->resSeq);
	// return -1;      
	printf("WARNING : no consecutive residues : %d - %d\n",
	       prevresPt->resSeq,theresPt->resSeq);

	curAAchainPt = (AAchain *) malloc(sizeof(AAchain)); 
	insert_pointer_in_list(curAAchainPt,(void ***)&(protPt->chainlist),&(protPt->nchains));
	curAAchainPt->nresidues = 0;
	theresPt->subchainind = prevresPt->subchainind + 1;
	// pointer to prev (if new chain)
	theresPt->prev = NULL;
	// pointer to next in prev
	prevresPt->next = theresPt;
      }
      else {    
	// pointer to next in prev
	prevresPt->next = theresPt;
	theresPt->subchainind = prevresPt->subchainind;
      }
    }
    else {
      theresPt->subchainind = 0;
    }

    // pointer to prev
    theresPt->prev = prevresPt;

    theresPt->chainPt = curAAchainPt;

    insert_pointer_in_list(theresPt,(void ***)&(curAAchainPt->reslist),&(curAAchainPt->nresidues));

    prevresPt = theresPt;    
    prevstate = state;

    // set peptide bond with previous residue
    if(theresPt->prev != NULL) {
      set_peptide_bond(theresPt,theresPt->prev);
    }
  }
  
  return 1;
}


/**********************************************************************/
// AUXILIAR FUNCTIONS

void free_protein(protein *protPt)
{

  // TO DO  
  
}
