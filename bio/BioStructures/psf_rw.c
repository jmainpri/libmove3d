
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "P3d-pkg.h"
#include "Bio-pkg.h"

//////// PRIVATE TYPES ////////////////////////////////////////////////////

// PDB READING STATE
typedef enum {
  RESIDU_ATOM_LINE, LIGAND_ATOM_LINE,
  PDB_END, OTHER_LINE, WRONG_LINE
} lineType;

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

/**********************************************************************/

// INTERMEDIATE AMINO-ACID STRUCTURE

#define MAX_BKB_ATOMS 15
#define MAX_SCH_ATOMS 17 //Il faut indiquer le max avec les H' compris !!!!
typedef struct s_res_atoms {
  //char chainID[2];
  char resName[4];  
  residueTypes resType;
  int resSeq;
  struct s_psf_atom *bkbatoms[MAX_BKB_ATOMS]; 
  struct s_psf_atom *schatoms[MAX_SCH_ATOMS]; 
} res_atoms;


typedef struct s_check_explored_atoms {
  int nbchecked;
  int* check_list;
} check_explored_atoms;

//////// GLOBAL VARIABLES ////////////////////////////////////////////////////




/////////// PROTOTYPE OF PRIVATE FUNCTIONS /////////////////////////////////////

static void noHresidueType(psf_residue *resPt);

static void set_bond(psf_atom *a1Pt, psf_atom *a2Pt);
static void set_peptide_bond(psf_residue *resPt, psf_residue *prevresPt);
static void set_residue_bonds(psf_residue *resPt);
static void set_ligand_bonds (psf_ligand* ligPt);

static int nb_No_HPrime_bkb_atoms(psf_residue *resPt);
static int nb_No_HPrime_sch_atoms(psf_residue *resPt);
static int consecutive_residues(psf_residue *res1Pt, psf_residue *res2Pt);

static void init_res_atoms(res_atoms *resatomsPt);
static psf_atom *copy_atom_data(pdb_atom_data *adataPt);
static int insert_bkb_adata_in_res_atoms(pdb_atom_data *adataPt, res_atoms *resatomsPt, residueTypes resType);
static int insert_sch_adata_in_res_atoms(pdb_atom_data *adataPt, res_atoms *resatomsPt, residueTypes resType);
static int insert_adata_in_res_atoms(pdb_atom_data *adataPt, res_atoms *resatomsPt, residueTypes resType);

static int extract_PRO_atoms(psf_residue *resPt, res_atoms *resatomsPt);
static int extract_bkb_atoms(psf_residue *resPt, res_atoms *resatomsPt);
static int extract_sch_atoms(psf_residue *resPt, res_atoms *resatomsPt);
static int extract_atoms_to_res(psf_residue *resPt, res_atoms *resatomsPt);

static int fill_residue_struct(psf_residue **resPtPt, res_atoms *resatomsPt);

static void compute_ligand_centroid(psf_ligand *ligPt, double *point);
static int get_ligand_atom_index(psf_ligand* ligPt, psf_atom* theAtom, int* index);
static int set_as_explored(psf_ligand* ligPt, psf_atom* bondedAtomPt, check_explored_atoms* check);
static int check_if_explored(psf_ligand* ligPt, psf_atom* bondedAtomPt, check_explored_atoms* check);

static int in_joint_axis(psf_ligand* ligPt, psf_atom *atomPt_i, psf_atom *atomPt_j, psf_joint **bondjntPtPt);
static int construct_rigid_from_root_atom(psf_ligand *ligPt, psf_atom *rootaPt, 
					  psf_atom ***nextrootalist, int *nnextroota,
					  check_explored_atoms* check);

static int generate_rigids_in_ligand(psf_ligand *ligPt);

static int is_protein_desc_line(char *pdbline);
static lineType read_atom(FILE *pdbfile, char *pdbLine, int length);

static lineType read_protein_atom(char* pdbLine, pdb_atom_data *adataPt);
static int read_residue(FILE *pdbfile, char* pdbLine, lineType* last_pdb_reading_state,
			psf_residue **resPtPt, char* chainID);

static char* get_ligand_name(char* pdbLine);
static lineType read_ligand_atom(char* pdbLine, psf_atom* aPt);

static int extract_psf_protein_struct(FILE *pdbfile, char* pdbLine, lineType* pdb_reading_state,
				      char *shortFileName, psf_protein *protPt);
static int extract_psf_ligand_struct(FILE *pdbfile, char* pdbLine, lineType* pdb_reading_state,
				     char *shortFileName, psf_ligand *ligPt);

static int read_and_treat_joint(char *pdbline, psf_ligand *ligPt);
static void write_joint_in_file(FILE* jntFile, psf_joint* joint);

//////// USEFUL FONCTIONS ////////////////////////////////////////////////////

// Protein

static void noHresidueType(psf_residue *resPt)
{
  switch(resPt->resType) {
  case psf_ALAH: 
    resPt->resType = psf_ALA;
    break;
  case psf_ARGH: 
    resPt->resType = psf_ARG;
    break;    
  case psf_ASNH: 
    resPt->resType = psf_ASN;
    break;
  case psf_ASPH: 
    resPt->resType = psf_ASP;
    break;
  case psf_CYSH: 
    resPt->resType = psf_CYS;
    break;
  case psf_GLNH: 
    resPt->resType = psf_GLN;
    break;
  case psf_GLUH: 
    resPt->resType = psf_GLU;
    break;
  case psf_GLYH: 
    resPt->resType = psf_GLY;
    break;
  case psf_HISH: 
    resPt->resType = psf_HIS;
    break;
  case psf_ILEH: 
    resPt->resType = psf_ILE;
    break;
  case psf_LEUH: 
    resPt->resType = psf_LEU;
    break;
  case psf_LYSH: 
    resPt->resType = psf_LYS;
    break;
  case psf_METH: 
    resPt->resType = psf_MET;
    break;
  case psf_PHEH: 
    resPt->resType = psf_PHE;
    break;
  case psf_PROH: 
    resPt->resType = psf_PRO;
    break;
  case psf_SERH: 
    resPt->resType = psf_SER;
    break;
  case psf_THRH: 
    resPt->resType = psf_THR;
    break;
  case psf_TRPH: 
    resPt->resType = psf_TRP;
    break;
  case psf_TYRH: 
    resPt->resType = psf_TYR;
    break;
  case psf_VALH: 
    resPt->resType = psf_VAL;
    break;
  default:
    break;
  }
}

/**********************************************************************/

// Bonds setting

static void set_bond(psf_atom *a1Pt, psf_atom *a2Pt)
{
  insert_pointer_in_list(a1Pt,(void ***)&(a2Pt->bondedAtomList),&(a2Pt->nbondedA));
  insert_pointer_in_list(a2Pt,(void ***)&(a1Pt->bondedAtomList),&(a1Pt->nbondedA));
}

static void set_peptide_bond(psf_residue *resPt, psf_residue *prevresPt)
{
  set_bond(get_N(resPt),get_C(prevresPt));
}


static void set_residue_bonds(psf_residue *resPt)
{

  // BE CAREFULL ! Each time a bond involving H' is going
  // to be made, the existence of this H' atom must be
  // tested (because H' atoms are present in some models
  // and missing in others)


  // set bonds between backbone atoms
  switch(resPt->resType) {
  case psf_GLY:
    set_bond(resPt->bkbAtomList[psf_GLY_N],resPt->bkbAtomList[psf_GLY_CA]);
    set_bond(resPt->bkbAtomList[psf_GLY_CA],resPt->bkbAtomList[psf_GLY_C]);
    set_bond(resPt->bkbAtomList[psf_GLY_C],resPt->bkbAtomList[psf_GLY_O]);
    if(resPt->flagCterm) {
      set_bond(resPt->bkbAtomList[psf_GLY_C],resPt->bkbAtomList[psf_GLY_OXT]);
    }
    break;
  case psf_GLYH:
    set_bond(resPt->bkbAtomList[psf_GLYH_N],resPt->bkbAtomList[psf_GLYH_CA]);
    set_bond(resPt->bkbAtomList[psf_GLYH_CA],resPt->bkbAtomList[psf_GLYH_C]);
    set_bond(resPt->bkbAtomList[psf_GLYH_C],resPt->bkbAtomList[psf_GLYH_O]);
    set_bond(resPt->bkbAtomList[psf_GLYH_CA],resPt->bkbAtomList[psf_GLYH_1HA]);
    set_bond(resPt->bkbAtomList[psf_GLYH_CA],resPt->bkbAtomList[psf_GLYH_2HA]);
    if(resPt->flagCterm) {
      set_bond(resPt->bkbAtomList[psf_GLYH_C],resPt->bkbAtomList[psf_GLYH_OXT]);
    }
    // H' atoms
    if (resPt->bkbAtomList[psf_GLYH_H]!=NULL)
      set_bond(resPt->bkbAtomList[psf_GLYH_N],resPt->bkbAtomList[psf_GLYH_H]);
    if (resPt->bkbAtomList[psf_GLYH_H_Nterm]!=NULL)
      set_bond(resPt->bkbAtomList[psf_GLYH_N],resPt->bkbAtomList[psf_GLYH_H_Nterm]);
    break;
  case psf_PRO:
    set_bond(resPt->bkbAtomList[psf_PRO_N],resPt->bkbAtomList[psf_PRO_CA]);
    set_bond(resPt->bkbAtomList[psf_PRO_CA],resPt->bkbAtomList[psf_PRO_C]);
    set_bond(resPt->bkbAtomList[psf_PRO_C],resPt->bkbAtomList[psf_PRO_O]);
    set_bond(resPt->bkbAtomList[psf_PRO_CA],resPt->bkbAtomList[psf_PRO_CB]);
    set_bond(resPt->bkbAtomList[psf_PRO_CB],resPt->bkbAtomList[psf_PRO_CG]);
    set_bond(resPt->bkbAtomList[psf_PRO_CG],resPt->bkbAtomList[psf_PRO_CD]);
    set_bond(resPt->bkbAtomList[psf_PRO_CD],resPt->bkbAtomList[psf_PRO_N]);
    if(resPt->flagCterm) {
      set_bond(resPt->bkbAtomList[psf_PRO_C],resPt->bkbAtomList[psf_PRO_OXT]);
    }
    break;
  case psf_PROH:
    set_bond(resPt->bkbAtomList[psf_PROH_N],resPt->bkbAtomList[psf_PROH_CA]);
    set_bond(resPt->bkbAtomList[psf_PROH_CA],resPt->bkbAtomList[psf_PROH_C]);
    set_bond(resPt->bkbAtomList[psf_PROH_C],resPt->bkbAtomList[psf_PROH_O]);
    set_bond(resPt->bkbAtomList[psf_PROH_CA],resPt->bkbAtomList[psf_PROH_CB]);
    set_bond(resPt->bkbAtomList[psf_PROH_CB],resPt->bkbAtomList[psf_PROH_CG]);
    set_bond(resPt->bkbAtomList[psf_PROH_CG],resPt->bkbAtomList[psf_PROH_CD]);
    set_bond(resPt->bkbAtomList[psf_PROH_CD],resPt->bkbAtomList[psf_PROH_N]);
    set_bond(resPt->bkbAtomList[psf_PROH_CA],resPt->bkbAtomList[psf_PROH_HA]);
    set_bond(resPt->bkbAtomList[psf_PROH_CB],resPt->bkbAtomList[psf_PROH_1HB]);
    set_bond(resPt->bkbAtomList[psf_PROH_CB],resPt->bkbAtomList[psf_PROH_2HB]);
    set_bond(resPt->bkbAtomList[psf_PROH_CG],resPt->bkbAtomList[psf_PROH_1HG]);
    set_bond(resPt->bkbAtomList[psf_PROH_CG],resPt->bkbAtomList[psf_PROH_2HG]);
    set_bond(resPt->bkbAtomList[psf_PROH_CD],resPt->bkbAtomList[psf_PROH_1HD]);
    set_bond(resPt->bkbAtomList[psf_PROH_CD],resPt->bkbAtomList[psf_PROH_2HD]);
    if(resPt->flagCterm) {
      set_bond(resPt->bkbAtomList[psf_PROH_C],resPt->bkbAtomList[psf_PROH_OXT]);
    }
    //  H' atoms
    // no H' on psf_PROH
    break;
  default:
    //if(((int) resPt->resType) < 20)
    if(resPt->flagH == 0) {
      set_bond(resPt->bkbAtomList[psf_gen_N],resPt->bkbAtomList[psf_gen_CA]);
      set_bond(resPt->bkbAtomList[psf_gen_CA],resPt->bkbAtomList[psf_gen_C]);
      set_bond(resPt->bkbAtomList[psf_gen_C],resPt->bkbAtomList[psf_gen_O]);
      set_bond(resPt->bkbAtomList[psf_gen_CA],resPt->bkbAtomList[psf_gen_CB]);
      if(resPt->flagCterm) {
	set_bond(resPt->bkbAtomList[psf_gen_C],resPt->bkbAtomList[psf_gen_OXT]);
      }
    }
    else {
      set_bond(resPt->bkbAtomList[psf_genH_N],resPt->bkbAtomList[psf_genH_CA]);
      set_bond(resPt->bkbAtomList[psf_genH_CA],resPt->bkbAtomList[psf_genH_C]);
      set_bond(resPt->bkbAtomList[psf_genH_C],resPt->bkbAtomList[psf_genH_O]);
      set_bond(resPt->bkbAtomList[psf_genH_CA],resPt->bkbAtomList[psf_genH_CB]);
      set_bond(resPt->bkbAtomList[psf_genH_CA],resPt->bkbAtomList[psf_genH_HA]);
      if(resPt->flagCterm) {
	set_bond(resPt->bkbAtomList[psf_genH_C],resPt->bkbAtomList[psf_genH_OXT]);
      }
      // H' atoms
      if(resPt->bkbAtomList[psf_genH_H]!=NULL)
	set_bond(resPt->bkbAtomList[psf_genH_N],resPt->bkbAtomList[psf_genH_H]);
      if(resPt->bkbAtomList[psf_genH_H_Nterm]!=NULL)
	set_bond(resPt->bkbAtomList[psf_genH_N],resPt->bkbAtomList[psf_genH_H_Nterm]);
    }
  }

  // set bonds between side-chain atoms
  switch(resPt->resType) {
  case psf_ALA:
    break;
  case psf_ARG:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_ARG_CG]);
    set_bond(resPt->schAtomList[psf_ARG_CG],resPt->schAtomList[psf_ARG_CD]);
    set_bond(resPt->schAtomList[psf_ARG_CD],resPt->schAtomList[psf_ARG_NE]);
    set_bond(resPt->schAtomList[psf_ARG_NE],resPt->schAtomList[psf_ARG_CZ]);
    set_bond(resPt->schAtomList[psf_ARG_CZ],resPt->schAtomList[psf_ARG_NH1]);
    set_bond(resPt->schAtomList[psf_ARG_CZ],resPt->schAtomList[psf_ARG_NH2]);
    break;
  case psf_ASN:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_ASN_CG]);
    set_bond(resPt->schAtomList[psf_ASN_CG],resPt->schAtomList[psf_ASN_OD1]);
    set_bond(resPt->schAtomList[psf_ASN_CG],resPt->schAtomList[psf_ASN_ND2]);
    break;
  case psf_ASP:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_ASP_CG]);
    set_bond(resPt->schAtomList[psf_ASP_CG],resPt->schAtomList[psf_ASP_OD1]);
    set_bond(resPt->schAtomList[psf_ASP_CG],resPt->schAtomList[psf_ASP_OD2]);
     break;
  case psf_CYS:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_CYS_SG]);
    break;
  case psf_GLN:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_GLN_CG]);
    set_bond(resPt->schAtomList[psf_GLN_CG],resPt->schAtomList[psf_GLN_CD]);
    set_bond(resPt->schAtomList[psf_GLN_CD],resPt->schAtomList[psf_GLN_OE1]);
    set_bond(resPt->schAtomList[psf_GLN_CD],resPt->schAtomList[psf_GLN_NE2]);
    break;
  case psf_GLU:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_GLU_CG]);
    set_bond(resPt->schAtomList[psf_GLU_CG],resPt->schAtomList[psf_GLU_CD]);
    set_bond(resPt->schAtomList[psf_GLU_CD],resPt->schAtomList[psf_GLU_OE1]);
    set_bond(resPt->schAtomList[psf_GLU_CD],resPt->schAtomList[psf_GLU_OE2]);
    break;
  case psf_GLY:
    break;
  case psf_HIS:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_HIS_CG]);
    set_bond(resPt->schAtomList[psf_HIS_CG],resPt->schAtomList[psf_HIS_ND1]);
    set_bond(resPt->schAtomList[psf_HIS_CG],resPt->schAtomList[psf_HIS_CD2]);
    set_bond(resPt->schAtomList[psf_HIS_ND1],resPt->schAtomList[psf_HIS_CE1]);
    set_bond(resPt->schAtomList[psf_HIS_CD2],resPt->schAtomList[psf_HIS_NE2]);
    set_bond(resPt->schAtomList[psf_HIS_CE1],resPt->schAtomList[psf_HIS_NE2]);
   break;
  case psf_ILE:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_ILE_CG1]);
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_ILE_CG2]);
    set_bond(resPt->schAtomList[psf_ILE_CG1],resPt->schAtomList[psf_ILE_CD1]);
    break;
  case psf_LEU:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_LEU_CG]);
    set_bond(resPt->schAtomList[psf_LEU_CG],resPt->schAtomList[psf_LEU_CD1]);
    set_bond(resPt->schAtomList[psf_LEU_CG],resPt->schAtomList[psf_LEU_CD2]);
    break;
  case psf_LYS:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_LYS_CG]);
    set_bond(resPt->schAtomList[psf_LYS_CG],resPt->schAtomList[psf_LYS_CD]);
    set_bond(resPt->schAtomList[psf_LYS_CD],resPt->schAtomList[psf_LYS_CE]);
    set_bond(resPt->schAtomList[psf_LYS_CE],resPt->schAtomList[psf_LYS_NZ]);
    break;
  case psf_MET:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_MET_CG]);
    set_bond(resPt->schAtomList[psf_MET_CG],resPt->schAtomList[psf_MET_SD]);
    set_bond(resPt->schAtomList[psf_MET_SD],resPt->schAtomList[psf_MET_CE]);
    break;
  case psf_PHE:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_PHE_CG]);
    set_bond(resPt->schAtomList[psf_PHE_CG],resPt->schAtomList[psf_PHE_CD1]);
    set_bond(resPt->schAtomList[psf_PHE_CG],resPt->schAtomList[psf_PHE_CD2]);
    set_bond(resPt->schAtomList[psf_PHE_CD1],resPt->schAtomList[psf_PHE_CE1]);
    set_bond(resPt->schAtomList[psf_PHE_CD2],resPt->schAtomList[psf_PHE_CE2]);
    set_bond(resPt->schAtomList[psf_PHE_CE1],resPt->schAtomList[psf_PHE_CZ]);
    set_bond(resPt->schAtomList[psf_PHE_CE2],resPt->schAtomList[psf_PHE_CZ]);
    break;
  case psf_PRO:
    break;
  case psf_SER:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_SER_OG]);
    break;
  case psf_THR:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_THR_OG1]);
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_THR_CG2]);
    break;
  case psf_TRP:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_TRP_CG]);
    set_bond(resPt->schAtomList[psf_TRP_CG],resPt->schAtomList[psf_TRP_CD1]);
    set_bond(resPt->schAtomList[psf_TRP_CG],resPt->schAtomList[psf_TRP_CD2]);
    set_bond(resPt->schAtomList[psf_TRP_CD1],resPt->schAtomList[psf_TRP_NE1]);
    set_bond(resPt->schAtomList[psf_TRP_NE1],resPt->schAtomList[psf_TRP_CE2]);
    set_bond(resPt->schAtomList[psf_TRP_CD2],resPt->schAtomList[psf_TRP_CE2]);
    set_bond(resPt->schAtomList[psf_TRP_CD2],resPt->schAtomList[psf_TRP_CE3]);
    set_bond(resPt->schAtomList[psf_TRP_CE2],resPt->schAtomList[psf_TRP_CZ2]);
    set_bond(resPt->schAtomList[psf_TRP_CE3],resPt->schAtomList[psf_TRP_CZ3]);
    set_bond(resPt->schAtomList[psf_TRP_CZ2],resPt->schAtomList[psf_TRP_CH2]);
    set_bond(resPt->schAtomList[psf_TRP_CZ3],resPt->schAtomList[psf_TRP_CH2]);
   break;
  case psf_TYR:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_TYR_CG]);
    set_bond(resPt->schAtomList[psf_TYR_CG],resPt->schAtomList[psf_TYR_CD1]);
    set_bond(resPt->schAtomList[psf_TYR_CG],resPt->schAtomList[psf_TYR_CD2]);
    set_bond(resPt->schAtomList[psf_TYR_CD1],resPt->schAtomList[psf_TYR_CE1]);
    set_bond(resPt->schAtomList[psf_TYR_CD2],resPt->schAtomList[psf_TYR_CE2]);
    set_bond(resPt->schAtomList[psf_TYR_CE1],resPt->schAtomList[psf_TYR_CZ]);
    set_bond(resPt->schAtomList[psf_TYR_CE2],resPt->schAtomList[psf_TYR_CZ]);
    set_bond(resPt->schAtomList[psf_TYR_CZ],resPt->schAtomList[psf_TYR_OH]);
    break;
  case psf_VAL:
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_VAL_CG1]);
    set_bond(resPt->bkbAtomList[psf_gen_CB],resPt->schAtomList[psf_VAL_CG2]);
    break;
  //- with H --------------------------------------------------
  case psf_ALAH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ALAH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ALAH_2HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ALAH_3HB]);
    break;
  case psf_ARGH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ARGH_CG]);
    set_bond(resPt->schAtomList[psf_ARGH_CG],resPt->schAtomList[psf_ARGH_CD]);
    set_bond(resPt->schAtomList[psf_ARGH_CD],resPt->schAtomList[psf_ARGH_NE]);
    set_bond(resPt->schAtomList[psf_ARGH_NE],resPt->schAtomList[psf_ARGH_CZ]);
    set_bond(resPt->schAtomList[psf_ARGH_CZ],resPt->schAtomList[psf_ARGH_NH1]);
    set_bond(resPt->schAtomList[psf_ARGH_CZ],resPt->schAtomList[psf_ARGH_NH2]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ARGH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ARGH_2HB]);
    set_bond(resPt->schAtomList[psf_ARGH_CG],resPt->schAtomList[psf_ARGH_1HG]);
    set_bond(resPt->schAtomList[psf_ARGH_CG],resPt->schAtomList[psf_ARGH_2HG]);
    set_bond(resPt->schAtomList[psf_ARGH_CD],resPt->schAtomList[psf_ARGH_1HD]);
    set_bond(resPt->schAtomList[psf_ARGH_CD],resPt->schAtomList[psf_ARGH_2HD]);
    // H' atoms
    if (resPt->schAtomList[psf_ARGH_1HH1]!=NULL)
      set_bond(resPt->schAtomList[psf_ARGH_NH1],resPt->schAtomList[psf_ARGH_1HH1]);
    if (resPt->schAtomList[psf_ARGH_2HH1]!=NULL)
      set_bond(resPt->schAtomList[psf_ARGH_NH1],resPt->schAtomList[psf_ARGH_2HH1]);
    if (resPt->schAtomList[psf_ARGH_1HH2]!=NULL)
      set_bond(resPt->schAtomList[psf_ARGH_NH2],resPt->schAtomList[psf_ARGH_1HH2]);
    if (resPt->schAtomList[psf_ARGH_2HH2]!=NULL)
      set_bond(resPt->schAtomList[psf_ARGH_NH2],resPt->schAtomList[psf_ARGH_2HH2]);
    if (resPt->schAtomList[psf_ARGH_HE]!=NULL)
      set_bond(resPt->schAtomList[psf_ARGH_NE],resPt->schAtomList[psf_ARGH_HE]);
    break;
  case psf_ASNH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ASNH_CG]);
    set_bond(resPt->schAtomList[psf_ASNH_CG],resPt->schAtomList[psf_ASNH_OD1]);
    set_bond(resPt->schAtomList[psf_ASNH_CG],resPt->schAtomList[psf_ASNH_ND2]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ASNH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ASNH_2HB]);
    // H' atoms
    if (resPt->schAtomList[psf_ASNH_1HD2]!=NULL)
      set_bond(resPt->bkbAtomList[psf_ASNH_ND2],resPt->schAtomList[psf_ASNH_1HD2]);
    if (resPt->schAtomList[psf_ASNH_2HD2]!=NULL)
      set_bond(resPt->bkbAtomList[psf_ASNH_ND2],resPt->schAtomList[psf_ASNH_2HD2]);
    break;
  case psf_ASPH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ASPH_CG]);
    set_bond(resPt->schAtomList[psf_ASPH_CG],resPt->schAtomList[psf_ASPH_OD1]);
    set_bond(resPt->schAtomList[psf_ASPH_CG],resPt->schAtomList[psf_ASPH_OD2]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ASPH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ASPH_2HB]);
    break;
  case psf_CYSH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_CYSH_SG]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_CYSH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_CYSH_2HB]);
    if(resPt->schAtomList[psf_CYSH_HG]!=NULL)
      set_bond(resPt->bkbAtomList[psf_CYSH_SG],resPt->schAtomList[psf_CYSH_HG]);
    break;
  case psf_GLNH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_GLNH_CG]);
    set_bond(resPt->schAtomList[psf_GLNH_CG],resPt->schAtomList[psf_GLNH_CD]);
    set_bond(resPt->schAtomList[psf_GLNH_CD],resPt->schAtomList[psf_GLNH_OE1]);
    set_bond(resPt->schAtomList[psf_GLNH_CD],resPt->schAtomList[psf_GLNH_NE2]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_GLNH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_GLNH_2HB]);
    set_bond(resPt->schAtomList[psf_GLNH_CG],resPt->schAtomList[psf_GLNH_1HG]);
    set_bond(resPt->schAtomList[psf_GLNH_CG],resPt->schAtomList[psf_GLNH_2HG]);
    // H' atoms
    if (resPt->schAtomList[psf_GLNH_1HE2]!=NULL)
      set_bond(resPt->schAtomList[psf_GLNH_NE2],resPt->schAtomList[psf_GLNH_1HE2]);
    if (resPt->schAtomList[psf_GLNH_2HE2]!=NULL)
      set_bond(resPt->schAtomList[psf_GLNH_NE2],resPt->schAtomList[psf_GLNH_2HE2]);
    break;
  case psf_GLUH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_GLUH_CG]);
    set_bond(resPt->schAtomList[psf_GLUH_CG],resPt->schAtomList[psf_GLUH_CD]);
    set_bond(resPt->schAtomList[psf_GLUH_CD],resPt->schAtomList[psf_GLUH_OE1]);
    set_bond(resPt->schAtomList[psf_GLUH_CD],resPt->schAtomList[psf_GLUH_OE2]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_GLUH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_GLUH_2HB]);
    set_bond(resPt->schAtomList[psf_GLUH_CG],resPt->schAtomList[psf_GLUH_1HG]);
    set_bond(resPt->schAtomList[psf_GLUH_CG],resPt->schAtomList[psf_GLUH_2HG]);
    break;
  case psf_GLYH:
    break;
  case psf_HISH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_HISH_CG]);
    set_bond(resPt->schAtomList[psf_HISH_CG],resPt->schAtomList[psf_HISH_ND1]);
    set_bond(resPt->schAtomList[psf_HISH_CG],resPt->schAtomList[psf_HISH_CD2]);
    set_bond(resPt->schAtomList[psf_HISH_ND1],resPt->schAtomList[psf_HISH_CE1]);
    set_bond(resPt->schAtomList[psf_HISH_CD2],resPt->schAtomList[psf_HISH_NE2]);
    set_bond(resPt->schAtomList[psf_HISH_CE1],resPt->schAtomList[psf_HISH_NE2]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_HISH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_HISH_2HB]);
    set_bond(resPt->schAtomList[psf_HISH_CD2],resPt->schAtomList[psf_HISH_HD2]);
    set_bond(resPt->schAtomList[psf_HISH_CE1],resPt->schAtomList[psf_HISH_HE1]);
    // H' atoms
    if(resPt->schAtomList[psf_HISH_HD1]!=NULL)
      set_bond(resPt->schAtomList[psf_HISH_ND1],resPt->schAtomList[psf_HISH_HD1]);
    if(resPt->schAtomList[psf_HISH_HE2]!=NULL)
      set_bond(resPt->schAtomList[psf_HISH_NE2],resPt->schAtomList[psf_HISH_HE2]);
    break;
  case psf_ILEH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ILEH_CG1]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ILEH_CG2]);
    set_bond(resPt->schAtomList[psf_ILEH_CG1],resPt->schAtomList[psf_ILEH_CD1]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_ILEH_HB]);
    set_bond(resPt->schAtomList[psf_ILEH_CG1],resPt->schAtomList[psf_ILEH_1HG1]);
    set_bond(resPt->schAtomList[psf_ILEH_CG1],resPt->schAtomList[psf_ILEH_2HG1]);
    set_bond(resPt->schAtomList[psf_ILEH_CG2],resPt->schAtomList[psf_ILEH_1HG2]);
    set_bond(resPt->schAtomList[psf_ILEH_CG2],resPt->schAtomList[psf_ILEH_2HG2]);
    set_bond(resPt->schAtomList[psf_ILEH_CG2],resPt->schAtomList[psf_ILEH_3HG2]);
    set_bond(resPt->schAtomList[psf_ILEH_CD1],resPt->schAtomList[psf_ILEH_1HD1]);
    set_bond(resPt->schAtomList[psf_ILEH_CD1],resPt->schAtomList[psf_ILEH_2HD1]);
    set_bond(resPt->schAtomList[psf_ILEH_CD1],resPt->schAtomList[psf_ILEH_3HD1]);
    break;
  case psf_LEUH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_LEUH_CG]);
    set_bond(resPt->schAtomList[psf_LEUH_CG],resPt->schAtomList[psf_LEUH_CD1]);
    set_bond(resPt->schAtomList[psf_LEUH_CG],resPt->schAtomList[psf_LEUH_CD2]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_LEUH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_LEUH_2HB]);
    set_bond(resPt->schAtomList[psf_LEUH_CG],resPt->schAtomList[psf_LEUH_HG]);
    set_bond(resPt->schAtomList[psf_LEUH_CD1],resPt->schAtomList[psf_LEUH_1HD1]);
    set_bond(resPt->schAtomList[psf_LEUH_CD1],resPt->schAtomList[psf_LEUH_2HD1]);
    set_bond(resPt->schAtomList[psf_LEUH_CD1],resPt->schAtomList[psf_LEUH_3HD1]);
    set_bond(resPt->schAtomList[psf_LEUH_CD2],resPt->schAtomList[psf_LEUH_1HD2]);
    set_bond(resPt->schAtomList[psf_LEUH_CD2],resPt->schAtomList[psf_LEUH_2HD2]);
    set_bond(resPt->schAtomList[psf_LEUH_CD2],resPt->schAtomList[psf_LEUH_3HD2]);
    break;
  case psf_LYSH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_LYSH_CG]);
    set_bond(resPt->schAtomList[psf_LYSH_CG],resPt->schAtomList[psf_LYSH_CD]);
    set_bond(resPt->schAtomList[psf_LYSH_CD],resPt->schAtomList[psf_LYSH_CE]);
    set_bond(resPt->schAtomList[psf_LYSH_CE],resPt->schAtomList[psf_LYSH_NZ]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_LYSH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_LYSH_2HB]);
    set_bond(resPt->schAtomList[psf_LYSH_CG],resPt->schAtomList[psf_LYSH_1HG]);
    set_bond(resPt->schAtomList[psf_LYSH_CG],resPt->schAtomList[psf_LYSH_2HG]);
    set_bond(resPt->schAtomList[psf_LYSH_CD],resPt->schAtomList[psf_LYSH_1HD]);
    set_bond(resPt->schAtomList[psf_LYSH_CD],resPt->schAtomList[psf_LYSH_2HD]);
    set_bond(resPt->schAtomList[psf_LYSH_CE],resPt->schAtomList[psf_LYSH_1HE]);
    set_bond(resPt->schAtomList[psf_LYSH_CE],resPt->schAtomList[psf_LYSH_2HE]);
    // H' atoms
    if(resPt->schAtomList[psf_LYSH_1HZ]!=NULL)
      set_bond(resPt->schAtomList[psf_LYSH_NZ],resPt->schAtomList[psf_LYSH_1HZ]);   
    if(resPt->schAtomList[psf_LYSH_2HZ]!=NULL)
      set_bond(resPt->schAtomList[psf_LYSH_NZ],resPt->schAtomList[psf_LYSH_2HZ]);   
    break;
   
  case psf_METH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_METH_CG]);
    set_bond(resPt->schAtomList[psf_METH_CG],resPt->schAtomList[psf_METH_SD]);
    set_bond(resPt->schAtomList[psf_METH_SD],resPt->schAtomList[psf_METH_CE]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_METH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_METH_2HB]);
    set_bond(resPt->schAtomList[psf_METH_CG],resPt->schAtomList[psf_METH_1HG]);
    set_bond(resPt->schAtomList[psf_METH_CG],resPt->schAtomList[psf_METH_2HG]);
    set_bond(resPt->schAtomList[psf_METH_CE],resPt->schAtomList[psf_METH_1HE]);
    set_bond(resPt->schAtomList[psf_METH_CE],resPt->schAtomList[psf_METH_2HE]);
    set_bond(resPt->schAtomList[psf_METH_CE],resPt->schAtomList[psf_METH_3HE]);
    break;
  case psf_PHEH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_PHEH_CG]);
    set_bond(resPt->schAtomList[psf_PHEH_CG],resPt->schAtomList[psf_PHEH_CD1]);
    set_bond(resPt->schAtomList[psf_PHEH_CG],resPt->schAtomList[psf_PHEH_CD2]);
    set_bond(resPt->schAtomList[psf_PHEH_CD1],resPt->schAtomList[psf_PHEH_CE1]);
    set_bond(resPt->schAtomList[psf_PHEH_CD2],resPt->schAtomList[psf_PHEH_CE2]);
    set_bond(resPt->schAtomList[psf_PHEH_CE1],resPt->schAtomList[psf_PHEH_CZ]);
    set_bond(resPt->schAtomList[psf_PHEH_CE2],resPt->schAtomList[psf_PHEH_CZ]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_PHEH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_PHEH_2HB]);
    set_bond(resPt->schAtomList[psf_PHEH_CD1],resPt->schAtomList[psf_PHEH_HD1]);
    set_bond(resPt->schAtomList[psf_PHEH_CD2],resPt->schAtomList[psf_PHEH_HD2]);
    set_bond(resPt->schAtomList[psf_PHEH_CE1],resPt->schAtomList[psf_PHEH_HE1]);
    set_bond(resPt->schAtomList[psf_PHEH_CE2],resPt->schAtomList[psf_PHEH_HE2]);
    set_bond(resPt->schAtomList[psf_PHEH_CZ],resPt->schAtomList[psf_PHEH_HZ]);
    break;
  case psf_PROH:
    break;
  case psf_SERH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_SERH_OG]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_SERH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_SERH_2HB]);
    // H' atoms
    if (resPt->schAtomList[psf_SERH_HG]!=NULL)
      set_bond(resPt->bkbAtomList[psf_SER_OG],resPt->schAtomList[psf_SERH_HG]);    
    break;
  case psf_THRH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_THRH_OG1]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_THRH_CG2]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_THRH_HB]);
    set_bond(resPt->schAtomList[psf_THRH_CG2],resPt->schAtomList[psf_THRH_1HG2]);
    set_bond(resPt->schAtomList[psf_THRH_CG2],resPt->schAtomList[psf_THRH_2HG2]);
    set_bond(resPt->schAtomList[psf_THRH_CG2],resPt->schAtomList[psf_THRH_3HG2]);
    // H' atoms
    if (resPt->schAtomList[psf_THRH_HG1]!=NULL)
      set_bond(resPt->bkbAtomList[psf_THRH_OG1],resPt->schAtomList[psf_THRH_HG1]);    
    break;
  case psf_TRPH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_TRPH_CG]);
    set_bond(resPt->schAtomList[psf_TRPH_CG],resPt->schAtomList[psf_TRPH_CD1]);
    set_bond(resPt->schAtomList[psf_TRPH_CG],resPt->schAtomList[psf_TRPH_CD2]);
    set_bond(resPt->schAtomList[psf_TRPH_CD1],resPt->schAtomList[psf_TRPH_NE1]);
    set_bond(resPt->schAtomList[psf_TRPH_NE1],resPt->schAtomList[psf_TRPH_CE2]);
    set_bond(resPt->schAtomList[psf_TRPH_CD2],resPt->schAtomList[psf_TRPH_CE2]);
    set_bond(resPt->schAtomList[psf_TRPH_CD2],resPt->schAtomList[psf_TRPH_CE3]);
    set_bond(resPt->schAtomList[psf_TRPH_CE2],resPt->schAtomList[psf_TRPH_CZ2]);
    set_bond(resPt->schAtomList[psf_TRPH_CE3],resPt->schAtomList[psf_TRPH_CZ3]);
    set_bond(resPt->schAtomList[psf_TRPH_CZ2],resPt->schAtomList[psf_TRPH_CH2]);
    set_bond(resPt->schAtomList[psf_TRPH_CZ3],resPt->schAtomList[psf_TRPH_CH2]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_TRPH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_TRPH_2HB]);
    set_bond(resPt->schAtomList[psf_TRPH_CD1],resPt->schAtomList[psf_TRPH_HD1]);
    set_bond(resPt->schAtomList[psf_TRPH_CE3],resPt->schAtomList[psf_TRPH_HE3]);
    set_bond(resPt->schAtomList[psf_TRPH_CZ2],resPt->schAtomList[psf_TRPH_HZ2]);
    set_bond(resPt->schAtomList[psf_TRPH_CZ3],resPt->schAtomList[psf_TRPH_HZ3]);
    set_bond(resPt->schAtomList[psf_TRPH_CH2],resPt->schAtomList[psf_TRPH_HH2]);
    // H' atoms
    if(resPt->schAtomList[psf_TRPH_HE1]!=NULL)
      set_bond(resPt->schAtomList[psf_TRPH_NE1],resPt->schAtomList[psf_TRPH_HE1]);
    break;
  case psf_TYRH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_TYRH_CG]);
    set_bond(resPt->schAtomList[psf_TYRH_CG],resPt->schAtomList[psf_TYRH_CD1]);
    set_bond(resPt->schAtomList[psf_TYRH_CG],resPt->schAtomList[psf_TYRH_CD2]);
    set_bond(resPt->schAtomList[psf_TYRH_CD1],resPt->schAtomList[psf_TYRH_CE1]);
    set_bond(resPt->schAtomList[psf_TYRH_CD2],resPt->schAtomList[psf_TYRH_CE2]);
    set_bond(resPt->schAtomList[psf_TYRH_CE1],resPt->schAtomList[psf_TYRH_CZ]);
    set_bond(resPt->schAtomList[psf_TYRH_CE2],resPt->schAtomList[psf_TYRH_CZ]);
    set_bond(resPt->schAtomList[psf_TYRH_CZ],resPt->schAtomList[psf_TYRH_OH]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_TYRH_1HB]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_TYRH_2HB]);
    set_bond(resPt->schAtomList[psf_TYRH_CD1],resPt->schAtomList[psf_TYRH_HD1]);
    set_bond(resPt->schAtomList[psf_TYRH_CD2],resPt->schAtomList[psf_TYRH_HD2]);
    set_bond(resPt->schAtomList[psf_TYRH_CE1],resPt->schAtomList[psf_TYRH_HE1]);
    set_bond(resPt->schAtomList[psf_TYRH_CE2],resPt->schAtomList[psf_TYRH_HE2]);
    // H' atoms
    if(resPt->schAtomList[psf_TYRH_HH]!=NULL)
      set_bond(resPt->schAtomList[psf_TYRH_OH],resPt->schAtomList[psf_TYRH_HH]);    
    break;
  case psf_VALH:
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_VALH_CG1]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_VALH_CG2]);
    set_bond(resPt->bkbAtomList[psf_genH_CB],resPt->schAtomList[psf_VALH_HB]);
    set_bond(resPt->schAtomList[psf_VALH_CG1],resPt->schAtomList[psf_VALH_1HG1]);
    set_bond(resPt->schAtomList[psf_VALH_CG1],resPt->schAtomList[psf_VALH_2HG1]);
    set_bond(resPt->schAtomList[psf_VALH_CG1],resPt->schAtomList[psf_VALH_3HG1]);
    set_bond(resPt->schAtomList[psf_VALH_CG2],resPt->schAtomList[psf_VALH_1HG2]);
    set_bond(resPt->schAtomList[psf_VALH_CG2],resPt->schAtomList[psf_VALH_2HG2]);
    set_bond(resPt->schAtomList[psf_VALH_CG2],resPt->schAtomList[psf_VALH_3HG2]);
    break;
  }
}

/**********************************************************************/

// counts the atoms which are not H' in the backbone of a residue

static int nb_No_HPrime_bkb_atoms(psf_residue *resPt)
{
  int nb_No_HPrime_atoms = 0;
  int i;

  for (i=0; i<resPt->nbkbatoms; i++) {
    if(resPt->bkbAtomList[i]->atomType != psf_HYDROGEN_P) {
      nb_No_HPrime_atoms++;
    }
  }
  return nb_No_HPrime_atoms;
}

/**********************************************************************/

// counts the atoms which are not H' in the side chain of a residue

static int nb_No_HPrime_sch_atoms(psf_residue *resPt)
{
  int nb_No_HPrime_atoms = 0;
  int i;
  for (i=0; i<resPt->nschatoms; i++) {
    if(resPt->schAtomList[i]->atomType != psf_HYDROGEN_P) {
      nb_No_HPrime_atoms++;
    }
  }
  return nb_No_HPrime_atoms;
}

/**********************************************************************/

static int consecutive_residues(psf_residue *res1Pt, psf_residue *res2Pt)
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

static void set_ligand_bonds (psf_ligand* ligPt) {

  psf_atom *atomPt_i, *atomPt_j;
  double dist;
  int i,j;
  
  for(i=0; i<(ligPt->natoms - 1); i++) {
    atomPt_i = ligPt->atomList[i];
    for(j=i+1; j<ligPt->natoms; j++) {
      atomPt_j = ligPt->atomList[j];
      compute_distance(atomPt_i->pos,atomPt_j->pos,&dist);
      if(dist < (atomPt_i->vdwR + atomPt_j->vdwR) * COVBOND_FACTOR) {
	set_bond(atomPt_i,atomPt_j);
      }
    }
  }
  
}

/**********************************************************************/

psf_atom *get_closest_atom_to_point(psf_atom **atomList, double natoms, double *point)
{
  psf_atom *bestatomPt;
  double dist, bestdist;
  int i;

  bestdist = HUGE_VAL;
  bestatomPt = NULL;
  for(i=0; i<natoms; i++) {
    compute_distance(atomList[i]->pos,point,&dist);
    if(dist < bestdist) {
      bestdist = dist;
      bestatomPt = atomList[i];
    }
  }
  return bestatomPt;
}

///////// INTERMEDIATE FUNCTIONS //////////////////////////////////////////////

// Protein

static void init_res_atoms(res_atoms *resatomsPt)
{
  int i;
  
  for(i=0; i<MAX_BKB_ATOMS; i++) 
    resatomsPt->bkbatoms[i] = NULL;
  for(i=0; i<MAX_SCH_ATOMS; i++) 
    resatomsPt->schatoms[i] = NULL;
}



/**********************************************************************/

static psf_atom *copy_atom_data(pdb_atom_data *adataPt) 
{
  double vdwR;
  int vdwR_OK = FALSE;
  psf_atom *aPt;
  int i;

  aPt = (psf_atom *) malloc(sizeof(psf_atom));
  
  vdwR_OK = get_vdwR_by_type(adataPt->type, &vdwR);
  if(!vdwR_OK)
    return NULL;
  aPt->vdwR = vdwR;
  aPt->serial = adataPt->serial;
  strcpy(aPt->name, adataPt->name);
  
  aPt->atomType = adataPt->type;

  for(i=0; i<3; i++) {
    aPt->pos[i] = adataPt->pos[i];
  }

  aPt->rigidPt = NULL;

  aPt->nbondedA = 0;
  aPt->bondedAtomList = NULL;

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

/**********************************************************************/

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

/**********************************************************************/

static int insert_adata_in_res_atoms(pdb_atom_data *adataPt, res_atoms *resatomsPt, residueTypes resType) {
  int state;
  
  state = insert_bkb_adata_in_res_atoms(adataPt,resatomsPt,resType);

  if(state < 0) {
    return -1;
  }
  else if(state == 1) {
    return 1;
  }

  state = insert_sch_adata_in_res_atoms(adataPt,resatomsPt,resType);
 
  if(state == 0) {
    printf("WARNING : no translated atom : %s - %d\n",adataPt->name,adataPt->serial);
  } 

  return 1;
}

/**********************************************************************/

// ONLY PARTICULAR CASE !!!
static int extract_PRO_atoms(psf_residue *resPt, res_atoms *resatomsPt)
{
  if(resatomsPt->bkbatoms[(int) psf_PROH_HA] == NULL) {
    // no-H version
    if(resatomsPt->bkbatoms[(int) psf_PROH_C] != NULL)
      resPt->bkbAtomList[(int) psf_PRO_C] = resatomsPt->bkbatoms[(int) psf_PROH_C];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_O] != NULL)
      resPt->bkbAtomList[(int) psf_PRO_O] = resatomsPt->bkbatoms[(int) psf_PROH_O];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_CA] != NULL)
      resPt->bkbAtomList[(int) psf_PRO_CA] = resatomsPt->bkbatoms[(int) psf_PROH_CA];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_CB] != NULL)
      resPt->bkbAtomList[(int) psf_PRO_CB] = resatomsPt->bkbatoms[(int) psf_PROH_CB];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_CG] != NULL)
      resPt->bkbAtomList[(int) psf_PRO_CG] = resatomsPt->bkbatoms[(int) psf_PROH_CG];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_CD] != NULL)
      resPt->bkbAtomList[(int) psf_PRO_CD] = resatomsPt->bkbatoms[(int) psf_PROH_CD];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_N] != NULL)
      resPt->bkbAtomList[(int) psf_PRO_N] = resatomsPt->bkbatoms[(int) psf_PROH_N];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_OXT] != NULL) {
      resPt->bkbAtomList[(int) psf_PRO_OXT] = resatomsPt->bkbatoms[(int) psf_PROH_OXT];
      resPt->flagCterm = 1;
      resPt->nbkbatoms = N_PRO_BKB_ATOMS;
    }
    else {
      resPt->flagCterm = 0;
      resPt->nbkbatoms = N_PRO_BKB_ATOMS - 1;
    }
    noHresidueType(resPt);
    resPt->flagH = 0;	  
  }
  else {
    // H version
    if(resatomsPt->bkbatoms[(int) psf_PROH_C] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_C] = resatomsPt->bkbatoms[(int) psf_PROH_C];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_O] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_O] = resatomsPt->bkbatoms[(int) psf_PROH_O];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_CA] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_CA] = resatomsPt->bkbatoms[(int) psf_PROH_CA];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_HA] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_HA] = resatomsPt->bkbatoms[(int) psf_PROH_HA];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_CB] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_CB] = resatomsPt->bkbatoms[(int) psf_PROH_CB];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_N] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_N] = resatomsPt->bkbatoms[(int) psf_PROH_N];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_1HB] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_1HB] = resatomsPt->bkbatoms[(int) psf_PROH_1HB];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_2HB] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_2HB] = resatomsPt->bkbatoms[(int) psf_PROH_2HB];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_CG] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_CG] = resatomsPt->bkbatoms[(int) psf_PROH_CG];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_1HG] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_1HG] = resatomsPt->bkbatoms[(int) psf_PROH_1HG];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_2HG] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_2HG] = resatomsPt->bkbatoms[(int) psf_PROH_2HG];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_CD] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_CD] = resatomsPt->bkbatoms[(int) psf_PROH_CD];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_1HD] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_1HD] = resatomsPt->bkbatoms[(int) psf_PROH_1HD];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_2HD] != NULL)
      resPt->bkbAtomList[(int) psf_PROH_2HD] = resatomsPt->bkbatoms[(int) psf_PROH_2HD];
    else
      return -1;
    if(resatomsPt->bkbatoms[(int) psf_PROH_OXT] != NULL) {
      resPt->bkbAtomList[(int) psf_PROH_OXT] = resatomsPt->bkbatoms[(int) psf_PROH_OXT];
      resPt->flagCterm = 1;
      resPt->nbkbatoms = N_PROH_BKB_ATOMS;
    }
    else {
      resPt->flagCterm = 0;
      resPt->nbkbatoms = N_PROH_BKB_ATOMS - 1;
    }    
    resPt->flagH = 1;
  }

  //return (!(resPt->flagCterm));  
  return 1;
}

/**********************************************************************/
// Note : to test if a residue is not incomplete, we consider the number
// of atoms which are not H'. this number in the current residue is
// compared with the constant number indicated in atoms.h 

static int extract_bkb_atoms(psf_residue *resPt, res_atoms *resatomsPt) {
  int i;
  int state;
  int n;
  
  if(resatomsPt->resType == psf_PROH) {
    // particular case : different relative order of heavy atoms between H and no-H versions    
    state = extract_PRO_atoms(resPt,resatomsPt);
    if(state < 0) {
      return FALSE;
    }
    return TRUE;
  }

  else {

    for(i=0; i<MAX_BKB_ATOMS; i++) {
      if(resatomsPt->bkbatoms[i] != NULL) {
	resPt->bkbAtomList[resPt->nbkbatoms] = resatomsPt->bkbatoms[i];
	if(strcmp(resatomsPt->bkbatoms[i]->name,"OXT") == 0)
	  resPt->flagCterm = 1;
	else
	  resPt->flagCterm = 0;
	resPt->nbkbatoms ++;
      }
    }

    n = number_bkb_atoms(resPt->resType);
    
    if(resPt->flagCterm) {
      // a OXT group exist, the number of no H' atoms is
      // the one indicated in atoms.h

      if(nb_No_HPrime_bkb_atoms(resPt) == n) {
	resPt->flagH = 1;
	return TRUE;
      }

      // The numbers are not coherent : the model may not
      // include H atoms      
      noHresidueType(resPt);
      resPt->flagH = 0;
      
      n = number_bkb_atoms(resPt->resType);
      if(nb_No_HPrime_bkb_atoms(resPt) == n) {     
	return TRUE;
      }    
    }
    else {
      // there is no OXT group, the number of no H' atoms is
      // the one indicated in atoms.h - 1

      if(nb_No_HPrime_bkb_atoms(resPt) == (n - 1)) {
	resPt->flagH = 1;
	return TRUE;
      }  
 
      // The numbers are not coherent : the model may not
      // include H atoms
      noHresidueType(resPt);
      resPt->flagH = 0;
      
      n = number_bkb_atoms(resPt->resType);
      if(nb_No_HPrime_bkb_atoms(resPt) == (n - 1)) {     
	return TRUE;
      }    
    }
  }
  return FALSE;
}

/**********************************************************************/

static int extract_sch_atoms(psf_residue *resPt, res_atoms *resatomsPt) {

  int i;
  int n;

  for (i=0; i<MAX_SCH_ATOMS; i++) {
    
    if (resatomsPt->schatoms[i] != NULL) {
      resPt->schAtomList[resPt->nschatoms] = resatomsPt->schatoms[i];
      resPt->nschatoms++;
    }
  }

  n = number_sch_atoms(resPt->resType);
  if (nb_No_HPrime_sch_atoms(resPt) == n)
    return TRUE;

  return FALSE;
}

/**********************************************************************/

static int extract_atoms_to_res(psf_residue *resPt, res_atoms *resatomsPt)
{
  int extracted;

  extracted = extract_bkb_atoms(resPt,resatomsPt);
  if(extracted) {
    extracted = extract_sch_atoms(resPt,resatomsPt);
  }
  return extracted;
}

/**********************************************************************/

static int fill_residue_struct(psf_residue **resPtPt, res_atoms *resatomsPt)
{
  int i;
  int extracted;
  psf_residue *resPt;

  resPt = (psf_residue *) malloc(sizeof(psf_residue)); 
  *resPtPt = resPt;

  //strcpy(resPt->chainPt->chainID,resatomsPt->chainID);
  resPt->resSeq = resatomsPt->resSeq;
  strcpy(resPt->resName,resatomsPt->resName);
  resPt->resType = resatomsPt->resType;

  // lists with constant size (could be allocated dynamically)
  resPt->bkbAtomList = (psf_atom**) malloc(sizeof(psf_atom *) * MAX_BKB_ATOMS);
  resPt->schAtomList = (psf_atom**) malloc(sizeof(psf_atom *) * MAX_SCH_ATOMS);
  for(i=0; i<MAX_BKB_ATOMS; i++) 
    resPt->bkbAtomList[i] = NULL;
  for(i=0; i<MAX_SCH_ATOMS; i++) 
    resPt->schAtomList[i] = NULL;  

  resPt->nbkbatoms = 0;
  resPt->nschatoms = 0;

  // NOTE : residues must be complete (i.e. no missing atoms) 
  //        in both H and no-H versions 
  //      * required by BCD and protein-to-p3d translator
  extracted = extract_atoms_to_res(resPt,resatomsPt);

  if(!extracted) {
    printf("ERROR : incomplete residue %d\n", resPt->resSeq);
    return FALSE;
  }

  // put in every atom a pointer to the residue
  for(i=0; i<resPt->nbkbatoms; i++) 
    resPt->bkbAtomList[i]->residuePt = resPt;
  for(i=0; i<resPt->nschatoms; i++) 
    resPt->schAtomList[i]->residuePt = resPt;

  resPt->nextResidue = NULL;

  return TRUE;
}

/**********************************************************************/

// Ligand

static void compute_ligand_centroid(psf_ligand *ligPt, double *point)
{
  double x,y,z,M;
  int i;
  psf_atom *atomPt;

  x = 0.0; y = 0.0; z = 0.0; M = 0.0;
  for(i=0; i<ligPt->natoms; i++) {
    atomPt = ligPt->atomList[i];
    x += (atomPt->pos[0]) * (atomPt->vdwR);
    y += (atomPt->pos[1]) * (atomPt->vdwR);
    z += (atomPt->pos[2]) * (atomPt->vdwR);
    M += (atomPt->vdwR);
  }
  point[0] = x/M;
  point[1] = y/M;
  point[2] = z/M;
}

/**********************************************************************/

static int get_ligand_atom_index(psf_ligand* ligPt, psf_atom* theAtom, int* index) {
  
  int found = FALSE;
  int i = 0;

  while (i<ligPt->natoms && !found) {
    if (ligPt->atomList[i] == theAtom) {
      *index = i;
      found = TRUE;
    }
    else
      i++;
  }
  
  return found;
}

/**********************************************************************/

static int set_as_explored(psf_ligand* ligPt, psf_atom* bondedAtomPt, check_explored_atoms* check) {
  
  int indBondedAtom;

  if (get_ligand_atom_index(ligPt, bondedAtomPt, &indBondedAtom))
    insert_integer_in_list(indBondedAtom, &(check->check_list), &(check->nbchecked));
  else
    return FALSE;

  return TRUE;
}


/**********************************************************************/

static int check_if_explored(psf_ligand* ligPt, psf_atom* bondedAtomPt, check_explored_atoms* check) {

  int indBondedAtom;
  int explored = FALSE;
  int i = 0;

  if (get_ligand_atom_index(ligPt, bondedAtomPt, &indBondedAtom)) {

    while (i<check->nbchecked && !explored) {
      if (check->check_list[i] == indBondedAtom) {
	explored = TRUE;
      }
      else
	i++;
    }

  }

  return explored;
}

/**********************************************************************/

static int in_joint_axis(psf_ligand* ligPt, psf_atom *atomPt_i, psf_atom *atomPt_j, psf_joint **bondjntPtPt) {
  
  int i;
  int notFound = TRUE;

  *bondjntPtPt = NULL;
  i = 0;

  while ((i<ligPt->njoints) && notFound) {
   
    if ((ligPt->jointList[i]->dihedralAtoms[1] == atomPt_i)
	&& (ligPt->jointList[i]->dihedralAtoms[2] == atomPt_j)) {
      notFound = FALSE;
      *bondjntPtPt = ligPt->jointList[i];
    }
    else if ((ligPt->jointList[i]->dihedralAtoms[1] == atomPt_j)
	     && (ligPt->jointList[i]->dihedralAtoms[2] == atomPt_i)) {
      notFound = FALSE;
      *bondjntPtPt = ligPt->jointList[i];
    }
    else {
      i++;
    }
  }
  //if (!notFound) printf("%d et %d\n", atomPt_i->serial, atomPt_j->serial);
  return !notFound;
}

/**********************************************************************/

// RIGID CONSTRUCTION

static int construct_rigid_from_root_atom(psf_ligand *ligPt, psf_atom *rootaPt, 
					  psf_atom ***nextrootalist, int *nnextroota,
					  check_explored_atoms* check) {
  psf_rigid *rigidPt;
  psf_joint *bondjntPt;
  psf_atom **curatomlist;
  psf_atom **nextatomlist, **auxlist;
  psf_atom *bondedaPt;
  int natoms;
  int nnextatoms;
  int i,j;
  int indRefAtom, indBondedAtom;
  
  rigidPt = init_rigid_struct();

  // insert root atom in rigid
  insert_pointer_in_list(rootaPt,(void***)&(rigidPt->atomList),&(rigidPt->natoms));
  rootaPt->rigidPt = rigidPt;

  natoms = 1;
  curatomlist = (psf_atom**) malloc(sizeof(psf_atom *));
  curatomlist[0] = rootaPt;

  while(natoms > 0) {
    nnextatoms = 0;
    nextatomlist = NULL;
    for(i=0; i<natoms; i++) {  
      
      if (!get_ligand_atom_index(ligPt, curatomlist[i], &indRefAtom))
	return FALSE;

      for(j=0; j<curatomlist[i]->nbondedA; j++) {  
	
	bondedaPt = curatomlist[i]->bondedAtomList[j];
	if (!get_ligand_atom_index(ligPt, bondedaPt, &indBondedAtom))
	  return FALSE;
	
	if(bondedaPt->rigidPt == NULL) {
	  // atoms defining the connection between to rigids are those 
	  // in dihedral angle axis (i.e. 2nd and 3rd in the list)
	  if(in_joint_axis(ligPt, bondedaPt,curatomlist[i],&bondjntPt)) {
	    insert_pointer_in_list(bondedaPt,(void***)nextrootalist,nnextroota);
	    // set "out-joint" pointer
	    bondjntPt->rigidOut = rigidPt;
	    insert_pointer_in_list(bondjntPt,(void***)&(rigidPt->outJntsList),&(rigidPt->noutJnts));
	  }
	  else {
	    insert_pointer_in_list(bondedaPt,(void***)&(nextatomlist),&nnextatoms);
	    insert_pointer_in_list(bondedaPt,(void***)&(rigidPt->atomList),&(rigidPt->natoms));
	    bondedaPt->rigidPt = rigidPt;
	  }
	}
	else {
	  // if bonded atom is alredy in current rigid -> do nothing 
	  // otherwise :
	  if(bondedaPt->rigidPt != rigidPt) {
	    // if the bond has been traveled yet -> set "in-joint" pointer (bodies linked in open chain)
	    if(!check_if_explored(ligPt, curatomlist[i], &(check[indBondedAtom]))) {
	      if(in_joint_axis(ligPt, bondedaPt,curatomlist[i],&bondjntPt)) {
		bondjntPt->rigidIn = rigidPt;
		insert_pointer_in_list(bondjntPt,(void***)&(rigidPt->inJntsList),&(rigidPt->ninJnts));
	      }
	      else {
		// PUEDE SUCEDER !!!???
		printf("Ooo oooooo\n");
	      }
	    }
	    // otherwise :
	    else {
	      printf("WARNING : closed kinematic chain : this case is not automatically treated\n");
	      printf("          A closure constraint must be considerer between atoms %d and %d\n",
		     curatomlist[i]->serial,bondedaPt->serial);
	    }
	  }
	  
	  if (!set_as_explored(ligPt, bondedaPt, &(check[indRefAtom])))
	    return FALSE;
	}
      }
    }
    natoms = nnextatoms;
    auxlist = curatomlist;
    curatomlist = nextatomlist;
    nextatomlist = auxlist;
    free(nextatomlist);
    nextatomlist = NULL;
  }
  free(curatomlist);
  curatomlist = NULL;

  insert_pointer_in_list(rigidPt,(void***)&(ligPt->rigidList),&(ligPt->nrigids));
  return TRUE;
}

/**********************************************************************/

static int generate_rigids_in_ligand(psf_ligand *ligPt)
{
  psf_atom **currootalist;
  psf_atom **nextrootalist, **auxlist;
  int nroota;
  int nnextroota;
  int state;
  int i;
  check_explored_atoms check[ligPt->natoms];

  if(ligPt->rigidList != NULL) {
    printf("ERROR while generating rigids : rigids already exist for that ligand!\n");
    return FALSE;
  }
  else {
    // set bonds in ligand
    set_ligand_bonds(ligPt);
    
    // Init checking structures
    for (i=0; i<ligPt->natoms; i++) {
      check[i].nbchecked = 0;
      check[i].check_list = NULL;
    }

    // choose atom for first rigid
    nroota = 1;
    currootalist = (psf_atom**) malloc(sizeof(psf_atom *));
    currootalist[0] = get_closest_atom_to_point(ligPt->atomList, ligPt->natoms, ligPt->oref);
    if(currootalist[0] == NULL) {
      printf("ERROR : problem computing distances : unable to choose best atom\n");
      return FALSE;
    }
    // construct rigids iteratively 
    // NOTE : a "breadth-first search" is used to construct the ligand
    //        (i.e. rigids are identified "level by level")
    while(nroota > 0) {
      nnextroota = 0;
      nextrootalist = NULL;
      for(i=0; i<nroota; i++) {
	state = construct_rigid_from_root_atom(ligPt,currootalist[i],&nextrootalist,&nnextroota, check);
	if(state < 0) {
	  free(nextrootalist);
	  nextrootalist = NULL;
	  free(currootalist);
	  currootalist = NULL;
	  return FALSE;
	}
      }
      nroota = nnextroota;
      auxlist = currootalist;
      currootalist = nextrootalist;
      nextrootalist = auxlist;
      free(nextrootalist);
      nextrootalist = NULL;
    }
    free(currootalist);
    currootalist = NULL;

    // free checking structures 
    for (i=0; i<ligPt->natoms; i++) {
      if (check[i].check_list != NULL) {
	free(check[i].check_list);
	check[i].check_list = NULL;
      }
    }
  }

  return TRUE;
}

int generate_rigids_in_all_ligands(psf_ligand** ligand_list, int nb_ligands) {

  int i = 0;
  int no_error = TRUE;

  while (i<nb_ligands && no_error) {
    no_error = generate_rigids_in_ligand(ligand_list[i]);
    if (no_error)
      i++;
  }
  
  return no_error;
}

/**********************************************************************/



///////// PDB READING FUNCTIONS ////////////////////////////////////////////////

// General reading function

static int is_protein_desc_line(char* pdbLine) {
  
  char possibleName[10];
  char parsedName[3];
  residueTypes typeResidue;

  strcpy(possibleName,"         ");
  strncpy(possibleName,pdbLine+17,3);
  if(sscanf(possibleName,"%s",parsedName) < 0) {
    return FALSE;
  }
  if (!resName_to_resType((char*)parsedName, &typeResidue)) {
    return FALSE;
  }

  return TRUE;
    
}

/**********************************************************************/

static lineType read_atom(FILE *pdbfile, char* pdbLine, int length) {

  char piece[10];
  char rec[10];
  char returned_s[length];
  char* read_line;
  
  int other_line = FALSE;
  lineType type = WRONG_LINE;

  do {
    
    read_line = fgets(returned_s,length,pdbfile);
        
    if (read_line == NULL) {
      other_line = FALSE;
      type = PDB_END;
    }
    else {
      strncpy(pdbLine,read_line,length-1);
      strcpy(piece,"         ");
      strncpy(piece,pdbLine,6);

      if(sscanf(piece,"%s",rec) < 0)
	other_line = TRUE;
      
      else if(strcmp(rec,"ATOM") != 0)
	other_line = TRUE;
      
      else {

	other_line = FALSE;
	if (is_protein_desc_line(pdbLine))
	  type = RESIDU_ATOM_LINE;
	else
	  // default : atom in ligand
	  type = LIGAND_ATOM_LINE;
      }
    }

  } while (other_line);

  return type;
}

/**********************************************************************/

// protein pdb reading function 

static lineType read_protein_atom(char* pdbLine, pdb_atom_data *adataPt) {

  char piece[10];
  char  chain[2];

  // extract only used data
  strcpy(piece,"         ");
  strncpy(piece,pdbLine+6,5);
  if(sscanf(piece,"%d",&adataPt->serial) < 0) return WRONG_LINE;
  
  strcpy(piece,"         ");
  strncpy(piece,pdbLine+12,4);
  if(sscanf(piece,"%s",adataPt->name) < 0) return WRONG_LINE;
 
  strcpy(piece,"         ");
  strncpy(piece,pdbLine+21,1);
  if (sscanf(piece,"%s",chain) < 0)
    strcpy(adataPt->chainID,"0");
  else
    strcpy(adataPt->chainID,chain);

  strcpy(piece,"         ");
  strncpy(piece,pdbLine+17,3);
  if(sscanf(piece,"%s",adataPt->resName) < 0) return WRONG_LINE;
  
  strcpy(piece,"         ");
  strncpy(piece,pdbLine+22,4);
  if(sscanf(piece,"%d",&adataPt->resSeq) < 0) return WRONG_LINE;
  
  strcpy(piece,"         ");
  strncpy(piece,pdbLine+30,8);
  if(sscanf(piece,"%lf",&adataPt->pos[0]) < 0) return WRONG_LINE;
  
  strcpy(piece,"         ");
  strncpy(piece,pdbLine+38,8);
  if(sscanf(piece,"%lf",&adataPt->pos[1]) < 0) return WRONG_LINE;
  
  strcpy(piece,"         ");
  strncpy(piece,pdbLine+46,8);
  if(sscanf(piece,"%lf",&adataPt->pos[2]) < 0) return WRONG_LINE; 

  // identify residue type (index)
  if(!resName_to_resType(adataPt->resName, &adataPt->resType)) {
    return WRONG_LINE;
  }
  
  return RESIDU_ATOM_LINE;
}

/**********************************************************************/
    
static int read_residue(FILE *pdbfile, char* pdbLine,
			lineType* last_pdb_reading_state,
			psf_residue **resPtPt, char* chainID) {

  pdb_atom_data current_atom_data;
  int residue_struct_filled = FALSE;
  int inserted = FALSE;
  res_atoms newAtoms;

  if (*last_pdb_reading_state == RESIDU_ATOM_LINE) {
    
    // read and insert first atom data extracted from the last read line
    *last_pdb_reading_state = read_protein_atom(pdbLine, &current_atom_data);

    if (*last_pdb_reading_state == WRONG_LINE)
      return FALSE;
    
    init_res_atoms(&newAtoms);
    newAtoms.resType = current_atom_data.resType;
    newAtoms.resSeq = current_atom_data.resSeq;
    strcpy(newAtoms.resName,current_atom_data.resName);
    strncpy(chainID,current_atom_data.chainID,2);

    do{

      inserted = insert_adata_in_res_atoms(&current_atom_data, &newAtoms, newAtoms.resType);

      // read new line in pdb file
      *last_pdb_reading_state = read_atom(pdbfile, pdbLine, 100);

      if (*last_pdb_reading_state == RESIDU_ATOM_LINE)
	*last_pdb_reading_state = read_protein_atom(pdbLine, &current_atom_data);
      
    } while((*last_pdb_reading_state == RESIDU_ATOM_LINE)
	    && inserted
	    && (current_atom_data.resSeq == newAtoms.resSeq));
    

    if(*last_pdb_reading_state == WRONG_LINE) {
      printf("ERROR : wrong PDB ATOM line\n");
      return FALSE;
    }

    if (!inserted) {
      printf("ERROR : ATOM not inserted in residue\n");
      return FALSE;  
    }
    
    // put res_atoms in residue structure (protein.h)
    residue_struct_filled = fill_residue_struct(resPtPt, &newAtoms);

    if(residue_struct_filled
       && (*last_pdb_reading_state!=RESIDU_ATOM_LINE)
       && (!(*resPtPt)->flagCterm)) {
      printf("WARNING : no terminal Oxigen in chain %s\n",chainID);
    }
  
  } 
  
  return residue_struct_filled;
    
}

/**********************************************************************/

// Ligand reading functions

static char* get_ligand_name(char* pdbLine) {
  
  char piece[10];
  char* ligandName = (char*)malloc(5*sizeof(char));

  strcpy(piece,"         ");
  strcpy(piece,"    ");
  strncpy(piece,pdbLine+17,4);
  if(sscanf(piece,"%s",ligandName) < 0) {
    return NULL;
  }

  return ligandName;
 
}

/**********************************************************************/

static lineType read_ligand_atom(char* pdbLine, psf_atom *aPt) {

  char piece[10];
  char atomic_Element_Name[3];
  double vdwR;
  int vdwR_OK = FALSE;

  // extract only used data
  strcpy(piece,"         ");
  strncpy(piece,pdbLine+6,5);
  if(sscanf(piece,"%d",&aPt->serial) < 0) return WRONG_LINE;

  strcpy(piece,"         ");
  strncpy(piece,pdbLine+12,4);
  if(sscanf(piece,"%s",aPt->name) < 0) return WRONG_LINE;

  strcpy(piece,"         ");
  strncpy(piece,pdbLine+30,8);
  if(sscanf(piece,"%lf",&aPt->pos[0]) < 0) return WRONG_LINE;

  strcpy(piece,"         ");
  strncpy(piece,pdbLine+38,8);
  if(sscanf(piece,"%lf",&aPt->pos[1]) < 0) return WRONG_LINE;

  strcpy(piece,"         ");
  strncpy(piece,pdbLine+46,8);
  if(sscanf(piece,"%lf",&aPt->pos[2]) < 0) return WRONG_LINE; 

  strcpy(piece,"         ");
  strncpy(piece,pdbLine+76,2);
  if(sscanf(piece,"%s",atomic_Element_Name) < 0) return WRONG_LINE; 

  if(elementName_to_lig_AtomType(atomic_Element_Name, &aPt->atomType) < 0) {
    return WRONG_LINE;
  }

  vdwR_OK = get_vdwR_by_type(aPt->atomType, &vdwR);
  if(!vdwR_OK)
    return WRONG_LINE;

  aPt->vdwR = vdwR;
  aPt->residuePt = NULL;
  aPt->rigidPt = NULL;
  aPt->nbondedA = 0;
  aPt->bondedAtomList = NULL;
  
  return LIGAND_ATOM_LINE;
}


///////// PROTEIN STRUCTURE (PSF) WRITING FUNCTIONS ////////////////////////////

// General writing function

int extract_psf_struct (FILE *pdbfile, char *shortFileName, psf_molecule* molecule) {

  lineType last_pdb_reading_state = OTHER_LINE;
  int extracted = FALSE;  
  char* pdbLine = (char*)malloc(100*sizeof(char));
  psf_protein* protPt;
  psf_ligand* ligPt;

  last_pdb_reading_state = read_atom(pdbfile, pdbLine, 100);
  
  if (last_pdb_reading_state == PDB_END) {
    printf("empty PDB file !\n");
    return FALSE;
  }
    
  do {
    
    if (last_pdb_reading_state == RESIDU_ATOM_LINE) {
      protPt = init_protein_struct(shortFileName);
      extracted = extract_psf_protein_struct(pdbfile, pdbLine, &last_pdb_reading_state,
					     shortFileName, protPt);
      insert_pointer_in_list((void*)protPt, (void***)&(molecule->proteinList), &(molecule->nproteins));
    }

    if (last_pdb_reading_state == LIGAND_ATOM_LINE) {
      ligPt = init_ligand_struct(shortFileName);
      extracted = extract_psf_ligand_struct(pdbfile,  pdbLine, &last_pdb_reading_state,
					    shortFileName, ligPt);
      insert_pointer_in_list((void*)ligPt, (void***)&(molecule->ligandList), &(molecule->nligands));
    }
  
  } while (extracted 
	   && ((last_pdb_reading_state == RESIDU_ATOM_LINE)
	       || (last_pdb_reading_state == LIGAND_ATOM_LINE)));
  
  return extracted;
 
}

/**********************************************************************/

// Protein psf writing function
    
static int extract_psf_protein_struct(FILE *pdbfile, char* pdbLine, lineType* pdb_reading_state,
				      char *shortFileName, psf_protein *protPt) {

  lineType previous_pdb_reading_state;
  int correctly_read = FALSE;
 
  psf_residue *theResPt=NULL;
  psf_residue *prevResPt=NULL;
  psf_AAchain *curAAchainPt;
  char currentChainID[2];

  // alloc first chain
  protPt->nchains = 0;
  curAAchainPt = (psf_AAchain *) malloc(sizeof(psf_AAchain));
  curAAchainPt->nresidues = 0;
  curAAchainPt->subchainind = 0;
  curAAchainPt->proteinPt = protPt;
  insert_pointer_in_list(curAAchainPt,(void ***)&(protPt->chainList),&(protPt->nchains));
  
  // Read residues
  while((*pdb_reading_state) == RESIDU_ATOM_LINE){
    
    correctly_read = read_residue(pdbfile, pdbLine, pdb_reading_state, &theResPt, currentChainID);

    if(!correctly_read) {
      return FALSE;
    }

    // if new chain
    if(prevResPt != NULL) {
      if(strcmp(prevResPt->chainPt->chainID,currentChainID) != 0) {

	curAAchainPt = (psf_AAchain *) malloc(sizeof(psf_AAchain)); 
	curAAchainPt->subchainind = 0;
	curAAchainPt->nresidues = 0;
	curAAchainPt->proteinPt = protPt;
	insert_pointer_in_list(curAAchainPt,(void ***)&(protPt->chainList),&(protPt->nchains));
	
	// pointer to prev (if new chain) 
	theResPt->prevResidue = NULL;
	// pointer to next in prev
	prevResPt->nextResidue = NULL; // Faire une fonction init_new_chain ?
      }
      // residues must be consecutive in the same chain
      // WARNING : they can be consecutive must can have non-consecutive resSeq
      // otherwise a new chain is created
      // (other possibility is to return ERROR)
      else if(!consecutive_residues(prevResPt,theResPt)) {
	// printf("ERROR : no consecutive residues : %d - %d\n",
	//         prevResPt->resSeq,theResPt->resSeq);
	// return -1;      
	printf("WARNING : no consecutive residues : %d - %d\n",
	       prevResPt->resSeq,theResPt->resSeq);

	curAAchainPt = (psf_AAchain *) malloc(sizeof(psf_AAchain)); 
	curAAchainPt->subchainind = prevResPt->chainPt->subchainind + 1;
	curAAchainPt->nresidues = 0;
	curAAchainPt->proteinPt = protPt;
	insert_pointer_in_list(curAAchainPt,(void ***)&(protPt->chainList),&(protPt->nchains));

	// pointer to prev (if new chain)
	theResPt->prevResidue = NULL;
	// pointer to next in prev
	prevResPt->nextResidue = theResPt;
      }
      else {    
	// pointer to next in prev
	prevResPt->nextResidue = theResPt;
      }
    }

    // pointer to prev
    theResPt->prevResidue = prevResPt;

    strncpy(curAAchainPt->chainID, currentChainID, 2);
    theResPt->chainPt = curAAchainPt;
    insert_pointer_in_list(theResPt,(void ***)&(curAAchainPt->resList),&(curAAchainPt->nresidues));

    prevResPt = theResPt;
    previous_pdb_reading_state = *pdb_reading_state;

    // set bonded atoms in residue 
    set_residue_bonds(theResPt);
    // set peptide bond with previous residue
    if(theResPt->prevResidue != NULL) {
      set_peptide_bond(theResPt,theResPt->prevResidue);
    }
  }

  return TRUE;
}


/**********************************************************************/
    
// Ligand psf writing function

static int extract_psf_ligand_struct(FILE *pdbfile, char* pdbLine, lineType* pdb_reading_state,
				     char *shortFileName, psf_ligand *ligPt) {

  psf_atom* read_aPt;
  char prevLigandName[5];
  char ligandName[5];

  if (*pdb_reading_state == LIGAND_ATOM_LINE) {

    snprintf(ligandName, 4, "%s", get_ligand_name(pdbLine));
      
    do {

      // get atom from a new line read in the current pdb file and
      // store that atom in ligand structure 
      read_aPt = (psf_atom*)malloc(sizeof(psf_atom)); 
      *pdb_reading_state = read_ligand_atom(pdbLine, read_aPt);
      
      if (*pdb_reading_state != WRONG_LINE) {
	insert_pointer_in_list(read_aPt,(void***)&(ligPt->atomList),&(ligPt->natoms));
      }
      
      // get new atom and the associated ligand (name)
      snprintf(prevLigandName, 4, "%s", ligandName);
      *pdb_reading_state = read_atom(pdbfile, pdbLine, 100);
      if (*pdb_reading_state == LIGAND_ATOM_LINE) {
	snprintf(ligandName, 4, "%s", get_ligand_name(pdbLine));
      }

    }  while ((*pdb_reading_state == LIGAND_ATOM_LINE)
	      && (strcmp(ligandName, prevLigandName) == 0));
  
  }
  
  if(*pdb_reading_state == WRONG_LINE) {
    printf("ERROR : wrong PDB ATOM line\n");
    return FALSE;
  }

  compute_ligand_centroid(ligPt, ligPt->oref);

  return TRUE;
}


///////// PSF READING/WRITING FUNCTIONS FROM/IN FILE ///////////////////////////

 // JOINT //
 
static int read_and_treat_joint(char *pdbline, psf_ligand *ligPt) {

  char piece[10];
  psf_joint *jointPt;
  int serial;
  psf_atom* theAtom;

  // create new joint
  jointPt = init_joint_struct();

  strcpy(piece,"         ");
  strncpy(piece,pdbline+7,5);
  if(sscanf(piece,"%d",&serial) < 0) return FALSE;
  if (psf_get_atom_in_list_by_serial(serial,&theAtom,ligPt->atomList,ligPt->natoms)) {
    jointPt->dihedralAtoms[0] = theAtom;
  }
  else {
    printf("ERROR : while reading joints : atom serial %d is not defined\n",serial);
    free(jointPt);
    free_pointer_list((void***)&(ligPt->jointList), &(ligPt->njoints));
    return FALSE;
  }

  strcpy(piece,"         ");
  strncpy(piece,pdbline+12,5);
  if(sscanf(piece,"%d",&serial) < 0) return FALSE;
  if (psf_get_atom_in_list_by_serial(serial,&theAtom,ligPt->atomList,ligPt->natoms)) {
    jointPt->dihedralAtoms[1] = theAtom;
  }
  else {
    printf("ERROR : while reading joints : atom serial %d is not defined\n",serial);
    free(jointPt);
    free_pointer_list((void***)&(ligPt->jointList), &(ligPt->njoints));
    return FALSE;
  }

  strcpy(piece,"         ");
  strncpy(piece,pdbline+17,5);
  if(sscanf(piece,"%d",&serial) < 0) return FALSE;
  if (psf_get_atom_in_list_by_serial(serial,&theAtom,ligPt->atomList,ligPt->natoms)) {
    jointPt->dihedralAtoms[2] = theAtom;
  }
  else {
    printf("ERROR : while reading joints : atom serial %d is not defined\n",serial);
    free(jointPt);
    free_pointer_list((void***)&(ligPt->jointList), &(ligPt->njoints));
    return FALSE;
  }

  strcpy(piece,"         ");
  strncpy(piece,pdbline+22,5);
  if(sscanf(piece,"%d",&serial) < 0) return FALSE;
  if (psf_get_atom_in_list_by_serial(serial,&theAtom,ligPt->atomList,ligPt->natoms)) {
    jointPt->dihedralAtoms[3] = theAtom;
  }
  else {
    printf("ERROR : while reading joints : atom serial %d is not defined\n",serial);
    free(jointPt);
    free_pointer_list((void***)&(ligPt->jointList), &(ligPt->njoints));
    return FALSE;
  }

  strcpy(piece,"         ");
  strncpy(piece,pdbline+27,7);
  if(sscanf(piece,"%lf",&(jointPt->vmin)) < 0) return FALSE;
  strcpy(piece,"         ");
  strncpy(piece,pdbline+35,7);
  if(sscanf(piece,"%lf",&(jointPt->vmax)) < 0) return FALSE;
  strcpy(piece,"         ");
  
  insert_pointer_in_list((void*)jointPt,(void***)&(ligPt->jointList),&(ligPt->njoints));
  return TRUE;
}

/**********************************************************************/

int load_joint_desc_from_file(const char* fullname, psf_ligand* ligPt) {

  FILE* jntFile;
  char *read_line;
  char piece[10];
  char rec[10];
  char returned_s[100];
  int read_OK = TRUE;
  int end = FALSE;

  jntFile = fopen(fullname, "r");
  if (jntFile == NULL) {
    printf("jnt file cannot be open\n");
    return FALSE;
  }

  do {
    
    read_line = fgets(returned_s,100,jntFile);
    
    if(read_line == NULL) {
      end = TRUE;
    }
    else {
      
      strcpy(piece,"         ");
      strncpy(piece,read_line,6);
      
      if ((sscanf(piece,"%s",rec) >= 0)
	  && (strcmp(rec,"JOINT") == 0)) {
	read_OK = read_and_treat_joint(read_line, ligPt);
      }
    }
  
  } while (read_OK && !end);

  fclose(jntFile);
    
  return read_OK;
}

/**********************************************************************/

/* JOINT record
   ------------
   CONTAINS :
     - columns  7-11 : atom serial of "base" rigid
     - columns 12-16 : atom serial of connected rigid
     - columns 17-21 : atom serial of connected rigid
     - columns 22-26 : atom serial of connected rigid
     - columns 27-34 : lower joint limit (degrees)
     - columns 35-42 : uper joint limit (degrees)
   NOTE1    : several intervals ???
   NOTE2    : parent joint ???
   WARNING  : up to 3 intervals ???
   EXAMPLE  :
            1         2         3         4         5         6         7         8
   12345678901234567890123456789012345678901234567890123456789012345678901234567890
   JOINT     7   15   17   22 -180.00  180.00
*/


static void write_joint_in_file(FILE* jntFile, psf_joint* joint) {

  int atomIndex;

  fprintf(jntFile,"%-6s","JOINT");
  for (atomIndex=0; atomIndex<4; atomIndex++) {
    fprintf(jntFile,"%5d",joint->dihedralAtoms[atomIndex]->serial);
  }
  fprintf(jntFile,"%8.2f",joint->vmin);
  fprintf(jntFile,"%8.2f",joint->vmax);
  fprintf(jntFile,"\n");

}

/**********************************************************************/

int save_joint_desc_in_file(const char* fullname, psf_ligand* ligPt) {

  FILE* jntFile;
  int jointIndex;

  jntFile = fopen(fullname, "w");
  if (jntFile == NULL) {
    printf("jnt file cannot be open\n");
    return FALSE;
  }
  
  for (jointIndex=0; jointIndex<ligPt->njoints; jointIndex++) {
    write_joint_in_file(jntFile, ligPt->jointList[jointIndex]);
  }

  fclose(jntFile);
  return TRUE;

}

/**********************************************************************/

