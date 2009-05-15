#ifndef PSF_TYPE
#define PSF_TYPE

#include "psf_defs_bio.h"

#define PSF_MAX_NAME_LENGTH 30

/**********************************************************************
 *                Protein Structure Format (PSF)                      *
 **********************************************************************/

typedef struct s_psf_atom {
  struct s_psf_residue *residuePt;
  int serial;
  char name[5];
  atomTypes atomType;
  double pos[3];
  double vdwR;
  int nbondedA;
  struct s_psf_atom **bondedAtomList;
  struct s_psf_rigid *rigidPt;
} psf_atom;


// PROTEIN

typedef struct s_psf_residue {
  struct s_psf_AAchain *chainPt;
  int resSeq;
  char resName[4];
  residueTypes resType;
  int flagH;      // indicates if it contains hydrogen atoms
  int flagCterm;  // indicates if it is a C terminal residue with OXT
  // NOTE : use pseudo backbone and side-chain
  int nbkbatoms;
  int nschatoms;
  // NOTE : atoms in lists have precise order (defined for BCD)
  struct s_psf_atom **bkbAtomList;
  struct s_psf_atom **schAtomList;
  struct s_psf_residue *nextResidue;
  struct s_psf_residue *prevResidue;
} psf_residue;


typedef struct s_psf_AAchain {
  struct s_psf_protein *proteinPt;
  char chainID[2];
  int subchainind;
  int nresidues;
  struct s_psf_residue **resList;
} psf_AAchain;


typedef struct s_psf_protein {
  char name[PSF_MAX_NAME_LENGTH];
  int nchains;
  struct s_psf_AAchain **chainList;
} psf_protein;


// LIGAND

typedef struct s_psf_rigid {
  int natoms;
  int noutJnts;
  int ninJnts;
  struct s_psf_atom **atomList;
  struct s_psf_joint **outJntsList;
  struct s_psf_joint **inJntsList;
  int indexparentj;
} psf_rigid;

typedef struct s_psf_joint {
  struct s_psf_atom *dihedralAtoms[4];
  double vmin;
  double vmax;
  double value;
  struct s_psf_rigid *rigidIn;
  struct s_psf_rigid *rigidOut;
} psf_joint;


typedef struct s_psf_ligand {
  char name[PSF_MAX_NAME_LENGTH];
  int natoms;
  int nrigids;
  int njoints;
  struct s_psf_atom **atomList;
  struct s_psf_rigid **rigidList;
  struct s_psf_joint **jointList;
  double oref[3];
} psf_ligand;

// MOLECULE

typedef struct s_psf_molecule {
  char name[PSF_MAX_NAME_LENGTH];
  int nproteins;
  int nligands;
  struct s_psf_protein** proteinList;
  struct s_psf_ligand** ligandList;
} psf_molecule;

#endif
