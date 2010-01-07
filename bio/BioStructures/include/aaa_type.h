#ifndef AAA_TYPE_H
#define AAA_TYPE_H

#define RIGID 0
#define MOBILE 1

#define MAX_PROTEIN_NAME_LENGTH 30

typedef struct {
  int resSeq;
  char resName[4];
  int art_bkb;
  int art_sch;
} AAA_residue_data;

typedef struct {
  char proteinName[MAX_PROTEIN_NAME_LENGTH];
  int nb_AAA_residues;
  AAA_residue_data** AAA_residueList;
} AAA_residue_data_list;

typedef struct {
  int nb_AAA_protein;
  AAA_residue_data_list** AAA_proteinList;
} AAA_protein_data_list;

#endif
