#ifndef PDBFORMAT_H
#define PDBFORMAT_H

#include "atoms.h"
#include "protein.h"

typedef enum {
  INSIGHT, AMBER
} formatTypes;

extern void translate_pdb_res_name(char* pdb_res_name, char* psf_res_name, 
				   formatTypes pdb_format);
extern void translate_pdb_atom_name(char* pdb_atom_name, char* psf_atom_name,
				    residueTypes resType, formatTypes pdb_format);
extern int scanFile(FILE* pdbfile, formatTypes* format);
extern int update_amber_serial(residue* resPt, int firstSer, int* lastSer);
extern void write_residue_with_amber_order(FILE* pdboutfile, residue* resPt);
extern int get_amber_serial(atom* aPt, int* serial);

#endif
