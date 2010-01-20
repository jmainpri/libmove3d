#ifndef BIO_2AMBER_H
#define BIO_2AMBER_H


/* amber */
#include <basics.h>
#include <stringExtra.h>
#include <vector.h>
#include <matrix.h>
#include <classes.h>
#include <dictionary.h>
#include <database.h>
#include <library.h>
#include <parmLib.h>
#include <pdbFile.h>
#include <help.h>
#include <parser.h>
#include <tools.h>
#include <amber.h>
#include <commands.h>
#include <defaults.h>
#include <leap.h>
#include <octree.h>
#include <tripos.h>
#include <block.h>
#include <leap.h>
#include <getline.h> 




void AMBER_addPath( STRING dir_name );
OBJEKT AMBER_add_source(char *filename);
OBJEKT AMBER_loadPdb(  FILE *fPdb);
OBJEKT AMBER_loadPrep(char*file_prep);
OBJEKT AMBER_loadMol2( char *mol2FileName );
OBJEKT AMBER_loadParams(char *frcmod);
void AMBER_saveParm(UNIT    uUnit,char *name_first );
void AMBER_savePdbInGivenFile( OBJEKT objProt, FILE *fOut);
void AMBER_savePdb( OBJEKT objProt, char *name_first);
OBJEKT AMBER_Combine(OBJEKT  objProt, OBJEKT objLig);

#endif /* #ifndef BIO_2AMBER_H */
