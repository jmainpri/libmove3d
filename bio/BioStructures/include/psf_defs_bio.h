/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#ifndef PSF_DEFS_BIO
#define PSF_DEFS_BIO

//////// BIO CONSTANT DEFINITIONS //////////////////////////////////////////////

/////////////////////////////////
// Peptide bond dihedral angle //
/////////////////////////////////

#define OMEGAMAX 190.0*(PI/180.0)
#define OMEGAMIN 170.0*(PI/180.0)

/////////////////////////
// Van Der Walls radii //
/////////////////////////

#define C_VDWR 1.53
#define N_VDWR 1.38
#define O_VDWR 1.35
#define S_VDWR 1.79
#define H_VDWR 1.10

#define Br_VDWR 1.94
#define Cl_VDWR 1.74
#define F_VDWR 1.29
#define P_VDWR 1.87
#define I_VDWR 2.14

/* #define C_VDWR 1.7 */
/* #define N_VDWR 1.55 */
/* #define O_VDWR 1.52 */
/* #define S_VDWR 1.8 */
/* #define H_VDWR 1.2 */

//////////////////////////////////
// Covalent factor necessary to //
//    make bond in ligands      //
//////////////////////////////////

#define COVBOND_FACTOR 0.65


////////////////
// atom names //
////////////////

typedef enum {
 psf_SULPHUR, psf_SULPHUR_H, psf_OXYGEN, psf_OXYGEN_H, psf_NITROGEN, psf_NITROGEN_H, psf_NITROGEN_FULL,
 psf_CARBON, psf_HYDROGEN, psf_HYDROGEN_P, psf_BROMINE, psf_IODINE, psf_FLUORINE, psf_PHOSPHORUS, psf_CHLORINE
} atomTypes; 

////////  RESIDUES /////////////////////////////////////////////////////////////


typedef enum {
  psf_ALA, psf_ARG, psf_ASN, psf_ASP, psf_CYS, psf_GLN, psf_GLU, psf_GLY, psf_HIS, psf_ILE, 
  psf_LEU, psf_LYS, psf_MET, psf_PHE, psf_PRO, psf_SER, psf_THR, psf_TRP, psf_TYR, psf_VAL,
  psf_ALAH, psf_ARGH, psf_ASNH, psf_ASPH, psf_CYSH, psf_GLNH, psf_GLUH, psf_GLYH, psf_HISH, psf_ILEH, 
  psf_LEUH, psf_LYSH, psf_METH, psf_PHEH, psf_PROH, psf_SERH, psf_THRH, psf_TRPH, psf_TYRH, psf_VALH 
} residueTypes;


//////// ATOMS IN RESIDUES ///////////////////////////////////////////////////// 


// atom indices (BCD order)
// NOTE 1 : BKBs, psf_ILE_sch, psf_LEU_sch, psf_VAL_sch have different order in .p3d file !!!

// NOTE 2 : H' are included in the model. Each enumeration contains H' if they exist.
// BE CAREFULL ! the number of atoms must not take them in account ! 


////////////////////
// backbone atoms //
////////////////////

# define N_GEN_BKB_ATOMS 6
typedef enum {
  psf_gen_C, psf_gen_O, psf_gen_CA, psf_gen_CB, psf_gen_N, psf_gen_OXT
} psf_gen_bkb_atoms;

# define N_GENH_BKB_ATOMS 7
typedef enum {
  psf_genH_C, psf_genH_O, psf_genH_CA, psf_genH_HA, psf_genH_CB, psf_genH_N,
  psf_genH_H, psf_genH_H_Nterm,
  psf_genH_OXT
} psf_genH_bkb_atoms;
// (psf_genH_H, psf_genH_H_Nterm) are H'
// Generally, there is no H on the terminal COO group in pdb files

# define N_GLY_BKB_ATOMS 5
typedef enum {
  psf_GLY_C, psf_GLY_O, psf_GLY_CA, psf_GLY_N, psf_GLY_OXT 
} psf_GLY_bkb_atoms;

# define N_GLYH_BKB_ATOMS 7
typedef enum {
  psf_GLYH_C, psf_GLYH_O, psf_GLYH_CA, psf_GLYH_1HA, psf_GLYH_2HA, psf_GLYH_N,
  psf_GLYH_H, psf_GLYH_H_Nterm,
  psf_GLYH_OXT 
} psf_GLYH_bkb_atoms;
// (psf_GLYH_H, psf_GLYH_H_Nterm) are H'
// Generally, there is no H on the terminal COO group in pdb files

# define N_PRO_BKB_ATOMS 8
typedef enum psf_PRO_bkb_atoms {
  psf_PRO_C, psf_PRO_O, psf_PRO_CA, psf_PRO_CB, psf_PRO_CG, psf_PRO_CD, psf_PRO_N,
  psf_PRO_OXT 
} psf_PRO_bkb_atoms;

# define N_PROH_BKB_ATOMS 15
typedef enum {
  psf_PROH_C, psf_PROH_O, psf_PROH_CA, psf_PROH_HA, psf_PROH_CB, psf_PROH_N, psf_PROH_1HB, psf_PROH_2HB,
  psf_PROH_CG, psf_PROH_1HG, psf_PROH_2HG, psf_PROH_CD, psf_PROH_1HD, psf_PROH_2HD, psf_PROH_OXT 
} psf_PROH_bkb_atoms;


///////////////////////
// side-chains atoms //
///////////////////////


# define N_ALA_SCH_ATOMS 0
# define N_ALAH_SCH_ATOMS 3
typedef enum {
  psf_ALAH_1HB, psf_ALAH_2HB, psf_ALAH_3HB 
} psf_ALAH_sch_atoms;

# define N_ARG_SCH_ATOMS 6
typedef enum {
  psf_ARG_CG, psf_ARG_CD, psf_ARG_NE, psf_ARG_CZ, psf_ARG_NH1, psf_ARG_NH2 
} psf_ARG_sch_atoms;

# define N_ARGH_SCH_ATOMS 12
typedef enum {
  psf_ARGH_1HB, psf_ARGH_2HB, psf_ARGH_CG, psf_ARGH_1HG, psf_ARGH_2HG, psf_ARGH_CD, psf_ARGH_1HD, psf_ARGH_2HD, psf_ARGH_NE, psf_ARGH_CZ, psf_ARGH_NH1, psf_ARGH_NH2,
  psf_ARGH_1HH1, psf_ARGH_2HH1, psf_ARGH_1HH2, psf_ARGH_2HH2, psf_ARGH_HE 
} psf_ARGH_sch_atoms ;
// (psf_ARGH_1HH1, psf_ARGH_2HH1, psf_ARGH_1HH2, psf_ARGH_2HH2, psf_ARGH_HE) are H'


# define N_ASN_SCH_ATOMS 3
typedef enum {
  psf_ASN_CG, psf_ASN_OD1, psf_ASN_ND2 
} psf_ASN_sch_atoms;

# define N_ASNH_SCH_ATOMS 5
typedef enum {
  psf_ASNH_1HB, psf_ASNH_2HB, psf_ASNH_CG, psf_ASNH_OD1, psf_ASNH_ND2,
  psf_ASNH_1HD2, psf_ASNH_2HD2
} psf_ASNH_sch_atoms;
// (psf_ASNH_1HD2, psf_ASNH_2HD2) are H'

# define N_ASP_SCH_ATOMS 3
typedef enum {
  psf_ASP_CG, psf_ASP_OD1, psf_ASP_OD2 
} psf_ASP_sch_atoms;

# define N_ASPH_SCH_ATOMS 5
typedef enum {
  psf_ASPH_1HB, psf_ASPH_2HB, psf_ASPH_CG, psf_ASPH_OD1, psf_ASPH_OD2 
} psf_ASPH_sch_atoms;

# define N_CYS_SCH_ATOMS 1
typedef enum {
  psf_CYS_SG 
} psf_CYS_sch_atoms;

# define N_CYSH_SCH_ATOMS 3
typedef enum {
  psf_CYSH_1HB, psf_CYSH_2HB, psf_CYSH_SG,
  psf_CYSH_HG
} psf_CYSH_sch_atoms;
// psf_CYSH_HG is H'

# define N_GLN_SCH_ATOMS 4
typedef enum {
  psf_GLN_CG, psf_GLN_CD, psf_GLN_OE1, psf_GLN_NE2 
} psf_GLN_sch_atoms;

# define N_GLNH_SCH_ATOMS 8
typedef enum {
  psf_GLNH_1HB, psf_GLNH_2HB, psf_GLNH_CG, psf_GLNH_1HG, psf_GLNH_2HG, psf_GLNH_CD, psf_GLNH_OE1, psf_GLNH_NE2, 
  psf_GLNH_1HE2, psf_GLNH_2HE2
} psf_GLNH_sch_atoms;
// (psf_GLNH_1HE2, psf_GLNH_2HE2) are H'

# define N_GLU_SCH_ATOMS 4
typedef enum {
  psf_GLU_CG, psf_GLU_CD, psf_GLU_OE1, psf_GLU_OE2 
} psf_GLU_sch_atoms;

# define N_GLUH_SCH_ATOMS 8
typedef enum {
  psf_GLUH_1HB, psf_GLUH_2HB, psf_GLUH_CG, psf_GLUH_1HG, psf_GLUH_2HG, psf_GLUH_CD, psf_GLUH_OE1, psf_GLUH_OE2 
} psf_GLUH_sch_atoms;

# define N_GLY_SCH_ATOMS 0
# define N_GLYH_SCH_ATOMS 0
// psf_GLY : no side-chain 

# define N_HIS_SCH_ATOMS 5
typedef enum {
  psf_HIS_CG, psf_HIS_ND1, psf_HIS_CD2, psf_HIS_CE1, psf_HIS_NE2 
} psf_HIS_sch_atoms;

# define N_HISH_SCH_ATOMS 9
typedef enum {
  psf_HISH_1HB, psf_HISH_2HB, psf_HISH_CG, psf_HISH_ND1, psf_HISH_CD2, psf_HISH_HD2, psf_HISH_CE1, psf_HISH_HE1, psf_HISH_NE2, 
  psf_HISH_HE2, psf_HISH_HD1 
} psf_HISH_sch_atoms;
// (psf_HISH_HE2, psf_HISH_HD1) is H'

# define N_ILE_SCH_ATOMS 3
typedef enum {
  psf_ILE_CG2, psf_ILE_CG1, psf_ILE_CD1 
} psf_ILE_sch_atoms;

# define N_ILEH_SCH_ATOMS 12
typedef enum {
  psf_ILEH_CG2, psf_ILEH_HB, psf_ILEH_CG1, psf_ILEH_1HG1, psf_ILEH_2HG1, psf_ILEH_CD1,
  psf_ILEH_1HG2, psf_ILEH_2HG2, psf_ILEH_3HG2, psf_ILEH_1HD1, psf_ILEH_2HD1, psf_ILEH_3HD1
} psf_ILEH_sch_atoms;

# define N_LEU_SCH_ATOMS 3
typedef enum {
  psf_LEU_CG, psf_LEU_CD1, psf_LEU_CD2 
} psf_LEU_sch_atoms;

# define N_LEUH_SCH_ATOMS 12
typedef enum {
  psf_LEUH_1HB, psf_LEUH_2HB, psf_LEUH_CG, psf_LEUH_HG, psf_LEUH_CD1, psf_LEUH_CD2,
  psf_LEUH_1HD1, psf_LEUH_2HD1, psf_LEUH_3HD1, psf_LEUH_1HD2, psf_LEUH_2HD2, psf_LEUH_3HD2 
} psf_LEUH_sch_atoms;

# define N_LYS_SCH_ATOMS 4
typedef enum {
  psf_LYS_CG, psf_LYS_CD, psf_LYS_CE, psf_LYS_NZ 
} psf_LYS_sch_atoms;

# define N_LYSH_SCH_ATOMS 12
typedef enum {
  psf_LYSH_1HB, psf_LYSH_2HB, psf_LYSH_CG, psf_LYSH_1HG, psf_LYSH_2HG, psf_LYSH_CD, psf_LYSH_1HD, psf_LYSH_2HD, 
  psf_LYSH_CE, psf_LYSH_1HE, psf_LYSH_2HE, psf_LYSH_NZ,
  psf_LYSH_1HZ, psf_LYSH_2HZ, psf_LYSH_3HZ
} psf_LYSH_sch_atoms;
// (psf_LYSH_1HZ, psf_LYSH_2HZ, psf_LYSH_3HZ ) are H'


# define N_MET_SCH_ATOMS 3
typedef enum {
  psf_MET_CG, psf_MET_SD, psf_MET_CE 
} psf_MET_sch_atoms;

# define N_METH_SCH_ATOMS 10
typedef enum {
  psf_METH_1HB, psf_METH_2HB, psf_METH_CG, psf_METH_1HG, psf_METH_2HG, psf_METH_SD, psf_METH_CE, psf_METH_1HE, psf_METH_2HE, psf_METH_3HE 
} psf_METH_sch_atoms;

# define N_PHE_SCH_ATOMS 6
typedef enum {
  psf_PHE_CG, psf_PHE_CD1, psf_PHE_CD2, psf_PHE_CE1, psf_PHE_CE2, psf_PHE_CZ 
} psf_PHE_sch_atoms;

# define N_PHEH_SCH_ATOMS 13
typedef enum {
  psf_PHEH_1HB, psf_PHEH_2HB, psf_PHEH_CG, psf_PHEH_CD1, psf_PHEH_CD2, psf_PHEH_HD1, psf_PHEH_HD2, 
  psf_PHEH_CE1, psf_PHEH_CE2, psf_PHEH_CZ, psf_PHEH_HE1, psf_PHEH_HE2, psf_PHEH_HZ 
} psf_PHEH_sch_atoms;

# define N_PRO_SCH_ATOMS 0
# define N_PROH_SCH_ATOMS 0
// psf_PRO : no side-chain

# define N_SER_SCH_ATOMS 1
typedef enum {
  psf_SER_OG 
} psf_SER_sch_atoms;

# define N_SERH_SCH_ATOMS 3
typedef enum {
  psf_SERH_1HB, psf_SERH_2HB, psf_SERH_OG,
  psf_SERH_HG
} psf_SERH_sch_atoms;
// psf_SERH_HG is H'

# define N_THR_SCH_ATOMS 2
typedef enum {
  psf_THR_OG1, psf_THR_CG2 
} psf_THR_sch_atoms;

# define N_THRH_SCH_ATOMS 6
typedef enum {
  psf_THRH_HB, psf_THRH_OG1, psf_THRH_CG2, psf_THRH_1HG2, psf_THRH_2HG2, psf_THRH_3HG2,
  psf_THRH_HG1 
} psf_THRH_sch_atoms;
// psf_THRH_HG1 is H'

# define N_TRP_SCH_ATOMS 9
typedef enum {
  psf_TRP_CG, psf_TRP_CD1, psf_TRP_CD2, psf_TRP_NE1, psf_TRP_CE2, psf_TRP_CE3, psf_TRP_CZ2, psf_TRP_CZ3, psf_TRP_CH2 
} psf_TRP_sch_atoms;

# define N_TRPH_SCH_ATOMS 16
typedef enum {
  psf_TRPH_1HB, psf_TRPH_2HB, psf_TRPH_CG, psf_TRPH_CD1, psf_TRPH_CD2, psf_TRPH_HD1, psf_TRPH_NE1, psf_TRPH_CE2, psf_TRPH_CE3, psf_TRPH_HE3,
  psf_TRPH_CZ2, psf_TRPH_HZ2, psf_TRPH_CZ3, psf_TRPH_HZ3, psf_TRPH_CH2, psf_TRPH_HH2,
  psf_TRPH_HE1
} psf_TRPH_sch_atoms;
// psf_TRPH_HE1 is H'

# define N_TYR_SCH_ATOMS 7
typedef enum {
  psf_TYR_CG, psf_TYR_CD1, psf_TYR_CD2, psf_TYR_CE1, psf_TYR_CE2, psf_TYR_CZ, psf_TYR_OH 
} psf_TYR_sch_atoms;

# define N_TYRH_SCH_ATOMS 13
typedef enum {
  psf_TYRH_1HB, psf_TYRH_2HB, psf_TYRH_CG, psf_TYRH_CD1, psf_TYRH_CD2, psf_TYRH_CE1, psf_TYRH_CE2, 
  psf_TYRH_HD1, psf_TYRH_HD2, psf_TYRH_HE1, psf_TYRH_HE2, psf_TYRH_CZ, psf_TYRH_OH, 
  psf_TYRH_HH
} psf_TYRH_sch_atoms;
// psf_TYRH_HH is H'

# define N_VAL_SCH_ATOMS 2
typedef enum {
  psf_VAL_CG1, psf_VAL_CG2 
} psf_VAL_sch_atoms;

# define N_VALH_SCH_ATOMS 9
typedef enum {
  psf_VALH_HB, psf_VALH_CG1, psf_VALH_CG2, psf_VALH_1HG1, psf_VALH_2HG1, psf_VALH_3HG1, psf_VALH_1HG2, psf_VALH_2HG2, psf_VALH_3HG2 
} psf_VALH_sch_atoms;


#endif
