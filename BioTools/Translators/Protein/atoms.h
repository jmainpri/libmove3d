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
#ifndef ATOM_H
#define ATOM_H

// atom indices (BCD order)
// NOTE 1 : BKBs, ILE_sch, LEU_sch, VAL_sch have different order in .p3d file !!!

// NOTE 2 : H' are included in the model. Each enumeration contains H' if they exist.
// BE CAREFULL ! the number of atoms must not take them in account ! 

// backbone

# define N_GEN_BKB_ATOMS 6
typedef enum {
  gen_C, gen_O, gen_CA, gen_CB, gen_N, gen_OXT
} gen_bkb_atoms;

# define N_GENH_BKB_ATOMS 7
typedef enum {
  genH_C, genH_O, genH_CA, genH_HA, genH_CB, genH_N,
  genH_OXT
} genH_bkb_atoms;

# define N_GLY_BKB_ATOMS 5
typedef enum {
  GLY_C, GLY_O, GLY_CA, GLY_N, GLY_OXT 
} GLY_bkb_atoms;

# define N_GLYH_BKB_ATOMS 7
typedef enum {
  GLYH_C, GLYH_O, GLYH_CA, GLYH_1HA, GLYH_2HA, GLYH_N,
  GLYH_OXT 
} GLYH_bkb_atoms;

# define N_PRO_BKB_ATOMS 8
typedef enum PRO_bkb_atoms {
  PRO_C, PRO_O, PRO_CA, PRO_CB, PRO_CG, PRO_CD, PRO_N,
  PRO_OXT 
} PRO_bkb_atoms;

# define N_PROH_BKB_ATOMS 15
typedef enum {
  PROH_C, PROH_O, PROH_CA, PROH_HA, PROH_CB, PROH_N, PROH_1HB, PROH_2HB,
  PROH_CG, PROH_1HG, PROH_2HG, PROH_CD, PROH_1HD, PROH_2HD, PROH_OXT 
} PROH_bkb_atoms;

// side-chains

# define N_ALA_SCH_ATOMS 0
# define N_ALAH_SCH_ATOMS 3
typedef enum {
  ALAH_1HB, ALAH_2HB, ALAH_3HB 
} ALAH_sch_atoms;

# define N_ARG_SCH_ATOMS 6
typedef enum {
  ARG_CG, ARG_CD, ARG_NE, ARG_CZ, ARG_NH1, ARG_NH2 
} ARG_sch_atoms;

# define N_ARGH_SCH_ATOMS 12
typedef enum {
  ARGH_1HB, ARGH_2HB, ARGH_CG, ARGH_1HG, ARGH_2HG, ARGH_CD, ARGH_1HD, ARGH_2HD, ARGH_NE, ARGH_CZ, ARGH_NH1, ARGH_NH2
} ARGH_sch_atoms ;


# define N_ASN_SCH_ATOMS 3
typedef enum {
  ASN_CG, ASN_OD1, ASN_ND2 
} ASN_sch_atoms;

# define N_ASNH_SCH_ATOMS 5
typedef enum {
  ASNH_1HB, ASNH_2HB, ASNH_CG, ASNH_OD1, ASNH_ND2
} ASNH_sch_atoms;

# define N_ASP_SCH_ATOMS 3
typedef enum {
  ASP_CG, ASP_OD1, ASP_OD2 
} ASP_sch_atoms;

# define N_ASPH_SCH_ATOMS 5
typedef enum {
  ASPH_1HB, ASPH_2HB, ASPH_CG, ASPH_OD1, ASPH_OD2 
} ASPH_sch_atoms;

# define N_CYS_SCH_ATOMS 1
typedef enum {
  CYS_SG 
} CYS_sch_atoms;

# define N_CYSH_SCH_ATOMS 3
typedef enum {
  CYSH_1HB, CYSH_2HB, CYSH_SG
} CYSH_sch_atoms;

# define N_GLN_SCH_ATOMS 4
typedef enum {
  GLN_CG, GLN_CD, GLN_OE1, GLN_NE2 
} GLN_sch_atoms;

# define N_GLNH_SCH_ATOMS 8
typedef enum {
  GLNH_1HB, GLNH_2HB, GLNH_CG, GLNH_1HG, GLNH_2HG, GLNH_CD, GLNH_OE1, GLNH_NE2
} GLNH_sch_atoms;

# define N_GLU_SCH_ATOMS 4
typedef enum {
  GLU_CG, GLU_CD, GLU_OE1, GLU_OE2 
} GLU_sch_atoms;

# define N_GLUH_SCH_ATOMS 8
typedef enum {
  GLUH_1HB, GLUH_2HB, GLUH_CG, GLUH_1HG, GLUH_2HG, GLUH_CD, GLUH_OE1, GLUH_OE2 
} GLUH_sch_atoms;

# define N_GLY_SCH_ATOMS 0
# define N_GLYH_SCH_ATOMS 0
// GLY : no side-chain 

# define N_HIS_SCH_ATOMS 5
typedef enum {
  HIS_CG, HIS_ND1, HIS_CD2, HIS_CE1, HIS_NE2 
} HIS_sch_atoms;

# define N_HISH_SCH_ATOMS 9
typedef enum {
  HISH_1HB, HISH_2HB, HISH_CG, HISH_ND1, HISH_CD2, HISH_HD2, HISH_CE1, HISH_HE1, HISH_NE2
} HISH_sch_atoms;

# define N_ILE_SCH_ATOMS 3
typedef enum {
  ILE_CG2, ILE_CG1, ILE_CD1 
} ILE_sch_atoms;

# define N_ILEH_SCH_ATOMS 12
typedef enum {
  ILEH_CG2, ILEH_HB, ILEH_CG1, ILEH_1HG1, ILEH_2HG1, ILEH_CD1,
  ILEH_1HG2, ILEH_2HG2, ILEH_3HG2, ILEH_1HD1, ILEH_2HD1, ILEH_3HD1
} ILEH_sch_atoms;

# define N_LEU_SCH_ATOMS 3
typedef enum {
  LEU_CG, LEU_CD1, LEU_CD2 
} LEU_sch_atoms;

# define N_LEUH_SCH_ATOMS 12
typedef enum {
  LEUH_1HB, LEUH_2HB, LEUH_CG, LEUH_HG, LEUH_CD1, LEUH_CD2,
  LEUH_1HD1, LEUH_2HD1, LEUH_3HD1, LEUH_1HD2, LEUH_2HD2, LEUH_3HD2 
} LEUH_sch_atoms;

# define N_LYS_SCH_ATOMS 4
typedef enum {
  LYS_CG, LYS_CD, LYS_CE, LYS_NZ 
} LYS_sch_atoms;

# define N_LYSH_SCH_ATOMS 12
typedef enum {
  LYSH_1HB, LYSH_2HB, LYSH_CG, LYSH_1HG, LYSH_2HG, LYSH_CD, LYSH_1HD, LYSH_2HD,
  LYSH_CE, LYSH_1HE, LYSH_2HE, LYSH_NZ,
} LYSH_sch_atoms;



# define N_MET_SCH_ATOMS 3
typedef enum {
  MET_CG, MET_SD, MET_CE 
} MET_sch_atoms;

# define N_METH_SCH_ATOMS 10
typedef enum {
  METH_1HB, METH_2HB, METH_CG, METH_1HG, METH_2HG, METH_SD, METH_CE, METH_1HE, METH_2HE, METH_3HE 
} METH_sch_atoms;

# define N_PHE_SCH_ATOMS 6
typedef enum {
  PHE_CG, PHE_CD1, PHE_CD2, PHE_CE1, PHE_CE2, PHE_CZ 
} PHE_sch_atoms;

# define N_PHEH_SCH_ATOMS 13
typedef enum {
  PHEH_1HB, PHEH_2HB, PHEH_CG, PHEH_CD1, PHEH_CD2, PHEH_HD1, PHEH_HD2,
  PHEH_CE1, PHEH_CE2, PHEH_CZ, PHEH_HE1, PHEH_HE2, PHEH_HZ 
} PHEH_sch_atoms;

# define N_PRO_SCH_ATOMS 0
# define N_PROH_SCH_ATOMS 0
// PRO : no side-chain

# define N_SER_SCH_ATOMS 1
typedef enum {
  SER_OG 
} SER_sch_atoms;

# define N_SERH_SCH_ATOMS 3
typedef enum {
  SERH_1HB, SERH_2HB, SERH_OG
} SERH_sch_atoms;

# define N_THR_SCH_ATOMS 2
typedef enum {
  THR_OG1, THR_CG2 
} THR_sch_atoms;

# define N_THRH_SCH_ATOMS 6
typedef enum {
  THRH_HB, THRH_OG1, THRH_CG2, THRH_1HG2, THRH_2HG2, THRH_3HG2
} THRH_sch_atoms;

# define N_TRP_SCH_ATOMS 9
typedef enum {
  TRP_CG, TRP_CD1, TRP_CD2, TRP_NE1, TRP_CE2, TRP_CE3, TRP_CZ2, TRP_CZ3, TRP_CH2 
} TRP_sch_atoms;

# define N_TRPH_SCH_ATOMS 16
typedef enum {
  TRPH_1HB, TRPH_2HB, TRPH_CG, TRPH_CD1, TRPH_CD2, TRPH_HD1, TRPH_NE1, TRPH_CE2, TRPH_CE3, TRPH_HE3,
  TRPH_CZ2, TRPH_HZ2, TRPH_CZ3, TRPH_HZ3, TRPH_CH2, TRPH_HH2
} TRPH_sch_atoms;

# define N_TYR_SCH_ATOMS 7
typedef enum {
  TYR_CG, TYR_CD1, TYR_CD2, TYR_CE1, TYR_CE2, TYR_CZ, TYR_OH 
} TYR_sch_atoms;

# define N_TYRH_SCH_ATOMS 13
typedef enum {
  TYRH_1HB, TYRH_2HB, TYRH_CG, TYRH_CD1, TYRH_CD2, TYRH_CE1, TYRH_CE2, 
  TYRH_HD1, TYRH_HD2, TYRH_HE1, TYRH_HE2, TYRH_CZ, TYRH_OH
} TYRH_sch_atoms;

# define N_VAL_SCH_ATOMS 2
typedef enum {
  VAL_CG1, VAL_CG2 
} VAL_sch_atoms;

# define N_VALH_SCH_ATOMS 9
typedef enum {
  VALH_HB, VALH_CG1, VALH_CG2, VALH_1HG1, VALH_2HG1, VALH_3HG1, VALH_1HG2, VALH_2HG2, VALH_3HG2 
} VALH_sch_atoms;

/********************************************************************/

// AMBER TYPES

// NO AMBER TYPE : !!! must be greater than maximal AMB_N
#define AMB_NONE 10000


#define AMB_N_GENH_BKB_ATOMS 7


#define AMB_N_GLYH_BKB_ATOMS 7
#define AMB_N_GLYH_SCH_ATOMS 0
typedef enum {
  AMB_GLYH_N,
  AMB_GLYH_H,
  AMB_GLYH_CA, 
  AMB_GLYH_2HA, 
  AMB_GLYH_3HA, 
  AMB_GLYH_C,
  AMB_GLYH_O,
  AMB_GLYH_OXT
} AMB_GLYH_atoms;


#define AMB_N_PROH_BKB_ATOMS 14
#define AMB_N_PROH_SCH_ATOMS 0
typedef enum {
  AMB_PROH_N, 
  AMB_PROH_CD, 
  AMB_PROH_2HD, 
  AMB_PROH_3HD, 
  AMB_PROH_CG, 
  AMB_PROH_2HG, 
  AMB_PROH_3HG, 
  AMB_PROH_CB, 
  AMB_PROH_2HB, 
  AMB_PROH_3HB,
  AMB_PROH_CA, 
  AMB_PROH_HA, 
  AMB_PROH_C, 
  AMB_PROH_O,
  AMB_PROH_OXT
} AMB_PROH_atoms;

#define AMB_N_ALAH_SCH_ATOMS 3
typedef enum {
  AMB_ALAH_N,
  AMB_ALAH_H,
  AMB_ALAH_CA, 
  AMB_ALAH_HA, 
  AMB_ALAH_CB, 
  AMB_ALAH_1HB, 
  AMB_ALAH_2HB, 
  AMB_ALAH_3HB, 
  AMB_ALAH_C,
  AMB_ALAH_O,
  AMB_ALAH_OXT
} AMB_ALAH_atoms;


#define AMB_N_ARGH_SCH_ATOMS 17
typedef enum {
  AMB_ARGH_N,
  AMB_ARGH_H,
  AMB_ARGH_CA, 
  AMB_ARGH_HA, 
  AMB_ARGH_CB, 
  AMB_ARGH_2HB, 
  AMB_ARGH_3HB, 
  AMB_ARGH_CG, 
  AMB_ARGH_2HG, 
  AMB_ARGH_3HG, 
  AMB_ARGH_CD, 
  AMB_ARGH_2HD, 
  AMB_ARGH_3HD, 
  AMB_ARGH_NE, 
  AMB_ARGH_HE,
  AMB_ARGH_CZ, 
  AMB_ARGH_NH1, 
  AMB_ARGH_1HH1,
  AMB_ARGH_2HH1,
  AMB_ARGH_NH2,
  AMB_ARGH_1HH2,
  AMB_ARGH_2HH2,
  AMB_ARGH_C,
  AMB_ARGH_O,
  AMB_ARGH_OXT
} AMB_ARGH_atoms ;


#define AMB_N_ASNH_SCH_ATOMS 7
typedef enum {
  AMB_ASNH_N,
  AMB_ASNH_H,
  AMB_ASNH_CA, 
  AMB_ASNH_HA, 
  AMB_ASNH_CB, 
  AMB_ASNH_2HB, 
  AMB_ASNH_3HB,
  AMB_ASNH_CG, 
  AMB_ASNH_OD1, 
  AMB_ASNH_ND2,
  AMB_ASNH_1HD2,
  AMB_ASNH_2HD2,
  AMB_ASNH_C,
  AMB_ASNH_O,
  AMB_ASNH_OXT
} AMB_ASNH_atoms;


#define AMB_N_ASPH_SCH_ATOMS 5
typedef enum {
  AMB_ASPH_N,
  AMB_ASPH_H,
  AMB_ASPH_CA, 
  AMB_ASPH_HA, 
  AMB_ASPH_CB, 
  AMB_ASPH_2HB, 
  AMB_ASPH_3HB, 
  AMB_ASPH_CG, 
  AMB_ASPH_OD1, 
  AMB_ASPH_OD2,
  AMB_ASPH_C,
  AMB_ASPH_O,
  AMB_ASPH_OXT
} AMB_ASPH_atoms;

#define AMB_N_ASHH_SCH_ATOMS 6
typedef enum {
  AMB_ASHH_N,
  AMB_ASHH_H,
  AMB_ASHH_CA, 
  AMB_ASHH_HA, 
  AMB_ASHH_CB, 
  AMB_ASHH_2HB, 
  AMB_ASHH_3HB, 
  AMB_ASHH_CG, 
  AMB_ASHH_OD1, 
  AMB_ASHH_OD2,
  AMB_ASHH_HD2,
  AMB_ASHH_C,
  AMB_ASHH_O,
  AMB_ASHH_OXT
} AMB_ASHH_atoms;


#define AMB_N_CYSH_SCH_ATOMS 4
typedef enum {
  AMB_CYSH_N,
  AMB_CYSH_H,
  AMB_CYSH_CA, 
  AMB_CYSH_HA, 
  AMB_CYSH_CB, 
  AMB_CYSH_2HB, 
  AMB_CYSH_3HB,
  AMB_CYSH_SG,
  AMB_CYSH_HG,
  AMB_CYSH_C,
  AMB_CYSH_O,
  AMB_CYSH_OXT
} AMB_CYSH_atoms;


#define AMB_N_GLNH_SCH_ATOMS 10
typedef enum {
  AMB_GLNH_N,
  AMB_GLNH_H,
  AMB_GLNH_CA, 
  AMB_GLNH_HA, 
  AMB_GLNH_CB, 
  AMB_GLNH_2HB, 
  AMB_GLNH_3HB, 
  AMB_GLNH_CG, 
  AMB_GLNH_2HG, 
  AMB_GLNH_3HG, 
  AMB_GLNH_CD, 
  AMB_GLNH_OE1, 
  AMB_GLNH_NE2,
  AMB_GLNH_1HE2,
  AMB_GLNH_2HE2,
  AMB_GLNH_C,
  AMB_GLNH_O,
  AMB_GLNH_OXT
} AMB_GLNH_atoms;


#define AMB_N_GLUH_SCH_ATOMS 8
typedef enum {
  AMB_GLUH_N,
  AMB_GLUH_H,
  AMB_GLUH_CA, 
  AMB_GLUH_HA, 
  AMB_GLUH_CB, 
  AMB_GLUH_2HB, 
  AMB_GLUH_3HB, 
  AMB_GLUH_CG, 
  AMB_GLUH_2HG, 
  AMB_GLUH_3HG, 
  AMB_GLUH_CD, 
  AMB_GLUH_OE1, 
  AMB_GLUH_OE2,
  AMB_GLUH_C,
  AMB_GLUH_O,
  AMB_GLUH_OXT
} AMB_GLUH_atoms;

#define AMB_N_GLHH_SCH_ATOMS 9
typedef enum {
 AMB_GLHH_N,
 AMB_GLHH_H,
 AMB_GLHH_CA, 
 AMB_GLHH_HA, 
 AMB_GLHH_CB, 
 AMB_GLHH_2HB, 
 AMB_GLHH_3HB, 
 AMB_GLHH_CG, 
 AMB_GLHH_2HG, 
 AMB_GLHH_3HG, 
 AMB_GLHH_CD, 
 AMB_GLHH_OE1, 
 AMB_GLHH_OE2,
 AMB_GLHH_HE2,
 AMB_GLHH_C,
 AMB_GLHH_O,
 AMB_GLHH_OXT
} AMB_GLHH_atoms;

#define AMB_N_HIPH_SCH_ATOMS 11
typedef enum {
  AMB_HIPH_N,
  AMB_HIPH_H,
  AMB_HIPH_CA,
  AMB_HIPH_HA,
  AMB_HIPH_CB,
  AMB_HIPH_2HB,
  AMB_HIPH_3HB,
  AMB_HIPH_CG,
  AMB_HIPH_ND1,
  AMB_HIPH_HD1,
  AMB_HIPH_CE1,
  AMB_HIPH_HE1,
  AMB_HIPH_NE2,
  AMB_HIPH_HE2,
  AMB_HIPH_CD2,
  AMB_HIPH_HD2, 
  AMB_HIPH_C,
  AMB_HIPH_O,
  AMB_HIPH_OXT
} AMB_HIPH_atoms;

#define AMB_N_HIEH_SCH_ATOMS 10
typedef enum {
  AMB_HIEH_N,
  AMB_HIEH_H,
  AMB_HIEH_CA,
  AMB_HIEH_HA,
  AMB_HIEH_CB,
  AMB_HIEH_2HB,
  AMB_HIEH_3HB,
  AMB_HIEH_CG,
  AMB_HIEH_ND1,
  AMB_HIEH_CE1,
  AMB_HIEH_HE1,
  AMB_HIEH_NE2,
  AMB_HIEH_HE2,
  AMB_HIEH_CD2,
  AMB_HIEH_HD2, 
  AMB_HIEH_C,
  AMB_HIEH_O,
  AMB_HIEH_OXT
} AMB_HIEH_atoms;

#define AMB_N_HIDH_SCH_ATOMS 10
typedef enum {
  AMB_HIDH_N,
  AMB_HIDH_H,
  AMB_HIDH_CA,
  AMB_HIDH_HA,
  AMB_HIDH_CB,
  AMB_HIDH_2HB,
  AMB_HIDH_3HB,
  AMB_HIDH_CG,
  AMB_HIDH_ND1,
  AMB_HIDH_HD1,
  AMB_HIDH_CE1,
  AMB_HIDH_HE1,
  AMB_HIDH_NE2,
  AMB_HIDH_CD2,
  AMB_HIDH_HD2, 
  AMB_HIDH_C,
  AMB_HIDH_O,
  AMB_HIDH_OXT
} AMB_HIDH_atoms;

#define AMB_N_ILEH_SCH_ATOMS 12
typedef enum {
  AMB_ILEH_N,
  AMB_ILEH_H, 
  AMB_ILEH_CA, 
  AMB_ILEH_HA, 
  AMB_ILEH_CB, 
  AMB_ILEH_HB, 
  AMB_ILEH_CG2, 
  AMB_ILEH_1HG2, 
  AMB_ILEH_2HG2, 
  AMB_ILEH_3HG2, 
  AMB_ILEH_CG1, 
  AMB_ILEH_2HG1, 
  AMB_ILEH_3HG1, 
  AMB_ILEH_CD1,
  AMB_ILEH_1HD1, 
  AMB_ILEH_2HD1, 
  AMB_ILEH_3HD1,
  AMB_ILEH_C,
  AMB_ILEH_O,
  AMB_ILEH_OXT
} AMB_ILEH_atoms;


#define AMB_N_LEUH_SCH_ATOMS 12
typedef enum {
  AMB_LEUH_N,
  AMB_LEUH_H, 
  AMB_LEUH_CA, 
  AMB_LEUH_HA, 
  AMB_LEUH_CB, 
  AMB_LEUH_1HB, 
  AMB_LEUH_2HB, 
  AMB_LEUH_CG, 
  AMB_LEUH_HG, 
  AMB_LEUH_CD1, 
  AMB_LEUH_1HD1, 
  AMB_LEUH_2HD1, 
  AMB_LEUH_3HD1, 
  AMB_LEUH_CD2,
  AMB_LEUH_1HD2, 
  AMB_LEUH_2HD2, 
  AMB_LEUH_3HD2,
  AMB_LEUH_C,
  AMB_LEUH_O,
  AMB_LEUH_OXT
} AMB_LEUH_atoms;


#define AMB_N_LYSH_SCH_ATOMS 15
typedef enum {
  AMB_LYSH_N,
  AMB_LYSH_H,
  AMB_LYSH_CA, 
  AMB_LYSH_HA, 
  AMB_LYSH_CB, 
  AMB_LYSH_2HB, 
  AMB_LYSH_3HB, 
  AMB_LYSH_CG, 
  AMB_LYSH_2HG, 
  AMB_LYSH_3HG, 
  AMB_LYSH_CD, 
  AMB_LYSH_2HD, 
  AMB_LYSH_3HD,
  AMB_LYSH_CE, 
  AMB_LYSH_2HE, 
  AMB_LYSH_3HE, 
  AMB_LYSH_NZ,
  AMB_LYSH_1HZ,
  AMB_LYSH_2HZ,
  AMB_LYSH_3HZ,
  AMB_LYSH_C,
  AMB_LYSH_O,
  AMB_LYSH_OXT
} AMB_LYSH_atoms;

#define AMB_N_LYNH_SCH_ATOMS 14
typedef enum {
  AMB_LYNH_N,
  AMB_LYNH_H,
  AMB_LYNH_CA, 
  AMB_LYNH_HA, 
  AMB_LYNH_CB, 
  AMB_LYNH_2HB, 
  AMB_LYNH_3HB, 
  AMB_LYNH_CG, 
  AMB_LYNH_2HG, 
  AMB_LYNH_3HG, 
  AMB_LYNH_CD, 
  AMB_LYNH_2HD, 
  AMB_LYNH_3HD,
  AMB_LYNH_CE, 
  AMB_LYNH_2HE, 
  AMB_LYNH_3HE, 
  AMB_LYNH_NZ,
  AMB_LYNH_1HZ,
  AMB_LYNH_2HZ,
  AMB_LYNH_C,
  AMB_LYNH_O,
  AMB_LYNH_OXT
} AMB_LYNH_atoms;


#define AMB_N_METH_SCH_ATOMS 10
typedef enum {
  AMB_METH_N,
  AMB_METH_H,
  AMB_METH_CA, 
  AMB_METH_HA, 
  AMB_METH_CB, 
  AMB_METH_2HB, 
  AMB_METH_3HB, 
  AMB_METH_CG, 
  AMB_METH_2HG, 
  AMB_METH_3HG, 
  AMB_METH_SD, 
  AMB_METH_CE, 
  AMB_METH_1HE, 
  AMB_METH_2HE, 
  AMB_METH_3HE, 
  AMB_METH_C,
  AMB_METH_O,
  AMB_METH_OXT
} AMB_METH_atoms;


#define AMB_N_PHEH_SCH_ATOMS 13
typedef enum {
  AMB_PHEH_N,
  AMB_PHEH_H,
  AMB_PHEH_CA, 
  AMB_PHEH_HA, 
  AMB_PHEH_CB, 
  AMB_PHEH_2HB, 
  AMB_PHEH_3HB, 
  AMB_PHEH_CG, 
  AMB_PHEH_CD1, 
  AMB_PHEH_HD1, 
  AMB_PHEH_CE1, 
  AMB_PHEH_HE1, 
  AMB_PHEH_CZ, 
  AMB_PHEH_HZ, 
  AMB_PHEH_CE2, 
  AMB_PHEH_HE2, 
  AMB_PHEH_CD2, 
  AMB_PHEH_HD2,
  AMB_PHEH_C,
  AMB_PHEH_O,
  AMB_PHEH_OXT
} AMB_PHEH_atoms;


#define AMB_N_SERH_SCH_ATOMS 4
typedef enum {
  AMB_SERH_N,
  AMB_SERH_H,
  AMB_SERH_CA, 
  AMB_SERH_HA, 
  AMB_SERH_CB, 
  AMB_SERH_2HB, 
  AMB_SERH_3HB, 
  AMB_SERH_OG,
  AMB_SERH_HG,
  AMB_SERH_C,
  AMB_SERH_O,
  AMB_SERH_OXT
} AMB_SERH_atoms;


#define AMB_N_THRH_SCH_ATOMS 7
typedef enum {
  AMB_THRH_N,
  AMB_THRH_H,
  AMB_THRH_CA, 
  AMB_THRH_HA, 
  AMB_THRH_CB, 
  AMB_THRH_HB, 
  AMB_THRH_CG2, 
  AMB_THRH_1HG2, 
  AMB_THRH_2HG2, 
  AMB_THRH_3HG2,
  AMB_THRH_OG1, 
  AMB_THRH_HG1,
  AMB_THRH_C,
  AMB_THRH_O,
  AMB_THRH_OXT
} AMB_THRH_atoms;


#define AMB_N_TRPH_SCH_ATOMS 17
typedef enum {
  AMB_TRPH_N,
  AMB_TRPH_H,
  AMB_TRPH_CA, 
  AMB_TRPH_HA, 
  AMB_TRPH_CB, 
  AMB_TRPH_2HB, 
  AMB_TRPH_3HB, 
  AMB_TRPH_CG, 
  AMB_TRPH_CD1, 
  AMB_TRPH_HD1, 
  AMB_TRPH_NE1, 
  AMB_TRPH_HE1,
  AMB_TRPH_CE2, 
  AMB_TRPH_CZ2, 
  AMB_TRPH_HZ2, 
  AMB_TRPH_CH2, 
  AMB_TRPH_HH2,
  AMB_TRPH_CZ3, 
  AMB_TRPH_HZ3, 
  AMB_TRPH_CE3, 
  AMB_TRPH_HE3,
  AMB_TRPH_CD2, 
  AMB_TRPH_C,
  AMB_TRPH_O,
  AMB_TRPH_OXT
} AMB_TRPH_atoms;


#define AMB_N_TYRH_SCH_ATOMS 14
typedef enum {
  AMB_TYRH_N,
  AMB_TYRH_H,
  AMB_TYRH_CA, 
  AMB_TYRH_HA, 
  AMB_TYRH_CB, 
  AMB_TYRH_2HB, 
  AMB_TYRH_3HB, 
  AMB_TYRH_CG, 
  AMB_TYRH_CD1, 
  AMB_TYRH_HD1, 
  AMB_TYRH_CE1, 
  AMB_TYRH_HE1, 
  AMB_TYRH_CZ, 
  AMB_TYRH_OH,
  AMB_TYRH_HH,
  AMB_TYRH_CE2, 
  AMB_TYRH_HE2, 
  AMB_TYRH_CD2, 
  AMB_TYRH_HD2, 
  AMB_TYRH_C,
  AMB_TYRH_O,
  AMB_TYRH_OXT
} AMB_TYRH_atoms;


#define AMB_N_VALH_SCH_ATOMS 9
typedef enum {
  AMB_VALH_N,
  AMB_VALH_H,
  AMB_VALH_CA, 
  AMB_VALH_HA, 
  AMB_VALH_CB, 
  AMB_VALH_HB,
  AMB_VALH_CG1, 
  AMB_VALH_1HG1, 
  AMB_VALH_2HG1, 
  AMB_VALH_3HG1, 
  AMB_VALH_CG2, 
  AMB_VALH_1HG2, 
  AMB_VALH_2HG2, 
  AMB_VALH_3HG2, 
  AMB_VALH_C,
  AMB_VALH_O,
  AMB_VALH_OXT
} AMB_VALH_atoms;

#endif
