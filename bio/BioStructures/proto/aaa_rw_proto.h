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


#ifndef __CEXTRACT__

extern AAA_residue_data* create_AAA_Residue(int resSeq, const char *resName, int art_bkb, int art_sch);
extern void set_Bkb_Art(AAA_residue_data* AAA_Residue, int art);
extern void set_Sch_Art(AAA_residue_data* AAA_Residue, int art);
extern void aaa_toggle_Bkb_Art(AAA_residue_data *AAA_Residue);
extern void aaa_toggle_Sch_Art(AAA_residue_data *AAA_Residue);
extern AAA_residue_data_list* copy_AAA_residue_data_list(AAA_residue_data_list* src_AAA_List);
extern AAA_protein_data_list* copy_AAA_protein_data_list(AAA_protein_data_list* src_AAA_List);
extern int update_AAA_residue_data_list(AAA_residue_data_list* src_AAA_List, AAA_residue_data_list* AAA_List_to_update);
extern int update_AAA_protein_data_list(AAA_protein_data_list* src_AAA_protein_list, AAA_protein_data_list* AAA_protein_list_to_update);
extern AAA_residue_data_list* AAA_read_psf_protein_struct(psf_protein *proteinPt);
extern AAA_protein_data_list* AAA_read_psf_struct(psf_protein** list, int nproteins);
extern int read_Art(AAA_residue_data_list* AAA_List, int num, int* bkb, int* sch);
extern void free_AAA_List(AAA_residue_data_list *AAA_List);
extern void free_AAA_protein_List(AAA_protein_data_list *AAA_protein_list);
extern int get_AAA_index_by_protein_Name(AAA_protein_data_list* AAA_protein_list, const char* name, int* index);
extern int read_aaa_desc_from_file(const char* fullname, AAA_residue_data_list* list);
extern int save_aaa_desc_in_file(const char* fullname, AAA_residue_data_list* aaa_list);
#endif /* __CEXTRACT__ */
