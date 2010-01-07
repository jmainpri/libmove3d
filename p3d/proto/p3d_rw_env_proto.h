/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.
 *
 *   Created: Wed Jun 23 14:30:04 2004
 */
#ifndef __CEXTRACT__

extern int p3d_read_desc ( char *file );
extern void p3d_set_directory ( char *dir );
extern void p3d_get_directory ( char *dir );
extern void p3d_get_filename ( char *fullname );
extern void p3d_set_filename ( char *fullname );
extern void p3d_convert_axe_to_mat ( p3d_matrix4 M, double * tabd );
extern void p3d_convert_dh_to_mat (p3d_matrix4 M, double * tabd, p3d_jnt * prevJnt);
extern int read_desc (FILE *fd, char* nameobj, double scale, int fileType);
extern int p3d_read_macro ( char *namemac, char *nameobj, double scale );
//extern int read_macro ( FILE *fd, char *nameobj, double scale );
extern int read_macro_ground(FILE *fd,char *nameobj, double scale );
extern int read_desc_type ( FILE *fd, int *type );
extern int read_desc_hyp_type ( FILE *fd, int *n, int *type );
extern int read_desc_name ( FILE *fd, char *name );
extern int read_desc_string ( FILE *fd, char *name );
extern int read_desc_double ( FILE *fd, int n, double *f );
extern int read_desc_line_double ( FILE *fd, int *n, double *dtab );
extern int read_desc_int ( FILE *fd, int n, int *itab );
extern int read_desc_line_int ( FILE *fd, int *n, int *itab );
extern int read_desc_mat ( FILE *fd, p3d_matrix4 mat );
extern int read_desc_mat_scaled ( FILE *fd, p3d_matrix4 mat, double scale );
extern int read_desc_error ( char *msg );

#endif /* __CEXTRACT__ */
