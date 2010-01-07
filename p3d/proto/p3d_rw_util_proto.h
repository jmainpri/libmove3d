/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.

 */
#ifndef __CEXTRACT__

extern int p3d_read_line ( FILE * f, char ** line, int size, int * num_line );
extern int p3d_read_line_next_function ( FILE * f, char ** line, int size, int * num_line );
extern int p3d_read_string_name ( char ** curpos, char ** name );
extern int p3d_increase_size_double ( double ** tab, int size, int final_size );
extern int p3d_read_string_line_double ( char ** curpos, int *n, double **dtab, int *size_max_dtab );
extern int p3d_read_string_line_string ( char ** curpos, char ** strval );
extern configPt p3d_read_word_and_config ( p3d_rob *robotPt, char *line, char *keyword, double version );
extern int p3d_read_word_and_n_int(p3d_rob *robotPt, char *line, char *keyword, int** itab, int nint);
extern int p3d_read_word_and_double ( char *line, char *keyword, double *result );
extern int p3d_read_string_double ( char ** curpos, int n, double *dtab );
extern int p3d_read_string_n_double ( char ** curpos, int n, double ** dtab, int * size_max_dtab );
extern int p3d_increase_size_int ( int ** tab, int size, int final_size );
extern int p3d_read_string_line_int ( char ** curpos, int *n, int **itab, int *size_max_itab );
extern int p3d_read_string_int ( char ** curpos, int n, int *itab );
extern int p3d_read_string_n_int ( char ** curpos, int n, int ** itab, int * size_max_itab );
extern int p3d_read_string_boolean ( char ** curpos, int n, int *btab );
extern int p3d_read_string_mat ( char ** curpos, p3d_matrix4 mat );
extern int p3d_read_word_and_int ( char *line, char *keyword, int *result );
extern void p3d_write_word_and_n_double ( FILE *file, char *keyword, double * dtab, int n );
extern void p3d_write_word_and_n_int(FILE *file, char *keyword,int * itab, int n);
extern int p3d_write_word_and_config ( p3d_rob *robotPt, FILE *file, char *keyword, configPt q );
extern char *p3d_copy_line ( char *in, char *out, int size_in );

#endif /* __CEXTRACT__ */
