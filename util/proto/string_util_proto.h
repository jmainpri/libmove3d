/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.

 */
#ifndef __CEXTRACT__

// strdup is not a standard option, it can be avalaible or not depending on compilers/options
#ifndef __cplusplus
extern char *strdup(const char *str);
#endif
extern int fixed_str_copy ( char * dest, const char * src, int * indice, int size );
extern char * dyna_str_dup ( const char * data );
extern void dyna_str_free ( char * data );

#endif /* __CEXTRACT__ */
