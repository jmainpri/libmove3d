#include "Util-pkg.h"
#include "P3d-pkg.h"

#ifdef P3D_LOCALPATH
#include "Localpath-pkg.h"
#endif

/*----------------------------------------------------------------------*/
/* Read a line and put the result in line, allocate memory if line is   */
/* larger than size, return the size of line or 0 if no data left       */
#define INIT_SIZE_CHAR 100
#define INC_SIZE_CHAR 30

#define INIT_SIZE_LINE_DATA 20
#define INC_SIZE_LINE_DATA 10


/*--------------------------------------------------------------------------*/
/*! \brief Funtion to read a line in a file
 *
 *  \note It supports DOS format and avoid space at the end of the string.
 *  \note It supports multi-line with "\".
 *  \note It increases the size of \a line if necessary and free
 *        it at the end of the file.
 *
 *  \param  f:    the file.
 *  \param  size: the allocated size of the string.
 *  
 *  \retval line:     the string that store the line.
 *  \retval num_line: the new line number.
 *
 *  \return the new allocated size of line (0 if there is no line left).
 *
 *  \internal
 */
int p3d_read_line(FILE * f, char ** line, int size, int * num_line)
{
  int i, len_str, test;

  if (size == 0) {
    *line = MY_ALLOC(char, INIT_SIZE_CHAR); 
    if (*line == NULL) {
      PrintError(("p3d_read_line: read_line: can't allocate memory\n"));
      return(0);
    }
    size = INIT_SIZE_CHAR;
  }
  (*line)[size-3] = '\0';
  if (fgets(*line, size-1, f) == NULL) { /* size-1 => let enough space to */
    MY_FREE(*line, char, size);          /* add '\n' at the end of *line  */
    *line = NULL;
    return(0);
  }
  do {
    test = FALSE;
    len_str = strlen(*line);
    /* Check for multi_line */
    if ((len_str>0) && ((*line)[len_str-1] == '\n')) {
      for(i=0; (i<len_str)&&((*line)[i]!='\n')&&((*line)[i]!='\\'); i++);
      if ((i<len_str) && ((*line)[i]=='\\')) {
	len_str = i;
	(*num_line) ++;
	test = TRUE;
      }
    }
    /* Increase the size of line if needed */
    if (((len_str >= size-2) && ((*line)[size-3] != '\n')) || 
	((len_str >= size-INC_SIZE_CHAR) && (test))) {
      (*line) = MY_REALLOC((*line), char ,size, size+INC_SIZE_CHAR);
      size += INC_SIZE_CHAR;
      test = TRUE;
      if ((*line) == NULL) {
	PrintError(("p3d_read_line: read_line: can't allocate memory\n"));
	return(0);
      }
    }
    if (test)
      { fgets((*line)+len_str, size-len_str-1, f); }    
  } while(test);

  /* avoid space at the end of a line */
  for(i=strlen(*line)-1; (i>=0)&&(((*line)[i]=='\n')||((*line)[i]=='\t') ||
				  ((*line)[i]==' ')||((*line)[i]=='\r')); i--);
  (*line)[i+1]='\0';
  
  return(size);
}

/*----------------------------------------------------------------------*/
/* Same function than before, but give the next line with a function,   */
/* remove comment, void line and '\n' char.                             */
/*  Add the number of line readed to num_line.                          */
int p3d_read_line_next_function(FILE * f, char ** line, int size, 
				int * num_line)
{
  int i, len;
  int test = TRUE;

  do {
    size = p3d_read_line(f, line, size, num_line);
    if (size) {
      (*num_line) ++;
      len = strlen(*line);

      /* Remove comment */
      for(i=0; (i<len) && ((*line)[i]!= '#'); i++);
      if ((*line)[i]=='#') { /* Remove previous spaces */
	for(i--; (i>=0)&&(((*line)[i]=='\t')||((*line)[i]==' ')); i--);
	(*line)[i+1] = '\0';
	test = (i < 0);
      } else {
	test = (len <= 0);
      }
    }
  } while((size != 0) && test);
  return size;
}

/*------------------------------------------------------------*/
/* Return a string on the first name of curpos, and change 
   curpos to point on the next name.
   Return FALSE if curpos is a void string. */

int p3d_read_string_name(char ** curpos, char ** name)
{
  char * pos;
  
  for(*name=*curpos;(**name!='\0')&&((**name==' ')||(**name=='\t'));(*name)++);
  if (**name == '\0') /* No name found */
    { return FALSE; }
  for(pos=*name; (*pos!='\0')&&(*pos!=' ')&&(*pos!='\t'); pos++);
  if (*pos != '\0') {
    *pos = '\0';
    pos ++;
  }
  *curpos = pos;
  return TRUE;
}


/*----------------------------------------------------------------------*/
/* Increase the size of an array od double */
/* return the size or 0 in case of failure */
int p3d_increase_size_double(double ** tab, int size, int final_size)
{
  double * tmp;

  if (size>=final_size) { 
    return size; 
  }
  
  tmp = MY_ALLOC(double, final_size);
  if (tmp == NULL) {
    if (size>0) {
      MY_FREE(*tab, double, size);
      *tab = NULL;
    }
    PrintError(("Cannot allocate memory\n"));
    return(0);
  }

  if (size != 0) {
    memcpy(tmp, *tab, size*sizeof(double));
    MY_FREE(*tab, double, size);
  }
  *tab = tmp;
  return final_size;
}

/*
 *  p3d_read_string_line_double --
 *
 * Return an array of double of size size_max_dtab with n items, and
 * change curpos to point on the next name.  Return FALSE if it
 * fails. 
 *
 *  ARGS IN  : a pointer on the string to read
 *             
 *  ARGS OUT : n the number of double precision numbers read,
 *             the number of double precision number allocated in dtab
 *             an array containing the number read.
*/
int p3d_read_string_line_double(char ** curpos, int *n, double **dtab,
				int *size_max_dtab)
{
  char * pos, * name;
  int test;

  *n = 0;
  *size_max_dtab = p3d_increase_size_double(dtab,*size_max_dtab, 
					    INIT_SIZE_LINE_DATA);
  if (*size_max_dtab == 0)
    { return FALSE; }
  pos = *curpos;
  do {
    if (*n>=*size_max_dtab) {
      *size_max_dtab = p3d_increase_size_double(dtab, *size_max_dtab, 
				    *size_max_dtab + INC_SIZE_LINE_DATA);
      if (*size_max_dtab == 0) 
	{ return FALSE; }
    }
    test = p3d_read_string_name(&pos, &name);
    if (test) {
      test = (sscanf(name, "%lf", &((*dtab)[*n]))==1);
      if (test) {
	(*n) ++; 
	*curpos = pos;
      }
    }
  } while(test);
  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Read a string between "" or a word. 
 *
 *  \note It avoids space before and after the string.
 *
 *  \param  curpos: the current position in the line.
 *
 *  \retval curpos: the new current position in the line (the end).
 *  \retval strval: the begining of the string.
 *
 *  \return TRUE if the string is not void.
 */
int p3d_read_string_line_string(char ** curpos, char ** strval)
{
  char * pos;

  pos = *curpos;
  while((pos[0]==' ') || (pos[0]=='\t') || (pos[0]=='\n') || (pos[0]=='\r'))
    { pos ++; }
  (*strval) = pos;
  if (pos[0]=='\0') {
    (*curpos) = pos;
    return FALSE;
  }
  if (pos[0]=='\"') {
    pos ++;
    (*strval) = pos;
    while((pos[0]!='\"') && (pos[0]!='\0'))
      { pos++; }
    if (pos[0] == '\"') {
      pos[0] = '\0';
      pos++;
    }
    (*curpos) = pos;
  } else {
    while((pos[0]!=' ') && (pos[0]!='\t') && (pos[0]!='\n') &&
	  (pos[0]!='\r') && (pos[0]!='\0'))
      { pos ++; }
    if (pos[0] != '\0') {
      pos[0] = '\0';
      pos++;
    }
    (*curpos) = pos;
  }
  return TRUE;
}


/* 
 *  p3d_read_word_and_config --
 *
 *  read in a string a keyword and the parameters of a configuration
 *
 *  ARGS IN  : a robot (for the dimension of the configuration)
 *             a string
 *             a keyword to read at the beginning of the string
 *             a version id to know how to read the config
 *
 *  ARGS OUT : a configuration if success,
 *             NULL otherwise
 */

configPt p3d_read_word_and_config(p3d_rob *robotPt, char *line, 
				  char *keyword, double version)
{
  configPt q=NULL;
  char *pos = line, *name=NULL;
  int success = TRUE, n=0;
  int i;
  int dimq = robotPt->nb_dof;
  static int size_max_dtab=0;
  static double *dtab=NULL;

  // PrintInfo(("p3d_read_word_and_config: version = %f, dimq = %i\n",version,dimq));

  if (p3d_read_string_name(&pos, &name) != TRUE) {
    success = FALSE;
  }

  if (success) {
    if (strcmp(name, keyword) != 0) {
      success=FALSE;
    }
  }
  
  /* read initial configuration */
  if (success) {
    if (p3d_read_string_line_double(&pos, &n, &dtab, 
				    &size_max_dtab) != TRUE) {
      success=FALSE;
    }
  }

  if (version == 0) {
    dimq = robotPt->njoints+4;
    // PrintInfo(("p3d_read_word_and_config: version == 0, dimq = %i\n",dimq));
  }
  /* check size of configuration */
  if (success) {
    if (n != dimq) {
      if (n == dimq - 2)
	{
	  dimq = robotPt->njoints+4;
	  version = 0;
	  PrintInfo(("version 0, dimq adapted to %i\n",dimq));
	}
      else
	{
	  PrintError(("dimq != n and dimq - 2 != n"));
	  success=FALSE;
	}
    }
  }
  /* copy numbers in initial configuration */
  if (success) {
    q = p3d_alloc_config(robotPt);

    if (version > 0) {
      for (i=0; i<n; i++) {
	q[i] = dtab[i];
      }
    }
    else {
      for (i=0; i<3; i++) {
	q[i] = dtab[i];
      }
      q[3] = 0;
      q[4] = 0;
      for (i=3; i<dimq; i++) {
	q[i+2] = dtab[i];
      }
    }
  }

  return q;
}

/* 
 *  p3d_read_word_and_n_int --
 *
 *  read in a string a keyword and the parameters of a configuration
 *
 *  ARGS IN  : a robot (for the dimension of the configuration)
 *             a string
 *             a keyword to read at the beginning of the string
 *             a version id to know how to read the config
 *
 *  ARGS OUT : a configuration if success,
 *             NULL otherwise
 */

int p3d_read_word_and_n_int(p3d_rob *robotPt, char *line, char *keyword, int** itab, int nint)
{
  char *pos = line, *name=NULL;
  int success = TRUE;
  int nint_max = nint;

  *itab = MY_ALLOC(int,nint);
  if (p3d_read_string_name(&pos, &name) != TRUE) {
    success = FALSE;
  }

  if (success) {
    if (strcmp(name, keyword) != 0) {
      success=FALSE;
    }
  }
  
  /* read initial configuration */
  if (success) {
    if (p3d_read_string_n_int(&pos, nint, itab, &nint_max) != TRUE) {
      success=FALSE;
    }
  }
  /* check size of itab */
  if (success) {
    if (nint_max > nint){
      PrintError(("nint != %d",nint));
      success=FALSE;
    }
  }
  /* copy numbers in initial configuration */
  if (!success) {
    MY_FREE(*itab,int,nint);
    return FALSE;
  }

  return TRUE;
}


/* 
 *  p3d_read_word_and_double --
 *
 *  read in a string a keyword and  a double precision number
 *
 *  ARGS IN  : a string
 *             a keyword to read at the beginning of the string
 *
 *  ARGS OUT : a double
 *
 *  RETURN   : TRUE if success, FALSE if failure
 */

int p3d_read_word_and_double(char *line, char *keyword, double *result)
{
  char *pos = line, *name=NULL;
  int success = TRUE, n=0;
  static int size_max_dtab=0;
  static double *dtab=NULL;

  if (p3d_read_string_name(&pos, &name) != TRUE) {
    success = FALSE;
  }

  if (success) {
    if (strcmp(name, keyword) != 0) {
      success=FALSE;
    }
  }
  
  /* read initial configuration */
  if (success) {
    if (p3d_read_string_line_double(&pos, &n, &dtab, 
				    &size_max_dtab) != TRUE) {
      success=FALSE;
    }
  }
  /* check size of configuration */
  if (success) {
    if (n != 1) {
      success=FALSE;
    }
    else {
      *result = dtab[0];
    }
  }

/*   if (size_max_dtab>0) { */
/*     MY_FREE(dtab, double, size_max_dtab); */
/*     dtab = NULL; */
/*   } */

  return success;
}


/*--------------------------------------------------------------------------*/
/*! \brief Return an array of double of fixed size.
 *
 * Return an array of double of size \a n, and
 * change \a curpos to point on the next name.  Return FALSE if it
 * fails.
 *
 * \param  curpos:        current position in the string
 * \param  n:             number of double that must be read
 *
 * \retval curpos:        the new current position in the string
 * \retval dtab:          the array of double
 *
 * \return TRUE if it is possible to read \a n double
 *
 * \warning The size of \a dtab must be more than \a n,
 *          if the size of \a dtab must change, use p3d_read_string_n_double()
 */
int p3d_read_string_double(char ** curpos, int n, double *dtab)
{
  char * pos, * name;
  int test, i;

  pos = *curpos;
  for(i=0; i<n; i++) {
    test = p3d_read_string_name(&pos, &name);
    if (test) {
      test = (sscanf(name, "%lf", &(dtab[i]))==1);
    }
    if (!test)
      { return FALSE; }
  }
  *curpos = pos;
  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Return an array of double of fixed size and allocate
 *         memory if needed.
 *
 * Return an array of double of size \a n, and
 * change \a curpos to point on the next name.  Return FALSE if it
 * fails. If \a dtab is not big enough, resize it.
 *
 * \param  curpos:        current position in the string
 * \param  n:             number of double that must be read
 * \param  size_max_dtab: the current size of \a dtab
 *
 * \retval curpos:        the new current position in the string
 * \retval dtab:          the array of double
 * \retval size_max_dtab: the new size of \a dtab
 *
 * \return TRUE if it is possible to read \a n double
 */
int p3d_read_string_n_double(char ** curpos, int n, double ** dtab,
			     int * size_max_dtab)
{
  if (n>0) {
    if (n>*size_max_dtab) {
      *size_max_dtab = p3d_increase_size_double(dtab,*size_max_dtab, n); 
      if (*size_max_dtab<=0)
	{ return FALSE; }
    }
    return p3d_read_string_double(curpos, n, *dtab);
  }
  return TRUE;
}


/*----------------------------------------------------------------------*/
/* Increase the size of an array of integer */
/* return the size or 0 in case of failure  */
int p3d_increase_size_int(int ** tab, int size, int final_size)
{
  int * tmp;

  if (size>=final_size) { 
    return size; 
  }
  
  tmp = MY_ALLOC(int, final_size);
  if (tmp == NULL) {
    if (size>0) {
      MY_FREE(*tab, int, size);
      *tab = NULL;
    }
    PrintError(("Cannot allocate memory\n"));
    return(0);
  }

  if (size != 0) {
    memcpy(tmp, *tab, size*sizeof(int));
    MY_FREE(*tab, int, size);
  }
  *tab = tmp;
  return final_size;
}


/*--------------------------------------------------------------------------*/
/*! \brief Return an array of integer of length variable and allocate
 *         memory if needed.
 *
 * Return an array of integer, its size \a n, and
 * change \a curpos to point on the next name.  Return FALSE if it
 * fails. If \a itab is not big enough, resize it.
 *
 * \param  curpos:        current position in the string
 * \param  size_max_itab: the current size of \a itab
 *
 * \retval curpos:        the new current position in the string
 * \param  n:             number of integer that have been read
 * \retval itab:          the array of integer
 * \retval size_max_itab: the new size of \a itab
 *
 * \return TRUE if it there is no memory error
 */
int p3d_read_string_line_int(char ** curpos, int *n, int **itab,
			     int *size_max_itab)
{
  char * pos, * name;
  int test;

  *n = 0;
  *size_max_itab = p3d_increase_size_int(itab, *size_max_itab, 
					 INIT_SIZE_LINE_DATA);
  if (*size_max_itab == 0)
    { return FALSE; }
  pos = *curpos;
  do {
    if (*n>=*size_max_itab) {
      *size_max_itab = p3d_increase_size_int(itab, *size_max_itab, 
				    *size_max_itab + INC_SIZE_LINE_DATA);
      if (*size_max_itab == 0) 
	{ return FALSE; }
    }
    test = p3d_read_string_name(&pos, &name);
    if (test) {
      test = (sscanf(name, "%i", &((*itab)[*n]))==1);
      if (test) {
	(*n) ++; 
	*curpos = pos;
      }
    }
  } while(test);
  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Return an array of int of fixed size.
 *
 * Return an array of int of size \a n, and
 * change \a curpos to point on the next name.  Return FALSE if it
 * fails.
 *
 * \param  curpos:        current position in the string
 * \param  n:             number of int that must be read
 *
 * \retval curpos:        the new current position in the string
 * \retval itab:          the array of int
 *
 * \return TRUE if it is possible to read \a n int
 *
 * \warning The size of \a itab must be more than \a n,
 *          if the size of \a itab must change, use p3d_read_string_n_int()
 */
int p3d_read_string_int(char ** curpos, int n, int *itab)
{
  char * pos, * name;
  int test, i;

  pos = *curpos;
  for(i=0; i<n; i++) {
    test = p3d_read_string_name(&pos, &name);
    if (test) {
      test = (sscanf(name, "%i", &(itab[i]))==1);
    }
    if (!test)
      { return FALSE; }
  }
  *curpos = pos;
  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Return an array of integer of fixed size and allocate
 *         memory if needed.
 *
 * Return an array of integer of size \a n, and
 * change \a curpos to point on the next name.  Return FALSE if it
 * fails. If \a itab is not big enough, resize it.
 *
 * \param  curpos:        current position in the string
 * \param  n:             number of integer that must be read
 * \param  size_max_itab: the current size of \a itab
 *
 * \retval curpos:        the new current position in the string
 * \retval itab:          the array of integer
 * \retval size_max_itab: the new size of \a itab
 *
 * \return TRUE if it is possible to read \a n integer
 */
int p3d_read_string_n_int(char ** curpos, int n, int ** itab,
			  int * size_max_itab)
{
  if (n>0) {
    if (n>*size_max_itab) {
      *size_max_itab = p3d_increase_size_int(itab,*size_max_itab, n);
      if (*size_max_itab == 0)
	{ return FALSE; }
    }
    return p3d_read_string_int(curpos, n, *itab);
  }
  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Return an array of boolean of fixed size.
 *
 * Return an array of boolean of size \a n, and
 * change \a curpos to point on the next name.  Return FALSE if it
 * fails.
 *
 * \param  curpos:        current position in the string
 * \param  n:             number of boolean that must be read
 *
 * \retval curpos:        the new current position in the string
 * \retval btab:          the array of boolean
 *
 * \return TRUE if it is possible to read \a n boolean
 */
int p3d_read_string_boolean(char ** curpos, int n, int *btab)
{
  char * pos, * name;
  int test, i;

  pos = *curpos;
  for(i=0; i<n; i++) {
    test = p3d_read_string_name(&pos, &name);
    if (test) {
      if ((strcmp(name, "T")==0) || (strcmp(name, "t")==0) ||
	  (strcmp(name, "1")==0) || (strcmp(name, "True")==0) ||
	  (strcmp(name, "true")==0) || (strcmp(name, "TRUE")==0))
	{ btab[i] = TRUE; }
      else if ((strcmp(name, "F")==0) || (strcmp(name, "f")==0) ||
	       (strcmp(name, "0")==0) || (strcmp(name, "False")==0) ||
	       (strcmp(name, "false")==0) || (strcmp(name, "FALSE")==0))
	{ btab[i] = FALSE; }
      else
	{ test = FALSE; }
    }
    if (!test)
      { return FALSE; }
  }
  *curpos = pos;
  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Return an matrix 4x4
 *
 * Return an matrix 4x4 of double, and
 * change \a curpos to point on the next name.  Return FALSE if it
 * fails.
 *
 * \param  curpos:        current position in the string
 *
 * \retval curpos:        the new current position in the string
 * \retval mat:           the matrix
 *
 * \return TRUE if it is possible to read the matrix (16 double)
 */
int p3d_read_string_mat(char ** curpos, p3d_matrix4 mat)
{
  char * pos, * name;
  int test, i, j;

  pos = *curpos;
  for(i=0; i<4; i++) {
    for(j=0; j<4; j++) {
      test = p3d_read_string_name(&pos, &name);
      if (test) {
	test = (sscanf(name, "%lf", &(mat[i][j]))==1);
      }
      if (!test)
	{ return FALSE; }
    }
  }
  *curpos = pos;
  return TRUE;
}


/* 
 *  p3d_read_word_and_int --
 *
 *  read in a string a keyword and  an integer
 *
 *  ARGS IN  : a string
 *             a keyword to read at the beginning of the string
 *
 *  ARGS OUT : an integer
 *
 *  RETURN   : TRUE if success, FALSE if failure
 */

int p3d_read_word_and_int(char *line, char *keyword, int *result)
{
  char *pos = line, *name=NULL;
  int success = TRUE;

  if (p3d_read_string_name(&pos, &name) != TRUE) {
    success = FALSE;
  }

  if (success) {
    if (strcmp(name, keyword) != 0) {
      success=FALSE;
    }
  }
  
  /* read initial configuration */
  if (success) {
    if (sscanf(pos, "%d", result) == 0) {
      success=FALSE;
    }
  }

  return success;
}



/*--------------------------------------------------------------------------*/ 
/*! \brief Write a word and n float values
 *
 *  \note This function write this on sevral lines
 *
 *  \param  file:    a file descriptor
 *  \param  keyword: the keyword to write at the beginning of the string
 *  \param  dtab:    an array of values
 *  \param  n:       the number of values
 */
void p3d_write_word_and_n_double(FILE *file, char *keyword,
				 double * dtab, int n)
{
  int i, k;
  #define NB_COLUMN 4

  fprintf(file, "%s", keyword);
  k = 0;
  while(k<n) {
    fprintf(file, "\\\n");
    for(i=0; (i<NB_COLUMN) && (k<n); i++) {
      fprintf(file, "\t%f", dtab[k]);
      k ++;
    }
  }
  fprintf(file, "\n");
}

/*--------------------------------------------------------------------------*/ 
/*! \brief Write a word and n int values
 *
 *  \note This function write this on sevral lines
 *
 *  \param  file:    a file descriptor
 *  \param  keyword: the keyword to write at the beginning of the string
 *  \param  dtab:    an array of values
 *  \param  n:       the number of values
 */
void p3d_write_word_and_n_int(FILE *file, char *keyword,
         int * itab, int n)
{
  int i, k;

  fprintf(file, "%s", keyword);
  k = 0;
  while(k<n) {
    fprintf(file, "\\\n");
    for(i=0; (i<NB_COLUMN) && (k<n); i++) {
      fprintf(file, "\t%d", itab[k]);
      k ++;
    }
  }
  fprintf(file, "\n");
}

/* 
 *  p3d_write_word_and_config --
 *
 *  write in a file a keyword and the parameters of a configuration
 *
 *  ARGS IN  : a robot (for the dimension of the configuration)
 *             a file descriptor
 *             a keyword to write at the beginning of the string
 *             a configuration
 *
 *  ARGS OUT : TRUE if success
 *             NULL otherwise
 *
 *  Configuration is written in old format: base of robot is supposed
 *  to have only 4 degrees of freedom.
 */

int p3d_write_word_and_config(p3d_rob *robotPt, FILE *file, 
			      char *keyword, configPt q)
{
  p3d_write_word_and_n_double(file, keyword, q, robotPt->nb_dof);

  return TRUE;
}



/*
 *  p3d_copy_line --
 *
 *  copy a string after allocating space if needed. return a pointer on 
 *  new string
 *
 */

char *p3d_copy_line(char *in, char *out, int size_in)
{
  out = (char*)realloc(out, size_in);
  strcpy(out, in);
  return out;
}
