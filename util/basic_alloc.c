#include "Util-pkg.h"

double p3d_version = 1.1;  /* set here before create some specific file */

// Since size_t is unsigned, -1 represents the highest integer which can be stored in this type
#define NO_LIMIT (size_t) -1

static size_t basic_alloc_size         = 0;
static size_t basic_alloc_max_size     = NO_LIMIT;

/***************************************************************/

#define LIMIT_DEBUG 1000000

static bool DEBUG = false;
// Arrays to store pointers, their address size, and wether a given
// pointer has been freed or not
static void* tab_ptr[LIMIT_DEBUG];
static size_t tab_ptr_size[LIMIT_DEBUG];
static bool tab_ptr_allocated[LIMIT_DEBUG];
static int tab_ptr_num = 0;

void basic_alloc_debugon() {
  DEBUG = true;
}

void basic_alloc_debugoff() {
  if(DEBUG) report_whats_left();
  DEBUG = false;
}

void report_whats_left() {
  int i;
  for(i = 0; i < tab_ptr_num; i++) {
    if(tab_ptr_allocated[i]) {
	PrintInfo(("debug: memory leak: ptr %p (size %zu) %d\n", tab_ptr[i], tab_ptr_size[i], i));
    }
  }
}

bool debug_find_ptr(void* ptr, int* n) {
  int i;
  for(i = tab_ptr_num - 1; i>=0; i--) {
    if(tab_ptr[i] == ptr) {
      *n = i;
      return true;
    }
  }
  return false;
}

void alloc_debug(void *ptr, size_t size) {
  int n;

  if(debug_find_ptr(ptr, &n)) {
    if(tab_ptr_allocated[n]) {
      PrintInfo(("debug: alloc: WARNING - allocation of unfree pointer - ptr %p (size %zu) %d\n", ptr, size, n));
    }
    tab_ptr_size[n] = size;
    tab_ptr_allocated[n] = true;
    //PrintInfo(("debug: alloc: allocating memory - ptr %p (size %zu) %d\n", ptr, tab_ptr_size[n], n));
  }
  else {
    if (tab_ptr_num < LIMIT_DEBUG) {
      tab_ptr[tab_ptr_num] = ptr;
      tab_ptr_size[tab_ptr_num] = size;
      tab_ptr_allocated[tab_ptr_num] = true;
      //PrintInfo(("debug: alloc: allocating memory - ptr %p (size %zu) %d\n", ptr, tab_ptr_size[tab_ptr_num], tab_ptr_num));
      tab_ptr_num++;
    }
    else {
      PrintError(("debug: alloc: memory debug array too small\n"));
    }
  }
}

void realloc_debug(void *ptr, void *newptr, size_t oldsize, size_t size) {
  int n;

  if(ptr == NULL) {
    if (oldsize != 0) {
      PrintInfo(("debug: realloc: WARNING - realloc request for null pointer with non zero size: ptr %p (size %zu)\n", ptr, oldsize));
    }
  }
  else {
    if(debug_find_ptr(ptr, &n)) {
      if(tab_ptr_size[n] != oldsize)
	PrintInfo(("debug: realloc: WARNING - specified size does not match stored size: ptr %p (specified size %zu) (stored size %zu)\n", ptr, oldsize, tab_ptr_size[n]));
      tab_ptr_size[n] = 0;
      tab_ptr_allocated[n] = false;
    }
  }

  if(debug_find_ptr(newptr, &n)) {
    tab_ptr_size[n] = size;
    tab_ptr_allocated[n] = true;
    //PrintInfo(("debug: realloc: reallocating memory - ptr %p (size %zu) %d\n", ptr, size, n));
  }
  else {
    if (tab_ptr_num < LIMIT_DEBUG) {
      tab_ptr[tab_ptr_num] = newptr;
      tab_ptr_size[tab_ptr_num] = size;
      tab_ptr_allocated[tab_ptr_num] = true;
      //PrintInfo(("debug: realloc: reallocating memory - ptr %p (size %zu) %d\n", newptr, size, tab_ptr_num));
      tab_ptr_num++;
    }
    else {
      PrintError(("debug: realloc: memory debug array too small\n"));
    }
  }
}

void free_debug(void *ptr, size_t size) {
  int n;
  if(debug_find_ptr(ptr, &n)) {
    if(tab_ptr_size[n] != size) {
      PrintInfo(("debug: free: WARNING - specified size does not match stored size: ptr %p (specified size %zu) (stored size %zu)\n", ptr, size, tab_ptr_size[n]));
      return;
    }
    tab_ptr_size[n] = 0;
    tab_ptr_allocated[n] = false;
    //PrintInfo(("debug: free: freeing memory - ptr %p (size %zu) %d\n", ptr, size, n));
  }
  else {
    if ((ptr != NULL) || (size != 0)) {
      PrintInfo(("debug: free: WARNING - ptr %p (size %zu) NOT FOUND\n", ptr, size));
    }
  }
}

/**********************************************************************/

void* basic_alloc(unsigned long n, size_t size) {
  size_t s = size*n;
  void* ptr;

  if (s==0)
    return NULL;

  if (basic_alloc_size + s > basic_alloc_max_size) {
    PrintError(("MP: basic_alloc: overflow, size=%zu ...\n", basic_alloc_size + s));
    return NULL;
  }

  if ((ptr = malloc(s)) == NULL) {
    PrintError(("MP: basic_alloc: can't alloc %lu * %ld = %ld of extra memory, current size %ld \n", n, size, (unsigned)s, basic_alloc_size));
    return NULL;
  }
  else
    basic_alloc_size += s;

  if(DEBUG) alloc_debug(ptr, s);

  return(ptr);
}

/**********************************************************************/

void* basic_realloc(void *ptr, unsigned long nold, unsigned long nnew, size_t size) {
  size_t dif_s = size*(nnew-nold);
  size_t new_s = size*nnew;
  void *oldptr = ptr;

  if (new_s == 0) {
    basic_free(ptr, nold, size);
    return NULL;
  }

  if (basic_alloc_size + dif_s > basic_alloc_max_size) {
    PrintError(("MP: basic_realloc: overflow, size=%zu ...\n", basic_alloc_size + dif_s));
    return(NULL);
  }

  ptr = realloc(ptr, new_s);

  if (ptr == NULL) {
    PrintError(("MP: basic_alloc: can't alloc extra memory\n"));
  }
  else
    basic_alloc_size += dif_s;

  if(DEBUG) realloc_debug(oldptr, ptr, size*nold, new_s);
  return(ptr);
}

/**********************************************************************/

void basic_free(void *ptr, unsigned long n, size_t size) {
  size_t s = size * n;

  if (ptr != NULL) {
    basic_alloc_size -= s;
    free(ptr);
    ptr = NULL;
  }
  if(DEBUG) free_debug(ptr, s);
}

/**********************************************************************/

void basic_alloc_set_maxsize(size_t s) {
  if (s == 0) s = NO_LIMIT;
  basic_alloc_max_size = s;
}

/**********************************************************************/

size_t basic_alloc_get_size() {
  return(basic_alloc_size);
}

/**********************************************************************/

void basic_alloc_info(const char *str) {
    PrintInfo(("MP: basic_alloc %s : %zu bytes allocated\n", str, basic_alloc_size));
}

/**********************************************************************/
