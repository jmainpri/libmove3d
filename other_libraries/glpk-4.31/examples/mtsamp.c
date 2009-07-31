/* mtsamp.c */

/*----------------------------------------------------------------------
-- This is an example program that illustrates how to use GLPK API in
-- multi-threaded application on W32 platform (VC++ 6.0).
--
-- To run this program use the following command:
--
--    mtsamp.exe mps-file-1 mps-file-2 ... mps-file-n
--
-- Each mps file specified in the command line is processed by separate
-- thread.
----------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <glpk.h>

DWORD APIENTRY solve_mps(VOID *arg)
{     char *fname = arg;
      LPX *lp;
      lp = lpx_read_mps(fname);
      if (lp == NULL)
      {  fprintf(stderr, "Cannot read mps file `%s'\n", fname);
         return 1;
      }
      lpx_simplex(lp);
      lpx_delete_prob(lp);
      return 0;
}

#define MAX_THREADS 20

int main(int argc, char *argv[])
{     HANDLE h[1+MAX_THREADS];
      DWORD foo;
      int t;
      if (argc < 2)
      {  fprintf(stderr, "At least one mps file must be specified\n");
         exit(EXIT_FAILURE);
      }
      if (argc-1 > MAX_THREADS)
      {  fprintf(stderr, "Too many mps files\n");
         exit(EXIT_FAILURE);
      }
      for (t = 1; t < argc; t++)
      {  h[t] = CreateThread(NULL, 0, solve_mps, argv[t], 0, &foo);
         if (h[t] == NULL)
         {  fprintf(stderr, "Unable to create thread for `%s'\n",
               argv[t]);
            exit(EXIT_FAILURE);
         }
      }
      WaitForMultipleObjects(argc-1, &h[1], TRUE, INFINITE);
      fprintf(stderr, "GLPK multi-threaded DLL seems to work\n");
      return 0;
}

/* eof */
