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
/***************************************************************

SUBJECT:  time functions

WHEN      WHO       WHAT
19/03/2001    N MOUTON    portage NT

FCT:
  ChronoOn ( void );
  ChronoOff ( void );
  ChronoTimes ( double *tu, double *ts );
  ChronoPrint ( char *msg );
  ChronoPrinter ( int flag );
  ChronoGet ( void );

***************************************************************/

#include "Util-pkg.h"

#if defined UNIX
#include <sys/time.h>
#include <sys/times.h>
#include <sys/resource.h>
#include <limits.h>
#include <unistd.h>
#elif defined WIN32
#include <time.h>
#include <limits.h>
//#include <unistd.h>
#elif defined VXWORKS
#include <vxWorks.h>
#include <stdlib.h>
#include "h2timeLib.h"
#else
#error Platform definition needed, like UNIX, WIN32 etc...
#endif

#if !defined(TRUE) || (TRUE!=1)
#define     TRUE            1
#endif
///////////////////////////////////////////////////////////////////////////////////////////////
#if defined UNIX
// code UNIX
//
#define NCHRONO 20

static clock_t  beg_s[NCHRONO], beg_u[NCHRONO];
static struct timeval begm_s[NCHRONO], begm_u[NCHRONO];
static int     counter = -1;
static int     print_flag   = TRUE;

// Get Time Of Day Chrono
static double t_init;

double ChronoGetTime(bool is_first_call)
{
  timeval tim;
  gettimeofday(&tim, NULL);
  double tu=tim.tv_sec+(tim.tv_usec/1000000.0);
  
  if (is_first_call) {
    t_init = tu;
  }
  
  return tu - t_init;
}

//
int ChronoOn ( void )
{
  struct tms debut;
  struct rusage ru;

  if ( ++counter >= NCHRONO )
  {
    fprintf ( stderr, "ChronoOn : Warning: Chrono stack overflow\n" );
    return ( --counter );
  }

  getrusage(RUSAGE_SELF, &ru);
  begm_u[counter] = ru.ru_utime;
  begm_s[counter] = ru.ru_stime;

  times ( &debut );
  beg_u[counter] = debut.tms_utime;
  beg_s[counter] = debut.tms_stime;

  return ( counter );
}
//
int ChronoOff ( void )
{
  if ( counter == -1 )
  {
    fprintf ( stderr, "ChronoOff : Warning: Chrono stack underflow\n" );
    return ( -1 );
  }
  return ( counter-- );
}

int ChronoTimes ( double *tu, double *ts )
{
  struct tms fin;
  long clk_ticks;

  clk_ticks = sysconf ( _SC_CLK_TCK );

  if ( ( counter == -1 ) || ( counter >= NCHRONO ) )
  {
    *tu = 0.0;
    *ts = 0.0;
    return ( -1 );
  }

  times ( &fin );

  *tu  = ( double ) ( fin.tms_utime - beg_u[counter] ) / clk_ticks;
  *ts  = ( double ) ( fin.tms_utime - beg_s[counter] ) / clk_ticks;

  return ( counter );
}

int ChronoMicroTimes ( double *tu, double *ts )
{
  struct rusage ru;
  double finu, fins;
  if ( ( counter == -1 ) || ( counter >= NCHRONO ) )
  {
    *tu = 0.0;
    *ts = 0.0;
    return ( -1 );
  }
  getrusage(RUSAGE_SELF, &ru);
  finu = (double)ru.ru_utime.tv_sec + (double)ru.ru_utime.tv_usec / 1000000.0;
  fins = (double)ru.ru_stime.tv_sec + (double)ru.ru_stime.tv_usec / 1000000.0;
  *tu  = finu - ((double)begm_u[counter].tv_sec + (double)begm_u[counter].tv_usec / 1000000.0);
  *ts  = fins - ((double)begm_s[counter].tv_sec + (double)begm_s[counter].tv_usec / 1000000.0);

  return ( counter );
}

void ChronoPrint ( const char *msg )
{
  double tu,ts;
  int    i;
  if ( print_flag )
  {
    ChronoTimes ( &tu,&ts );
    /*
    fprintf(stdout,"Clock[%2d]: Utime=%6.3f sec , Stime=%6.3f sec : %s\n",
        counter,tu,ts,msg);
        */
    fprintf ( stdout,"Clock = %6.3f sec : ",tu );
    //calcul en microsecondes
    ChronoMicroTimes ( &tu, &ts );
    fprintf ( stdout,"Clock user = %f sec ",tu);

    for ( i=0;i<counter;i++ ) fprintf ( stdout,"  " );
    fprintf ( stdout,"[%d] %s\n",counter,msg );
  }
}

void ChronoPrinter ( int flag )
{
  print_flag = flag;
}

unsigned long  ChronoGet ( void )
{
  unsigned long msec;
  struct tms debut;

  times ( &debut );
  msec = debut.tms_utime;
  return ( msec );
}

///////////////////////////////////////////////////////////////////////////////////////////////

static struct timeval beg_tod[NCHRONO];
static int counter_tod = -1;
static int print_flag_tod = -1;
//
int ChronoTimeOfDayOn ( void )
{
  if ( ++counter_tod >= NCHRONO )
  {
    fprintf ( stderr, "ChronoTimeOfDayOn : Warning: Chrono stack overflow\n" );
    return ( --counter_tod );
  }
  
  gettimeofday(&beg_tod[counter_tod],NULL);
  return ( counter_tod );
}


int ChronoTimeOfDayOff ( void )
{
  if ( counter_tod == -1 )
  {
    fprintf ( stderr, "ChronoTimeOfDayOff : Warning: Chrono stack underflow\n" );
    return ( -1 );
  }
  return ( counter_tod-- );
}

int ChronoTimeOfDayTimes ( double *tu, double *ts )
{
  if ( ( counter_tod == -1 ) || ( counter_tod >= NCHRONO ) )
  {
    *tu = 0.0;
    *ts = 0.0;
    return ( -1 );
  }
  
  timeval tim;
  gettimeofday(&tim,NULL);
  
  *tu  = double( tim.tv_usec - beg_tod[counter_tod].tv_usec );
  *ts  = double( tim.tv_sec  - beg_tod[counter_tod].tv_sec );
  
  return ( counter );
}

int ChronoTimeOfDayTimes ( double *ts )
{
  if ( ( counter_tod == -1 ) || ( counter_tod >= NCHRONO ) )
  {
    *ts = 0.0;
    return ( -1 );
  }
  
  timeval tim;
  gettimeofday(&tim,NULL);
  
  double tu;
  
  tu  = double( tim.tv_usec - beg_tod[counter_tod].tv_usec );
  *ts = double( tim.tv_sec  - beg_tod[counter_tod].tv_sec ); 
  
  *ts = *ts + tu/1000000.0;
  
  return ( counter );
}

void ChronoTimeOfDayPrint ( const char *msg )
{
  double ts;
  int    i;
  if ( print_flag )
  {
    for ( i=0;i<counter_tod;i++ ) 
    {
      ChronoTimeOfDayTimes ( &ts );
      fprintf ( stdout,"Clock = %6.3f sec : ", ts );
      fprintf ( stdout,"  " );
      fprintf ( stdout,"[%d] %s\n", counter_tod, msg );
    }
    
  }
}

void ChronoTimeOfDayPrinter ( int flag )
{
  print_flag_tod = flag;
}

///////////////////////////////////////////////////////////////////////////////////////////////
#elif defined WIN32
// code WIN32
//
#include <sys/types.h>
#include <sys/timeb.h>

#define NCHRONO 20

//void _ftime( struct _timeb *timeptr );

static double beg_s[NCHRONO], beg_u[NCHRONO];
static int     counter = -1;
static int     print_flag   = TRUE;


int ChronoOn ( void )
{

  struct _timeb debut;

  if ( ++counter >= NCHRONO )
  {
#ifdef _DEBUG
    fprintf ( stderr, "ChronoOn : Warning: Chrono stack overflow\n" );
#endif
    return ( --counter );
  }

  _ftime ( &debut );

  beg_u[counter] = 0.;
  beg_s[counter] = 0.;

  beg_u[counter] = debut.time + debut.millitm /1000.;

  return ( counter );

}

//
int ChronoOff ( void )
{
  if ( counter == -1 )
  {
#ifdef _DEBUG
    fprintf ( stderr, "ChronoOff : Warning: Chrono stack underflow\n" );
#endif
    return ( -1 );
  }
  return ( counter-- );
}

//
int ChronoTimes ( double *tu, double *ts )
/*
use tu in second, ts allways = 0
*/
{
  struct _timeb fin;


  if ( ( counter == -1 ) || ( counter >= NCHRONO ) )
  {
    *tu = 0.0;
    *ts = 0.0;
    return ( -1 );
  }

  _ftime ( &fin );

  *tu  = fin.time + fin.millitm / 1000. - beg_u[counter];
  *ts  = 0;

  return ( counter );
}

//
void ChronoPrint ( char *msg )
{
#ifdef _DEBUG
  double tu,ts;
  int    i;
  if ( print_flag )
  {
    ChronoTimes ( &tu,&ts );
    // fprintf(stdout,"Clock[%2d]: Utime=%6.3f sec : %s\n", counter, tu, msg);
    PrintInfo ( ( "Clock = %6.3f sec : ",tu ) );
    for ( i=0;i<counter;i++ )
      PrintInfo ( ( "  " ) );
    PrintInfo ( ( "[%d] %s\n",counter,msg ) );
  }
#endif
}

//
void ChronoPrinter ( int flag )
{
#ifdef _DEBUG
  print_flag = flag;
#endif
}

//
unsigned long  ChronoGet ( void )
{
#ifdef _DEBUG
  unsigned long msec;
  struct _timeb debut;

  _ftime ( &debut );

  msec = ( debut.time * 1000 ) + debut.millitm;

  return ( msec );
#else
return 0;
#endif
}

////////////////////////
#elif defined VXWORKS
// code VXWORKS
//
#define NCHRONO 20

static unsigned long beg_s[NCHRONO], beg_u[NCHRONO];
static int     counter = -1;
static int     print_flag   = TRUE;

static unsigned long get_time_milli ( void )
{
  unsigned long msec;
  H2TIME time_str;

  if ( h2timeGet ( &time_str ) != OK )
    perror ( "h2timeget" );

  msec = time_str.msec + ( 60 * ( ( 60 * ( long ) time_str.hour ) + ( long ) time_str.minute ) + time_str.sec ) * 1000;
  return ( msec );
}
//
int ChronoOn ( void )
{
  if ( ++counter >= NCHRONO )
  {
    fprintf ( stderr, "ChronoOn : Warning: Chrono stack overflow\n" );
    return ( --counter );
  }

  beg_u[counter] = get_time_milli();
  beg_s[counter] = 0;
  return ( counter );
}
//
int ChronoOff ( void )
{
  if ( counter == -1 )
  {
    fprintf ( stderr, "ChronoOff : Warning: Chrono stack underflow\n" );
    return ( -1 );
  }
  return ( counter-- );
}
//
int ChronoTimes ( double *tu, double *ts )
{
  unsigned long msec;
  msec = get_time_milli();

  if ( ( counter == -1 ) || ( counter >= NCHRONO ) )
  {
    *tu = 0.0;
    *tu = 0.0;
    return ( -1 );
  }

  *tu  = ( double ) ( msec - beg_u[counter] ) / 1000;
  *ts  = 0.0;

  return ( counter );
}
//
void ChronoPrint ( char *msg )
{
  double tu,ts;
  int    i;
  if ( print_flag )
  {
    ChronoTimes ( &tu,&ts );
    /*
    fprintf(stdout,"Clock[%2d]: Utime=%6.3f sec , Stime=%6.3f sec : %s\n",
    counter,tu,ts,msg);
    */
    fprintf ( stdout,"Clock = %6.3f sec : ",tu );
    for ( i=0;i<counter;i++ ) fprintf ( stdout,"  " );
    fprintf ( stdout,"[%d] %s\n",counter,msg );
  }
}
//
void ChronoPrinter ( int flag )
{
  print_flag = flag;
}
//
unsigned long  ChronoGet ( void )
{
  unsigned long msec;

  msec =  get_time_milli();
  return ( msec );
}
//////////////////////

#else
#error Platform definition needed like UNIX, WIN32 etc...
#endif
