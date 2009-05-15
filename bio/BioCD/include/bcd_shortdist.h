/****************************************************************************/
/*!		bcd_shortdist.h
 *
 *   
 *
 ****************************************************************************/
 
#if !defined (BCD_SHORTDIST)
#define BCD_SHORTDIST

void create_residual_SDs(int *autocolindex, int *intercolindex,int *loopindex);

void general_SD_function(SDpt SD);

/* developer */
void write_SD_function(SDpt SD);
#endif