/****************************************************************************/
/*!			bcd_init.h
 *
 *  \brief bio structures
 *
 ****************************************************************************/
 
#if !defined (BCD_INIT)
#define BCD_INIT

#include "P3d-pkg.h"

void my_create_molecule(pp3d_rob rob, int nrobot, int njoint, double scale);

struct bcd_init_structure *pass_init_variables(void);

#endif