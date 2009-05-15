#ifndef __CEXTRACT__

/**
 * p3d_SetDistConfigChoice 
 * Set the current choice of metric used to compute 
 * the distance between
 * two configurations   
 * @param[In] DistConfChoice: the selected choice of 
 * metric to compute the distance.
 */
void p3d_SetDistConfigChoice(int DistConfChoice);

/**
 * p3d_GetDisConfigChoice 
 * Get the current metric used to compute 
 * the distance between two configurations   
 * @return:  the current metric used to compute 
 * the distance.
 */
int p3d_GetDistConfigChoice(void);

/**
 * SelectedDistConfig
 * Compute the distance between two robot configurations using the
 * current metric defined by  p3d_SetDisConfigChoice
 * @param[in] robotPt: a pointer to the current robot
 * @param[in] Config1: the first configuration
 * @param[in] Config2: the second configuration
 * @return: the distance between the two configurations,
 * and -1. if the computation of the ditances failed
 */
double SelectedDistConfig(p3d_rob* robotPt, configPt Config1,
			  configPt Config2);

#endif
