#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"

/**
 * Note: the integer values of the different
 * EXPANSION_DIRECTION_METHODs are defined in the
 * p3d_type.h file
 */
static int EXPANSION_DIRECTION_METHOD = GLOBAL_CS_EXP;

/**
 * Value of the bias toward the goal componant
 * if the expansion process is biased.
 */
static double GOAL_BIAS = 0.1;

/**
 * Flag TRUE if the expansion process
 * is biased toward a  goal configuration
 */
static int IS_GOAL_BIAS = FALSE;


/**
 * Flag to say if a sampling direction must
 * validate the Rlg constraints or not
 */
static int IsDirSampleWithRlg = FALSE;

/**
 * Get the value of the bias toward the goal componant
 * if the expansion process is biased.
 * @return: the value of the bias toward the goal componant
 */
double p3d_GetGoalBiasValue(void){
  return GOAL_BIAS;
}

/**
 * Set the value of the bias toward the goal componant
 * if the expansion process is biased.
 * @param[In]: the value of the bias toward the goal componant
 */
void p3d_SetGoalBiasValue(double GoalBias) {
  GOAL_BIAS = GoalBias;
}

/**
 * p3d_GetIsGoalBias
 * Get if the expansion is biased toward the goal
 * return: TRUE if there is a bias toward the goal
 */
int p3d_GetIsGoalBias(void) {
  return IS_GOAL_BIAS;
}

/**
 * p3d_SetIsGoalBias
 * Set if the expansion is biased toward the goal
 * @param[In] IsGoalBias: TRUE if there is a bias toward the goal
 */
void p3d_SetIsGoalBias(int IsGoalBias) {
  IS_GOAL_BIAS = IsGoalBias;
}

/**
 * p3d_SetExpansionDirectionMethod
 * Set the current value of the method used to
 * choose the direction of expansion
 * @param[in] the expansion direction method
*/
void p3d_SetExpansionDirectionMethod(int ExpDirMethod)
{
  EXPANSION_DIRECTION_METHOD = ExpDirMethod;
}

/**
 * p3d_GetExpansionDirectionMethod
 * Get the  value of the current method used to
 * choose the direction of expansion
 * @return: the current expansion direction method
*/
int p3d_GetExpansionDirectionMethod(void)
{
  return EXPANSION_DIRECTION_METHOD;
}




/**
 * p3d_GetIsDirSampleWithRlg
 * Get if the  sampling direction must
 * validate the Rlg constraints or not
 * @return: TRUE if if the  sampling direction
 * must validate the Rlg constraints
 */
int p3d_GetIsDirSampleWithRlg(void) {
  return IsDirSampleWithRlg;
}

/**
 * p3d_GetIsDirSampleWithRlg
 * Set if the  sampling direction must
 * validate the Rlg constraints or not
 * @param[In] isDirSampleWithRlg: TRUE
 * if if the  sampling direction
 * must validate the Rlg constraints
 */
void p3d_SetIsDirSampleWithRlg(int isDirSampleWithRlg) {
  IsDirSampleWithRlg = isDirSampleWithRlg;
}
