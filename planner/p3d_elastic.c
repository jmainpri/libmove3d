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
/* ---------------------------------------------------------------------*/
/*!\file p3d_elastic.c
 * \brief elastic strips optimization
 * \ingroup planner
 *
 * \author E.Ferre
 * \date   Aug 2001
 *
 * Set of functions for elastic band optimization and tracjectory tracking
 *
 */
/* ---------------------------------------------------------------------*/

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Localpath-pkg.h"
#include "Planner-pkg.h"



/************************************************************************/
/*! \fn static void ReplaceStripPoint(p3d_rob *rob, p3d_strippoint *sp, configPt q, p3d_localpath *path, p3d_localpath *nextpath)
 *
 * \brief replace a strippoint in a strip
 *
 * \param rob      the robot
 * \param sp       the strippoint
 * \param q        the new configuration
 * \param path     the previous local path
 * \param nextpath the next local path

 * \warning q, path and nextpath are copied and need to be destroy latter
 * \sa DeleteStripPoint(),InsertStripPoint()
 */
/************************************************************************/

static void ReplaceStripPoint(p3d_rob *rob, p3d_strippoint *sp, configPt q, p3d_localpath *path, p3d_localpath *nextpath) {
  p3d_copy_config_into(rob, q, &(sp->q));
  if (sp->path)
    sp->path->destroy(rob, sp->path);
  if (path)
    sp->path = path->copy(rob, path);
  else
    sp->path = NULL;
  if (sp->next) {
    sp->next->path->destroy(rob, sp->next->path);
    sp->next->path = nextpath->copy(rob, nextpath);
  }
}

/************************************************************************/
/*! \fn static int InsertStripPoint(p3d_rob *rob, p3d_strippoint *sp, double k)
 * \brief insert a strippoint in a strip
 *
 * \param rob the robot
 * \param sp  the next strippoint
 * \param k   where the strippoint is inserted. must be inside [0, 1]
 *
 * \return TRUE if insertion is not possible
 *
 * \note the strippoint is inserted at k*range_param on the local path
 *       before sp
 *
 * \sa DeleteStripPoint(),ReplaceStripPoint()
 * \todo this function should not test if the strippoint is in collision
 *       because it must be collision free
 */
/************************************************************************/

static int InsertStripPoint(p3d_rob *rob, p3d_strippoint *sp, double k) {
  p3d_strippoint *newsp = MY_ALLOC(p3d_strippoint, 1);
  newsp->q = sp->path->config_at_param(rob, sp->path, (sp->path->range_param) * k);
  // test if strippoint is in collision
  // modif Juan
  if (!p3d_set_and_update_robot_conf(newsp->q)) {
    PrintInfo(("ERROR ! : inserted invalid strippoint !!\n"));
    p3d_destroy_config(rob, newsp->q);
    MY_FREE(newsp, p3d_strippoint, 1);
    return TRUE;
  }
  // p3d_set_and_update_robot_conf(newsp->q);

  if (p3d_col_test()) {
    PrintInfo(("ERROR ! : inserted strippoint in collision !!\n"));
    p3d_destroy_config(rob, newsp->q);
    MY_FREE(newsp, p3d_strippoint, 1);
    return TRUE;
  }
  // fmodif Juan
  newsp->Kes = k;
  newsp->path = p3d_local_planner(rob, sp->prev->q, newsp->q);
  if (!newsp->path) {
    PrintInfo(("ERROR ! : invalid path !!\n"));
    p3d_destroy_config(rob, newsp->q);
    MY_FREE(newsp, p3d_strippoint, 1);
    return TRUE;
  }
  sp->path->destroy(rob, sp->path);
  sp->path = p3d_local_planner(rob, newsp->q, sp->q);
  if (!sp->path) {
    PrintInfo(("ERROR ! : invalid path !!\n"));
    p3d_destroy_config(rob, newsp->q);
    newsp->path->destroy(rob, newsp->path);
    MY_FREE(newsp, p3d_strippoint, 1);
    return TRUE;
  }
  newsp->next = sp;
  newsp->prev = sp->prev;
  newsp->prev->next = newsp;
  sp->prev = newsp;

  if (newsp->prev->path)
    newsp->prev->Kes = (newsp->prev->path->range_param) / (newsp->prev->path->range_param + newsp->path->range_param);
  if (sp->next)
    sp->Kes = (sp->path->range_param) / (sp->path->range_param + sp->next->path->range_param);

  return FALSE;
}

/************************************************************************/
/*! \fn static void DeleteStripPoint(p3d_rob *rob, p3d_strippoint **sp, p3d_localpath *path)
 *
 * \brief delete a strippoint in a strip
 *
 * \param rob  the robot
 * \param sp   the strippoint
 * \param path the new local path between previous and next strippoints
 *
 * \note sp is returned with the value of the previous strippoint
 * \warning path is copied and need to be destroy latter
 * \sa DeleteStripPoint(),ReplaceStripPoint()
 */
/************************************************************************/

static void DeleteStripPoint(p3d_rob *rob, p3d_strippoint **sp, p3d_localpath *path) {
  p3d_strippoint *prevsp = (*sp)->prev;
  prevsp->next = (*sp)->next;
  prevsp->next->prev = prevsp;
  prevsp->next->path->destroy(rob, prevsp->next->path);
  prevsp->next->path = path->copy(rob, path);
  // Kes is updated
  if (prevsp->path)
    prevsp->Kes = (prevsp->path->range_param) / (prevsp->path->range_param + prevsp->next->path->range_param);
  if (prevsp->next->next)
    prevsp->next->Kes = (prevsp->next->path->range_param) / (prevsp->next->path->range_param + prevsp->next->next->path->range_param);
  p3d_destroy_config(rob, (*sp)->q);
  (*sp)->path->destroy(rob, (*sp)->path);
  MY_FREE(*sp, p3d_strippoint, 1);
  *sp = prevsp; // previous strippoint is returned
}

//-----------------------------------------------------



/************************************************************************/
/*! \fn static p3d_strip *CreateStrip(p3d_traj *traj)
 * \brief transform a trajectory into a strip
 *
 * \param traj the trajectory
 * \return pointer to a strip
 * \sa  CopyStripIntoTraj(),DeleteStrip()
 */
/************************************************************************/

static p3d_strip *CreateStrip(p3d_traj *traj) {
  int compteur = 0;
  p3d_localpath *path = NULL;
  p3d_strip *S = NULL;
  p3d_strippoint *SPoint = NULL;
  if (!traj)
    return NULL;
  path = traj->courbePt;
  S = MY_ALLOC(p3d_strip, 1);
  S->rob = traj->rob;
  SPoint = MY_ALLOC(p3d_strippoint, 1);
  compteur++;
  SPoint->q = path->config_at_param(S->rob, path, 0);
  SPoint->Kes = 0;
  SPoint->path = NULL;
  SPoint->prev = NULL;
  SPoint->next = MY_ALLOC(p3d_strippoint, 1);
  compteur++;
  SPoint->next->prev = SPoint;
  S->sp = SPoint;
  SPoint = SPoint->next;
  while (path->next_lp) {
    SPoint->q = path->config_at_param(S->rob, path, path->range_param);
    SPoint->Kes = path->range_param / (path->range_param + path->next_lp->range_param);
    SPoint->path = path->copy(S->rob, path);
    SPoint->next = MY_ALLOC(p3d_strippoint, 1);
    compteur++;
    SPoint->next->prev = SPoint;
    SPoint = SPoint->next;
    path = path->next_lp;
  }
  SPoint->q = path->config_at_param(S->rob, path, path->range_param);
  SPoint->Kes = 0;
  SPoint->path = path->copy(S->rob, path);
  SPoint->next = NULL;
  return S;
}

/************************************************************************/
/*! \fn static void CopyStripIntoTraj(p3d_strip *strip, p3d_traj *traj)
 * \brief transform a strip to a trajectory
 *
 * \param strip the strip
 * \param traj  the trajectory to modify
 *
 * \warning traj must already exist
 * \sa CreateStrip(),DeleteStrip()
 */
/************************************************************************/

static void CopyStripIntoTraj(p3d_strip *strip, p3d_traj *traj) {
  p3d_localpath *path = NULL;
  p3d_localpath *prevpath = NULL;
  p3d_strippoint *sp = NULL;

  if ((!traj) || (!strip))
    return;
  destroy_list_localpath(traj->rob, traj->courbePt);
  sp = strip->sp->next;
  traj->courbePt = sp->path->copy(traj->rob, sp->path);
  path = traj->courbePt;
  prevpath = NULL;
  while (sp->next) {
    path->prev_lp = prevpath;
    path->next_lp = sp->next->path->copy(traj->rob, sp->next->path);
    sp = sp->next;
    prevpath = path;
    path = path->next_lp;
  }
  path->prev_lp = prevpath;
  path->next_lp = NULL;
  traj->nlp = p3d_compute_traj_nloc(traj);
  traj->range_param = p3d_compute_traj_rangeparam(traj);
}

/************************************************************************/
/*! \fn static void DeleteStrip (p3d_strip *strip)
 * \brief destroy a strip
 *
 * \param strip the strip
 */
/************************************************************************/

static void DeleteStrip(p3d_strip *strip) {
  p3d_strippoint *sp = NULL, *nextsp = NULL;
  if (!strip)
    return;
  sp = strip->sp;
  nextsp = sp->next;
  p3d_destroy_config(strip->rob, sp->q);
  MY_FREE(sp, p3d_strippoint, 1);
  sp = nextsp;
  while (sp) {
    nextsp = sp->next;
    p3d_destroy_config(strip->rob, sp->q);
    if (sp->path)
      sp->path->destroy(strip->rob, sp->path);
    MY_FREE(sp, p3d_strippoint, 1);
    sp = nextsp;
  }
  MY_FREE(strip, p3d_strip, 1);
}

/************************************************************************/
/*! \fn static double ComputeRangeStrip(p3d_strip *strip)
 * \brief compute the parameter range of a strip
 *
 * \param strip the strip
 * \return  the parameter range
 */
/************************************************************************/

static double ComputeRangeStrip(p3d_strip *strip) {
  double range = 0;
  p3d_strippoint *sp = strip->sp->next;
  while (sp) {
    range += sp->path->range_param;
    sp = sp->next;
  }
  return range;
}

/************************************************************************/
/*! \fn static int clearStrip (p3d_strip *strip, double tend, int (*fct_stop)(void),int *ntest)
 * \brief delete useless strippoints in a strip
 *
 * \param strip the trajectory strip
 * \param tend     maximum time of the optimization (tend=0 means tend=infini)
 * \param fct_stop if fct_stop returns FALSE, optimization is stopped
 *
 * \retval ntest update the number of collision tests
 *
 * \return TRUE in case of user stop or time out
 *
 * a strippoint is useless iff the previous and next strippoints can be
 * linked by a collision free local path
 *
 */
/************************************************************************/
static int clearStrip(p3d_strip *strip, double tend, int (*fct_stop)(void), int *ntest) {
  p3d_localpath *newpath;// path between prev and next
  p3d_rob *rob = strip->rob;
  p3d_strippoint *sp = NULL; // current strippoint
  int stripchange = TRUE, stop = FALSE; // boolean to know if function can be stopped
  int  ndelpoint = 0;
  double tu = 0, ts = 0;

  while ((stripchange) && (!stop)) {
    stripchange = FALSE;
    sp = strip->sp->next;
    while ((sp->next) && (!stop)) {
      newpath = p3d_local_planner(rob, sp->prev->q, sp->next->q);
      //if (!p3d_col_test_localpath(rob,  newpath, ntest))
      if (!p3d_unvalid_localpath_test(rob,  newpath, ntest)) { // <- modif Juan
        DeleteStripPoint(rob, &sp, newpath);
        stripchange = TRUE;
        ndelpoint++;
      }
      sp = sp->next;
      newpath->destroy(rob, newpath);
      ChronoTimes(&tu, &ts);
      if ((tu > tend) && (tend > 0)) {
        stop = TRUE;
      }
      if (fct_stop) {
        if (!(*fct_stop)())
          stop = TRUE;
      }
    }
  }
  return stop;
}


static int elasticOptimization(){
 return TRUE;
}


/************************************************************************/
/*! \fn int p3d_trackCurrentTraj(p3d_rob *rob, int Nend, double epsilon, double d0,int Quick)
 * \brief deform current trajectory of a robot
 *
 * It the same algorithm than gradientDescentOptimize(), but it must be used
 * for a very short time (Nend=3) because there is no strip cleaning during the
 * optimization. the random optimization is called at the end to try to break
 * the strip.
 *
 * \param rob     the robot
 * \param Nend    maximum number of iterations
 * \param epsilon if the gain between two steps is less than epsilon,
 *                optimization is stopped
 * \param d0      contact distance of the obstacles. the strippoints should not
 *                be closer than d0.
 * \param Quick    if TRUE, quickdescent algorithm is applied as gradient descent
 *                 else projected gradient descent is applied.
 *
 * \warning current trajectory must be collision free
 *
 * \sa gradientDescentOptimize()
 */
/************************************************************************/

int p3d_trackCurrentTraj(p3d_rob *rob, int Nend, double epsilon, double d0, int Quick) {
  p3d_localpath *newpath, *newnextpath;
  int i, ntest = 0, spchange, stripchange, stop = FALSE, error = FALSE;
  double range1, range2, gain;
  p3d_traj *traj = rob->tcur;
  p3d_strip *strip = NULL;
  p3d_strippoint *sp = NULL;
  configPt q = NULL;

  if (!traj)
    return FALSE;

  //--------- insert the new current config -------------
  strip = CreateStrip(traj);
  sp = strip->sp;
  
  if (!(p3d_equal_config(rob, rob->ROBOT_POS, sp->q))) {
    //verifier si la ROBOT_POS est valide (contraites + collisions)
    if (!p3d_set_and_update_robot_conf(rob->ROBOT_POS)) {
      DeleteStrip(strip);
      return TRUE;
    }
    if (p3d_col_test()) {
      DeleteStrip(strip);
      return TRUE;
    }
    //si oui relier la config a la traj
    //on commence par tester si ROBOT_POS est connectable a la seconde config de la traj (derniere config du premier lp de la traj)
    newpath = p3d_local_planner(rob, rob->ROBOT_POS, sp->next->q);
    if (p3d_unvalid_localpath_test(rob,  newpath, &ntest)) {
      newpath->destroy(rob, newpath);
      newpath = p3d_local_planner(rob, rob->ROBOT_POS, sp->q);
      //si non, tester avec la premiere config de la traj
      if (p3d_unvalid_localpath_test(rob,  newpath, &ntest)) {
        //si non, revoyer qu'on ne trouve pas et qu'il faut planifier de nouveau
        newpath->destroy(rob, newpath);
        DeleteStrip(strip);
        return FALSE;
      }
      //si oui, creer un nouveau strip point qu'on relira au tout premier
      sp = MY_ALLOC(p3d_strippoint, 1);
      sp->q = p3d_copy_config(rob,  rob->ROBOT_POS);
      sp->path = NULL;
      sp->Kes = 0;
      sp->prev = NULL;
      sp->next = strip->sp;
      strip->sp = sp;
      sp = sp->next;
      sp->prev = strip->sp;
      sp->path = newpath->copy(rob, newpath);
      sp->Kes = sp->path->range_param / (sp->path->range_param + sp->next->path->range_param);
    } else{
      //si oui on remplace la config et le lp du premier strip point
      ReplaceStripPoint(rob, sp, rob->ROBOT_POS, NULL, newpath);
    }
    newpath->destroy(rob, newpath);
  }

  //--------- insert the new goal config -------------
  //aller au dernier strip point
  sp = strip->sp;
  while (sp->next)
    sp = sp->next;
  //verifier si la ROBOT_GOTO est valide (contraites + collisions)
  if (!(p3d_equal_config(rob, rob->ROBOT_GOTO, sp->q))) {
    if (!p3d_set_and_update_robot_conf(rob->ROBOT_GOTO) || (p3d_col_test())) {
      DeleteStrip(strip);
      return TRUE;
    }
    //si oui relier la config a la traj
    //on commence par tester si ROBOT_GOTO est connectable a l'avant derniere config de la traj (premiere config du dernier lp de la traj)
    newpath = p3d_local_planner(rob, sp->prev->q, rob->ROBOT_GOTO);
    if (p3d_unvalid_localpath_test(rob,  newpath, &ntest)) {
      newpath->destroy(rob, newpath);
      newpath = p3d_local_planner(rob, sp->q, rob->ROBOT_GOTO);
      //si non, tester avec la premiere config de la traj
      if (p3d_unvalid_localpath_test(rob,  newpath, &ntest)) {
        //si non, revoyer qu'on ne trouve pas et qu'il faut planifier de nouveau
        newpath->destroy(rob, newpath);
        DeleteStrip(strip);
        return FALSE;
      }
      //si oui, creer un nouveau strip point qu'on relira a tout dernier
      sp->next = MY_ALLOC(p3d_strippoint, 1);
      sp->next->q = p3d_copy_config(rob,  rob->ROBOT_GOTO);
      sp->next->path = newpath->copy(rob, newpath);
      sp->Kes = 0;
      sp->next->prev = sp;
      sp->next->next = NULL;
      sp->Kes = sp->path->range_param / (sp->path->range_param + sp->next->path->range_param);
    } else {
      //si oui on remplace la config et le lp du dernier strip point
      ReplaceStripPoint(rob, sp, rob->ROBOT_GOTO, newpath, NULL);
    }
    newpath->destroy(rob, newpath);
  }

  // strip initialization
  q = p3d_alloc_config(rob);
  stripchange = TRUE;
  range1 = ComputeRangeStrip(strip);
  i = 0;

  // -------------- gradient decrease -----------------
  //stop = TRUE;
  while (!stop) {
    i++;
    sp = strip->sp->next;
    stripchange = FALSE;
    while ((sp->next) && (!stop)) {
      // calculate the new strippoint configuration
      if (Quick)
        spchange = p3d_quickGradientDescent(rob, sp, d0, q, &ntest);
      else
        spchange = p3d_projectedGradientDescent(rob, sp, d0, q, &ntest);

      if (spchange) {
        newpath = p3d_local_planner(rob, sp->prev->q, q);
        newnextpath = p3d_local_planner(rob, q, sp->next->q);
        // insert strippoint if new paths are not valid
        //if (p3d_col_test_localpath(rob,  newpath, &ntest))
        if (p3d_unvalid_localpath_test(rob,  newpath, &ntest)) { // <- modif Juan
          error = InsertStripPoint(rob, sp, 0.5);
          stop = error;
          stripchange = TRUE;
        } else
          //if (p3d_col_test_localpath(rob, newnextpath, &ntest))
          if (p3d_unvalid_localpath_test(rob,  newnextpath, &ntest)) { // <- modif Juan
            error = InsertStripPoint(rob, sp->next, 0.5);
            stop = error;
            stripchange = TRUE;
          } else
            // modify strippoint
            ReplaceStripPoint(rob, sp, q, newpath, newnextpath);
        newpath->destroy(rob, newpath);
        newnextpath->destroy(rob, newnextpath);
      }
      sp = sp->next;
    }
    // optimization stop
    if (!stop) {
      range2 = range1;
      range1 = ComputeRangeStrip(strip);
      if (i > Nend)
        stop = TRUE;
      if ((!stripchange) && (!stop))
        stop = (fabs((range2 - range1) / range2) < epsilon);
      if ((stop) && (!error)) {
        clearStrip(strip, 0, NULL, &ntest);
        CopyStripIntoTraj(strip, traj);
      }
    }
  }
  // random optimization
  p3d_optim_traj(traj, &gain, &ntest);
  if (error)
    PrintInfo(("erreur dans TrajTrack\n"));
  p3d_destroy_config(rob, q);
  DeleteStrip(strip);
  return TRUE;
}

/************************************************************************/
/*! \fn void p3d_gradientDescentOptimize(p3d_traj *traj, double tend, double epsilon, double d0,int Quick, int (*fct_stop)(void), void (*fct_draw)(void))
 *
 * \brief optimize a trajectory with a gradient descent method
 *
 * \param traj     the trajectory
 * \param tend     maximum time of the optimization (tend=0 means tend=infini)
 * \param epsilon  if the gain between two interations if less than
 *                 epsilon, optimization is stopped
 * \param d0       contact distance of the obstacles. the strippoints should not
 *                 be closer than d0.
 * \param Quick    if TRUE, quickdescent algorithm is applied as gradient descent
 *                 else projected gradient descent is applied.
 * \param fct_stop if fct_stop returns FALSE, optimization is stopped
 * \param fct_draw function drawing the trajectory at the end of an iteration
 *
 * The trajectory is deformed by altering each of strippoints in turn.
 * To change a strippoint, a gradient descent method is applied. If one
 * of local path linking this strippoint is not valid, the strippoint is
 * not change and a strippoint is inserted on the old local path.
 *
 * \sa TrajTrack()
 */
/************************************************************************/

void p3d_gradientDescentOptimize(p3d_traj *traj, double tend, double epsilon, double d0, int Quick, int (*fct_stop)(void), void (*fct_draw)(void)) {
  p3d_localpath *newpath, *newnextpath;
  int i, ntest, spchange, stripchange, stop = FALSE, userstop = FALSE;
  int nc = 0;
  double init_range_param = p3d_compute_traj_rangeparam(traj);
  double range0, range1, range2;
  p3d_strip *strip = NULL;
  p3d_strippoint *sp = NULL;
  p3d_rob *rob = traj->rob;
  configPt q = p3d_alloc_config(rob);
  double tu = 0, ts = 0;
  ChronoOn();

  // initialization
  strip = CreateStrip(traj);
  ntest = 0;
  stripchange = TRUE;
  range1 = ComputeRangeStrip(strip);
  range0 = range1;

  // gradient descent
  i = 0;
  while (!stop) {
    // the strip is cleaned every 10 iterations
    if (!(i % 10)) {
      userstop = clearStrip(strip, tend - tu, fct_stop, &ntest);
      stop = userstop;
    }

    i++;
    sp = strip->sp->next;
    stripchange = FALSE;
    while ((sp->next) && (!stop)) {
      // beginning of an iteration on the strip

      // calculate the new strippoint configuration
      if (Quick)
        spchange = p3d_quickGradientDescent(rob, sp, d0, q, &ntest);
      else
        spchange = p3d_projectedGradientDescent(rob, sp, d0, q, &ntest);
      nc++;

      // if strippoint can be changed we update it
      if (spchange) {
        newpath = p3d_local_planner(rob, sp->prev->q, q);
        newnextpath = p3d_local_planner(rob, q, sp->next->q);

        // we insert strippoint if new paths are not valid
        //if (p3d_col_test_localpath(rob,  newpath, &ntest))
        if (p3d_unvalid_localpath_test(rob,  newpath, &ntest)) { // <- modif Juan
          stop = InsertStripPoint(rob, sp, 0.5);
          stripchange = TRUE;
        } else {
          //if (p3d_col_test_localpath(rob,  newnextpath, &ntest))
          if (p3d_unvalid_localpath_test(rob,  newnextpath, &ntest)) { // <- modif Juan
            stop = InsertStripPoint(rob, sp->next, 0.5);
            stripchange = TRUE;
          } else
            // update strippoint if new paths are valid
            ReplaceStripPoint(rob, sp, q, newpath, newnextpath);
        }
        newpath->destroy(rob, newpath);
        newnextpath->destroy(rob, newnextpath);
      }
      sp = sp->next;
      // test if we must stop
      ChronoTimes(&tu, &ts);
      if ((tu > tend) && (tend > 0)) {
        stop = TRUE;
        userstop = TRUE;
        PrintInfo(("          \nOptimization stopped : time out\n"));
      }
      if (fct_stop) {
        if (!(*fct_stop)()) {
          PrintInfo(("          \nOptimization canceled\n"));
          stop = TRUE;
          userstop = TRUE;
        }
      }
    }
    // end of an iteration on the strip

    // optimization stop
    range2 = range1;
    range1 = ComputeRangeStrip(strip);
    if ((!stripchange) && (!stop)) {
      stop = (fabs((range2 - range1) / range2) <= epsilon);

      if (stop) {
        /*fflush(stdout);*/
        PrintInfo(("          \noptimization process completed\n"));
      }
    }
    if (fct_draw) {
      CopyStripIntoTraj(strip, traj);
      (*fct_draw)();
    }
    PrintInfo(("    %7.4f %%          \r" , (range0 - range1)*100 / range0));
  }

  if (!userstop)
    clearStrip(strip, tend - tu, fct_stop, &ntest);

  // destroy struct
  p3d_destroy_config(rob, q);
  CopyStripIntoTraj(strip, traj);
  DeleteStrip(strip);
  p3d_set_and_update_robot_conf(rob->ROBOT_POS);

  PrintInfo(("gradient calculation    : %d\n", nc));
  PrintInfo(("collision test          : %d\n", ntest));
  PrintInfo(("nb strippoints           : %d\n", traj->nlp - 1));
  PrintInfo(("optimization efficiency : %f%%\n", (init_range_param - traj->range_param)*100 / init_range_param));
  ChronoPrint("");
  ChronoOff();
}

/************************************************************************/
/*! \fn void p3d_clearTraj(p3d_traj *traj,double tend,int (*fct_stop)(void))
 * \brief clear a trajectory by deleting useless strippoints
 *
 * \param traj the trajectory
 * \param tend     maximum time of the optimization (tend=0 means tend=infini)
 * \param fct_stop if fct_stop returns FALSE, optimization is stopped
 *
 */
/************************************************************************/

void p3d_clearTraj(p3d_traj *traj, double tend, int (*fct_stop)(void)) {
  int ntest = 0;
  double beforeCost = p3d_compute_traj_cost(traj);
  ChronoOn();
  p3d_strip *strip = CreateStrip(traj);
  
  clearStrip(strip, 0, NULL, &ntest);

  CopyStripIntoTraj(strip, traj);
  DeleteStrip(strip);
  PrintInfo(("Gain = %f \n", (1 - (p3d_compute_traj_cost(traj)/beforeCost)) * 100));
  ChronoPrint("Clear Trajectory");
  ChronoOff();
  PrintInfo(("Collision test : %d\n", ntest));
}
