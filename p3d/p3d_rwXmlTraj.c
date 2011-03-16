/*
 *  p3d_rwXmlTraj.c
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 14/03/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */

#include "P3d-pkg.h"

/*------------------------------------------------------------*/
/* Save a trajectory file,
 return FALSE in case of failure */
int writeXmlTraj(xmlNodePtr trajNode, p3d_traj * trajPt) 
{
  pp3d_rob robotPt;
  pp3d_localpath localpathPt;
  time_t t = time(NULL);
  
  robotPt = trajPt->rob;
  localpathPt = trajPt->courbePt;
  
  xmlNewProp(trajNode, xmlCharStrdup("p3d_env"), xmlCharStrdup(p3d_get_desc_curname(P3D_ENV)));
  xmlNewProp(trajNode, xmlCharStrdup("p3d_rob"), xmlCharStrdup(robotPt->name));
  xmlNewProp(trajNode, xmlCharStrdup("p3d_traj"), xmlCharStrdup(trajPt->name));

  while (localpathPt != NULL) {
    writeXmlLocalpath( trajNode , robotPt, localpathPt);
    localpathPt = localpathPt->next_lp;
  }
}

/*------------------------------------------------------------*/
/* Save a trajectory file,
 return FALSE in case of failure */
int p3d_writeXmlTraj(const char *file, p3d_traj * traj) 
{
  xmlDocPtr doc = NULL;
  xmlNodePtr root = NULL;
  
  //Creating the file Variable version 1.0
  doc = xmlNewDoc(xmlCharStrdup("1.0"));
  
  //Writing the initial node
  root = xmlNewNode (NULL, xmlCharStrdup("move3dTraj"));
  
  // Start traj
  xmlDocSetRootElement(doc, root);
  
  writeXmlTraj(root, traj);
  
  //Writing the file on HD
  xmlSaveFormatFile (file, doc, 1);
  xmlFreeDoc(doc);
  
  return true;
}

/*------------------------------------------------------------*/
/* Read a trajectory from a file,
 return FALSE in case of failure */
p3d_traj* p3d_readXmlTraj(const char* file)
{
  
}
