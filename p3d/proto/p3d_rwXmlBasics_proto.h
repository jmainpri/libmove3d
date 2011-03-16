/*
 *  p3d_rwXmlBasics_proto.h
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 14/03/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef P3D_RW_XML_BASICS_H
#define P3D_RW_XML_BASICS_H

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

void writeXmlMatrix(xmlNodePtr parent,p3d_matrix4 m);
void writeXmlIkSol(xmlNodePtr parent, p3d_rob* rob, int* iksol );
void writeXmlConfig(xmlNodePtr parent, p3d_rob* rob, configPt q );
xmlNodePtr writeXmlLocalpath(xmlNodePtr parent, p3d_rob* rob, p3d_localpath * localpath );

#endif