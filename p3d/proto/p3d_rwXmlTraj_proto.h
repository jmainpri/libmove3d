/*
 *  p3d_rwXmlTraj_proto.h
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 15/03/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef P3D_RW_XML_TRAJ_H
#define P3D_RW_XML_TRAJ_H

int p3d_writeXmlTraj(const char* file, p3d_traj* traj);
p3d_traj* p3d_readXmlTraj(const char* file);

#endif