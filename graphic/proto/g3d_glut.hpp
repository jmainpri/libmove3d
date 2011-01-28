/*
 *  glutWindow.hpp
 *  OOMove3D
 *
 *  Created by Jim Mainprice on 25/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef GLUT_DISPLAY_WIN_HPP
#define GLUT_DISPLAY_WIN_HPP

void g3d_glut_paintGL();

//! Class that displays a Glut window
//! this class displays a Glut based window of the scene
class GlutWindowDisplay 
{
public:
	GlutWindowDisplay(int argc, char *argv[]);
	void initDisplay();
};

#endif
