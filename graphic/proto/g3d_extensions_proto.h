/***************************************************************************
 *   Copyright (C) 2007  Antony Martin                                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU General Public License           *
 *   as published by the Free Software Foundation; either version 2        *
 *   of the License, or (at your option) any later version.                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA          *
 ***************************************************************************/



#ifndef G3D_EXTENSIONS_H
#define G3D_EXTENSIONS_H


extern int g3d_is_extension_supported(const char *name);

extern int g3d_init_extensions();

extern char* g3d_load_source(const char *filename);

extern GLuint g3d_load_shader(GLenum type, const char *filename);

extern GLuint g3d_load_program(const char *vsname, const char *psname);

#endif 
