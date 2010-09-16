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




#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <GL/glew.h>

#include "../graphic/proto/g3d_extensions_proto.h"



int g3d_is_extension_supported(const char *name)
{
//     const GLubyte *extensions = glGetString(GL_EXTENSIONS);
    
    if( glewIsSupported(name) != GL_FALSE)
        return 1;
    else
    {
        printf("extension %s is not supported\n", name);
        return 0;
    }
}


int g3d_init_extensions(void)
{
    int state = 1;
    
    if(!g3d_is_extension_supported("GL_ARB_shading_language_100") ||
       !g3d_is_extension_supported("GL_ARB_shader_objects") ||
       !g3d_is_extension_supported("GL_ARB_vertex_shader") ||
       !g3d_is_extension_supported("GL_ARB_fragment_shader")
       /* add here the extensions you want*/)
    {
        state = 0;
    }
    
    return state;
}



/** Loads the content of a shader program.
 * 
 * @param filename  name of the shader file
 * @return 
 */
char* g3d_load_source(const char *filename)
{
    char *src = NULL;   /* code source de notre shader */
    FILE *fp = NULL;    /* fichier */
    long size;          /* taille du fichier */
    long i;             /* compteur */
    
    
    /* on ouvre le fichier */
    fp = fopen(filename, "r");
    /* on verifie si l'ouverture a echoue */
    if(fp == NULL)
    {
      printf( "%s: %d: g3d_load_source(): can not open '%s'\n", __FILE__,__LINE__, filename);
      return NULL;
    }
    
    /* on recupere la longueur du fichier */
    fseek(fp, 0, SEEK_END);
    size = ftell(fp);
    
    /* on se replace au debut du fichier */
    rewind(fp);
    
    /* on alloue de la memoire pour y placer notre code source */
    src = (char *) malloc(size+1); /* +1 pour le caractere de fin de chaine '\0' */
    if(src == NULL)
    {
        fclose(fp);
        fprintf(stderr, "%s: %d: g3d_load_source(): can not allocate memory\n", __FILE__,__LINE__);
        return NULL;
    }
    
    /* lecture du fichier */
    for(i=0; i<size; i++)
        src[i] = fgetc(fp);
    
    /* on place le dernier caractere a '\0' */
    src[size] = '\0';
    
    fclose(fp);
    
    return src;
}


GLuint g3d_load_shader(GLenum type, const char *filename)
{
    GLuint shader = 0;
    GLsizei logsize = 0;
    GLint compile_status = GL_TRUE;
    char *log = NULL;
    char *src = NULL;
    
    /* creation d'un shader de sommet */
    shader = glCreateShader(type);
    if(shader == 0)
    {
        printf("%s: %d: g3d_load_shader(): can not create shader \n", __FILE__,__LINE__);
        return 0;
    }
    
    /* chargement du code source */
    src = g3d_load_source(filename);
    if(src == NULL)
    {
        /* theoriquement, la fonction LoadSource a deja affiche un message
           d'erreur, nous nous contenterons de supprimer notre shader
           et de retourner 0 */
        
        glDeleteShader(shader);
        return 0;
    }
    
    /* assignation du code source */
    glShaderSource(shader, 1, (const GLchar**)&src, NULL);
    
    
    /* compilation du shader */
    glCompileShader(shader);
    
    /* liberation de la memoire du code source */
    free(src);
    src = NULL;
    
    /* verification du succes de la compilation */
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compile_status);
    if(compile_status != GL_TRUE)
    {
        /* erreur a la compilation recuperation du log d'erreur */
        
        /* on recupere la taille du message d'erreur */
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logsize);
        
        /* on alloue un esapce memoire dans lequel OpenGL ecrira le message */
        log = (char *) malloc(logsize + 1);
        if(log == NULL)
        {
            printf("%s: %d: g3d_load_shader(): can not allocate memory \n", __FILE__,__LINE__);
            return 0;
        }
        /* initialisation du contenu */
        memset(log, '\0', logsize + 1);
        
        glGetShaderInfoLog(shader, logsize, &logsize, log);
        fprintf(stderr, "%s: %d: g3d_load_shader(): can not compile shader %s\n%s", __FILE__,__LINE__,filename,log);
        
        /* ne pas oublier de liberer la memoire et notre shader */
        free(log);
        glDeleteShader(shader);
        
        return 0;
    }
    
    return shader;
}

/** Loads the content of a shader program.
 * 
 * @param vsname name of the vertex shader
 * @param psname name of the fragment shader
 * @return 
 */
GLuint g3d_load_program(const char *vsname, const char *psname)
{

  GLuint prog = 0;
  GLuint vs = 0, ps = 0;
  GLint link_status = GL_TRUE;
  GLint logsize = 0;
  char *log = NULL;
  
  
  /* verification des arguments */
  if(vsname == NULL && psname == NULL)
  {
      fprintf(stderr, "%s: %d: g3d_load_program(): creation d'un program demande, mais aucun "
                      "noms de fichiers source envoye, arret.\n", __FILE__,__LINE__);
      
      return 0;
  }
  
  
  /* chargement des shaders */
  if(vsname != NULL)
  {
      vs = g3d_load_shader(GL_VERTEX_SHADER, vsname);
//       printf("%s: %d: g3d_load_program(): load %s\n", __FILE__,__LINE__, vsname); 
      if(vs == 0)
          return 0;
  }
  if(psname != NULL)
  {
      ps = g3d_load_shader(GL_FRAGMENT_SHADER, psname);
//       printf("%s: %d: g3d_load_program(): load %s\n", __FILE__,__LINE__, psname); 
      if(ps == 0)
      {
          if(glIsShader(vs))
              glDeleteShader(vs);
          return 0;
      }
  }
  
  
  /* creation du program */
  prog = glCreateProgram();
  
  /* on envoie nos shaders a notre program */
  if(vs)
      glAttachShader(prog, vs);
  if(ps)
      glAttachShader(prog, ps);
  
  /* on lie le tout */
  glLinkProgram(prog);
  
  /* on verifie que tout s'est bien passe */
  glGetProgramiv(prog, GL_LINK_STATUS, &link_status);
  if(link_status != GL_TRUE)
  {
      glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &logsize);
      log = (char *) malloc(logsize + 1);
      if(log == NULL)
      {
          glDeleteProgram(prog);
          glDeleteShader(vs);
          glDeleteShader(ps);
          
          fprintf(stderr, "%s: %d: g3d_load_program(): impossible d'allouer de la memoire!\n", __FILE__,__LINE__);
          return 0;
      }
      memset(log, '\0', logsize + 1);
      glGetProgramInfoLog(prog, logsize, &logsize, log);
      
      fprintf(stderr, "%s: %d: g3d_load_program(): impossible de lier le program :\n%s", __FILE__,__LINE__, log);
      
      free(log);
      glDeleteProgram(prog);
      glDeleteShader(vs);
      glDeleteShader(ps);
      
      return 0;
  }
  
  /* les shaders sont dans le program maintenant, on en a plus besoin */
  glDeleteShader(vs);
  glDeleteShader(ps);
  
  return prog;
}
