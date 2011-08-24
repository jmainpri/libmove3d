#include <iostream>
#include "collada_parser.h"
#include "urdf_interface/model.h"
#include "urdf_interface/link.h"

#include "P3d-pkg.h"

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC FOREACH

using namespace urdf;
using namespace std;

void parcoursArbre(boost::shared_ptr<const Link> link_parent, int num_prev_jnt, int * num_last_jnt, Pose pos_abs_jnt_parent);

// Ajout d'un FREEFLYER pour avoir un objet movable
void add_root_freeflyer()
{
  p3d_read_jnt_data * data = p3d_create_read_jnt_data(P3D_FREEFLYER);

  // p3d_set_prev_jnt
  data->prev_jnt=0;
  data->flag_prev_jnt = TRUE;

  // p3d_set_pos_xyz
  double dtab[6] = {0,0,0,0,0,0};
  dtab[3] = DTOR(dtab[3]);
  dtab[4] = DTOR(dtab[4]);
  dtab[5] = DTOR(dtab[5]);
  p3d_mat4Pos(data->pos, dtab[0], dtab[1], dtab[2],
  dtab[3], dtab[4], dtab[5]);
  data->flag_pos = TRUE;

  // p3d_set_dof
  for(int i=0; i<6; i++)
          data->v[i]=dtab[i];
  data->flag_v = TRUE;

  // p3d_set_dof_vmin
  double dtab2[6]={-10,-10,-2.500,-180,-180,-180};
  for(int i=0; i<6; i++)
          data->vmin[i]=dtab2[i];
  data->flag_vmin = TRUE;

  // p3d_set_dof_vmax
  double dtab3[6]={10,10,2,180,180,180};
  for(int i=0; i<6; i++)
          data->vmax[i]=dtab3[i];
  data->flag_vmax = TRUE;

  data->scale=1;

  s_p3d_build_jnt_data(data);

  p3d_destroy_read_jnt_data(data);
}

void add_joint( boost::shared_ptr<Joint> joint, Pose pos_abs_jnt_enf, int num_prev_jnt)
{
  p3d_read_jnt_data * data;

  switch(joint->type)
  {
    case Joint::REVOLUTE:
    case Joint::CONTINUOUS: // Pas cette jointure dans p3d donc traité comme une rotation
    cout << "p3d_beg_desc_jnt " << "P3D_ROTATE" << endl;
    data = p3d_create_read_jnt_data(P3D_ROTATE);
    data->vmin[0]=joint->limits->lower*180/3.14;
    data->vmax[0]=joint->limits->upper*180/3.14;
    break;
    case Joint::PRISMATIC:
    cout << "p3d_beg_desc_jnt " << "P3D_TRANSLATE" << endl;
    data = p3d_create_read_jnt_data(P3D_TRANSLATE);
    data->vmin[0]=joint->limits->lower;
    data->vmax[0]=joint->limits->upper;
    break;
    default :
    break;
  }

  // p3d_set_name
  strcpy(data->name, joint->name.c_str());
  data->flag_name = TRUE;

  // p3d_set_prev_jnt
  data->prev_jnt=num_prev_jnt;
  data->flag_prev_jnt = TRUE;

  // p3d_set_pos_axe
  double rx, ry, rz;
  pos_abs_jnt_enf.rotation.getRPY(rx,ry,rz);
  p3d_matrix4 posMatrix1;
  p3d_mat4Pos(posMatrix1, pos_abs_jnt_enf.position.x, pos_abs_jnt_enf.position.y, pos_abs_jnt_enf.position.z, rx, ry, rz);

  p3d_matrix3 rotMatrix;
  rotMatrix[0][0]=posMatrix1[0][0];
  rotMatrix[0][1]=posMatrix1[0][1];
  rotMatrix[0][2]=posMatrix1[0][2];
  rotMatrix[1][0]=posMatrix1[1][0];
  rotMatrix[1][1]=posMatrix1[1][1];
  rotMatrix[1][2]=posMatrix1[1][2];
  rotMatrix[2][0]=posMatrix1[2][0];
  rotMatrix[2][1]=posMatrix1[2][1];
  rotMatrix[2][2]=posMatrix1[2][2];

  p3d_vector3 axis;
  axis[0]=joint->axis.x;
  axis[1]=joint->axis.y;
  axis[2]=joint->axis.z;

  p3d_vector3 vector;
  vector[0]=rotMatrix[0][0]*axis[0]+rotMatrix[0][1]* axis[1]+rotMatrix[0][2]*axis[2];
  vector[1]=rotMatrix[1][0]*axis[0]+rotMatrix[1][1]* axis[1]+rotMatrix[1][2]*axis[2];
  vector[2]=rotMatrix[2][0]*axis[0]+rotMatrix[2][1]* axis[1]+rotMatrix[2][2]*axis[2];


  double dtab[6] = {pos_abs_jnt_enf.position.x,pos_abs_jnt_enf.position.y,pos_abs_jnt_enf.position.z, vector[0], vector[1], vector[2]};
  cout << joint->axis.x << joint->axis.y << joint->axis.z << endl;
  p3d_convert_axe_to_mat(data->pos, dtab);
  data->flag_pos = TRUE;

  // p3d_set_pos_xyz
//  double rx, ry, rz;
//  pos_abs_jnt_enf.rotation.getRPY(rx,ry,rz);
//  cout << rx << " " << ry << " " << rz << endl;
//  double dtab1[6] = {pos_abs_jnt_enf.position.x,pos_abs_jnt_enf.position.y,pos_abs_jnt_enf.position.z,rx*180/3.14,ry*180/3.14,rz*180/3.14};
//  dtab1[3] = DTOR(dtab1[3]);
//  dtab1[4] = DTOR(dtab1[4]);
//  dtab1[5] = DTOR(dtab1[5]);
//  p3d_mat4Pos(data->pos, dtab1[0], dtab1[1], dtab1[2],
//  dtab1[3], dtab1[4], dtab1[5]);
//  data->flag_pos = TRUE;

  // p3d_set_dof_vmin
  data->flag_vmin = TRUE;

  // p3d_set_dof_vmax
  data->flag_vmax = TRUE;

  data->scale=1;

  s_p3d_build_jnt_data(data);

  p3d_destroy_read_jnt_data(data);
}

int p3d_load_collada(char* filename, char* modelName)
{
    boost::shared_ptr<ModelInterface> model;
    cout << filename << endl;
    model = parseCollada(filename);

    boost::shared_ptr<Link> root_link;
    root_link = model->root_link_;

    if(!root_link)
        return 0;

    cout << "p3d_beg_desc P3D_ROBOT" << endl;
    p3d_beg_desc(P3D_ROBOT, modelName);

    cout << "p3d_beg_desc_jnt P3D_FREEFLYER # J1" << endl;
    cout << "p3d_set_name root" << endl;
    cout <<  "p3d_set_prev_jnt 0" << endl;
    cout <<  "p3d_set_pos_xyz 0 0 0 0 0 0" << endl;
    cout << "p3d_set_dof 0 0 0 0 0 0"<< endl;
    cout << "p3d_set_dof_vmin -10 -10 -2.500 -180 -180 -180" << endl;
    cout <<  "p3d_set_dof_vmax 10 10 2.500 180 180 180" << endl;
    cout << "p3d_end_desc" << endl;

    add_root_freeflyer();

    cout << "p3d_beg_desc P3D_BODY " << root_link->name << endl;
    p3d_beg_desc(P3D_BODY, (char *)root_link->name.c_str());

    cout << "p3d_add_desc_poly " << root_link->name << endl;
    p3d_add_desc_poly((char *)root_link->name.c_str(),P3D_GRAPHIC);

    urdf::Mesh* mesh = (urdf::Mesh*) root_link->visual->geometry.get();
    FOREACH(it,mesh->vertices){
    	//cout << "p3d_add_desc_vert "<< it->x << " " << it->y << " " << it->z  << endl;
        p3d_add_desc_vert(it->x, it->y, it->z );
    }
    for(int i=0; i<mesh->indices.size(); i+=3){
    	int indices[3]={mesh->indices.at(i)+1, mesh->indices.at(i+1)+1 , mesh->indices.at(i+2)+1 };
        //cout << "p3d_add_desc_face " << indices[0] << " " << indices[1] << " " << indices[2] << endl;
        p3d_add_desc_face(indices, 3);
    }

    cout << "p3d_end_desc_poly" << endl;
    p3d_end_desc_poly();

    // Ajout de la position du mesh
    Vector3 pos = root_link->visual->origin.position;
    cout << "p3d_set_prim_pos " << root_link->name  << " " << pos.x << " " << pos.y << " " << pos.z  << " " << 0  << " " << 0  << " " << 0  << endl;
    p3d_set_prim_pos_deg(p3d_poly_get_poly_by_name((char *)root_link->name.c_str()), pos.x, pos.y, pos.z, 0, 0, 0);
    // Ajout de la couleur du mesh
    cout << "p3d_set_prim_color " << root_link->name << " Any " <<  mesh->diffuseColor.r << " " <<  mesh->diffuseColor.g << " " <<  mesh->diffuseColor.b << endl;
    double color_vect[3]={mesh->diffuseColor.r,mesh->diffuseColor.g, mesh->diffuseColor.b};
    p3d_poly_set_color(p3d_poly_get_poly_by_name((char*)root_link->name.c_str()), Any, color_vect);

    cout << "p3d_end_desc" << endl;
    p3d_end_desc();

    int num_prev_jnt = 1;
    int num_last_jnt = 1;

    parcoursArbre(root_link, num_prev_jnt, &num_last_jnt, root_link->visual->origin);

    return 0;
}

// link_parent : lien parent
// num_prev_jnt : numéro de la jointure parent
// pos_abs_jnt_parent : position aboslue de la jointure parent
// num_last_jnt : numéro de la dernière jointure déclarée
void parcoursArbre(boost::shared_ptr<const Link> link_parent, int num_prev_jnt, int * num_last_jnt, Pose pos_abs_jnt_parent)
{
  // pour tous les enfants du lien
  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link_parent->child_links.begin(); child != link_parent->child_links.end(); child++)
  {
    if (*child)
    {
      double r_jnt_parent_abs, p_jnt_parent_abs, y_jnt_parent_abs;
      double r_jnt_rel, p_jnt_rel, y_jnt_rel;
      Pose pos_jnt_enf_abs;
      p3d_matrix4 posJntEnf;

      (*num_last_jnt)++;
      int num_joint = *num_last_jnt;
      boost::shared_ptr<Joint> joint = (*child)->parent_joint;
      if(joint)
      {
        // Récupère la jointure parent
        pos_abs_jnt_parent.rotation.getRPY(r_jnt_parent_abs, p_jnt_parent_abs, y_jnt_parent_abs);

        // Récupère la position relative et son angle de la jointure par rapport à la jointure parente
        Pose pos_rel_jnt;
        pos_rel_jnt.position.x=joint->parent_to_joint_origin_transform.position.x;
        pos_rel_jnt.position.y=joint->parent_to_joint_origin_transform.position.y;
        pos_rel_jnt.position.z=joint->parent_to_joint_origin_transform.position.z;
        joint->parent_to_joint_origin_transform.rotation.getRPY(r_jnt_rel, p_jnt_rel, y_jnt_rel);

        // Calcul de la position aboslu de la jointure enfant
          // on remplit une premiere matrice de la situation de la jointure parent
          p3d_matrix4 posMatrix1;
          p3d_mat4Pos(posMatrix1, pos_abs_jnt_parent.position.x, pos_abs_jnt_parent.position.y, pos_abs_jnt_parent.position.z, r_jnt_parent_abs, p_jnt_parent_abs, y_jnt_parent_abs);

          // on remplit une deuxieme matrice de la transformation relative entre les repères
          p3d_matrix4 posMatrix2;
          p3d_mat4Pos(posMatrix2, pos_rel_jnt.position.x, pos_rel_jnt.position.y, pos_rel_jnt.position.z, r_jnt_rel, p_jnt_rel, y_jnt_rel);


          // par multiplication, on obtient la matrice de situation de la jointure enfant
          p3d_mat4Mult(posMatrix1, posMatrix2, posJntEnf);

          // on remplit la Pos
          double Rx, Ry, Rz;
          p3d_mat4ExtractPosReverseOrder(posJntEnf, &(pos_jnt_enf_abs.position.x), &(pos_jnt_enf_abs.position.y), &(pos_jnt_enf_abs.position.z), &Rx, &Ry, &Rz);
          pos_jnt_enf_abs.rotation.setFromRPY(Rx,Ry,Rz);

          add_joint(joint,pos_jnt_enf_abs, num_prev_jnt);
      }

      // Récupère le mesh du lien
      // TODO : traiter le cas des formes géométriques simples (si elles ne sont pas transformées en mesh lors du parsing)
      urdf::Mesh* mesh = (urdf::Mesh*) (*child)->visual->geometry.get();
      if(mesh->vertices.size()==0)
      continue;

      // on remplit une troisièùe matrice de la transformation relative entre les repères
      // Récupère la position relative du lien par rapport à la jointure parente
      double r_rel_link, p_rel_link, y_rel_link;
      (*child)->visual->origin.rotation.getRPY(r_rel_link, p_rel_link, y_rel_link);
      p3d_matrix4 posMatrix3;
      p3d_mat4Pos(posMatrix3, (*child)->visual->origin.position.x, (*child)->visual->origin.position.y, (*child)->visual->origin.position.z, r_rel_link, p_rel_link, y_rel_link);

      // par multiplication, on obtient la matrice de situation de la jointure enfant
      p3d_matrix4 posLink;
      p3d_mat4Mult(posJntEnf, posMatrix3, posLink);

      double Tx, Ty, Tz, Rx, Ry, Rz;
      p3d_mat4ExtractPosReverseOrder(posLink, &Tx, &Ty, &Tz, &Rx, &Ry, &Rz);

      // Ajout dans le monde du lien
      cout << "p3d_beg_desc P3D_BODY " << (*child)->name << endl;
      p3d_beg_desc(P3D_BODY, (char*)(*child)->name.c_str());
      cout << "p3d_add_desc_poly " << (*child)->name << endl;
      p3d_add_desc_poly((char*)(*child)->name.c_str(),P3D_GRAPHIC);

      FOREACH(it,mesh->vertices) {
        //cout << "p3d_add_desc_vert "<< it->x << " " << it->y << " " << it->z  << endl;
        p3d_add_desc_vert(it->x, it->y, it->z );
      }

      for(int i=0; i<mesh->indices.size(); i+=3){
        int indices[3]={mesh->indices.at(i)+1, mesh->indices.at(i+1)+1 , mesh->indices.at(i+2)+1 };
        //cout << "p3d_add_desc_face " << indices[0] << " " << indices[1] << " " << indices[2] << endl;
        p3d_add_desc_face(indices, 3);
      }
      cout << "p3d_end_desc_poly" << endl;
      p3d_end_desc_poly();

      // Ajout de la position du mesh
      //cout << "p3d_set_prim_pos " << (*child)->name  << " " << Tx << " " << Ty << " " << Tz << " " <<  Rx << " " << Ry << " " << Rz << endl;
      //p3d_set_prim_pos_deg(p3d_poly_get_poly_by_name((char*)(*child)->name.c_str()), Tx, Ty, Tz, Rx, Ry, Rz);
      p3d_set_prim_pos_by_mat(p3d_poly_get_poly_by_name((char*)(*child)->name.c_str()), posLink);

      // Ajout de la couleur du mesh
      cout << "p3d_set_prim_color " << (*child)->name << " Any " <<  mesh->diffuseColor.r << " " <<  mesh->diffuseColor.g << " " <<  mesh->diffuseColor.b << endl;
      double color_vect[3]={mesh->diffuseColor.r,mesh->diffuseColor.g, mesh->diffuseColor.b};
      p3d_poly_set_color(p3d_poly_get_poly_by_name((char*)(*child)->name.c_str()), Any, color_vect);

      cout << "p3d_end_desc" << endl;
      p3d_end_desc();

      parcoursArbre(*child, num_joint, num_last_jnt,pos_jnt_enf_abs);
    }
    else
    {
      std::cout << "link: " << link_parent->name << " has a null child!" << *child << std::endl;
    }
  }
}
