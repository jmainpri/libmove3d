/**
 * \file urdf_p3d_converter.cpp
 * \brief Fonctions pour parcourir une structure URDF C++ et créer un modèle p3d associé.
 * \author F. Lancelot
 * \version 0.1
 * \date 26 août 2011
 *
 * Programme qui parcourt une structure URDF C++ et crée un modèle p3d associé.
 * Le modèle URDF ne semble pas proposer de structures pour sauvegarder les vertices et indices.
 *
 * Une structure a donc été rajouté dans le modèle URDF pour sauvegarder les vertices, indices et couleurs.
 *  - Class Mesh (link.h)
 *    - std::vector<Vector3> vertices;
 *    - std::vector<int> indices;
 *    - Color diffuseColor;
 *
 * Ne peut donc fonctionner pour l'instant qu'avec un modèle URDF créé par le collada_parser qui remplit ces nouvelles structures
 */


#include "proto/urdf_p3d_converter.h"
#include <iostream>
#include "urdf_interface/link.h"
#include "P3d-pkg.h"

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC FOREACH

using namespace urdf;
using namespace std;

void parcoursArbre(boost::shared_ptr<const Link> link_parent, int num_prev_jnt, int * num_last_jnt, Pose pos_abs_jnt_parent);
void add_joint( boost::shared_ptr<Joint> joint, Pose pos_abs_jnt_enf, int num_prev_jnt);
void add_root_freeflyer();
void calcul_pos_abs_link( struct Position * pos_abs_jnt_enf, const Pose * pos_rel_link, struct Position * pos_abs_link);
void calcul_pos_abs_jnt_enf(const Pose * pos_abs_jnt_parent, const Pose * pos_rel_jnt_enfant, struct Position * position_abs_jnt_enf);

/**
 * \fn int urdf_p3d_converter(boost::shared_ptr<ModelInterface> model, char* modelName)
 * \brief Fonction qui parcourt une structure URDF C++ et crée un modèle p3d associé
 * \param model Modèle URDF à convertir
 * \param modelName Nom du modèle
 */
int urdf_p3d_converter(boost::shared_ptr<ModelInterface> model, char* modelName)
{
    boost::shared_ptr<Link> root_link;
    root_link = model->root_link_;
    if(!root_link)
    {
        cout << "Le lien 'root' n'a pas été trouvé" << std::endl;
        return 0;
    }

    p3d_beg_desc(P3D_ROBOT, modelName);

    /*
     **********************************
     ** CAS PARTICULIER DU LIEN ROOT **
     **********************************
     */
    add_root_freeflyer();

    p3d_beg_desc(P3D_BODY, (char *)root_link->name.c_str());

    p3d_add_desc_poly((char *)root_link->name.c_str(),P3D_GRAPHIC);

    urdf::Mesh* mesh = (urdf::Mesh*) root_link->visual->geometry.get();
    FOREACH(it,mesh->vertices){
        p3d_add_desc_vert(it->x, it->y, it->z );
    }
    for(int i=0; i<mesh->indices.size(); i+=3){
        int indices[3]={mesh->indices.at(i)+1, mesh->indices.at(i+1)+1 , mesh->indices.at(i+2)+1 };
        p3d_add_desc_face(indices, 3);
    }

    p3d_end_desc_poly();

    // Ajout de la position du mesh
    double r_root, p_root, y_root;
    Vector3 pos = root_link->visual->origin.position;
    root_link->visual->origin.rotation.getRPY(r_root, p_root, y_root);
    p3d_matrix4 posMatrixRoot;
    p3d_mat4Pos(posMatrixRoot, pos.x*2.0, pos.y*2.0, pos.z*2.0, r_root, p_root, y_root);
    p3d_set_prim_pos_by_mat(p3d_poly_get_poly_by_name((char*)root_link->name.c_str()), posMatrixRoot);
    //p3d_set_prim_pos_deg(p3d_poly_get_poly_by_name((char *)root_link->name.c_str()), pos.x, pos.y, pos.z, 0, 0, 0);

    // Ajout de la couleur du mesh
    double color_vect[3]={mesh->diffuseColor.r,mesh->diffuseColor.g, mesh->diffuseColor.b};
    p3d_poly_set_color(p3d_poly_get_poly_by_name((char*)root_link->name.c_str()), Any, color_vect);

    p3d_end_desc();

    int num_prev_jnt = 1;
    int num_last_jnt = 1;

    parcoursArbre(root_link, num_prev_jnt, &num_last_jnt, root_link->visual->origin);

    return 0;
}

/**
 * \struct Position
 * \brief Contient la position sous forme de Pose ou de Matrix4
 *
 */

struct Position
{
  Pose position_pose;
  p3d_matrix4 position_matrix4;
};

/**
 * \fn void calcul_pos_abs_jnt_enf(const Pose * pos_abs_jnt_parent, const Pose * pos_rel_jnt_enfant, struct Position * position)
 * \brief Calcul la position absolue d'une articulation enfant connaissant sa position relative et la position absolue de l'articulation parente
 * \param pos_abs_jnt_parent Position absolue de l'articulation parente
 * \param pos_rel_jnt_enfant Position relative de l'articulation enfant
 * \param position_abs_jnt_enf Position absolue de l'articulation enfant (résultat)
 */
void calcul_pos_abs_jnt_enf(const Pose * pos_abs_jnt_parent, const Pose * pos_rel_jnt_enfant, struct Position * position_abs_jnt_enf)
{
  double r_jnt_rel, p_jnt_rel, y_jnt_rel;
  double r_jnt_parent_abs, p_jnt_parent_abs, y_jnt_parent_abs;

  pos_abs_jnt_parent->rotation.getRPY(r_jnt_parent_abs, p_jnt_parent_abs, y_jnt_parent_abs);

  // Récupère la position relative et son angle de la jointure enfant par rapport à la jointure parente
  pos_rel_jnt_enfant->rotation.getRPY(r_jnt_rel, p_jnt_rel, y_jnt_rel);

  // Remplit une première matrice de la situation de la jointure parent
  p3d_matrix4 posMatrix1;
  p3d_mat4Pos(posMatrix1, pos_abs_jnt_parent->position.x, pos_abs_jnt_parent->position.y, pos_abs_jnt_parent->position.z, r_jnt_parent_abs, p_jnt_parent_abs, y_jnt_parent_abs);

  // Remplit une deuxième matrice de la transformation relative entre les repères de la jointure parent et de la jointure enfant
  p3d_matrix4 posMatrix2;
  p3d_mat4Pos(posMatrix2, pos_rel_jnt_enfant->position.x, pos_rel_jnt_enfant->position.y, pos_rel_jnt_enfant->position.z, r_jnt_rel, p_jnt_rel, y_jnt_rel);

  // Par multiplication, obtient la matrice de situation de la jointure enfant
  p3d_mat4Mult(posMatrix1, posMatrix2, position_abs_jnt_enf->position_matrix4);

  // Remplit la structure spécifique (Pose) à partir de la matrix4 position_abs_jnt_enf
  double Rx, Ry, Rz;
  p3d_mat4ExtractPosReverseOrder(position_abs_jnt_enf->position_matrix4, &(position_abs_jnt_enf->position_pose.position.x), &(position_abs_jnt_enf->position_pose.position.y), &(position_abs_jnt_enf->position_pose.position.z), &Rx, &Ry, &Rz);
  position_abs_jnt_enf->position_pose.rotation.setFromRPY(Rx,Ry,Rz);

}
/**
 * \fn void calcul_pos_abs_link( struct Position * pos_abs_jnt_enf, const Pose * pos_rel_link, struct Position * pos_abs_link)
 * \brief Calcul la position absolue d'un lien connaissant sa position relative et la position absolue de l'articulation parente
 * \param pos_abs_jnt_enf Position absolue de l'articulation parente
 * \param pos_rel_link Position relative du lien par rapport à l'articulation parente
 * \param pos_abs_link Position absolue du lien (résultat)
 */
void calcul_pos_abs_link( struct Position * pos_abs_jnt_parent, const Pose * pos_rel_link, struct Position * pos_abs_link)
{
  //Récupère la position relative du lien par rapport à la jointure enfant
  double r_rel_link, p_rel_link, y_rel_link;
  pos_rel_link->rotation.getRPY(r_rel_link, p_rel_link, y_rel_link);

  // Remplit une matrice de la transformation relative entre les repères de la jointure enfant et du body
  p3d_matrix4 posMatrix3;
  p3d_mat4Pos(posMatrix3, pos_rel_link->position.x,  pos_rel_link->position.y,  pos_rel_link->position.z, r_rel_link, p_rel_link, y_rel_link);

  // Par multiplication, obtient la matrice de situation du lien
  p3d_mat4Mult(pos_abs_jnt_parent->position_matrix4, posMatrix3, pos_abs_link->position_matrix4);
}

/**
 * \fn void parcoursArbre(boost::shared_ptr<const Link> link_parent, int num_prev_jnt, int * num_last_jnt, Pose pos_abs_jnt_parent)
 * \brief Fonction qui convertit un link URDF dans les structures p3d et fait de même pour tous ses enfants
 * \param model link_parent lien à convertir
 * \param num_prev_jnt Numéro de l'articulation parente
 * \param num_last_jnt Dernier numéro utilisé par une articulation
 * \param pos_abs_jnt_parent Position absolue de l'articulation parente
 */
void parcoursArbre(boost::shared_ptr<const Link> link_parent, int num_prev_jnt, int * num_last_jnt, Pose pos_abs_jnt_parent)
{
  // Pour tous les enfants du lien
  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link_parent->child_links.begin(); child != link_parent->child_links.end(); child++)
  {
    if (*child)
    {
      struct Position pos_abs_jnt_enf;
      struct Position pos_abs_link;

      // Caractérise la jointure par un id non encore utilisé
      (*num_last_jnt)++;
      int num_joint = *num_last_jnt;

      boost::shared_ptr<Joint> joint = (*child)->parent_joint;
      if(joint)
      {
        // Calcul de la position absolue de l'articulation enfant
        calcul_pos_abs_jnt_enf(&pos_abs_jnt_parent, &(joint->parent_to_joint_origin_transform), &pos_abs_jnt_enf);
        add_joint(joint,pos_abs_jnt_enf.position_pose, num_prev_jnt);
      }

      // Récupère le mesh du lien
      urdf::Mesh* mesh = (urdf::Mesh*) (*child)->visual->geometry.get();

      // Ajout dans le monde du lien
      p3d_beg_desc(P3D_BODY, (char*)(*child)->name.c_str());
      p3d_add_desc_poly((char*)(*child)->name.c_str(),P3D_GRAPHIC);

      FOREACH(it,mesh->vertices) {
        p3d_add_desc_vert(it->x, it->y, it->z );
      }

      for(int i=0; i<mesh->indices.size(); i+=3){
        int indices[3]={mesh->indices.at(i)+1, mesh->indices.at(i+1)+1 , mesh->indices.at(i+2)+1 };
        p3d_add_desc_face(indices, 3);
      }
      p3d_end_desc_poly();

      // Calcul de la position absolue du body
      calcul_pos_abs_link(&pos_abs_jnt_enf,  &((*child)->visual->origin), &pos_abs_link);

      // Ajout de la position du mesh
      p3d_set_prim_pos_by_mat(p3d_poly_get_poly_by_name((char*)(*child)->name.c_str()), pos_abs_link.position_matrix4);

      // Ajout de la couleur du mesh
      double color_vect[3]={mesh->diffuseColor.r,mesh->diffuseColor.g, mesh->diffuseColor.b};
      p3d_poly_set_color(p3d_poly_get_poly_by_name((char*)(*child)->name.c_str()), Any, color_vect);

      p3d_end_desc();

      parcoursArbre(*child, num_joint, num_last_jnt,pos_abs_jnt_enf.position_pose);
    }
    else
    {
      std::cout << "link: " << link_parent->name << " has a null child!" << *child << std::endl;
    }
  }
}


/**
 * \fn void add_root_freeflyer()
 * \brief Fonction qui ajoute 6 degrés de liberté au noeud root.
 *
 */
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

/**
 * \fn void add_joint( boost::shared_ptr<Joint> joint, Pose pos_abs_jnt_enf, int num_prev_jnt)
 * \brief Fonction qui fixe une articulation au body en cours.
 * \param joint Articulation URDF à ajouter
 * \param pos_abs_jnt_enf Position absolue de l'articulation à ajouter
 * \param num_prev_jnt Numéro de l'articulation parente
 */
void add_joint( boost::shared_ptr<Joint> joint, Pose pos_abs_jnt_enf, int num_prev_jnt)
{
  p3d_read_jnt_data * data;

  switch(joint->type)
  {
    case Joint::REVOLUTE:
    case Joint::CONTINUOUS: // Pas cette jointure dans p3d donc traité comme une rotation
      // p3d_beg_desc_jnt P3D_ROTATE
      data = p3d_create_read_jnt_data(P3D_ROTATE);
      // p3d_set_dof_vmin
      data->vmin[0]=joint->limits->lower*180/3.14;
      // p3d_set_dof_vmax
      data->vmax[0]=joint->limits->upper*180/3.14;
      break;
    case Joint::PRISMATIC:
      // p3d_beg_desc_jnt P3D_TRANSLATE
      data = p3d_create_read_jnt_data(P3D_TRANSLATE);
      // p3d_set_dof_vmin
      data->vmin[0]=joint->limits->lower;
      // p3d_set_dof_vmax
      data->vmax[0]=joint->limits->upper;
      break;
    case Joint::FLOATING:
     cout << "La jointure de type 'floating' n'est pas traitée" << endl;
     break;
    case Joint::PLANAR:
      cout << "La jointure de type 'planar' n'est pas traitée" << endl;
      break;
    case Joint::FIXED:
      cout << "La jointure de type 'fixed' n'est pas traitée" << endl;
      break;
    default:
      cout << "La jointure est de type 'unknown'" << endl;
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
  p3d_convert_axe_to_mat(data->pos, dtab);
  data->flag_pos = TRUE;

  // p3d_set_dof_vmin
  data->flag_vmin = TRUE;

  // p3d_set_dof_vmax
  data->flag_vmax = TRUE;

  data->scale=1;

  s_p3d_build_jnt_data(data);

  p3d_destroy_read_jnt_data(data);
}
