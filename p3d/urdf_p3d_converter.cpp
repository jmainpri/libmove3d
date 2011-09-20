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
 */


#include "proto/urdf_p3d_converter.h"
#include <iostream>
#include <RobotModelParser/urdf_interface/link.h>
#include "P3d-pkg.h"

#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC FOREACH

using namespace urdf;
using namespace std;

void parcoursArbre(boost::shared_ptr<const Link> link_parent, int num_prev_jnt, int * num_last_jnt, Pose pos_abs_jnt_parent, double scale);
void add_joint( boost::shared_ptr<Joint> joint, Pose pos_abs_jnt_enf, int num_prev_jnt, double scale);
void add_root_freeflyer();
void calcul_pos_abs_link( struct Position * pos_abs_jnt_enf, const Pose * pos_rel_link, struct Position * pos_abs_link);
void calcul_pos_abs_jnt_enf(const Pose * pos_abs_jnt_parent, const Pose * pos_rel_jnt_enfant, struct Position * position_abs_jnt_enf);

void loadMesh(std::string const& filename);
void loadBox(urdf::Box * box, string name);
void loadCylinder(urdf::Cylinder * cylinder, string name);
void loadSphere(urdf::Sphere * sphere, string name);

Assimp::Importer _importer;

/**
 * \fn int urdf_p3d_converter(boost::shared_ptr<ModelInterface> model, char* modelName, double scale)
 * \brief Fonction qui parcourt une structure URDF C++ et crée un modèle p3d associé
 * \param model Modèle URDF à convertir
 * \param modelName Nom du modèle
 * \param scale Echelle du modèle
 */
int urdf_p3d_converter(URDFModel* model, char* modelName, double scale)
{
    //_importer.SetIOHandler(new ResourceIOSystem);

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

    if(root_link->visual)
    {
      Geometry* geom = root_link->visual->geometry.get();
      urdf::Mesh* mesh = (urdf::Mesh*) root_link->visual->geometry.get();
      double meshScale =1.0;

      if(geom)
      {
      switch(geom->type)
        {
          case urdf::Geometry::MESH:
            if(!mesh->filename.empty())
            {
              p3d_add_desc_poly((char *)root_link->name.c_str(),P3D_GRAPHIC);
              loadMesh(mesh->filename);
              p3d_end_desc_poly();
              // Même échelle pour tous les axes
              meshScale=mesh->scale.x;
            }
            break;
          case urdf::Geometry::BOX:
            loadBox((urdf::Box*)(geom), root_link->name);
            break;
          case urdf::Geometry::CYLINDER:
            loadCylinder((urdf::Cylinder*)(geom), root_link->name);
            break;
          case urdf::Geometry::SPHERE:
            loadSphere((urdf::Sphere*)(geom), root_link->name);
            break;
        }
      }

      // Ajout de la position du mesh
      double r_root, p_root, y_root;
      Vector3 pos = root_link->visual->origin.position;
      root_link->visual->origin.rotation.getRPY(r_root, p_root, y_root);
      p3d_matrix4 posMatrixRoot;
      // *2.0 car le centre dans le modèle URDF est le centre de l'objet alors que dans Move3d c'est le bas de l'objet
      p3d_mat4Pos(posMatrixRoot, pos.x*2.0, pos.y*2.0, pos.z*2.0, r_root, p_root, y_root);
      p3d_set_prim_pos_by_mat(p3d_poly_get_poly_by_name((char*)root_link->name.c_str()), posMatrixRoot);
      //p3d_set_prim_pos_deg(p3d_poly_get_poly_by_name((char *)root_link->name.c_str()), pos.x, pos.y, pos.z, 0, 0, 0);

      // Ajout de la couleur du mesh
      if(root_link->visual->material)
      {
        double color_vect[3]={root_link->visual->material->color.r,root_link->visual->material->color.g, root_link->visual->material->color.b};
        p3d_poly_set_color(p3d_poly_get_poly_by_name((char*)root_link->name.c_str()), Any, color_vect);
      }

      // Mise à l'échelle
      p3d_scale_prim(p3d_poly_get_poly_by_name((char*)root_link->name.c_str()), scale*meshScale);
    }
    p3d_end_desc();


    int num_prev_jnt = 1;
    int num_last_jnt = 1;

    parcoursArbre(root_link, num_prev_jnt, &num_last_jnt, root_link->visual->origin, scale);

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

  if(!pos_abs_jnt_parent || !pos_rel_jnt_enfant)
  {
    cout << "Calcul de la position impossible" << endl;
    return;
  }

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

  if(!pos_abs_jnt_parent || !pos_rel_link)
  {
    cout << "Calcul de la position impossible" << endl;
    return;
  }

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
 * \param scale Echelle du modèle
 */
void parcoursArbre(boost::shared_ptr<const Link> link_parent, int num_prev_jnt, int * num_last_jnt, Pose pos_abs_jnt_parent, double scale)
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
        add_joint(joint,pos_abs_jnt_enf.position_pose, num_prev_jnt, scale);
      }

      // Ajout dans le monde du lien
      p3d_beg_desc(P3D_BODY, (char*)(*child)->name.c_str());

      if((*child)->visual)
      {
        Geometry* geom = (*child)->visual->geometry.get();
        urdf::Mesh* mesh = (urdf::Mesh*) (*child)->visual->geometry.get();
        double meshScale =1.0;

        if(geom)
        {
          switch(geom->type)
          {
            case urdf::Geometry::MESH:
              if(!mesh->filename.empty())
              {
                p3d_add_desc_poly((char *)(*child)->name.c_str(),P3D_GRAPHIC);
                loadMesh(mesh->filename);
                p3d_end_desc_poly();
                // Même échelle pour tous les axes
                meshScale=mesh->scale.x;
              }
              break;
            case urdf::Geometry::BOX:
              loadBox((urdf::Box*)(geom), (*child)->name);
              break;
            case urdf::Geometry::CYLINDER:
              loadCylinder((urdf::Cylinder*)(geom), (*child)->name);
              break;
            case urdf::Geometry::SPHERE:
              loadSphere((urdf::Sphere*)(geom), (*child)->name);
              break;
          }
        }

        // Calcul de la position absolue du body
        calcul_pos_abs_link(&pos_abs_jnt_enf,  &((*child)->visual->origin), &pos_abs_link);

        // Ajout de la position du mesh
        p3d_set_prim_pos_by_mat(p3d_poly_get_poly_by_name((char*)(*child)->name.c_str()), pos_abs_link.position_matrix4);

        // Ajout de la couleur du mesh
        if((*child)->visual->material)
        {
          double color_vect[3]={(*child)->visual->material->color.r,(*child)->visual->material->color.g, (*child)->visual->material->color.b};
          p3d_poly_set_color(p3d_poly_get_poly_by_name((char*)(*child)->name.c_str()), Any, color_vect);
        }
        // Mise à l'échelle
        //cout << scale << " " << meshScale << endl;
        p3d_scale_prim(p3d_poly_get_poly_by_name((char*)(*child)->name.c_str()), scale*meshScale);

      }

      p3d_end_desc();

      parcoursArbre(*child, num_joint, num_last_jnt,pos_abs_jnt_enf.position_pose, scale);
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
 * \fn void add_joint( boost::shared_ptr<Joint> joint, Pose pos_abs_jnt_enf, int num_prev_jnt, double scale)
 * \brief Fonction qui fixe une articulation au body en cours.
 * \param joint Articulation URDF à ajouter
 * \param pos_abs_jnt_enf Position absolue de l'articulation à ajouter
 * \param num_prev_jnt Numéro de l'articulation parente
 * \param scale Echelle du modèle
 */
void add_joint( boost::shared_ptr<Joint> joint, Pose pos_abs_jnt_enf, int num_prev_jnt, double scale)
{
  p3d_read_jnt_data * data;

  if(!joint)
  {
    cout << "Impossible d'ajouter une articulation" << endl;
    return;
  }

  switch(joint->type)
  {
    case Joint::REVOLUTE:
    case Joint::CONTINUOUS: // Pas cette jointure dans p3d donc traité comme une rotation
      // p3d_beg_desc_jnt P3D_ROTATE
      data = p3d_create_read_jnt_data(P3D_ROTATE);
      // p3d_set_dof_vmin
      data->vmin[0]=-180;
      // p3d_set_dof_vmax
      data->vmax[0]=180;
      if(joint->limits)
      {
        data->vmin[0]=joint->limits->lower*180/3.14;
        data->vmax[0]=joint->limits->upper*180/3.14;
      }
      break;
    case Joint::PRISMATIC:
      // p3d_beg_desc_jnt P3D_TRANSLATE
      data = p3d_create_read_jnt_data(P3D_TRANSLATE);
      // p3d_set_dof_vmin
      data->vmin[0]=-10;
      // p3d_set_dof_vmax
      data->vmax[0]=10;
      if(joint->limits)
      {
        data->vmin[0]=joint->limits->lower;
        data->vmax[0]=joint->limits->upper;
      }
      break;
    case Joint::FLOATING:
     cout << "La jointure de type 'floating' n'est pas traitée" << endl;
     break;
    case Joint::PLANAR:
      cout << "La jointure de type 'planar' n'est pas traitée" << endl;
      break;
    case Joint::FIXED:
      data = p3d_create_read_jnt_data(P3D_FIXED);
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

  data->scale=scale;

  s_p3d_build_jnt_data(data);

  p3d_destroy_read_jnt_data(data);
}

void traiterMesh(aiNode* node, const aiScene* scene)
{
    for(int i=0; i<node->mNumChildren; i++)
    {
        aiNode* mChildren =node->mChildren[i];
        //cout << "nombre de meshs" << mChildren->mNumMeshes <<  endl;

        for(unsigned int j=0; j< mChildren->mNumMeshes; j++)
        {
            aiMesh* input_mesh = scene->mMeshes[mChildren->mMeshes[j]];
            // vertices
            for(unsigned int k=0; k<input_mesh->mNumVertices; k++){
                //cout << "parcourt des vertices" << endl;
                aiVector3D p = input_mesh->mVertices[k];
                p3d_add_desc_vert(p.x, p.y, p.z );
            }
            // faces
            for(unsigned int k=0; k<input_mesh->mNumFaces; k++)
            {
                //cout << "parcourt des faces" << endl;
                aiFace& face = input_mesh->mFaces[k];
                if( face.mNumIndices == 3 ) {
                    int indices[3]={face.mIndices[0]+1, face.mIndices[1]+1, face.mIndices[2]+1};
                    p3d_add_desc_face(indices, 3);
                }
            }
        }
        traiterMesh(node->mChildren[i], scene);
    }
}

void loadBox(urdf::Box * box, string name){
p3d_beg_desc(P3D_BODY, (char *)name.c_str());
p3d_add_desc_box((char *)name.c_str(), box->dim.x, box->dim.y, box->dim.z, P3D_REAL);
}

void loadCylinder(urdf::Cylinder * cylinder, string name)
{
p3d_beg_desc(P3D_BODY, (char *)name.c_str());
p3d_add_desc_cylindre((char *)name.c_str(), cylinder->radius, cylinder->length, P3D_REAL);
}

void loadSphere(urdf::Sphere * sphere, string name)
{
p3d_beg_desc(P3D_BODY, (char *)name.c_str());
p3d_add_desc_sphere((char *)name.c_str(), sphere->radius, P3D_REAL);

}

void loadMesh(std::string const& filename)
{

  //cout << "Chargement du mesh :" <<  filename.c_str() << endl;
  const aiScene* scene = _importer.ReadFile(filename, aiProcess_SortByPType|aiProcess_Triangulate);
  if(!scene){
    cout << "failed to load " << filename.c_str() << endl;
    return;
  }
  if(!scene->mRootNode){
    cout << "resource " << filename.c_str() << " has no data" << endl;
    return;
  }
  if(!scene->HasMeshes()){
    cout << "no meshes found in file " << filename.c_str() << endl;
  }

  aiNode* node = scene->mRootNode;

  traiterMesh(node, scene);
}
