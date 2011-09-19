#include <RobotModelParser/urdf_model.h>

/**
 * \file urdf_p3d_converter.h
 * \brief Fonction qui parcourt une structure URDF C++ et crée un modèle p3d associé.
 * \author F. Lancelot
 * \version 0.1
 * \date 26 août 2011
 *
 * Programme qui parcourt une structure URDF C++ et crée un modèle p3d associé.
 * Le modèle URDF ne semble pas proposer de structures pour sauvegarder les vertices et indices.
 *
 */


int urdf_p3d_converter(urdf::URDFModel* model, char* modelName, double scale);
