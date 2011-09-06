#include "../urdf_interface/model.h"

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
 * Une structure a donc été rajouté dans le modèle URDF pour sauvegarder les vertices, indices et couleurs.
 *  - Class Mesh (link.h)
 *    - std::vector<Vector3> vertices;
 *    - std::vector<int> indices;
 *    - Color diffuseColor;
 *
 * Ne peut donc fonctionner pour l'instant qu'avec un modèle URDF créé par le collada_parser qui remplit ces nouvelles structures
 */


int urdf_p3d_converter(boost::shared_ptr<urdf::ModelInterface> model, char* modelName, double scale);
