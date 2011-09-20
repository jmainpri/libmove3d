/**
 * \file p3d_load_model.cpp
 * \brief Fonction qui charge un fichier Collada 1.5 dans les structures p3d.
 * \author F. Lancelot
 * \version 0.1
 * \date 26 août 2011
 *
 * Programme qui charge un fichier Collada 1.5 dans les structures p3d.
 *      - Le modèle COLLADA 1.5 est parsé par la fonction initFile de la classe URDFModel.
 *      - Le parsing retourne un modèle URDF (qui décrit sous forme d'un arbre les links et joints du modèle).
 *      - La fonction urdf_p3d_converter() crée à l'aide de ce modèle, un modèle p3d.
 *
 */

#include <iostream>
#include "urdf_p3d_converter.h"
#include <RobotModelParser/urdf_model.h>

int p3d_load_model(char* filename, char* modelName, double scale)
{
    urdf::URDFModel model;
    std::cout << "Fichier " << filename << " en cours de parsing."<< std::endl;

    if(!model.initFile(filename))
    {
        std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
        return -1;
    }

    std::cout << "Fichier " << filename << " en cours de conversion."<< std::endl;
    urdf_p3d_converter(&model, modelName, scale);

    return 0;
}
