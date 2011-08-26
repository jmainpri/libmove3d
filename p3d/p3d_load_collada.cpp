/**
 * \file urdf_p3d_converter.h
 * \brief Programme qui charge un fichier Collada 1.5 dans les structures p3d.
 * \author Francois L.
 * \version 0.1
 * \date 26 août 2011
 *
 * Programme qui charge un fichier Collada 1.5 dans les structures p3d.
 *      - Le modèle COLLADA 1.5 est parsé par la fonction parseCollada() de collada_parser.h.
 *      - Le parsing retourne un modèle URDF (qui décrit sous forme d'un arbre les links et joints du modèle).
 *      - La fonction urdf_p3d_converter() crée à l'aide de ce modèle, un modèle p3d.
 *
 */

#include <iostream>
#include "collada_parser.h"
#include "urdf_p3d_converter.h"
#include "urdf_interface/model.h"

int p3d_load_collada(char* filename, char* modelName)
{
    boost::shared_ptr<urdf::ModelInterface> model;
    std::cout << "Fichier " << filename << " en cours de parsing."<< std::endl;
    model = urdf::parseCollada(filename);

    if(!model)
    {
        std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
        return -1;
    }

    std::cout << "Fichier " << filename << " en cours de conversion."<< std::endl;
    urdf_p3d_converter(model, modelName);

    return 0;
}
