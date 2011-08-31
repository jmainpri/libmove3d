/**
 * \file p3d_load_collada.h
 * \brief Fonction qui charge un fichier Collada 1.5 dans les structures p3d.
 * \author F. Lancelot
 * \version 0.1
 * \date 26 août 2011
 *
 * Programme qui charge un fichier Collada 1.5 dans les structures p3d.
 *      - Le modèle COLLADA 1.5 est parsé par la fonction parseCollada() de collada_parser.h.
 *      - Le parsing retourne un modèle URDF (qui décrit sous forme d'un arbre les links et joints du modèle).
 *      - La fonction urdf_p3d_converter() crée à l'aide de ce modèle, un modèle p3d.
 *
 */

int p3d_load_collada(char* filename, char* modelName);
