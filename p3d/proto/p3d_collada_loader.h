#ifndef P3D_COLLADA_LOADER_H
#define P3D_COLLADA_LOADER_H

/**
 * \file p3d_collada_loader.h
 * \brief Programme qui charge un fichier Collada 1.5 dans les structures p3d.
 * \author Francois L.
 * \version 0.1
 * \date 10 août 2011
 *
 * Programme qui charge un fichier Collada 1.5 dans les structures p3d.
 * Il gère pour l'instant que les fichiers Collada 1.5 simple : faces polylists et triangles, matériaux
 * avec une simple couleur RGB, positions des corps par translation et rotation en x, y et z (pas de matrice).
 * Ne gère pas encore  la cinématique.
 *
 *
 */

#include <dae.h>
#include <dom/domCOLLADA.h>


 /*! \class p3d_collada_loader
   * \brief Classe représentant le chargeur d'un fichier Collada 1.5
   *
   *  La classe gère le chargement d'un fichier Collada 1.5 dans les structures p3d
   */
class p3d_collada_loader{

public :

    /*!
     *  \brief Constructeur de la classe p3d_collada_loader
     */
	p3d_collada_loader();

    /*!
     *  \brief Destructeur de la classe p3d_collada_loader
     */
	virtual ~p3d_collada_loader();

    /*!
     *  \brief Charge un fichier Collada 1.5 dans les structures p3d
     *
     *  \param filename : le fichier Collada à charger
     *  \return true si le fichier a correctement été parsé et chargé, false sinon
     */
	bool load(const char* filename);

    /*!
     *  \brief Charge un fichier Collada 1.5 dans un environnement P3D (P3D_env)
     *
     *  \param filename : le fichier Collada à charger
     *  \return true si le fichier a correctement été parsé et chargé, false sinon
     */
	bool p3d_read_collada(char* filename, char* rootName, double scale);


private :

	DAE* m_collada; /*!< Accès vers la librairie ColladaDOM*/
	domCOLLADA* m_root; /*!< Noeud root du fichier Collada*/
	float scale; /*!< Echelle de la scène du fichier Collada*/

    /*!
     *  \brief Extrait les informations utiles et remplit les structures p3d
     *
     *	Extrait les informations utiles d'un fichier Collada et remplit les structures p3d associées
     *
     *  \return true si le fichier a correctement été parsé, false sinon
     */
	bool extractAndFill(char* nomObjet);

    /*!
     *  \brief Extrait les informations utiles et remplit les structures p3d d'un sous-noeud
     *
     *	Extrait les informations utiles d'un sous-noeud d'une scène visuelle (visual_scene/node/node) d'un fichier Collada et remplit les structures p3d associées
     */
	void extractAndFillGeometryPositionColor(const domNodeRef node);

    /*!
     *  \brief Extrait les couleurs et remplit les structures p3d d'un sous-noeud
     *
     *	Extrait les couleurs d'un sous-noeud d'une scène visuelle (visual_scene/node/node) et remplit les structures p3d associées
     */
	void extractAndFillColor(const domMaterialRef pmat, const domNodeRef littleNode );

	/*!
     *  \brief Extrait les sommets et remplit les structures p3d d'un sous-noeud
     *
     *	Extrait les sommets d'un sous-noeud d'une scène visuelle (visual_scene/node/node) et remplit les structures p3d associées
     */
	void extractAndFillVertices(const domVerticesRef vertices);

	/*!
     *  \brief Extrait les faces (polynomes) et remplit les structures p3d d'un sous-noeud
     *
     *	Extrait les faces (polynomes) d'un sous-noeud d'une scène visuelle (visual_scene/node/node) et remplit les structures p3d associées
     */
	void extractAndFillPolylist(const domPolylistRef polylist);

	/*!
     *  \brief Extrait les faces (triangles) et remplit les structures p3d d'un sous-noeud
     *
     *	Extrait les faces (triangles) d'un sous-noeud d'une scène visuelle (visual_scene/node/node) et remplit les structures p3d associées
     */
	void extractAndFillTriangles(const domTrianglesRef triangles);

};

#endif //P3D_COLLADA_LOADER_H
