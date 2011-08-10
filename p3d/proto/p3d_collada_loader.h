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

class p3d_collada_loader{

public :

	p3d_collada_loader();
	~p3d_collada_loader();

	bool load(const char* filename);

private :

	DAE* m_collada;
	domCOLLADA* m_root;
	float scale;

	bool extract();
	void fillGeometryColor(const domMaterialRef pmat, const domNodeRef littleNode );
	void extractVertices(const domVerticesRef vertices);
	void extractPolylist(const domPolylistRef polylist);
	void extractTriangles(const domTrianglesRef triangles);
	void extractGeometryMaterialPosition(const domNodeRef node);

};

#endif //P3D_COLLADA_LOADER_H
