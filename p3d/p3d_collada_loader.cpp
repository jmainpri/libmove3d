/**
 * \file p3d_collada_loader.c
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

#include "./proto/p3d_collada_loader.h"

#include <iostream>

/*
 *
 *  FONCTIONS P3D UTILISEES ET LEUR FICHIERS
 *
 *  #include "p3d/proto/p3d_env_proto.h"
 *  void *p3d_beg_desc ( int type, char *name );
 *  void p3d_add_desc_box(char name[20], double a, double b, double c, int type)
 *  int p3d_end_desc(void)
 *  void p3d_add_desc_vert(double x, double y, double z)
 *  void p3d_add_desc_poly(char name[20], int type)
 *
 *
 *
 *
 *  #include "p3d/proto/p3d_setpos_proto.h"
 *  void p3d_set_prim_pos(p3d_poly *poly, double tx, double ty, double tz,
                      double rx, double ry, double rz) {
 *
 *
 *  #include "p3d/proto/p3d_poly_proto.h"
 *  p3d_poly *p3d_poly_get_poly_by_name(char *name) {
 *
 *
 */
#include "P3d-pkg.h"

using namespace std;

p3d_collada_loader::p3d_collada_loader(): m_collada(NULL), m_root(NULL)
{
	scale = 1.0;
}

p3d_collada_loader::~p3d_collada_loader()
{
	if(m_collada)
		delete m_collada;

	m_collada=NULL;
	m_root=NULL;
}

bool p3d_collada_loader::load(const char* filename)
{
	bool chargement_reussi = true;

	// Initialise les variables ColladaDom et ouvre le fichier
	m_collada = new DAE;
	m_root = m_collada->open(filename);

	if(!m_root){
		cout << "L'ouverture du fichier Collada a échoué" << endl;
		return false;
	}

	// Récupère l'échelle de la scène
	if( !!m_root->getAsset() ) {
        if( !!m_root->getAsset()->getUnit() ) {
        	scale = m_root->getAsset()->getUnit()->getMeter();
        }
    }

	// Initialise le début de la description et de l'environnement à la main
	cout << "p3d_beg_desc P3D_ENV Env " << endl;
	p3d_beg_desc(P3D_ENV, (char *)"Env");

	cout << "p3d_beg_desc P3D_OBSTACLE floor" << endl;
	p3d_beg_desc(P3D_OBSTACLE, (char *)"floor");
		p3d_add_desc_box((char *)"floor", 4, 4, 0.05, P3D_REAL);
		p3d_poly *poly;
		poly = p3d_poly_get_poly_by_name ((char *)"floor");
		p3d_set_prim_pos_deg (poly, 0,0,-1,0,0,0 );
	cout << "p3d_end_desc" << endl;
	p3d_end_desc();

	chargement_reussi = extractAndFill();

	cout << "p3d_end_desc" << endl;
	p3d_end_desc();

	return chargement_reussi;
}


bool p3d_collada_loader::extractAndFill()
{
	domCOLLADA::domSceneRef all_scene = m_root->getScene();
	if(!all_scene)
	{
		cout << "Le chargement de la balise 'scene' a échoué" << endl;
		return false;
	}
	// Récupère l'instance_visual_scene
	if(all_scene->getInstance_visual_scene())
	{
		// Grâce à l'URL, conduit directement à la description de la visual_scene dans la library_visual_scene
		domVisual_sceneRef visual_scene = daeSafeCast<domVisual_scene>(all_scene->getInstance_visual_scene()->getUrl().getElement().cast());

		// Parcourt tous les noeuds de la visual scene
		for(size_t i=0; i < visual_scene->getNode_array().getCount(); i++)
		{
			domNodeRef node = visual_scene->getNode_array()[i];

			cout << "p3d_beg_desc P3D_ROBOT" << endl;
			p3d_beg_desc(P3D_ROBOT, (char*)"greiffer");

			// TODO
			// Ajout d'un FREEFLYER pour avoir un objet movable
			//p3d_read_jnt_data * data;
			//data = p3d_create_read_jnt_data(P3D_FREEFLYER);

			// Parcours tous les noeuds fils (sous-noeud, littleNode : node/node)
			for(size_t j=0; j<node->getNode_array().getCount(); j++)
			{
				domNodeRef littleNode = node->getNode_array()[j];

				cout << "p3d_beg_desc P3D_BODY " << littleNode->getID() << endl;
				p3d_beg_desc(P3D_BODY, (char*)littleNode->getID());

				extractAndFillGeometryPositionColor(littleNode);

				cout << "p3d_end_desc"<< endl;
				p3d_end_desc();
			}

			cout << "p3d_end_desc"<< endl;
			p3d_end_desc();
		}
	}
	return true;
}

void p3d_collada_loader::extractAndFillGeometryPositionColor(const domNodeRef littleNode)
{
	// Récupère les instances géométriques
	for(size_t k=0; k<littleNode->getInstance_geometry_array().getCount(); k++)
	{
		cout << "p3d_add_desc_poly " << littleNode->getID() << " P3D_GRAPHIC" << endl;
		p3d_add_desc_poly((char *)littleNode->getID(),P3D_GRAPHIC);

		domInstance_geometryRef instance_geo = littleNode->getInstance_geometry_array()[k];

		// Grâce à l'URL, conduit directement à la description de la géométrie dans la library_geometries
		domGeometryRef geometry = daeSafeCast<domGeometry>(instance_geo->getUrl().getElement().cast());

		if(geometry->getMesh())
		{
			// Récupère les vertices
			domVerticesRef vertices = geometry->getMesh()->getVertices();
			extractAndFillVertices(vertices);

			// Récupère les polylist
			for(size_t l=0; l<geometry->getMesh()->getPolylist_array().getCount(); l++)
			{
				domPolylistRef polylist = geometry->getMesh()->getPolylist_array()[l];
				extractAndFillPolylist(polylist);
			}
			// Récupère les triangles
			for(size_t l=0; l<geometry->getMesh()->getTriangles_array().getCount(); l++)
			{
				domTrianglesRef triangles = geometry->getMesh()->getTriangles_array()[l];
				extractAndFillTriangles(triangles);
			}
			cout << "p3d_end_desc_poly" << endl;
			p3d_end_desc_poly();

			// TODO à revoir
			// Ajoute du matérial (couleur) de cette instance_geometry
			if (!!instance_geo->getBind_material() && !!instance_geo->getBind_material()->getTechnique_common()) {
				const domInstance_material_Array& matarray = instance_geo->getBind_material()->getTechnique_common()->getInstance_material_array();
				for (size_t imat = 0; imat < matarray.getCount(); ++imat) {
					domMaterialRef p_mat = daeSafeCast<domMaterial>(matarray[imat]->getTarget().getElement());
					if (!!p_mat) {
						extractAndFillColor(p_mat, littleNode);
					}
				}
			}
		}
		// Ajout de la position du littleNode
		if(littleNode)
		{
			float x=0, y=0, z=0;
			float rx=0, ry=0, rz=0;
			// Translation
			for(size_t n_translate=0; n_translate < littleNode->getTranslate_array().getCount(); n_translate++)
			{
				x+=littleNode->getTranslate_array()[n_translate]->getValue()[0];
				y+=littleNode->getTranslate_array()[n_translate]->getValue()[1];
				z+=littleNode->getTranslate_array()[n_translate]->getValue()[2];
			}
			// Rotation
			for(size_t n_rotate=0; n_rotate < littleNode->getRotate_array().getCount(); n_rotate++)
			{
				if(littleNode->getRotate_array()[n_rotate]->getValue()[0]==1)
					rx+=littleNode->getRotate_array()[n_rotate]->getValue()[3];
				if(littleNode->getRotate_array()[n_rotate]->getValue()[1]==1)
					ry+=littleNode->getRotate_array()[n_rotate]->getValue()[3];
				if(littleNode->getRotate_array()[n_rotate]->getValue()[2]==1)
					rz+=littleNode->getRotate_array()[n_rotate]->getValue()[3];
			}

			cout << "p3d_set_prim_pos " << littleNode->getID() << " " << x << " " << y << " " << z << " " << rx << " "<< ry << " " << rz << endl;
			p3d_poly* poly= p3d_poly_get_poly_by_name ((char*)littleNode->getID());
			p3d_set_prim_pos(poly, x, y, z, rx, ry, rz);
	        p3d_scale_prim(poly, scale);

		}
	}

	// Parcours récursif pour tous les sous-noeuds
	for(size_t i=0; i<littleNode->getNode_array().getCount(); i++)
	{
		domNodeRef underLittleNode = littleNode->getNode_array()[i];
		extractAndFillGeometryPositionColor(underLittleNode);
	}
}

void p3d_collada_loader::extractAndFillVertices(const domVerticesRef vertices)
{
	for (size_t l=0; l<vertices->getInput_array().getCount(); l++)
	{
		domInput_localRef local_ref = vertices->getInput_array()[l];
		daeString str = local_ref->getSemantic();
		if ( strcmp(str,"POSITION") == 0 )
		{
			const domSourceRef source = daeSafeCast<domSource>(local_ref->getSource().getElement());
			if( !source )
			{
				continue;
			}
            const domFloat_arrayRef float_array = source->getFloat_array();
            const domList_of_floats& list_floats = float_array->getValue();

            int vertexStride = 3;     //instead of hardcoded stride, should use the 'accessor'
            for(size_t m=0; m<float_array->getValue().getCount(); m=m+vertexStride)
            {
            	cout << "p3d_add_desc_vert " << list_floats.get(m) << " " << list_floats.get(m+1) << " " << list_floats.get(m+2) << endl;
            	p3d_add_desc_vert(list_floats.get(m), list_floats.get(m+1), list_floats.get(m+2) );

            }
		}
	}
}

void p3d_collada_loader::extractAndFillPolylist(const domPolylistRef polylist)
{
	// Récupère l'offset des vertex
	size_t vertexoffset = -1;
	size_t polylist_stride = 0;
	for (unsigned int w=0; w<polylist->getInput_array().getCount(); w++) {
		size_t offset = polylist->getInput_array()[w]->getOffset();
		daeString str = polylist->getInput_array()[w]->getSemantic();
		if (!strcmp(str,"VERTEX")) {
			vertexoffset = offset;
		}
		if (offset> polylist_stride)
		{
			polylist_stride = offset;
		}
	}
	polylist_stride++;

	int repere=0;
	// Parcourt toutes les faces
	int * listeV = NULL;
	for(size_t m=0; m<polylist->getVcount()->getValue().getCount(); m++)
	{
		cout << "p3d_add_desc_face ";

		// Parcourt toutes les vertices des faces
		size_t num_verts = polylist->getVcount()->getValue()[m];

		listeV = new int[num_verts];
		for(size_t n=0; n<num_verts; n++)
		{
			cout << polylist->getP()->getValue().get(repere+vertexoffset+polylist_stride*n)+1 << " ";
			listeV[n]=polylist->getP()->getValue().get(repere+vertexoffset+polylist_stride*n)+1;
		}
		cout << endl;

		p3d_add_desc_face(listeV, num_verts);
		delete listeV;
		listeV=NULL;

		repere+=polylist_stride*(num_verts);
	}
}

void p3d_collada_loader::extractAndFillTriangles(const domTrianglesRef triangles)
{
	// Récupère l'offset des vertex
	size_t vertexoffset = -1;
	size_t triangles_stride = 0;
	for (unsigned int w=0; w<triangles->getInput_array().getCount(); w++) {
		size_t offset = triangles->getInput_array()[w]->getOffset();
		daeString str = triangles->getInput_array()[w]->getSemantic();
		if (!strcmp(str,"VERTEX")) {
			vertexoffset = offset;
		}
		if (offset> triangles_stride)
		{
			triangles_stride = offset;
		}
	}
	triangles_stride++;

	int repere=0;
	// Parcourt toutes les faces
	int * listeV = NULL;
	for(size_t m=0; m<triangles->getCount(); m++)
	{
		cout << "p3d_add_desc_face ";

		// Parcourt toutes les vertices des faces
		size_t num_verts = 3; //comme c'est un triangle, il y a en a tj 3
		listeV = new int[num_verts];
		for(size_t n=0; n<num_verts; n++)
		{
			cout << triangles->getP()->getValue().get(repere+vertexoffset+triangles_stride*n)+1 << " ";
			listeV[n]=triangles->getP()->getValue().get(repere+vertexoffset+triangles_stride*n)+1 ;
		}
		cout << endl;

		p3d_add_desc_face(listeV, num_verts);
		delete listeV;
		listeV=NULL;

		repere+=triangles_stride*(num_verts);
	}
}

void p3d_collada_loader::extractAndFillColor(const domMaterialRef pmat, const domNodeRef littleNode )
{
    if( !!pmat && !!pmat->getInstance_effect() ) {
        domEffectRef peffect = daeSafeCast<domEffect>(pmat->getInstance_effect()->getUrl().getElement().cast());
        if( !!peffect ) {
            domProfile_common::domTechnique::domPhongRef pphong = daeSafeCast<domProfile_common::domTechnique::domPhong>(peffect->getDescendant(daeElement::matchType(domProfile_common::domTechnique::domPhong::ID())));
            if( !!pphong ) {
                //if( !!pphong->getAmbient() && !!pphong->getAmbient()->getColor() ) {
                  //  geom.ambientColor = getVector4(pphong->getAmbient()->getColor()->getValue());
                //}
                if( !!pphong->getDiffuse() && !!pphong->getDiffuse()->getColor() ) {
                    domFx_color& diffuseColor = pphong->getDiffuse()->getColor()->getValue();
                    // TODO : question
                    // A quoi sert le dernier indice des couleurs ??
                    cout << "p3d_set_prim_color " << littleNode->getID() << " Any " << diffuseColor.get(0) << " " << diffuseColor.get(1) << " " << diffuseColor.get(2) << endl;
            		p3d_poly *poly;
            		double color_vect[3];
            		poly = p3d_poly_get_poly_by_name ((char *)littleNode->getID());
            		for(int i=0; i<3; i++)
            			color_vect[i]=diffuseColor.get(i);
            		p3d_poly_set_color(poly, Any, color_vect);
                }
            }
        }
    }
}
