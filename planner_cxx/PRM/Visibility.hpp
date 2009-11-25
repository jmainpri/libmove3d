#ifndef VIS_PRM_HPP
#define VIS_PRM_HPP

#include "PRM.hpp"
/**
	\brief Classe représentant l'algorithme Vis_PRM
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Vis_PRM : public PRM
{
public:
    /**
     * Constructeur de la classe
     * @param WS Le WorkSpace de l'application
     */
    Vis_PRM(Robot* R, Graph* G);

    /**
     * Destructeur de la classe
     */
    ~Vis_PRM();

    /**
     * fonction principale de l'algorithme Vis_PRM
     * @param Graph_Pt le graphPt affiché
     * @param (*fct_stop)(void) la fonction d'arret
     * @param (*fct_draw)(void) la fonction d'affichage
     * @return le nombre de Node ajoutés au Graph
     */
    uint expand(p3d_graph* Graph_Pt,int (*fct_stop)(void), void (*fct_draw)(void));
};

#endif
