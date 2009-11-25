#ifndef ACR_HPP
#define ACR_HPP

#include "PRM.hpp"
/**
	\brief Classe représentant l'algorithme ACR
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class ACR : public PRM
{
public:
    /**
     * Constructeur de la classe
     * @param WS Le WorkSpace de l'application
     */
    ACR(Robot* R, Graph* G);

    /**
     * Destructeur de la classe
     */
    ~ACR();

    /**
     * fonction principale de l'algorithme ACR
     * @param Graph_Pt le graphPt affiché
     * @param (*fct_stop)(void) la fonction d'arret
     * @param (*fct_draw)(void) la fonction d'affichage
     * @return le nombre de Node ajoutés au Graph
     */
    uint expand(p3d_graph* Graph_Pt,int (*fct_stop)(void), void (*fct_draw)(void));

};

#endif
