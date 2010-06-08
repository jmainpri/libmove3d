#ifndef CPP_GRAPH_HPP
#define CPP_GRAPH_HPP

#include "../planningAPI.hpp"

/**
  @ingroup ROADMAP
  \brief Graph class
*/
class cpp_Graph{

public:
    //contructors and destructor
    /**
     * Constructeur de la classe
     * @param G la structure p3d_graph qui sera stockée
     * @param R le Robot pour lequel le Graph est créé
     */
    cpp_Graph(Robot* R, p3d_graph* ptGraph = NULL);

    /**
     * Frees the graph nodes and edges
     */
    ~cpp_Graph();
	
	/**
	 * import p3d graph struct
	 */
	void import(p3d_graph* G);

	/**
	 * export p3d graph struct
	 */
	p3d_graph* exportGraphStruct();
	
	/**
	 * Returns the Robot structure
	 */
	Robot* getRobot() { return m_Robot; };
	

private:
	/**
	 * Frees graph
	 */
    void freeResources();
	
	/**
	 * Puts a name to the graph
	 */
	void setName() { m_Name = "The graph"; } 
	
private:
    Robot* m_Robot;

    std::vector<Node*> m_Nodes;
    std::vector<Edge*> m_Edges;
	std::vector<ConnectedComponent*> m_Comp;

    Node* m_Start;
    Node* m_Goal;

    std::string m_Name;

};

#endif
