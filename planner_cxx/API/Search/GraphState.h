#ifndef GRAPHSTATE_HPP
#define GRAPHSTATE_HPP

#include "AStar/State.h"
#include "Planner-pkg.h"

class GraphState : public State
{
public:
    GraphState();
    GraphState(p3d_node* n);

    std::vector<State*> getSuccessors();

    p3d_node* getGraphNode() { return _GraphNode; }

    bool equal(State* other);
    bool isLeaf();
    void print();

    void setClosed(std::vector<State*>& closedStates,std::vector<State*>& openStates);
    bool isColsed(std::vector<State*>& closedStates);

    void setOpen(std::vector<State*>& openStates);
    bool isOpen(std::vector<State*>& openStates);

protected:
    double computeLength(State *parent);       /* g */
    double computeHeuristic(State *parent);    /* h */

private:
    p3d_node* _GraphNode;
};

#endif // GRAPHSTATE_HPP
