#include <iostream>
#include "State.h"
#include "AStar.h"

using namespace std;


/**
 * Tree Element Class
 */
TreeNode::TreeNode(State *st, TreeNode *prnt=NULL)
{
    if(st!=NULL)
    {
        _State = st;
        _Parent = prnt;
    }
    else
    {
        cerr << "Error in TreeNode::TreeNode(State *st, TreeNode *prnt=NULL)\n";
        exit(1);
    }
}



/**
 * A Star
 */
vector<State*> AStar::getSolution(QueueElement q_tmp)
{
    _AStarState = FOUND;
    TreeNode *solution_leaf = q_tmp.getTreeNode();
    TreeNode *t_tmp = solution_leaf;
    int step_n = 0;

    while(t_tmp)
    {
        ++step_n;
        t_tmp = t_tmp->getParent();
    }

    t_tmp = solution_leaf;
    for(int i=step_n-1;i>=0;i--)
    {
        _Solution.push_back(t_tmp->getState());
        t_tmp = t_tmp->getParent();
    }

    return _Solution;
}


bool AStar::isGoal(State* state)
{
//    cout << "AStar:: is Goal"<< endl;
    if(_GoalIsDefined)
    {
        return _Goal->equal(state);
    }
}

void AStar::cleanStates()
{
    for(int i=0;i<_Explored.size();i++)
    {
        _Explored[i]->reset();
    }
}


vector<State*> AStar::solve(State* initialState, int& solution_n)
{
    cout << "start solve" << endl;

//    _Explored.reserve(20000);

    _AStarState = NOT_FOUND;
    _SolutionLeaf = NULL;

    //    initialState->computeCost(NULL);
    _Root = new TreeNode(initialState);

    vector<State*> closedSet;
    vector<State*> openSet;
    openSet.push_back(initialState);

    _OpenSet.push(* new QueueElement(_Root));
    _Explored.push_back(initialState);


    QueueElement q_tmp;

    while( !_OpenSet.empty() )
    {
        q_tmp = _OpenSet.top();
        _OpenSet.pop();

        State* currentState = q_tmp.getTreeNode()->getState();
//        cout << "State = "<< currentState << endl;
        currentState->setClosed(openSet,closedSet);

        /* The solution is found */
        if(currentState->isLeaf() || this->isGoal(currentState) )
        {
            cout <<  "Solution Found" << endl;
            cleanStates();
            return getSolution(q_tmp);
        }

        vector<State*> branchedStates = currentState->getSuccessors();

        for(int i=0; i<branchedStates.size(); i++)
        {
            if(branchedStates[i]!=NULL)
            {
                TreeNode* parent = q_tmp.getTreeNode()->getParent();

                if(!((parent != NULL) && (parent->getState()->equal(branchedStates[i]))))
                {
                    if(!(branchedStates[i]->isColsed(closedSet)))
                    {
                        if(!(branchedStates[i]->isOpen(openSet)))
                        {
                            branchedStates[i]->computeCost(currentState);
                            branchedStates[i]->setOpen(openSet);
                            _Explored.push_back(branchedStates[i]);
                            _OpenSet.push(*new QueueElement(new TreeNode(branchedStates[i],(q_tmp.getTreeNode()))));
                        }
                    }
                }
            }
        }
    }

    cleanStates();

    if(NOT_FOUND==_AStarState)
    {
        cerr << "The Solution does not exist\n";
        return _Solution;
    }

    return _Solution;
}
