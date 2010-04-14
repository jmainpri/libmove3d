#include "RRTExpansion.h"

#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

RRTExpansion::RRTExpansion() :
        BaseExpansion()
{
}

RRTExpansion::RRTExpansion(Graph* ptrGraph) :
        BaseExpansion(ptrGraph)
{
}

RRTExpansion::~RRTExpansion()
{
}

shared_ptr<Configuration> RRTExpansion::getExpansionDirection(
        Node* expandComp, Node* goalComp, bool samplePassive,
        Node*& directionNode)
{

    if (p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH)
    {

        shared_ptr<Configuration> q = mGraph->getRobot()->shootDir(
                samplePassive);

        p3d_addConfig(mGraph->getRobot()->getRobotStruct(),
                      q->getConfigStruct(),
                      expandComp->getCompcoStruct()->dist_nodes->N->q,
                      q->getConfigStruct());

        return (q);

    }

    shared_ptr<Configuration> q;
    int savedRlg;

    if (IsDirSampleWithRlg)
    {
        // Save the previous Rlg setting to shoot without Rlg
        savedRlg = p3d_get_RLG();
        p3d_set_RLG(false);
    }

    // Selection in the entire CSpace and
    // biased to the Comp of the goal configuration
    if (ENV.getBool(Env::isGoalBiased) && p3d_random(0., 1.) <= ENV.getDouble(Env::Bias))
    {
        // select randomly a node in the goal component as direction of expansion
        directionNode = mGraph->randomNodeFromComp(goalComp);
        q = directionNode->getConfiguration();
    }
    else
    {
        switch (ExpansionDirectionMethod)
        {
            case SUBREGION_CS_EXP:
                // Selection in a subregion of the CSpace
                // (typically close to the current tree)
                // and  biased to the goal configuration
                q = shared_ptr<Configuration> (
                        new Configuration(mGraph->getRobot()));

                p3d_shoot_inside_box(mGraph->getRobot()->getRobotStruct(),
                                     /*expandComp->getConfiguration()->getConfigStruct(),*/
                                     q->getConfigStruct(), expandComp->getCompcoStruct()->box_env_small,
                                     (int) samplePassive);
            break;

          case GLOBAL_CS_EXP:
              default:
                // Selection in the entire CSpace
                q = mGraph->getRobot()->shoot(samplePassive);
        }
    }
    if (!IsDirSampleWithRlg)
    {
        //Restore the previous Rlg setting
        p3d_set_RLG(savedRlg);
    }
    return (q);
}

Node* RRTExpansion::getExpansionNode(Node* compNode, shared_ptr<Configuration> direction, int distance)
{
//    cout << "Distance == " << distance << endl;

    if (p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH)
    {
        return mGraph->getNode(compNode->getCompcoStruct()->dist_nodes->N);
    }

    int KNearest = -1;
    int NearestPercent;

    switch (distance)
    {

    case NEAREST_EXP_NODE_METH:
        /* Choose the nearest node of the componant*/
        return (mGraph->nearestWeightNeighbour(compNode, direction,
                                               p3d_GetIsWeightedChoice(), distance));

    case K_NEAREST_EXP_NODE_METH:
        /* Select randomly among the K nearest nodes of a componant */
        NearestPercent = kNearestPercent;
        KNearest
                = MAX(1,(int)((NearestPercent*(compNode->getCompcoStruct()->nnode))/100.));
        // TODO : fix
        //   ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ,
        // KNearest);

        return (mGraph->nearestWeightNeighbour(compNode, direction,
                                               p3d_GetIsWeightedChoice(), distance));

    case BEST_SCORE_EXP_METH:
        /* Select the node which has the best score: weight*dist */
        return (mGraph->nearestWeightNeighbour(compNode, direction,
                                               p3d_GetIsWeightedChoice(), distance));

    case K_BEST_SCORE_EXP_METH:
        NearestPercent = kNearestPercent;
        KNearest
                = MAX(1,(int)((NearestPercent*(compNode->getCompcoStruct()->nnode))/100.));
        // TODO : fix
        // ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ, KNearest);
        return (mGraph->nearestWeightNeighbour(compNode, direction,
                                               p3d_GetIsWeightedChoice(), distance));

    case RANDOM_IN_SHELL_METH:
        /* Select randomly among all the nodes inside a given portion of shell */
        return (mGraph->getNode(hrm_selected_pb_node(mGraph->getGraphStruct(),
                                                     direction->getConfigStruct(), compNode->getNodeStruct()->comp)));

    case RANDOM_NODE_METH:
        return (mGraph->getNode(p3d_RandomNodeFromComp(
                compNode->getCompcoStruct())));

    default:
        /* By default return the nearest node of the componant */
        return (mGraph->nearestWeightNeighbour(compNode, direction,
                                               p3d_GetIsWeightedChoice(), distance));
    }
}

int RRTExpansion::expandProcess(Node* expansionNode, shared_ptr<
                                Configuration> directionConfig, Node* directionNode,
                                Env::expansionMethod method)
{
    bool extensionSucceeded(false);
    bool failed(false);
    int nbCreatedNodes(0);
    Node fromNode(*expansionNode);
    Node* extensionNode(NULL);
    shared_ptr<LocalPath> directionLocalpath;
    double positionAlongDirection(0.);
    shared_ptr<LocalPath> extensionLocalpath;
    bool firstIteration(true);

    // Perform extension toward directionConfig
    // Additional nodes creation in the nExtend case, but without checking for expansion control
    while (firstIteration || (method == Env::nExtend && !failed
                              && positionAlongDirection < 1.))
    {
        directionLocalpath = shared_ptr<LocalPath> (new LocalPath(
                fromNode.getConfiguration(), directionConfig));

        extensionSucceeded = this->nextStep(*directionLocalpath, directionNode,
                                            positionAlongDirection, extensionLocalpath, method);

//        cout << directionLocalpath->length() << endl;
//        cout << positionAlongDirection << endl;

        failed |= !extensionSucceeded;

        //                if(failed)
        //                {
        //                    cout << " Path not valid" << endl;
        //                }

        // Expansion Control
        if (firstIteration && !failed)
        {
            if (ENV.getBool(Env::expandControl)
                && !this->expandControl(*directionLocalpath,
                                        positionAlongDirection, *expansionNode))
                {
                failed = true;
            }
        }
        // Add node to graph if everything succeeded
        if (!failed)
        {
            extensionNode = addNode(&fromNode, *extensionLocalpath,
                                    positionAlongDirection, directionNode,
                                    nbCreatedNodes);

//            extensionNode->getConfiguration()->setConstraints();
        }
        if (firstIteration && failed)
        {
            this->expansionFailed(*expansionNode);
        }

        if (!failed)
        {
            fromNode = *extensionNode;
        }
        firstIteration = false;

    }
    directionNode = extensionNode;
    return nbCreatedNodes;
}

bool RRTExpansion::expandToGoal(Node* expansionNode,
                                std::tr1::shared_ptr<Configuration> directionConfig)
{
    return false;
}
