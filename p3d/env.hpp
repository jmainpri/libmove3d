/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#ifndef ENV_HPP
#define ENV_HPP

#ifdef QT_LIBRARY
#undef CursorShape
#include <QtCore/QObject>
#endif

#include <map>
#include <utility>
#include <string>
#include <vector>

#include "ParametersEnv.hpp"

/**
 \brief Classe permettant de se lier à l'interface graphique
 @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */
class Env
        #ifdef QT_LIBRARY
        : public QObject
        #endif
{
public:

#ifdef QT_LIBRARY
    Q_OBJECT;
    Q_ENUMS(boolParameter);
    Q_ENUMS(intParameter);
    Q_ENUMS(doubleParameter);
    Q_ENUMS(stringParameter);
    Q_ENUMS(vectorParameter);
    Q_ENUMS(expansionMethod);
#endif

public:

    enum boolParameter {
        // Controls wether the C++ planner maps the C Graph or not
        // Only works with RRT
        use_p3d_structures,
        // Controls wether the planner is PRM or Diffusion
        isPRMvsDiffusion,
        isGoalBiased,
        treePlannerIsEST,
        // Controls wether the diffusion method expand towards a given goal.
        expandToGoal,
        // Controls wether the expansion process for bi-RRT is balanced.
        expandBalanced,
        // Controls wether the diffusion method is mono or bidirectional (bi-RRT).
        biDir,
        minimize,
        // Controls wether cycles are added during the diffusion process.
        // Currently, a cycle is added if from a new node,
        // we can find  another node that is far in the graph structure.
        addCycles,
        // Wether the exporation controls the refinement vs expansion nodes
        expandControl,
        refinementControl,
        // Wether nodes are discarded for selection in RRT,
        // after a maximum number of allowed failures.
        discardNodes,
        //enable or disable edge validation (used in RRT only)
        findLowCostConf,
        // Manhattan-like RRT
        // In a first step only the active parameters are expanded
        // then we try to expand the passive parameter by recursivly
        // expanding  only the passive parameters which were in collision
        // during the previsous expansion
        isManhattan,
        isMultiRRT,
        drawDisabled,
        drawFrame,
        drawExploration,
        drawGraph,
        drawEdges,
        drawTraj,
        drawOTPTraj,
        drawTrajVector,
        drawAll,
        drawLightSource,
        drawGrid,
        drawEntireGrid,
        drawDistance,
        drawPoints,
        drawGaze,
        drawBox,
        drawOnlyOneLine,
        drawVectorField,
        drawMultiColorLocalpath,
        // Variables Hri
        useHriDis,
        useHriPen,
        useHriNat,
        enableHri,
        computeGrid,
        HRIPlannerTS,
        HRIPlannerWS,
        HRIPlannerCS,
        HRIPlannerTRRT,
        HRIPathDistance,
        HRIleftArmVsRightArm,
        HRIcameraBehindHuman,
        HRINoRobot,
        HRIComputeOTP,
        HRIAutoLoadGrid,

        // Variable Visualisation
        printTemp,
        printRadius,
        printNbQRand,
        printCollFail,
        printCostFail,

        // Ligand exit trajectory simulation :
        // The rrt expansion is stopped when the distance from start
        // reaches a certain value.
        ligandExitTrajectory,
        useRefiRadius,

        // Cost Traj && Cost Space
        setActiveJointsGroup,
        setStompPlanner,
        debugCostOptim,
        trajCostRecompute,
        isCostSpace,
        isPasExtWhenAct,
        useDist,
        costBeforeColl,
        costExpandToGoal,
        costThresholdRRT,
        costThresholdPlanner,
        costStarRRT,

        //OTP
        FastComputingRobotBase,
        DrawRobotBaseGridCosts,

        // Smoothing stage
        withMaxIteration,
        withGainLimit,
        withSmoothing,
        withShortCut,
        withDeformation,
        saveTrajCost,

        withCleaning,

        useTRRT,
        useBoxDist,
        useBallDist,
        isInverseKinematics,
        isRunning,
        initPlot,
        isWeightedRotation,
        randomConnectionToGoal,
        tryClosest,
        StopMultiRun,
        tRrtComputeGradient,
        FKShoot,
        FKDistance,
        RecomputeCellCost,
        UseDPGGrids,
        startWithFKCntrt,
        showOneCell
#ifdef MULTILOCALPATH
        ,plotSoftMotionCurve,
        writeSoftMotionFiles,
        smoothSoftMotionTraj
#endif
    };

    enum intParameter {
        // number of consecutive times the optimization
        // of a cost trajectory fails before we stop the
        // optimization
        // WARNING: Currently, this parameter is also
        // used for other applications:
        // - Set the speed of the threshold increase in
        // the MAXIMAL_THRESHOLD variant
        // - Set the temperature in the MONTE_CARLO_SEARCH
        PRMType,
        nbOfSeeds,
        jntToDraw,
        maxCostOptimFailures,
        nbQRand,
        nbCostTransFailed,
        nbCollExpanFailed,
        nbCostOptimize,
        nbGreedyTraj,
        maxNodeCompco,
        maxNode,
        maxConnect,
        NbTry,
        MaxExpandNodeFail,
        MaxPassiveExpand,
        DistConfigChoice,
        ExpansionNodeMethod,
        costMethodChoice,
        test,
        nbRound,
        nbMultiRun,
        nbMultiSmooth,
        akinJntId,
        heightFactor,
        progress,
        costDeltaMethod,
        hriCostType,
        hriActiveGrid,
        hriShownGridLine,
        tRrtNbtry,
        cellToShow,
        lineToShow,
        nbCells,

        //OTP
        typeRobotBaseGrid, // 0, 1, 2 or 3 for all, dist only, robot dist only and visual field only
        setOfActiveJoints,
        currentArmId
    };

    enum doubleParameter {
        // Mirrors the env dmax
        dmax,
        // Frame per seconds in the QT interface
        FPS,
        showTrajFPS,
        // the extension length in the extend method is equal to
        // mExtensionStep*Dmax
        extensionStep,
        costStep,
        costThreshold,

        // Controls the increasement of the temperature in Cost Spaces.
        temperatureRate,
        temperatureStart,
        temperatureGoal,

        // Temperature parameter of the T-RRT algorithm.
        // (by analogy with simulated annealing methods)
        initialTemperature,
        alpha,

        // Variables Hri
        zone_size,
        coeffPen,
        coeffDis,
        coeffNat,
        coeffJoint,
        coeffEnerg,
        coeffConfo,
        coeffArmPr,
        multCost,
        Kdistance,
        Kvisibility,
        Knatural,
        Kreachable,
        KlengthWeight,

        // Optimization Variables
        MaxFactor,
        MinStep,

        //Other variables
        refiRadius,
        manhatRatio,
        dist,
        visThresh,
        PlanCellSize,
        CellSize,
        Bias,
        RotationWeight,
        colorThreshold1,
        colorThreshold2,
        findLowCostThreshold,
        bestCost,
        costMax,
        minimalFinalExpansionGap,
        //Optimisation time
        timeOptimize,

        //OTP
        optimalDist,
        robotMaximalDist,
        gazeAngle,

        optimalDistFactor,
        robotMaximalDistFactor,
        gazeAngleFactor
    };

    enum stringParameter {
        nameOfFile,
        numberOfCollisionPerSec,
        numberOfLocalPathPerSec,
        numberOfCostPerSec,
        ObjectToCarry,
        ActiveGrid
    };

    enum  vectorParameter {
        costAlongTraj
    };

    enum expansionMethod {
        Extend,
        nExtend,
        Connect,
        costConnect
    };

    /**
   * Constructeur de la classe
   */
    Env();

    /**
   * Destructeur de la classe
   */
    ~Env();

    /**
   * get the value of a string parameter
   * @param p le paramètre
   * @return la valeur
   */
    std::string getString(stringParameter p);

    /**
   * modifies the value of the string
   * @param p le paramètre
   * @param v la nouvelle valeur
   */
    void setString(stringParameter p, std::string v);

    /**
   * get the value of a vector parameter
   * @param p le paramètre
   * @return la valeur
   */
    std::vector<double> getVector(vectorParameter p);

    /**
   * modifies the value of the vector
   * @param p le paramètre
   * @param v la nouvelle valeur
   */
    void setVector(vectorParameter p, std::vector<double> v);

    /**
   * obtient la valeur d'un paramètre int
   * @param p le paramètre
   * @return la valeur
   */
    int getInt(intParameter p);

    /**
   * modifie la valeur d'un paramètre int
   * @param p le paramètre
   * @param v la nouvelle valeur
   */
    void setInt(intParameter p, int v);

    /**
   * obtient la valeur d'un paramètre double
   * @param p le paramètre
   * @return la valeur
   */
    double getDouble(doubleParameter p);

    /**
   * modifie la valeur d'un paramètre double
   * @param p le paramètre
   * @param v la nouvelle valeur
   */
    void setDouble(doubleParameter p, double v);

    /**
   * obtient la valeur d'un paramètre bool
   * @param p le paramètre
   * @return la valeur
   */
    bool getBool(boolParameter p);

    /**
   * modifie la valeur d'un paramètre bool
   * @param p le paramètre
   * @param v la nouvelle valeur
   */
    void setBool(boolParameter p, bool v);

    /**
   * obtient le QObject correspondant à un paramètre int
   * @param p le paramètre
   * @return le QObject
   */
#ifdef QT_LIBRARY
    QObject* getObject(intParameter p);
#endif

    /**
   * obtient le QObject correspondant à un paramètre bool
   * @param p le paramètre
   * @return le QObject
   */
#ifdef QT_LIBRARY
    QObject* getObject(boolParameter p);
#endif

    /**
   * obtient le QObject correspondant à un paramètre double
   * @param p le paramètre
   * @return le QObject
   */
#ifdef QT_LIBRARY
    QObject* getObject(doubleParameter p);
#endif

    /**
   * obtient le QObject correspondant à un paramètre double
   * @param p le paramètre
   * @return le QObject
   */
#ifdef QT_LIBRARY
    QObject* getObject(stringParameter p);
#endif

    /**
   * obtient le QObject correspondant à un paramètre double
   * @param p le paramètre
   * @return le QObject
   */
#ifdef QT_LIBRARY
    QObject* getObject(vectorParameter p);
#endif


    /**
   * modifie le type de méthode d'expansion stocké
   * @param method le nouveau type stocké
   */
    void setExpansionMethod(expansionMethod method);

    /**
   * obtient le type de méthode d'expansion stocké
   * @return le type de méthode d'expansion stocké
   */
    expansionMethod getExpansionMethod();


    /**
   * Maps Getters and Setters
   */
    typedef std::pair<intParameter, intContainer*> intMap_t;
    typedef std::pair<stringParameter, stringContainer*> stringMap_t;
    typedef std::pair<vectorParameter, vectorContainer*> vectorMap_t;
    typedef std::pair<doubleParameter, doubleContainer*> doubleMap_t;
    typedef std::pair<boolParameter, boolContainer*> boolMap_t;

    std::map<intParameter, intContainer*>			getIntMap() {return mIntMap; }
    std::map<stringParameter, stringContainer*> 	getStringMap() {return mStringMap; }
    std::map<vectorParameter, vectorContainer*> 	getVectorMap() {return mVectorMap; }
    std::map<doubleParameter, doubleContainer*> 	getDoubleMap() {return mDoubleMap; }
    std::map<boolParameter, boolContainer*>			getBoolMap() {return mBoolMap; }


#ifdef QT_LIBRARY
public slots:
#endif
    /**
   * stocke le type de méthode d'expansion
   * @param method le type de méthode d'expansion
   */
    void setExpansionMethodSlot(int method);

#ifdef QT_LIBRARY
signals:
#endif
    /**
   * signal émis lorsque le type de méthode d'expansion change
   * @param method le nouveau type de méthode d'expansion
   */
    void expansionMethodChanged(int method);


private:
    std::map<intParameter,		intContainer*>				mIntMap;
    std::map<stringParameter, stringContainer*>			mStringMap;
    std::map<vectorParameter, vectorContainer*>			mVectorMap;
    std::map<doubleParameter, doubleContainer*>			mDoubleMap;
    std::map<boolParameter,		boolContainer*>				mBoolMap;


    /*The method used to expand a node toward a direction
     configuration selected as direction of expansion*/
    expansionMethod mExpansionMethod;

};

extern Env ENV;

#endif
