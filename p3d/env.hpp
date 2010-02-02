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

class intContainer
#ifdef QT_LIBRARY
        : public QObject
#endif
{
#ifdef QT_LIBRARY
    Q_OBJECT;
#endif
    int _Value;

public:
    intContainer(int v = false);
    int get();

#ifdef QT_LIBRARY
public slots:
#endif
    void set(int v);

#ifdef QT_LIBRARY
signals:
#endif
    void valueChanged(int v);
};

class doubleContainer
#ifdef QT_LIBRARY
        : public QObject
#endif
{
#ifdef QT_LIBRARY
    Q_OBJECT;
#endif
    double _Value;

public:
    doubleContainer(double v = false);
    double get();

#ifdef QT_LIBRARY
public slots:
#endif
    void set(double v);

#ifdef QT_LIBRARY
signals:
#endif
    void valueChanged(double v);
};

class boolContainer
#ifdef QT_LIBRARY
        : public QObject
#endif
{
#ifdef QT_LIBRARY
    Q_OBJECT;
#endif
    bool _Value;

public:
    boolContainer(bool v = false);
    bool get();
#ifdef QT_LIBRARY
public slots:
#endif
    void set(bool v);

#ifdef QT_LIBRARY
signals:
#endif
    void valueChanged(bool v);
};

/**
 * @ingroup Interface
 * @brief String Container
 */
class stringContainer
#ifdef QT_LIBRARY
        : public QObject
#endif
{
#ifdef QT_LIBRARY
    Q_OBJECT;

    QString _Value;

public:
    stringContainer(QString v = "");
    QString get();


public slots:
    void set(QString v);


signals:

    void valueChanged(QString v);
#endif
};

/**
 * @ingroup Interface
 * @brief String Container
 */
class vectorContainer
#ifdef QT_LIBRARY
        : public QObject
#endif
{
#ifdef QT_LIBRARY
    Q_OBJECT;
#endif
    std::vector<double> _Value;

public:
    vectorContainer() { }
    vectorContainer(std::vector<double>);
    std::vector<double> get();

#ifdef QT_LIBRARY
public slots:
#endif
    void set(std::vector<double> v);

#ifdef QT_LIBRARY
signals:
#endif
    void valueChanged(std::vector<double> v);

};

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
#endif

public:

    enum boolParameter {
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
        // Wether nodes are discarded for selection in RRT,
        // after a maximum number of allowed failures.
        discardNodes,
        // Manhattan-like RRT
        // In a first step only the active parameters are expanded
        // then we try to expand the passive parameter by recursivly
        // expanding  only the passive parameters which were in collision
        // during the previsous expansion
        isManhattan,
        drawFrame,
        drawGraph,
        drawTraj,
        drawAll,
        drawLightSource,
        drawGrid,
        drawDistance,
        drawPoints,
        // Variables Hri
        useHriDis,
        useHriPen,
        useHriNat,
        enableHri,
        computeGrid,
        HRIPlannerTS,
        HRIPlannerWS,
        HRIPlannerCS,
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
        debugCostOptim,
        trajCostRecompute,
        isCostSpace,
        isPasExtWhenAct,
        useDist,
        CostBeforeColl,
        withShortCut,
        useTRRT,
        useBoxDist,
        useBallDist,
        isInverseKinematics,
        isRunning,
        initPlot,
        isWeightedRotation,
        randomConnectionToGoal,
        tryClosest,
        StopMultiRun
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
        maxCostOptimFailures,
        nbQRand,
        nb_rounds,
        nbCostTransFailed,
        nbCollExpanFailed,
        nbCostOptimize,
        nbGreedyTraj,
        maxNodeCompco,
        maxNode,
        NbTry,
        MaxExpandNodeFail,
        MaxPassiveExpand,
        DistConfigChoice,
        ExpansionNodeMethod,
        CostMethodChoice,
        test,
        nbRound,
        akinJntId,
        heightFactor,
        progress,
        hriCostType
    };

    enum doubleParameter {
        // Frame per seconds in the QT interface
        FPS,

        // the extension length in the extend method is equal to
        // mExtensionStep*Dmax
        extensionStep,
        CostStep,

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
        coeffLim,
        coeffTas,
        coeffHei,
        multCost,
        refiRadius,
        MaxFactor,
        MinStep,
        manhatRatio,
        dist,
        Kdistance,
        Kvisibility,
        visThresh,
        CellSize,
        Bias,
        RotationWeight
    };

    enum stringParameter {
        nameOfFile,
        numberOfCollisionPerSec,
        numberOfLocalPathPerSec,
        numberOfCostPerSec,
        ObjectToCarry
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

#ifdef QT_LIBRARY
    /**
   * get the value of a string parameter
   * @param p le paramètre
   * @return la valeur
   */
    QString getString(stringParameter p);

    /**
   * modifies the value of the string
   * @param p le paramètre
   * @param v la nouvelle valeur
   */
    void setString(stringParameter p, QString v);
#endif

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
    std::map<boolParameter, boolContainer*> 		getBoolMap() {return mBoolMap; }


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
    std::map<intParameter, intContainer*> mIntMap;
    std::map<stringParameter, stringContainer*> mStringMap;
    std::map<vectorParameter, vectorContainer*> mVectorMap;
    std::map<doubleParameter, doubleContainer*> mDoubleMap;
    std::map<boolParameter, boolContainer*> mBoolMap;


    /*The method used to expand a node toward a direction
  configuration selected as direction of expansion*/
    expansionMethod mExpansionMethod;

};

extern Env ENV;

#endif
