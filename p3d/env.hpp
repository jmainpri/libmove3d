#ifndef ENV_HPP
#define ENV_HPP

#ifdef QT_LIB
#include <QtCore/QObject>
#endif

#include <map>
#include <utility>

class intContainer
#ifdef QT_LIB
: public QObject
#endif
{
#ifdef QT_LIB
  Q_OBJECT;
#endif
  int _Value;

public:
  intContainer(int v = false);
  int get();

#ifdef QT_LIB
public slots:
#endif
  void set(int v);

#ifdef QT_LIB
signals:
#endif
  void valueChanged(int v);
};

class doubleContainer
#ifdef QT_LIB
: public QObject
#endif
{
#ifdef QT_LIB
  Q_OBJECT;
#endif
  double _Value;

public:
  doubleContainer(double v = false);
  double get();

#ifdef QT_LIB
public slots:
#endif
  void set(double v);

#ifdef QT_LIB
signals:
#endif
  void valueChanged(double v);
};

class boolContainer
#ifdef QT_LIB
: public QObject
#endif
{
#ifdef QT_LIB
  Q_OBJECT;
#endif
  bool _Value;

public:
  boolContainer(bool v = false);
  bool get();
#ifdef QT_LIB
public slots:
#endif
  void set(bool v);

#ifdef QT_LIB
signals:
#endif
  void valueChanged(bool v);
};

/**
 * @ingroup Interface
 * @brief String Container
 */
class stringContainer : public QObject
{
#ifdef QT_LIB
  Q_OBJECT;
#endif
  std::string _Value;

public:
  stringContainer(std::string v = "");
  std::string get();

#ifdef QT_LIB
public slots:
#endif
  void set(std::string v);

#ifdef QT_LIB
signals:
#endif
  void valueChanged(std::string v);
};

/**
	\brief Classe permettant de se lier à l'interface graphique
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Env
#ifdef QT_LIB
: public QObject
#endif
{
#ifdef QT_LIB
  Q_OBJECT;
#endif
public:
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
	CostMethodChoice
  };

  enum stringParameter {
	  nameOfFile
    };

  enum doubleParameter {

	// the extension length in the extend method is equal to
    // mExtensionStep*Dmax
    extensionStep,

    // Controls the increasement of the temperature in Cost Spaces.
    temperatureRate,
    temperature,

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
	dist
  };
  enum boolParameter {
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
    drawGraph,
    drawTraj,
    drawAll,
    // Variables Hri
    costEnv,
    useHriDis,
    useHriPen,
    useHriNat,
    enableHri,
    computeGrid,
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
    debugCostOptim,
	isCostSpace,
	isPasExtWhenAct,
	useDist
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
#ifdef QT_LIB
  QObject* getObject(intParameter p);
#endif

  /**
   * obtient le QObject correspondant à un paramètre bool
   * @param p le paramètre
   * @return le QObject
   */
#ifdef QT_LIB
  QObject* getObject(boolParameter p);
#endif

  /**
   * obtient le QObject correspondant à un paramètre double
   * @param p le paramètre
   * @return le QObject
   */
#ifdef QT_LIB
  QObject* getObject(doubleParameter p);
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

#ifdef QT_LIB
public slots:
#endif
  /**
   * stocke le type de méthode d'expansion
   * @param method le type de méthode d'expansion
   */
  void setExpansionMethodSlot(int method);

#ifdef QT_LIB
signals:
#endif
  /**
   * signal émis lorsque le type de méthode d'expansion change
   * @param method le nouveau type de méthode d'expansion
   */
  void expansionMethodChanged(int method);


protected:
  std::map<intParameter, intContainer*> mIntMap;
  std::map<stringParameter, stringContainer*> mStringMap;
  std::map<doubleParameter, doubleContainer*> mDoubleMap;
  std::map<boolParameter, boolContainer*> mBoolMap;
  typedef std::pair<intParameter, intContainer*> intMap_t;
  typedef std::pair<stringParameter, stringContainer*> stringMap_t;
  typedef std::pair<doubleParameter, doubleContainer*> doubleMap_t;
  typedef std::pair<boolParameter, boolContainer*> boolMap_t;

  /*The method used to expand a node toward a direction
  configuration selected as direction of expansion*/
  expansionMethod mExpansionMethod;

};

extern Env ENV;

#endif
