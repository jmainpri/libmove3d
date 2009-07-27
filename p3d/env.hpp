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
    /*number of consecutive times the optimization
    of a cost trajectory fails before we stop the
    optimization*/
    /*WARNING: Currently, this parameter is also
    used for other applications:
    - Set the speed of the threshold increase in
    the MAXIMAL_THRESHOLD variant
    - Set the temperature in the MONTE_CARLO_SEARCH*/
    maxCostOptimFailures,
    /*!< number of consecutive times the optimization
    of a cost trajectory fails before we stop the
    optimization*/
    maxNodeCompco,
    /*!< max number of node in a compco*/
    maxNode,
    /*!< max number of node in graph*/
    NbTry,
    /*!< max number of try by step*/

    MaxExpandNodeFail,
    MaxPassiveExpand,
    DistConfigChoice,
    ExpansionNodeMethod,
    CostMethodChoice
  };
  enum doubleParameter {
    extensionStep,
    /*!< the extension length in the extend method is equal to
    mExtensionStep*Dmax*/
    temperatureRate,
    /*!< Controls the increasement of the temperature in Cost Spaces.*/
    initialTemperature,
    /*!< Temperature parameter of the T-RRT algorithm.
    (by analogy with simulated annealing methods)*/
    alpha,

    /*Variables Hri*/
    zone_size,
    coeffPen,
    coeffDis,
    /*Variables pour ML-RRT*/
    manhatRatio,
    dist
    /*!< la distance max de connection*/
  };
  enum boolParameter {
    expandToGoal,
    /*!< Controls wether the diffusion method expand towards a given goal.*/
    expandBalanced,
    /*!< Controls wether the expansion process for bi-RRT is balanced.*/
    biDir,
    /*!< Controls wether the diffusion method is mono or bidirectional (bi-RRT).*/
    minimize,
    addCycles,
    /*!< Controls wether cycles are added during the diffusion process.*/
    /*Currently, a cycle is added if from a new node,
    we can find  another node that is far in the graph structure.*/
    expandControl,
    /*!< Wether the exporation controls the refinement vs expansion nodes*/
    discardNodes,
    /*!< Wether nodes are discarded for selection in RRT,
    after a maximum number of allowed failures.*/
    isManhattan,
    /*!< Manhattan-like RRT*/
    /*In a first step only the active parameters are expanded
    then we try to expand the passive parameter by recursivly
    expanding  only the passive parameters which were in collision
    during the previsous expansion*/
    drawGraph,
    /*!< affiche le Graph*/
    isCostSpace,
    isPasExtWhenAct,
     /* Variables Hri*/
    notUseHriDis,
    notUseHriPen,
    enable,
    ligandExitTrajectory,
    /*!< Ligand exit trajectory simulation*/
    /*The rrt expansion is stopped when the distance from start
    reaches a certain value.*/
    useDist
    /*!< indique si l'on prend en compte la distance max de connection ou non*/
  };
  enum expansionMethod {
    Connect,
    /*!< méthode d'expansion de type Connect*/
    nExtend,
    /*!< méthode d'expansion de type nExtend*/
    Extend
    /*!< méthode d'expansion de type Extend*/
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
  std::map<doubleParameter, doubleContainer*> mDoubleMap;
  std::map<boolParameter, boolContainer*> mBoolMap;
  typedef std::pair<intParameter, intContainer*> intMap_t;
  typedef std::pair<doubleParameter, doubleContainer*> doubleMap_t;
  typedef std::pair<boolParameter, boolContainer*> boolMap_t;

  /*The method used to expand a node toward a direction
  configuration selected as direction of expansion*/
  expansionMethod mExpansionMethod;

};

extern Env ENV;

#endif
