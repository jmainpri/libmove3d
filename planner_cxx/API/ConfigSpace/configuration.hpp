#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include "../planningAPI.hpp"
#include "other_libraries/Eigen/Geometry"

class Node;
/**
  * @ingroup CPP_API
  * @defgroup CONFIG_SPACE Configuration space
  * @brief C-Space make generic motion planners possible
  */

/**
        @ingroup CONFIG_SPACE
        @brief Classe représentant une Configuration d'un Robot
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Configuration{

public:
  //constructor and destructor
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructeur de la classe
     * @param R le Robot pour lequel la Configuration est créée
     */
    Configuration(Robot* R);

    /**
     * Constructeur de la classe
     * @param R le Robot pour lequel la Configuration est créée
     * @param C la structure de Configuration qui sera stockée
     * @param noCopy if set to true, _Configuration is set to C,
     * otherwise a copy of C is made.
     */
  Configuration(Robot* R, configPt C, bool noCopy = false);

    /**
      * Copy constructor of the class
      * @param confguration
      **/
    Configuration(const Configuration& conf);

    /**
     * Destructeur de la classe
     */
    ~Configuration();

    /**
     * détruie la configPt stockée
     */
    void Clear();

  //Accessors
    /**
     * obtient le Robot pour lequel la Configuration est créée
     * @return le Robot pour lequel la Configuration est créée
     */
    Robot* getRobot();

    /**
     * Gets the quaternion
     * @return le vecteur des Quaternions
     */
    Eigen::Quaterniond getQuaternion();

    /**
      * Sets EulersAngles
      */
    void setQuaternionsToEuler();

    /**
     * obtient le pointeur sur la ConfigPt
     * @return la pointeur sur la ConfigPt
     */
    configPt getConfigStruct();

    /**
     * modifie la structure configPt stockée
     * @param C la nouvelle structure configPt
     */
    void setConfiguration(configPt C);
    /**
     * modifie la structure configPt stockée
     * @param C la Configuration contentant la nouvelle structure
     */
    void setConfiguration(Configuration& C);

    /**
     * indique si le vecteur de Quaternions est initialisé
     * @return le vecteur de Quaternions est initialisé
     */
//    bool isQuatInit();

    /**
     * initialise le vecteur de Quaternions
     */
//    void initQuaternions();

    /**
      * Set Quaternions
      */
//    void initQuaternions(int quatDof,Eigen::Quaternion<double> quat);

    /**
     * Convert Configuration in radian
     */
    void convertToRadian();

    /**
     * calcule la distance à une Configuration
     * @param Conf la Configuration entrée
     * @return la distance
     */
    double dist(Configuration& Conf);

    /**
     * calcule la distance à une Configuration
     * @param q la Configuration entrée
     * @param distChoice le type de calcul de distance
     * @return la distance
     */
    double dist(Configuration& q, int distChoice);

    /**
     * indique si la Configuration est en collision
     * @return la Configuration est en collision
     */
    bool IsInCollision();


    double distEnv();

    /**
     * compare à une autre Configuration
     * @param Conf la Configuration entrée
     * @return les deux Configurations sont égales
     */
    bool equal(Configuration& Conf);

    /**
     * Compare tow configurations
     */
    bool operator==(Configuration& Conf) { return this->equal(Conf); }

    /**
      * Compare tow configurations
      */
    bool operator!=(Configuration& Conf) { return !(this->equal(Conf)); }

    /**
     * copie une Configuration
     * @return une copie de la Configuration
     */
    std::tr1::shared_ptr<Configuration> copy();

    /**
     * copie les joints passifs de la Configuration courante dans la Configuration entrée
     * @param C in/out la Configuration à modifier
     */
    void copyPassive(Configuration& C);

    /**
     * obtient le cout de la Configuration suivant l'espace des fonctions de cout
     * @return le cout de la Configuration
     */
    double cost();

    /**
	 * Sets the configuration to respect robot constraints
	 */
    bool setConstraints();

    /**
     *
     */
    std::tr1::shared_ptr<Configuration> add(Configuration& C);

    /**
      * Adds tow configurations
      */
    std::tr1::shared_ptr<Configuration> operator+(Configuration& Conf) { return this->add(Conf); }

    /**
     * Checks that Configruation is not out of bands
     */
    bool isOutOfBands();

    /**
     *
     */
    void print();


private:
        bool _flagInitQuaternions;/*!< Booleen indiquant que les Quaternions ont été initialisés*/
        int _QuatDof;
//        Eigen::Quaterniond _Quaternions;

	bool _CollisionTested;
	bool _InCollision;

	bool _CostTested;
	double _Cost;

	Robot* _Robot;/*!< Le Robot pour lequel la Configuration est créée*/
	configPt _Configuration;/*!< une structure de congitPt contenant les données sur la Configuration*/
};

#endif
