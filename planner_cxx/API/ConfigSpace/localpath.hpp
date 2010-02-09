#ifndef LOCALPATH_HPP
#define LOCALPATH_HPP

#include "../planningAPI.hpp"

class Graph;

/**
 @ingroup CONFIG_SPACE
 \brief Classe représentant un chemin local
 @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */
class LocalPath {

public:
	/**
	 * constructors and destructors
	 */

	/**
	 * Class Constructor
	 * The type is linear by default
	 * @param B la Configuration initiale du LocalPath
	 * @param E la Configuration finale du LocalPath
	 */
	LocalPath(std::tr1::shared_ptr<Configuration> B, std::tr1::shared_ptr<
			Configuration> E);

	/*contruit un localpath a partir d'un localpath et d'un parametre*/
	/**
	 * constructeur de la classe
	 * @param path un LocalPath
	 * @param p in/out le paramètre correspondant à la dernière Configauration valide
	 */
	LocalPath(LocalPath& path, double& p);

	/**
	 * Copy constructor
	 * @param path a LocalPath
	 */
	LocalPath(const LocalPath& path);

	/**
	 * Constructor from a struct
	 * @param p3d struct
	 */
	LocalPath(Robot* R,p3d_localpath* lpPtr);

	/**
	 * Destructor
	 */
	~LocalPath();

	//Accessor
	/**
	 * obtient la structure p3d_localpath stockée
	 * @return la structure p3d_localpath stockée
	 */
	p3d_localpath* getLocalpathStruct();

	/**
	 * obtient la configuration initiale
	 * @return la configuration initiale
	 */
	std::tr1::shared_ptr<Configuration> getBegin();
	/**
	 * obtient la configuration finale
	 * @return la configuration finale
	 */
	std::tr1::shared_ptr<Configuration> getEnd();

	/**
	 * obtient le Graph pour lequel le LocalPath est créé
	 * @return le Graph pour lequel le LocalPath est créé
	 */
//	Graph* getGraph();
	/**
	 * obtient le Robot pour lequel le LocalPath est créé
	 * @return le Robot pour lequel le LocalPath est créé
	 */
	Robot* getRobot();

	/**
	 * teste si le LocalPath est valide
	 * @return le LocalPath est valide
	 */
	bool getValid();

	/**
	 * Returns the number of
	 * Colision test done to test the local path
	 */
	int getNbColTest();

	/**
	 * teste si le LocalPath à été évalué
	 * @return le LocalPath à été évalué
	 */
	bool getEvaluated();

	/**
	 * obtient le type de LocalPath
	 * @return le type de LocalPath
	 */
	p3d_localpath_type getType();

	/**
	 * obtient la dernière Configuration valide le long du LocalPath
	 * @param p in/out le paramètre correspondant à la dernière Configauration valide
	 * @return la dernière Configuration valide le long du LocalPath
	 */
	std::tr1::shared_ptr<Configuration> getLastValidConfig(double& p);

	/**
	 * test si le LocalPath est valide
	 * @param R le Robot pour lequel le LocalPath est testé
	 * @param ntest in/out le nombre de tests
	 * @return ! le LocalPath n'est pas valide
	 */
	bool unvalidLocalpathTest(Robot* R, int* ntest);

	/*test le localpath*/
	bool classicTest();

	/**
	 * obtient la longueur du LocaPath
	 * @return la longueur du LocalPath
	 */
	double length();

	/**
	 *
	 */
	double getParamMax();

	/**
	 * obtient une Configuration se trouvant à une distance donnée du début du LocalPath
	 * @param R le Robot pour lequel le LocalPath est créé
	 * @param dist la distance par rapport au début
	 * @return la Configuration
	 */
	std::tr1::shared_ptr<Configuration> configAtDist(double dist);
	/**
	 * obtient une Configuration se trouvant sur le LocalPath à un paramètre donnée
	 * @param R le Robot pour lequel le LocalPath est créé
	 * @param param le paramètre
	 * @return la Configuration
	 */
	std::tr1::shared_ptr<Configuration> configAtParam(double param);

	double getResolution();

	double cost();

        void resetCostComputed() { _costEvaluated = false; }

	void print();

private:
	p3d_localpath* _LocalPath;
	std::tr1::shared_ptr<Configuration> _Begin;
	std::tr1::shared_ptr<Configuration> _End;
//	Graph* _Graph;
	Robot* _Robot;

	bool _Valid;
	bool _Evaluated;
	double _lastValidParam;
	std::tr1::shared_ptr<Configuration> _lastValidConfig;
	bool _lastValidEvaluated;
	int _NbColTest;

	bool _costEvaluated;
	double _Cost;

	bool _ResolEvaluated;
	double _Resolution;

	p3d_localpath_type _Type; //type du local path(mahantan, linear ...)

};

#endif
