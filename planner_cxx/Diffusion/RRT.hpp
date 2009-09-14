#ifndef RRT_HPP
#define RRT_HPP

#include "Expansion/TreeExpansionMethod.hpp"
#include "TreePlanner.hpp"
/**
 * @ingroup Diffusion
 *
 * ! \brief RRT
 *
 * This class implements the following RRT algorithms:<BR>
 * RRT, T-RRT and ML-RRT.<BR>
 * The expansion can be mono- or bi-directional,
 * with or without a goal.<BR>
 * The possible expansion methods are:<BR>
 * "extend", "extend n steps" and "connect".<BR>
 * There are some restrictions on the use of those methods:<BR>
 * connect cannot be used with T-RRT,
 * while ML-RRT should be used with connect.
 */
class RRT: public TreePlanner
{

public:
	/** Constructor from a WorkSpace object
	 * @param WS the WorkSpace
	 */
	RRT(WorkSpace* WS);

	/**
	 * Destructor
	 */
	~RRT();

	/**
	 * Initialzation of the plannificator
	 * @return the number of node added during the init phase
	 */
	virtual int init();

	/**
	 * Checks out the Stop condition
	 */
	bool checkStopConditions();

	/**
	 * Checks out the preconditions
	 */
	bool preConditions();

	/**
	 * Three phases One Step Expansion
	 *  - Direction
	 *  - Node
	 *  - Process
	 *
	 *  @param fromComp the component which is expanded
	 *  @param toComp the goal component
	 */
	virtual int expandOneStep(Node* fromComp, Node* toComp);


	/**
	 * Shoots a new configuration randomly at a fix step
	 * @param qCurrent la Configuration limitant la distance
	 * @return la Configuration tirée
	 */
	std::tr1::shared_ptr<Configuration> diffuseOneConf(std::tr1::shared_ptr<
			Configuration> qCurrent)
	{
		std::tr1::shared_ptr<LocalPath> path = std::tr1::shared_ptr<LocalPath> (new LocalPath(
				qCurrent, _Robot->shoot()));

		return path->configAtParam(std::min(path->length(), _expan->step()));
	};

	/**
	* Returns number of consecutive failure
	* during plannification
	*/
	TreeExpansionMethod* getExpansion()
		{
			return _expan;
		};

protected:
	TreeExpansionMethod* _expan;


};

#endif