#ifndef LOCALPATH_FACTORY_HPP
#define LOCALPATH_FACTORY_HPP

#include "../planningAPI.hpp"

class LocalpathFactory
{
public:
	LocalpathFactory();

	virtual ~LocalpathFactory();

	virtual Localpath* create(
			std::tr1::shared_ptr<Configuration> q1,
			std::tr1::shared_ptr<Configuration> q2);
};

#endif
