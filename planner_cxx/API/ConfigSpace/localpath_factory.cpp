#include "../planningAPI.hpp"

using std::tr1::shared_ptr;

LocalpathFactory::LocalpathFactory() {}

LocalpathFactory::~LocalpathFactory() {}

Localpath* LocalpathFactory::create(
		shared_ptr<Configuration> q1,
		shared_ptr<Configuration> q2)
{
	return(new Localpath(q1, q2));
}
