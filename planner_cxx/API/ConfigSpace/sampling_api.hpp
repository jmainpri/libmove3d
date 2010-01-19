#ifndef SAMPLING_API_HPP
#define SAMPLING_API_HPP

#include "../planningAPI.hpp"

class Robot;

/**
    @ingroup CONFIG_SPACE
    @brief The sampling API
    */
class SamplingAPI
{
protected:
	Robot* mR;

public:
	SamplingAPI(Robot* r) : mR(r) {}

	virtual ~SamplingAPI();

	virtual std::tr1::shared_ptr<Configuration> sample(bool samplePassive = true);

	std::tr1::shared_ptr<Configuration> shootCollisionFree();
};

#endif
