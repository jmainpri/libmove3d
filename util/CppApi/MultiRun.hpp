#ifndef MULTIRUNS_HPP
#define MULTIRUNS_HPP

#include <vector>
#include <string>

/**
  * Enables mutliple runs
  */
class MultiRun
{
public:
    MultiRun();

    void runMutliRRT();
    void runMutliGreedy();

private:
    void saveVectorToFile();

    std::vector<std::string>                mNames;
    std::vector< std::vector<double> >      mVectDoubles;
    std::vector<double>                     mTime;

};

#endif // MULTIRUNS_HPP
