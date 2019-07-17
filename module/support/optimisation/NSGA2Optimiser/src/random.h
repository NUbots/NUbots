#ifndef RANDOM_GEN_H
#define RANDOM_GEN_H

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/uniform_int_distribution.hpp>

namespace nsga2 {
class RandomGenerator {
public:
    RandomGenerator(uint32_t seed = 0);
    virtual ~RandomGenerator();
    double Realu();
    double Real(double _low, double _high);
    int Integer(int _low, int _high);
    void SetSeed(uint32_t _seed);
    int GetSeed() const;
    boost::random::mt19937 generator;
    int seed;
    boost::random::uniform_01<double> u01d;
    boost::random::uniform_int_distribution<int> uintd;
};
}  // namespace nsga2

#endif
