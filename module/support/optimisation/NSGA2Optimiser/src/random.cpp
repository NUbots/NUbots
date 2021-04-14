#include "random.h"

using namespace nsga2;

RandomGenerator::RandomGenerator(uint32_t _seed) {
    seed = _seed;
    generator.seed(_seed);
}

RandomGenerator::~RandomGenerator() {}

double RandomGenerator::Realu() {
    return u01d(generator);
}

double RandomGenerator::Real(double _low, double _high) {
    return (_low + (_high - _low) * Realu());
}

int RandomGenerator::Integer(int _low, int _high) {
    uintd.param(boost::random::uniform_int_distribution<int>::param_type(_low, _high));
    return uintd(generator);
}

void RandomGenerator::SetSeed(uint32_t _seed) {
    seed = _seed;
    generator.seed(seed);
}

int RandomGenerator::GetSeed() const {
    return seed;
}
