
#ifndef RANDOM_GEN_H
#define RANDOM_GEN_H

#include <random>

namespace nsga2 {
    template <typename IntType = int, typename RealType = double>
    class RandomGenerator {
    public:
        RandomGenerator(const uint32_t& seed_ = 0) : seed(seed_), generator(random_device()), u01d(0, 1), uintd(0, 1) {
            generator.seed(seed_);
        }
        RealType Realu() {
            return u01d(generator);
        }
        RealType Real(const RealType& low, const RealType& high) {
            return (low + (high - low) * Realu());
        }
        IntType Integer(const IntType& low, const IntType& high) {
            uintd.param(typename std::uniform_int_distribution<IntType>::param_type(low, high));
            return uintd(generator);
        }
        void SetSeed(const uint32_t& seed_) {
            seed = seed_;
            generator.seed(seed_);
        }
        uint32_t GetSeed() const {
            return seed;
        }

    private:
        uint32_t seed;
        std::random_device random_device;
        std::mt19937 generator;
        std::uniform_real_distribution<RealType> u01d;
        std::uniform_int_distribution<IntType> uintd;
    };
}  // namespace nsga2

#endif
