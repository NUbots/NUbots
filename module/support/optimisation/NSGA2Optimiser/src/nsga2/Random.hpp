/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_RANDOM_HPP
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_RANDOM_HPP

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
        void set_seed(const uint32_t& seed_) {
            seed = seed_;
            generator.seed(seed_);
        }
        uint32_t get_seed() const {
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

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_RANDOM_HPP
