/*
 * Copyright (c) Hamburg Bit-Bots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
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
 *
 * This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
 * The original files can be found at:
 * https://github.com/Rhoban/model/
 */

#ifndef UTILITY_MOTION_SPLINES_COMBINATION_HPP
#define UTILITY_MOTION_SPLINES_COMBINATION_HPP

#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace utility::motion::splines {

    /**
     * Combination
     *
     * Implement binomial coefficient computation using Pascal Triangle and iterate thought all (n choose k)
     * combinations
     */
    class Combination {
    public:
        /**
         * Compute the number of possible combinations for (n choose k) (using dynamic progamming)
         */
        [[nodiscard]] constexpr unsigned long binomialCoefficient(size_t k, size_t n) {
            if (n == 0 || k == 0) {
                return 1;
            }
            if (k > n) {
                throw std::logic_error("Combination not valid k>n");
            }

            if (n - k < k) {
                return binomialCoefficient(n - k, n);
            }
            if (k == 1 || k == n) {
                return n;
            }

            Pair pair(k, n);
            if (pascal_triangle.count(pair) == 0) {
                const unsigned long val1 = binomialCoefficient(k - 1, n - 1);
                const unsigned long val2 = binomialCoefficient(k, n - 1);
                const unsigned long test = std::numeric_limits<unsigned long>::max() - val1;
                if (val2 < test) {
                    pascal_triangle[pair] = val1 + val2;
                }
                else {
                    throw std::runtime_error("Combination overflow");
                }
            }

            return pascal_triangle[pair];
        }

        /**
         * Start combination iteration for given (n choose k)
         */
        void inline startCombination(size_t k, size_t n) {
            if (n == 0 || k == 0) {
                throw std::logic_error("Combination zero");
            }
            if (k > n) {
                throw std::logic_error("Combination not valid k>n");
            }

            indexes.clear();
            k = k;
            n = n;
            for (size_t i = 0; i < k; i++) {
                indexes.push_back(i);
            }
        }

        /**
         * Return the next combination.
         * Return empty std::vector when iteration is finished
         */
        [[nodiscard]] inline std::vector<size_t> nextCombination() {
            std::vector<size_t> result = indexes;

            if (!indexes.empty()) {
                bool isEnd = incrIndexes(k - 1);
                if (isEnd) {
                    indexes.clear();
                }
            }

            return result;
        }

    private:
        /**
         * Typedefs
         */
        using Pair = std::pair<unsigned long, unsigned long>;
        using Comb = std::vector<size_t>;

        /**
         * Hold (n choose k) number of possible combinations for dynamic programming
         */
        std::map<Pair, unsigned long> pascal_triangle{};

        /**
         * Current indexes container and iteration n and k parameter
         */
        std::vector<size_t> indexes{};
        size_t n = 0;
        size_t k = 0;

        /**
         * Increment by one the indexes container at digit i (recursively).
         * Return true on iteration end.
         */
        inline bool incrIndexes(const size_t& i) {
            if (indexes[i] == n - (k - i)) {
                if (i == 0) {
                    return true;
                }
                const bool isEnd = incrIndexes(i - 1);
                if (isEnd) {
                    return true;
                }
                indexes[i] = indexes[i - 1] + 1;
                return false;
            }
            indexes[i]++;
            return false;
        }
    };

}  // namespace utility::motion::splines

#endif
