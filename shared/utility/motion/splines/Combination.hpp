/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef UTILITY_MOTION_SPLINES_COMBINATION_HPP
#define UTILITY_MOTION_SPLINES_COMBINATION_HPP

#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace utility {
namespace motion {
    namespace splines {

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
            unsigned long binomialCoefficient(size_t k, size_t n) {
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
                    unsigned long val1 = binomialCoefficient(k - 1, n - 1);
                    unsigned long val2 = binomialCoefficient(k, n - 1);
                    unsigned long test = std::numeric_limits<unsigned long>::max() - val1;
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
            void startCombination(size_t k, size_t n) {
                if (n == 0 || k == 0) {
                    throw std::logic_error("Combination zero");
                }
                if (k > n) {
                    throw std::logic_error("Combination not valid k>n");
                }

                indexes = std::vector<size_t>();
                k       = k;
                n       = n;
                for (size_t i = 0; i < k; i++) {
                    indexes.push_back(i);
                }
            }

            /**
             * Return the next combination.
             * Return empty sdt::vector when iteration is finished
             */
            std::vector<size_t> nextCombination() {
                std::vector<size_t> result = indexes;

                if (indexes.size() > 0) {
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
            typedef std::pair<unsigned long, unsigned long> Pair;
            typedef std::vector<size_t> Comb;

            /**
             * Hold (n choose k) number of possible combinations for dynamic programming
             */
            std::map<Pair, unsigned long> pascal_triangle;

            /**
             * Current indexes container and iteration n and k parameter
             */
            std::vector<size_t> indexes;
            size_t n;
            size_t k;

            /**
             * Increment by one the indexes container at digit i (recursively).
             * Return true on iteration end.
             */
            bool incrIndexes(size_t i) {
                if (indexes[i] == n - (k - i)) {
                    if (i == 0) {
                        return true;
                    }
                    else {
                        bool isEnd = incrIndexes(i - 1);
                        if (isEnd) {
                            return true;
                        }
                        else {
                            indexes[i] = indexes[i - 1] + 1;
                            return false;
                        }
                    }
                }
                else {
                    indexes[i]++;
                    return false;
                }
            }
        };

    }  // namespace splines
}  // namespace motion
}  // namespace utility

#endif
