/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
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

#ifndef MODULE_EXTENSION_DIRECTOR_LCS_HPP
#define MODULE_EXTENSION_DIRECTOR_LCS_HPP

#include <Eigen/Core>

namespace util {

    /**
     * Longest common subsequence algorithm that returns which elements from a and b form the common subsequence.
     *
     * This algorithm compares two lists and finds the longest subsequence you can make that is included in both of the
     * lists. It returns a pair of bool vectors that indicate which elements are a part of the subsequence.
     *
     * @tparam T    the type of the elements to be compared
     *
     * @param a the first list to compare
     * @param b the second list to compare
     *
     * @return two vectors of bools indicating which elements participate in the longest common subexpression
     */
    template <typename T>
    std::pair<std::vector<bool>, std::vector<bool>> lcs(const std::vector<T>& a, const std::vector<T>& b) {

        // Start with nothing matching
        std::vector<bool> match_a(a.size(), false);
        std::vector<bool> match_b(b.size(), false);

        // Nothing matches if one is empty
        if (a.empty() || b.empty()) {
            return std::make_pair(match_a, match_b);
        }

        // Directions matrix for dynamic algorithm
        // 0x1 = diagonal, 0x2 = left, 0x4 = top
        using Matrix      = Eigen::MatrixXi;
        Matrix directions = Eigen::MatrixXi::Zero(a.size(), b.size());

        const int insert_weight = 3;
        std::vector<int> last_weights(a.size(), 0);
        std::vector<int> curr_weights(a.size(), 0);
        for (int i = 0, weight = insert_weight; i < int(a.size()); ++i, weight += insert_weight) {
            last_weights[i] = weight;
        }

        for (int y = 0; y < int(b.size()); ++y) {
            for (int x = 0; x < int(a.size()); ++x) {
                // Calculate the weights
                int weight_from_left = x == 0 ? (y + 2) * insert_weight : curr_weights[x - 1] + insert_weight;
                int weight_from_top  = last_weights[x] + insert_weight;
                int weight_from_diagonal =
                    a[x] == b[y] ? (x == 0 ? (y + 1) * insert_weight : last_weights[x - 1]) : 0x7FFFFFFF;

                // Find the smallest weight
                int min_weight  = std::min(std::min(weight_from_left, weight_from_top), weight_from_diagonal);
                curr_weights[x] = min_weight;

                int direction = (min_weight == weight_from_diagonal ? 0x01 : 0x0)  //
                                | (min_weight == weight_from_left ? 0x02 : 0x0)    //
                                | (min_weight == weight_from_top ? 0x04 : 0x0);    //

                directions(x, y) = direction;
            }

            // Swap the weights
            std::swap(last_weights, curr_weights);
        }

        // Rewrite directions(y,x) to match_a[i] and match_b[i].
        int x   = int(a.size()) - 1;
        int y   = int(b.size()) - 1;
        int i_a = x;
        int i_b = y;
        while (x >= 0 && y >= 0) {
            int direction = directions(x, y);
            if (direction & 0x01) {
                match_a[i_a--] = true;
                match_b[i_b--] = true;
                --x;
                --y;
            }
            else if (direction & 0x02) {
                --i_a;
                --x;
            }
            else {  // (direction & 0x04)
                --i_b;
                --y;
            }
        }

        return std::make_pair(match_a, match_b);
    }

}  // namespace util

#endif  // MODULE_EXTENSION_DIRECTOR_LCS_HPP
