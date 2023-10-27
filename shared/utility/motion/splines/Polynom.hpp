/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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

/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef UTILITY_MOTION_SPLINES_POLYNOM_HPP
#define UTILITY_MOTION_SPLINES_POLYNOM_HPP

#include <cstdlib>
#include <iostream>
#include <vector>

#include "Combination.hpp"

namespace utility::motion::splines {

    /**
     * Polynom
     *
     * Simple one dimensional polynom class for spline generation
     */
    template <typename Scalar>
    class Polynom {
    public:
        /**
         * Default and inital degree initialization
         */
        constexpr Polynom() = default;
        constexpr Polynom(const size_t& degree) : coefs(degree + 1, static_cast<Scalar>(0)) {}
        constexpr Polynom(const std::vector<Scalar>& coefs) : coefs(coefs) {}

        /**
         * Access to coefficient indexed from constant to higher degree
         */
        [[nodiscard]] constexpr const std::vector<Scalar>& getCoefs() const {
            return coefs;
        }
        [[nodiscard]] std::vector<Scalar>& getCoefs() {
            return coefs;
        }

        /**
         * Access to coefficient
         */
        [[nodiscard]] constexpr const Scalar& operator()(const size_t& index) const {
            return coefs.at(index);
        }
        [[nodiscard]] Scalar& operator()(const size_t& index) {
            return coefs.at(index);
        }

        /**
         * Return polynom degree -1 means empty polynom
         */
        [[nodiscard]] constexpr size_t degree() const {
            return coefs.size() - 1;
        }

        /**
         * Polynom evaluation, its first, second and third derivative at given x
         */
        [[nodiscard]] constexpr Scalar pos(const Scalar& x) const {
            auto xx  = static_cast<Scalar>(1);
            auto val = static_cast<Scalar>(0);
            for (size_t i = 0; i < coefs.size(); i++) {
                val += xx * coefs[i];
                xx *= x;
            }
            return val;
        }

        [[nodiscard]] constexpr Scalar vel(const Scalar& x) const {
            auto xx  = static_cast<Scalar>(1);
            auto val = static_cast<Scalar>(0);
            for (size_t i = 1; i < coefs.size(); i++) {
                val += i * xx * coefs[i];
                xx *= x;
            }
            return val;
        }

        [[nodiscard]] constexpr Scalar acc(const Scalar& x) const {
            auto xx  = static_cast<Scalar>(1);
            auto val = static_cast<Scalar>(0);
            for (size_t i = 2; i < coefs.size(); i++) {
                val += (i - 1) * i * xx * coefs[i];
                xx *= x;
            }
            return val;
        }

        [[nodiscard]] constexpr Scalar jerk(const Scalar& x) const {
            auto xx  = static_cast<Scalar>(1);
            auto val = static_cast<Scalar>(0);
            for (size_t i = 3; i < coefs.size(); i++) {
                val += (i - 2) * (i - 1) * i * xx * coefs[i];
                xx *= x;
            }
            return val;
        }

        /**
         * Some useful operators
         */
        constexpr void operator*=(const Scalar& coef) {
            for (size_t i = 0; i < coefs.size(); i++) {
                coefs[i] *= coef;
            }
        }

        constexpr void operator+=(const Polynom& p) {
            while (p.coefs.size() > coefs.size()) {
                coefs.push_back(static_cast<Scalar>(0));
            }

            for (size_t i = 0; i < p.coefs.size(); i++) {
                coefs[i] += p.coefs[i];
            }
        }

        /**
         * Update the polynom coefficients by applying delta offset on X abscisse
         */
        constexpr void shift(const Scalar& delta) {
            Polynom<Scalar> n(coefs.size() - 1);
            n.coefs[0] = coefs[0];

            for (size_t k = 1; k < coefs.size(); k++) {
                const Polynom<Scalar> tmp = expandBinomial(delta, k);
                for (size_t l = 0; l <= k; l++) {
                    n.coefs[l] += coefs[k] * tmp.coefs[l];
                }
            }

            *this = n;
        }

        /**
         * Expand the given formula (x + y)^degree and return the polynom in x whose coefficient are computed
         * using binomial coefficient
         */
        [[nodiscard]] static Polynom<Scalar> expandBinomial(const Scalar& y, const size_t& degree) {
            Combination combination{};
            Polynom polynom{};

            polynom.getCoefs().resize(degree + 1);

            for (size_t k = 0; k <= degree; k++) {
                polynom.getCoefs()[k] = combination.binomialCoefficient(k, degree) * std::pow(y, degree - k);
            }

            return polynom;
        }

    private:
        /**
         * Polynom coefficients
         */
        std::vector<Scalar> coefs{};
    };

    /**
     * Print operator
     */
    template <typename Scalar>
    constexpr std::ostream& operator<<(std::ostream& os, const Polynom<Scalar>& p) {
        os << "degree=" << p.degree() << " ";
        for (size_t i = 0; i < p.degree() + 1; i++) {
            os << p(i) << " ";
        }

        return os;
    }

}  // namespace utility::motion::splines

#endif
