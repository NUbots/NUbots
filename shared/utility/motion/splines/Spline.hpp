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
#ifndef UTILITY_MOTION_SPLINES_SPLINE_HPP
#define UTILITY_MOTION_SPLINES_SPLINE_HPP

#include <iomanip>
#include <iostream>
#include <vector>

#include "Polynom.hpp"

namespace utility::motion::splines {

    /**
     * Spline
     *
     * Generic one dimentional polynomial spline generator
     */
    template <typename Scalar>
    class Spline {
    public:
        /**
         * Internal spline part structure with a polynom valid on an interval
         */
        struct Spline_t {
            Polynom<Scalar> polynom{};
            Scalar min = 0;
            Scalar max = 0;
        };

        /**
         * Return spline interpolation at given t. Compute spline value, its first, second and third derivative
         */
        [[nodiscard]] constexpr Scalar pos(const Scalar& t) const {
            return interpolation(t, &Polynom<Scalar>::pos);
        }
        [[nodiscard]] constexpr Scalar vel(const Scalar& t) const {
            return interpolation(t, &Polynom<Scalar>::vel);
        }
        [[nodiscard]] constexpr Scalar acc(const Scalar& t) const {
            return interpolation(t, &Polynom<Scalar>::acc);
        }
        [[nodiscard]] constexpr Scalar jerk(const Scalar& t) const {
            return interpolation(t, &Polynom<Scalar>::jerk);
        }

        /**
         * Return spline interpolation value, first, second and third derivative with given t bound between 0
         * and 1
         */
        [[nodiscard]] constexpr Scalar posMod(const Scalar& t) const {
            return interpolationMod(t, &Polynom<Scalar>::pos);
        }
        [[nodiscard]] constexpr Scalar velMod(const Scalar& t) const {
            return interpolationMod(t, &Polynom<Scalar>::vel);
        }
        [[nodiscard]] constexpr Scalar accMod(const Scalar& t) const {
            return interpolationMod(t, &Polynom<Scalar>::acc);
        }
        [[nodiscard]] constexpr Scalar jerkMod(const Scalar& t) const {
            return interpolationMod(t, &Polynom<Scalar>::jerk);
        }

        /**
         * Return minimum and maximum abscisse value for which spline is defined
         */
        [[nodiscard]] constexpr Scalar min() const {
            if (splines.empty()) {
                return static_cast<Scalar>(0);
            }
            return splines.front().min;
        }
        [[nodiscard]] constexpr Scalar max() const {
            if (splines.empty()) {
                return static_cast<Scalar>(0);
            }
            return splines.back().max;
        }

        /**
         * Write and read splines data into given iostream in ascii format
         */
        constexpr void exportData(std::ostream& os) const {
            for (size_t i = 0; i < splines.size(); i++) {
                os << std::setprecision(17) << splines[i].min << " ";
                os << std::setprecision(17) << splines[i].max << " ";
                os << std::setprecision(17) << splines[i].polynom.getCoefs().size() << " ";
                for (size_t j = 0; j < splines[i].polynom.getCoefs().size(); j++) {
                    os << std::setprecision(17) << splines[i].polynom.getCoefs()[j] << " ";
                }
            }
            os << std::endl;
        }

        constexpr void importData(std::istream& is) {
            bool isFormatError = true;
            while (is.good()) {
                isFormatError = true;
                Scalar min;
                Scalar max;
                size_t size = 0;
                Polynom<Scalar> p;
                // Load spline interval and degree
                is >> min;
                if (!is.good()) {
                    break;
                }
                is >> max;
                if (!is.good()) {
                    break;
                }
                is >> size;
                // Load polynom coefficients
                p.getCoefs().resize(size);
                for (size_t i = 0; i < size; i++) {
                    if (!is.good()) {
                        break;
                    }
                    is >> p.getCoefs()[i];
                }
                // Save spline part
                isFormatError = false;
                splines.push_back({p, min, max});
                // Exit on line break
                while (is.peek() == ' ') {
                    if (!is.good()) {
                        break;
                    }
                    is.ignore();
                }
                if (is.peek() == '\n') {
                    break;
                }
            }
            if (isFormatError) {
                throw std::logic_error("Spline import format invalid");
            }
            // Call possible post import
            importCallBack();
        }

        /**
         * Return the number of internal polynom
         */
        [[nodiscard]] constexpr size_t size() const {
            return splines.size();
        }

        /**
         * Access to given by its index
         */
        [[nodiscard]] constexpr const Spline_t& part(size_t index) const {
            return splines.at(index);
        }

        /**
         * Add a part with given polynom and min/max time range
         */
        constexpr void addPart(const Polynom<Scalar>& poly, const Scalar& min, const Scalar& max) {
            splines.emplace_back(poly, min, max);
        }

        /**
         * Replace this spline part with the internal data of given spline
         */
        constexpr void copyData(const Spline<Scalar>& sp) {
            splines = sp.splines;
            // Call possible post import
            importCallBack();
        }

    protected:
        /**
         * Spline part container
         */
        std::vector<Spline_t> splines{};

        /**
         * Possible override callback after importation
         */
        virtual void importCallBack() {}

    private:
        /**
         * Return spline interpolation of given value and used given polynom evaluation function (member
         * function pointer)
         */
        [[nodiscard]] constexpr Scalar interpolation(Scalar x,
                                                     Scalar (Polynom<Scalar>::*func)(const Scalar&) const) const {
            // Empty case
            if (splines.empty()) {
                return static_cast<Scalar>(0);
            }
            // Bound asked abscisse into spline range
            if (x <= splines.front().min) {
                x = splines.front().min;
            }
            if (x >= splines.back().max) {
                x = splines.back().max;
            }
            // Bijection spline search
            size_t indexLow = 0;
            size_t indexUp  = splines.size() - 1;
            while (indexLow != indexUp) {
                size_t index = (indexUp + indexLow) / 2;
                if (x < splines[index].min) {
                    indexUp = index - 1;
                }
                else if (x > splines[index].max) {
                    indexLow = index + 1;
                }
                else {
                    indexUp  = index;
                    indexLow = index;
                }
            }
            // Compute and return spline value
            return (splines[indexUp].polynom.*func)(x - splines[indexUp].min);
        }

        /**
         * Return interpolation with x bound between 0 and 1
         */
        [[nodiscard]] constexpr Scalar interpolationMod(Scalar x,
                                                        Scalar (Polynom<Scalar>::*func)(const Scalar&) const) const {
            if (x < static_cast<Scalar>(0)) {
                x = static_cast<Scalar>(1) + (x - (static_cast<int>(x) / static_cast<Scalar>(1)));
            }
            else if (x > static_cast<Scalar>(1)) {
                x = (x - ((int) x / 1));
            }
            return interpolation(x, func);
        }
    };

}  // namespace utility::motion::splines

#endif
