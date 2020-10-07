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

namespace utility {
namespace motion {
    namespace splines {

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
            Polynom() {}
            Polynom(size_t degree) : coefs(degree + 1, static_cast<Scalar>(0)) {}
            Polynom(std::vector<Scalar> coefs) : coefs(coefs) {}

            /**
             * Access to coefficient indexed from constant to higher degree
             */
            const std::vector<Scalar>& getCoefs() const {
                return coefs;
            }
            std::vector<Scalar>& getCoefs() {
                return coefs;
            }

            /**
             * Access to coefficient
             */
            const Scalar& operator()(size_t index) const {
                return coefs.at(index);
            }
            Scalar& operator()(size_t index) {
                return coefs.at(index);
            }

            /**
             * Return polynom degree -1 means empty polynom
             */
            size_t degree() const {
                return coefs.size() - 1;
            }

            /**
             * Polynom evaluation, its first, second and third derivative at given x
             */
            Scalar pos(Scalar x) const {
                Scalar xx  = static_cast<Scalar>(1);
                Scalar val = static_cast<Scalar>(0);
                for (size_t i = 0; i < coefs.size(); i++) {
                    val += xx * coefs[i];
                    xx *= x;
                }
                return val;
            }

            Scalar vel(Scalar x) const {
                Scalar xx  = static_cast<Scalar>(1);
                Scalar val = static_cast<Scalar>(0);
                for (size_t i = 1; i < coefs.size(); i++) {
                    val += i * xx * coefs[i];
                    xx *= x;
                }
                return val;
            }

            Scalar acc(Scalar x) const {
                Scalar xx  = static_cast<Scalar>(1);
                Scalar val = static_cast<Scalar>(0);
                for (size_t i = 2; i < coefs.size(); i++) {
                    val += (i - 1) * i * xx * coefs[i];
                    xx *= x;
                }
                return val;
            }

            Scalar jerk(Scalar x) const {
                Scalar xx  = static_cast<Scalar>(1);
                Scalar val = static_cast<Scalar>(0);
                for (size_t i = 3; i < coefs.size(); i++) {
                    val += (i - 2) * (i - 1) * i * xx * coefs[i];
                    xx *= x;
                }
                return val;
            }

            /**
             * Some useful operators
             */
            void operator*=(Scalar coef) {
                for (size_t i = 0; i < coefs.size(); i++) {
                    coefs[i] *= coef;
                }
            }

            void operator+=(const Polynom& p) {
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
            void shift(Scalar delta) {
                Polynom<Scalar> n(coefs.size() - 1);
                n.coefs[0] = coefs[0];

                for (size_t k = 1; k < coefs.size(); k++) {
                    Polynom<Scalar> tmp = expandBinomial(delta, k);
                    for (size_t l = 0; l <= k; l++) {
                        n.coefs[l] += coefs[k] * tmp.coefs[l];
                    }
                }

                *this = n;
            }

            /**
             * Expand the given formula (x + y)^degree and return the polynom in x whose coefficient are computed using
             * binomial coefficient
             */
            static Polynom<Scalar> expandBinomial(Scalar y, size_t degree) {
                Combination combination;
                Polynom polynom;

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
            std::vector<Scalar> coefs;
        };

        /**
         * Print operator
         */
        template <typename Scalar>
        std::ostream& operator<<(std::ostream& os, const Polynom<Scalar>& p) {
            os << "degree=" << p.degree() << " ";
            for (size_t i = 0; i < p.degree() + 1; i++) {
                os << p(i) << " ";
            }

            return os;
        }

    }  // namespace splines
}  // namespace motion
}  // namespace utility

#endif
