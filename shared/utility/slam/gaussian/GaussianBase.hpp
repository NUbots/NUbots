/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

 #ifndef UTILITY_SLAM_GAUSSIAN_GAUSSIAN_BASE_HPP
 #define UTILITY_SLAM_GAUSSIAN_GAUSSIAN_BASE_HPP

 #include <cassert>
 #include <cmath>
 #include <ctime>
 #include <numbers>

 #include <Eigen/Core>

 #include <boost/math/special_functions/gamma.hpp>
 #include <boost/random/mersenne_twister.hpp>
 #include <boost/random/normal_distribution.hpp>

 #include "DensityBase.hpp"

 namespace utility::slam::gaussian {

     /**
      * @brief Base class for Gaussian distributions.
      *
      * This class provides a common interface and utility functions for Gaussian distributions.
      *
      * @tparam Scalar The scalar type used for calculations (default: double).
      */
     template <typename Scalar = double>
     class GaussianBase : public DensityBase<Scalar> {
     public:
         /**
          * @brief Virtual destructor.
          */
         virtual ~GaussianBase() override = default;

         /**
          * @brief Returns the dimension of the Gaussian distribution.
          * @return The dimension of the distribution.
          */
         virtual Eigen::Index dim() const = 0;

         /**
          * @brief Returns the mean of the Gaussian distribution.
          * @return The mean vector.
          */
         virtual Eigen::VectorX<Scalar> mean() const = 0;

         /**
          * @brief Returns the square root of the covariance matrix.
          * @return The square root of the covariance matrix.
          */
         virtual Eigen::MatrixX<Scalar> sqrtCov() const = 0;

         /**
          * @brief Returns the covariance matrix.
          * @return The covariance matrix.
          */
         virtual Eigen::MatrixX<Scalar> cov() const = 0;

         /**
          * @brief Returns the information matrix (inverse of covariance).
          * @return The information matrix.
          */
         virtual Eigen::MatrixX<Scalar> infoMat() const = 0;

         /**
          * @brief Returns the information vector.
          * @return The information vector.
          */
         virtual Eigen::VectorX<Scalar> infoVec() const = 0;

         /**
          * @brief Returns the square root of the information matrix.
          * @return The square root of the information matrix.
          */
         virtual Eigen::MatrixX<Scalar> sqrtInfoMat() const = 0;

         /**
          * @brief Returns the square root of the information vector.
          * @return The square root of the information vector.
          */
         virtual Eigen::VectorX<Scalar> sqrtInfoVec() const = 0;

         /**
          * @brief Simulates a sample from the Gaussian distribution.
          * @return A random vector drawn from the distribution.
          */
         Eigen::VectorX<Scalar> simulate() const {
             static boost::random::mt19937 rng(std::time(0));  // Initialise and seed once
             boost::random::normal_distribution<> dist;

             // Draw w ~ N(0, I)
             Eigen::VectorX<Scalar> w(dim());
             for (Eigen::Index i = 0; i < dim(); ++i) {
                 w(i) = dist(rng);
             }

             return mean() + sqrtCov().transpose() * w;
         }

         /**
          * @brief Computes the inverse of the chi-squared distribution function.
          * @param p Probability value.
          * @param nu Degrees of freedom.
          * @return The inverse of the chi-squared distribution function.
          */
         static double chi2inv(double p, double nu) {
             assert(p >= 0);
             assert(p < 1);
             return 2 * boost::math::gamma_p_inv(nu / 2.0, p);
         }

         /**
          * @brief Computes the cumulative distribution function of the standard normal distribution.
         * @param w The input value.
         * @return The cumulative probability.
         */
        static double normcdf(double w) {
            return 0.5 * erfc(-w * std::numbers::sqrt2 / 2.0);
        }

        /**
         * @brief Computes the cumulative distribution function of a normal distribution.
         * @param x The input value.
         * @param mu The mean of the normal distribution.
         * @param sigma The standard deviation of the normal distribution.
         * @return The cumulative probability.
         */
        static double normcdf(double x, double mu, double sigma) {
            return normcdf((x - mu) / sigma);
        }

        /**
         * @brief Computes the scaled complementary error function.
         * @param x The input value.
         * @return The scaled complementary error function value.
         */
        static double erfcx(double x) {
            // https://stackoverflow.com/a/39777361
            double a, d, e, m, p, q, r, s, t;

            a = std::fmax(x, 0.0 - x);  // NaN preserving absolute value computation

            /* Compute q = (a-4)/(a+4) accurately. [0,INF) -> [-1,1] */
            m = a - 4.0;
            p = a + 4.0;
            r = 1.0 / p;
            q = m * r;
            t = std::fma(q + 1.0, -4.0, a);
            e = std::fma(q, -a, t);
            q = std::fma(r, e, q);

            /* Approximate (1+2*a)*exp(a*a)*erfc(a) as p(q)+1 for q in [-1,1] */
            p = 0x1.edcad78fc8044p-31;                    //  8.9820305531190140e-10
            p = std::fma(p, q, 0x1.b1548f14735d1p-30);    //  1.5764464777959401e-09
            p = std::fma(p, q, -0x1.a1ad2e6c4a7a8p-27);   // -1.2155985739342269e-08
            p = std::fma(p, q, -0x1.1985b48f08574p-26);   // -1.6386753783877791e-08
            p = std::fma(p, q, 0x1.c6a8093ac4f83p-24);    //  1.0585794011876720e-07
            p = std::fma(p, q, 0x1.31c2b2b44b731p-24);    //  7.1190423171700940e-08
            p = std::fma(p, q, -0x1.b87373facb29fp-21);   // -8.2040389712752056e-07
            p = std::fma(p, q, 0x1.3fef1358803b7p-22);    //  2.9796165315625938e-07
            p = std::fma(p, q, 0x1.7eec072bb0be3p-18);    //  5.7059822144459833e-06
            p = std::fma(p, q, -0x1.78a680a741c4ap-17);   // -1.1225056665965572e-05
            p = std::fma(p, q, -0x1.9951f39295cf4p-16);   // -2.4397380523258482e-05
            p = std::fma(p, q, 0x1.3be1255ce180bp-13);    //  1.5062307184282616e-04
            p = std::fma(p, q, -0x1.a1df71176b791p-13);   // -1.9925728768782324e-04
            p = std::fma(p, q, -0x1.8d4aaa0099bc8p-11);   // -7.5777369791018515e-04
            p = std::fma(p, q, 0x1.49c673066c831p-8);     //  5.0319701025945277e-03
            p = std::fma(p, q, -0x1.0962386ea02b7p-6);    // -1.6197733983519948e-02
            p = std::fma(p, q, 0x1.3079edf465cc3p-5);     //  3.7167515521269866e-02
            p = std::fma(p, q, -0x1.0fb06dfedc4ccp-4);    // -6.6330365820039094e-02
            p = std::fma(p, q, 0x1.7fee004e266dfp-4);     //  9.3732834999538536e-02
            p = std::fma(p, q, -0x1.9ddb23c3e14d2p-4);    // -1.0103906603588378e-01
            p = std::fma(p, q, 0x1.16ecefcfa4865p-4);     //  6.8097054254651804e-02
            p = std::fma(p, q, 0x1.f7f5df66fc349p-7);     //  1.5379652102610957e-02
            p = std::fma(p, q, -0x1.1df1ad154a27fp-3);    // -1.3962111684056208e-01
            p = std::fma(p, q, 0x1.dd2c8b74febf6p-3);     //  2.3299511862555250e-01

            /* Divide (1+p) by (1+2*a) ==> exp(a*a)*erfc(a) */
            d = a + 0.5;
            r = 1.0 / d;
            r = r * 0.5;
            q = std::fma(p, r, r);  // q = (p+1)/(1+2*a)
            t = q + q;
            e = (p - q) + std::fma(t, -a, 1.0);  // residual: (p+1)-q*(1+2*a)
            r = std::fma(e, r, q);

            /* Handle argument of infinity */
            if (a > 0x1.fffffffffffffp1023)
                r = 0.0;

            /* Handle negative arguments: erfcx(x) = 2*exp(x*x) - erfcx(|x|) */
            if (x < 0.0) {
                s = x * x;
                d = std::fma(x, x, -s);
                e = std::exp(s);
                r = e - r;
                r = std::fma(e, d + d, r);
                r = r + e;
                if (e > 0x1.fffffffffffffp1023)
                    r = e;  // avoid creating NaN
            }
            return r;
        }

        /**
         * @brief Check if a given point is within the confidence region of the Gaussian distribution.
         *
         * This method determines whether the input vector x is within the confidence region
         * defined by nSigma standard deviations from the mean of the Gaussian distribution.
         *
         * @param x The input vector to check.
         * @param nSigma The number of standard deviations defining the confidence region (default: 3.0).
         * @return True if the point is within the confidence region, false otherwise.
         */
        virtual bool isWithinConfidenceRegion(const Eigen::VectorX<Scalar>& x, double nSigma = 3.0) const = 0;

        /**
         * @brief Computes points on the boundary of the confidence ellipse.
         * @param nSigma The number of standard deviations (default: 3.0).
         * @param nSamples The number of samples to generate (default: 100).
         * @return Matrix of points on the ellipse boundary.
         */
        Eigen::Matrix<Scalar, 2, Eigen::Dynamic> confidenceEllipse(double nSigma = 3.0, int nSamples = 100) const {
            const Eigen::Index& n = dim();
            assert(n == 2);

            // from matlab directly
            const double c = normcdf(nSigma) - normcdf(-nSigma);
            const Scalar r = Scalar(std::sqrt(chi2inv(c, 2.0)));

            // angles in [0, 2π] using vectorized operations
            const Scalar twoPi = Scalar(2.0) * Scalar(std::numbers::pi);
            Eigen::Matrix<Scalar, 1, Eigen::Dynamic> t =
                twoPi * Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::LinSpaced(nSamples, Scalar(0), Scalar(1));

            // circle samples in whitened coords using vectorized trig functions
            Eigen::Matrix<Scalar, 2, Eigen::Dynamic> W(2, nSamples);
            W.row(0) = r * t.array().cos();
            W.row(1) = r * t.array().sin();

            // map back: X = μ + Sᵀ W
            Eigen::Matrix<Scalar, 2, Eigen::Dynamic> X = sqrtCov().transpose() * W;
            X.colwise() += mean();

            assert(X.rows() == 2);
            assert(X.cols() == nSamples);
            return X;
        }

        /**
         * @brief Computes the log of the integral of the Gaussian distribution between two points.
         *
         * This function returns the following value:
         *
         * \f[
         * \log \int_a^b \mathcal{N}(x; \mu, \sigma^2) \,\mathrm{d}x
         * \f]
         *
         * where \f$\mu\f$ is the mean and \f$\sigma\f$ is the standard deviation of the Gaussian distribution.
         *
         * @param a The lower bound of the integral.
         * @param b The upper bound of the integral.
         * @return The log of the integral value.
         */
        Scalar logIntegral(Scalar a, Scalar b) const {
            Scalar za, zb, ta, tb;
            return logIntegralImpl(a, b, za, zb, ta, tb);
        }

        /**
         * @brief Computes the log of the integral of the Gaussian distribution between two points and its derivative
         * with respect to the mean.
         *
         * This function returns the following value:
         *
         * \f[
         * l = \log \int_a^b \mathcal{N}(x; \mu, \sigma^2) \,\mathrm{d}x
         * \f]
         *
         * and its derivative with respect to the mean:
         *
         * \f[
         * \frac{\partial l}{\partial \mu} = \frac{\mathcal{N}(b; \mu, \sigma^2) - \mathcal{N}(a; \mu,
         * \sigma^2)}{\int_a^b \mathcal{N}(x; \mu, \sigma^2) \,\mathrm{d}x} \f]
         *
         * where \f$\mu\f$ is the mean and \f$\sigma\f$ is the standard deviation of the Gaussian distribution.
         *
         * @param a The lower bound of the integral.
         * @param b The upper bound of the integral.
         * @param dldmu Reference to store the derivative with respect to the mean.
         * @return The log of the integral value.
         */
        Scalar logIntegral(Scalar a, Scalar b, Scalar& dldmu) const {
            Scalar za, zb, ta, tb;
            Scalar l     = logIntegralImpl(a, b, za, zb, ta, tb);
            Scalar sigma = sqrtCov()(0, 0);
            using std::exp, std::log;
            dldmu = exp(-za * za - l - 0.5 * log(2 * std::numbers::pi * sigma * sigma))
                    - exp(-zb * zb - l - 0.5 * log(2 * std::numbers::pi * sigma * sigma));

            return l;
        }

    private:
        /**
         * @brief Implementation of the log integral computation.
         * @param a The lower bound of the integral.
         * @param b The upper bound of the integral.
         * @param za Standardized lower bound.
         * @param zb Standardized upper bound.
         * @param ta Temporary variable for computation.
         * @param tb Temporary variable for computation.
         * @return The log of the integral value.
         */
        Scalar logIntegralImpl(Scalar a, Scalar b, Scalar& za, Scalar& zb, Scalar& ta, Scalar& tb) const {
            assert(dim() == 1 && "Expected univariate Gaussian to compute the log-integral");
            assert(b >= a && "Expected integral to be non-negative so that the log exists");

            using std::abs, std::exp, std::log, std::log1p;

            Scalar mu    = mean()(0);
            Scalar sigma = sqrtCov()(0, 0);

            za = -(std::numbers::sqrt2 / 2.0) * (a - mu) / sigma;
            zb = -(std::numbers::sqrt2 / 2.0) * (b - mu) / sigma;
            ta = log(0.5 * erfcx(abs(za))) - za * za;
            tb = log(0.5 * erfcx(abs(zb))) - zb * zb;

            if (za >= 0) {
                if (zb >= 0) {
                    return tb + log1p(-exp(ta - tb));
                }
                else  // zb < 0
                {
                    return log1p(-exp(ta) - exp(tb));
                }
            }
            else  // za < 0
            {
                assert(zb < 0 && "Case forbidden, since b >= a implies 0 <= zb <= za < 0");
                return ta + log1p(-exp(tb - ta));
            }
        }
    };

}  // namespace utility::slam::gaussian

#endif  // UTILITY_SLAM_GAUSSIAN_GAUSSIAN_BASE_HPP
