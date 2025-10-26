#include <stdexcept>
#include <Eigen/Core>
#include "../Event.hpp"
#include "../system/SystemEstimator.hpp"
#include "../funcmin.hpp"
#include "MeasurementGaussianLikelihood.hpp"

namespace utility::slam::measurement {

    MeasurementGaussianLikelihood::MeasurementGaussianLikelihood(double time, const Eigen::VectorXd & y)
        : Measurement(time)
        , y_(y)
    {
        updateMethod_= UpdateMethod::BFGSTRUSTSQRT;
    }

    MeasurementGaussianLikelihood::MeasurementGaussianLikelihood(double time, const Eigen::VectorXd & y, int verbosity)
        : Measurement(time, verbosity)
        , y_(y)
    {
        updateMethod_= UpdateMethod::BFGSTRUSTSQRT;
    }

    MeasurementGaussianLikelihood::~MeasurementGaussianLikelihood() = default;

    gaussian::GaussianInfo<double> MeasurementGaussianLikelihood::predictDensity(const Eigen::VectorXd & x, const SystemEstimator & system) const
    {
        Eigen::VectorXd h = predict(x, system);
        const Eigen::MatrixXd & SR = noiseDensity(system).sqrtCov();
        return gaussian::GaussianInfo<double>::fromSqrtMoment(h, SR);
    }

    gaussian::GaussianInfo<double> MeasurementGaussianLikelihood::predictDensity(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::MatrixXd & dhdx) const
    {
        Eigen::VectorXd h = predict(x, system, dhdx);
        const Eigen::MatrixXd & SR = noiseDensity(system).sqrtCov();
        return gaussian::GaussianInfo<double>::fromSqrtMoment(h, SR);
    }

    gaussian::GaussianInfo<double> MeasurementGaussianLikelihood::predictDensity(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::MatrixXd & dhdx, Eigen::Tensor<double, 3> & d2hdx2) const
    {
        Eigen::VectorXd h = predict(x, system, dhdx, d2hdx2);
        const Eigen::MatrixXd & SR = noiseDensity(system).sqrtCov();
        return gaussian::GaussianInfo<double>::fromSqrtMoment(h, SR);
    }

    // Augmented measurement model
    // [ y ] = [ h(x) + v ]
    // [ x ]   [     x    ]
    // \___/   \__________/
    //   ya  =   ha(x, v)
    //
    // Evaluate ha(x, v) and its Jacobian Ja = [dha/dx, dha/dv]
    Eigen::MatrixXd MeasurementGaussianLikelihood::augmentedPredict(const Eigen::VectorXd & xv, Eigen::MatrixXd & Ja, const SystemEstimator & system) const
    {
        const Eigen::Index & nxv = xv.size();
        const Eigen::Index & ny = y_.size();
        const Eigen::Index & nx = nxv - ny;
        Eigen::VectorXd x = xv.head(nx);
        Eigen::VectorXd v = xv.tail(ny);
        Eigen::MatrixXd J;
        Eigen::VectorXd y = predict(x, system, J) + v;

        Eigen::VectorXd ha(nx + ny);
        ha << y,
            x;

        Ja.resize(nx + ny, nx + ny);
        Ja <<                                  J, Eigen::MatrixXd::Identity(ny, ny),
            Eigen::MatrixXd::Identity(nx, nx), Eigen::MatrixXd::Zero(nx, ny);

        return ha;
    }

    void MeasurementGaussianLikelihood::update(SystemBase & system_)
    {
        // Downcast since we know that MeasurementGaussianLikelihood events only occur to SystemEstimator objects
        SystemEstimator & system = dynamic_cast<SystemEstimator &>(system_);

        const Eigen::Index & nx = system.density.dim();
        switch (updateMethod_)
        {
            case UpdateMethod::AFFINE:  // Update using affine transformation
            {
                const Eigen::Index & ny = y_.size();
                auto pxv = system.density*noiseDensity(system);    // p(x, v) = p(x)*p(v)
                auto func = [&](const Eigen::VectorXd & x, Eigen::MatrixXd & J){ return augmentedPredict(x, J, system); };
                auto pyx = pxv.affineTransform(func);
                system.density = pyx.conditional(Eigen::lastN(nx), Eigen::seqN(0, ny), y_);
                break;
            }
            case UpdateMethod::GAUSSNEWTON:
            {
                throw std::runtime_error("Gauss-Newton method not yet implemented");
            }
            case UpdateMethod::LEVENBERGMARQUARDT:
            {
                throw std::runtime_error("Levenberg-Marquardt method not yet implemented");
            }
            default:
                Measurement::update(system_);
        }
    }

    Eigen::VectorXd MeasurementGaussianLikelihood::simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const
    {
        return predictDensity(x, system).simulate();
    }

    double MeasurementGaussianLikelihood::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const
    {
        auto likelihood = predictDensity(x, system);

        // Evaluate log N(y; h(x), R)
        double logLik = likelihood.log(y_);
        return logLik;
    }

    double MeasurementGaussianLikelihood::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const
    {
        Eigen::MatrixXd dhdx;
        auto likelihood = predictDensity(x, system, dhdx);

        // Evaluate log N(y; h(x), R) and d/dy log N(y; h(x), R)
        Eigen::VectorXd loglikGrad;
        double logLik = likelihood.log(y_, loglikGrad);
        // Note:
        //  d                        d
        // -- log N(y; h(x), R) = - -- log N(y; h(x), R)
        // dh                       dy

        // Gradient of log likelihood:
        //
        //         d
        // g_i = ---- log N(y; h(x), R)
        //       dx_i
        //
        //             dh_k     d
        // g_i = sum_k ---- * ---- log N(y; h(x), R)
        //             dx_i   dh_k
        //
        //               dh_k     d
        // g_i = - sum_k ---- * ---- log N(y; h(x), R)
        //               dx_i   dy_k
        //
        g = -dhdx.transpose()*loglikGrad;
        return logLik;
    }

    double MeasurementGaussianLikelihood::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const
    {
        Eigen::MatrixXd dhdx;
        Eigen::Tensor<double, 3> d2hdx2;
        auto likelihood = predictDensity(x, system, dhdx, d2hdx2);

        // Evaluate log N(y; h(x), R), d/dy log N(y; h(x), R) and d^2/dy^2 log N(y; h(x), R)
        Eigen::VectorXd loglikGrad;
        Eigen::MatrixXd logLikHess;
        double logLik = likelihood.log(y_, loglikGrad, logLikHess);
        // Note:
        //  d                        d
        // -- log N(y; h(x), R) = - -- log N(y; h(x), R)
        // dh                       dy
        //
        //  d^2                      d^2
        // ---- log N(y; h(x), R) = ---- log N(y; h(x), R)
        // dh^2                     dy^2

        // Gradient of log likelihood:
        //
        //         d
        // g_i = ---- log N(y; h(x), R)
        //       dx_i
        //
        //             dh_k     d
        // g_i = sum_k ---- * ---- log N(y; h(x), R)
        //             dx_i   dh_k
        //
        //               dh_k     d
        // g_i = - sum_k ---- * ---- log N(y; h(x), R)
        //               dx_i   dy_k
        //
        g = -dhdx.transpose()*loglikGrad;

        // Hessian of log likelihood:
        //
        //              d                                 d  ( dh_k     d                    )
        // H_{ij} = --------- log N(y; h(x), R) = sum_k ---- ( ---- * ---- log N(y; h(x), R) )
        //          dx_i dx_j                           dx_j ( dx_i   dh_k                   )
        //
        //                      dh_k   d^2 log N(y; h(x), R)   dh_l          d^2 h_k      d
        // H_{ij} = sum_k sum_l ---- * --------------------- * ---- + sum_k --------- * ---- log N(y; h(x), R)
        //                      dx_i         dh_k dh_l         dx_j         dx_i dx_j   dh_k
        //
        //                      dh_k   d^2 log N(y; h(x), R)   dh_l          d^2 h_k      d
        // H_{ij} = sum_k sum_l ---- * --------------------- * ---- - sum_k --------- * ---- log N(y; h(x), R)
        //                      dx_i         dy_k dy_l         dx_j         dx_i dx_j   dy_k
        //

        // In MATLAB, this operation would look like the following:
        //       nh = length(h);
        //       nx = length(x);
        //       H = dhdx.'*logLikHess*dhdx - reshape(sum(d2hdx2 .* reshape(loglikGrad, [nh, 1, 1]), 1), [nx, nx]);

        // secondTerm(i, j) = sum_k d2hdx2(k, i, j) * loglikGrad(k)
        Eigen::TensorMap<Eigen::Tensor<double, 3>> logLikGradTensorView(loglikGrad.data(), likelihood.dim(), 1, 1);
        const Eigen::array<Eigen::Index, 3> bdims = {1, x.size(), x.size()};        // broadcast dimensions (cardinals)
        const Eigen::array<Eigen::Index, 1> rdims = {0};                            // reduction dimensions (ordinals)
        Eigen::Tensor<double, 2> secondTerm = (d2hdx2 * logLikGradTensorView.broadcast(bdims)).sum(rdims);
        Eigen::Map<Eigen::MatrixXd> secondTermMatrixView(secondTerm.data(), x.size(), x.size());

        H = dhdx.transpose()*logLikHess*dhdx - secondTermMatrixView;
        return logLik;
    }

} // namespace utility::slam::measurement
