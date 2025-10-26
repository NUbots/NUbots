/**
 * @file MeasurementGaussianLikelihood.h
 * @brief Defines the MeasurementGaussianLikelihood class for measurements with Gaussian likelihood.
 */
#ifndef MEASUREMENTGAUSSIANLIKELIHOOD_H
#define MEASUREMENTGAUSSIANLIKELIHOOD_H

#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>
#include "../gaussian/GaussianInfo.hpp"
#include "../system/SystemEstimator.hpp"
#include "Measurement.hpp"

/**
 * @class MeasurementGaussianLikelihood
 * @brief Class for measurements with Gaussian likelihood.
 *
 * This class represents measurements that have a Gaussian likelihood function.
 * It inherits from the Measurement class and adds functionality specific to Gaussian likelihoods.
 */
namespace utility::slam::measurement {

    class MeasurementGaussianLikelihood : public Measurement
    {
    public:
        /**
         * @brief Construct a new MeasurementGaussianLikelihood object.
         * @param time The time at which the measurement occurs.
         * @param y The measurement vector.
         */
        MeasurementGaussianLikelihood(double time, const Eigen::VectorXd & y);

        /**
         * @brief Construct a new MeasurementGaussianLikelihood object with verbosity setting.
         * @param time The time at which the measurement occurs.
         * @param y The measurement vector.
         * @param verbosity The verbosity level for logging and output.
         */
        MeasurementGaussianLikelihood(double time, const Eigen::VectorXd & y, int verbosity);

        /**
         * @brief Destroy the MeasurementGaussianLikelihood object.
         */
        virtual ~MeasurementGaussianLikelihood() override;

        /**
         * @brief Predict the measurement given a state and system.
         * @param x The state vector.
         * @param system The system estimator.
         * @return The predicted measurement.
         */
        virtual Eigen::VectorXd predict(const Eigen::VectorXd & x, const SystemEstimator & system) const = 0;

        /**
         * @brief Predict the measurement and calculate its Jacobian.
         * @param x The state vector.
         * @param system The system estimator.
         * @param J Output parameter for the Jacobian.
         * @return The predicted measurement.
         */
        virtual Eigen::VectorXd predict(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::MatrixXd & J) const = 0;

        /**
         * @brief Predict the measurement, calculate its Jacobian and Hessian.
         * @param x The state vector.
         * @param system The system estimator.
         * @param J Output parameter for the Jacobian.
         * @param H Output parameter for the Hessian.
         * @return The predicted measurement.
         */
        virtual Eigen::VectorXd predict(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::MatrixXd & J, Eigen::Tensor<double, 3> & H) const = 0;

        /**
         * @brief Get the noise density for the measurement.
         * @param system The system estimator.
         * @return The noise density.
         */
        virtual gaussian::GaussianInfo<double> noiseDensity(const SystemEstimator & system) const = 0;

        /**
         * @brief Predict the density of the measurement.
         * @param x The state vector.
         * @param system The system estimator.
         * @return The predicted density.
         */
        virtual gaussian::GaussianInfo<double> predictDensity(const Eigen::VectorXd & x, const SystemEstimator & system) const;

        /**
         * @brief Predict the density of the measurement and calculate its Jacobian.
         * @param x The state vector.
         * @param system The system estimator.
         * @param dhdx Output parameter for the Jacobian.
         * @return The predicted density.
         */
        virtual gaussian::GaussianInfo<double> predictDensity(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::MatrixXd & dhdx) const;

        /**
         * @brief Predict the density of the measurement, calculate its Jacobian and Hessian.
         * @param x The state vector.
         * @param system The system estimator.
         * @param dhdx Output parameter for the Jacobian.
         * @param d2hdx2 Output parameter for the Hessian.
         * @return The predicted density.
         */
        virtual gaussian::GaussianInfo<double> predictDensity(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::MatrixXd & dhdx, Eigen::Tensor<double, 3> & d2hdx2) const;

        // Inherited virtual functions
        virtual Eigen::VectorXd simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
        virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
        virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const override;
        virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const override;

    protected:
        /**
         * @brief Augment the prediction with additional information.
         * @param xv The augmented state vector.
         * @param Ja Output parameter for the augmented Jacobian.
         * @param system The system estimator.
         * @return The augmented prediction.
         */
        Eigen::MatrixXd augmentedPredict(const Eigen::VectorXd & xv, Eigen::MatrixXd & Ja, const SystemEstimator & system) const;

        /**
         * @brief Update the system based on this measurement.
         * @param system The system to update.
         */
        virtual void update(SystemBase & system) override;

        Eigen::VectorXd y_;  ///< The measurement vector.
    };

} // namespace utility::slam::measurement

#endif
