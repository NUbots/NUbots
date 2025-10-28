/**
 * @file MeasurementGyroscope.hpp
 * @brief Defines the MeasurementGyroscope class for gyroscope measurements
 */
#ifndef UTILITY_SLAM_MEASUREMENT_MEASUREMENT_GYROSCOPE_HPP
#define UTILITY_SLAM_MEASUREMENT_MEASUREMENT_GYROSCOPE_HPP

#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>

#include "../gaussian/GaussianInfo.hpp"
#include "../system/SystemEstimator.hpp"
#include "MeasurementGaussianLikelihood.hpp"

namespace utility::slam::measurement {

    /**
     * @class MeasurementGyroscope
     * @brief Class for gyroscope measurements (angular velocity)
     *
     * This class represents gyroscope measurements in the SLAM system.
     * It inherits from MeasurementGaussianLikelihood and implements gyroscope-specific functionality.
     */
    class MeasurementGyroscope : public MeasurementGaussianLikelihood {
    public:
        /**
         * @brief Construct a new MeasurementGyroscope object
         * @param time The time at which the measurement occurs
         * @param y The measurement vector (3D angular velocity in rad/s)
         */
        MeasurementGyroscope(double time, const Eigen::VectorXd& y);

        /**
         * @brief Construct a new MeasurementGyroscope object with verbosity
         * @param time The time at which the measurement occurs
         * @param y The measurement vector (3D angular velocity in rad/s)
         * @param verbosity The verbosity level for logging and output
         */
        MeasurementGyroscope(double time, const Eigen::VectorXd& y, int verbosity);

        /**
         * @brief Destroy the MeasurementGyroscope object
         */
        virtual ~MeasurementGyroscope() override;

        // Inherited virtual functions
        virtual Eigen::VectorXd predict(const Eigen::VectorXd& x, const system::SystemEstimator& system) const override;

        virtual Eigen::VectorXd predict(const Eigen::VectorXd& x,
                                        const system::SystemEstimator& system,
                                        Eigen::MatrixXd& J) const override;

        virtual Eigen::VectorXd predict(const Eigen::VectorXd& x,
                                        const system::SystemEstimator& system,
                                        Eigen::MatrixXd& J,
                                        Eigen::Tensor<double, 3>& H) const override;

        virtual gaussian::GaussianInfo<double> noiseDensity(const system::SystemEstimator& system) const override;

    protected:
        /**
         * @brief Get a string representation of the gyroscope measurement process
         * @return std::string A string describing the gyroscope measurement process
         */
        virtual std::string getProcessString() const override;

    private:
        static constexpr double sigma_gyro_ = 1;  ///< Gyroscope noise std dev [rad/s]
    };

}  // namespace utility::slam::measurement

#endif  // UTILITY_SLAM_MEASUREMENT_MEASUREMENT_GYROSCOPE_HPP
