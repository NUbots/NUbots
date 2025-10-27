/**
 * @file MeasurementAnchor.hpp
 * @brief Defines the MeasurementAnchor class for anchor measurements
 */
#ifndef UTILITY_SLAM_MEASUREMENT_MEASUREMENT_ANCHOR_HPP
#define UTILITY_SLAM_MEASUREMENT_MEASUREMENT_ANCHOR_HPP

#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>
#include "../gaussian/GaussianInfo.hpp"
#include "../system/SystemEstimator.hpp"
#include "MeasurementGaussianLikelihood.hpp"

namespace utility::slam::measurement {

    /**
     * @class MeasurementAnchor
     * @brief Class for anchor measurements (position)
     *
     * This class represents anchor measurements in the SLAM system.
     * It inherits from MeasurementGaussianLikelihood and implements anchor-specific functionality.
     */
    class MeasurementAnchor : public MeasurementGaussianLikelihood {
    public:
        /**
         * @brief Construct a new MeasurementAnchor object
         * @param time The time at which the measurement occurs
         * @param y The measurement vector (3D anchor position in meters)
         */
        MeasurementAnchor(double time, const Eigen::VectorXd& y);

        /**
         * @brief Construct a new MeasurementAnchor object with verbosity
         * @param time The time at which the measurement occurs
         * @param y The measurement vector (3D anchor position in meters)
         * @param verbosity The verbosity level for logging and output
         */
        MeasurementAnchor(double time, const Eigen::VectorXd& y, int verbosity);

        /**
         * @brief Destroy the MeasurementAnchor object
         */
        virtual ~MeasurementAnchor() override;

        // Inherited virtual functions
        virtual Eigen::VectorXd predict(const Eigen::VectorXd& x,
                                       const system::SystemEstimator& system) const override;

        virtual Eigen::VectorXd predict(const Eigen::VectorXd& x,
                                       const system::SystemEstimator& system,
                                       Eigen::MatrixXd& J) const override;

        virtual Eigen::VectorXd predict(const Eigen::VectorXd& x,
                                       const system::SystemEstimator& system,
                                       Eigen::MatrixXd& J,
                                       Eigen::Tensor<double, 3>& H) const override;

        virtual gaussian::GaussianInfo<double> noiseDensity(
            const system::SystemEstimator& system) const override;

    protected:
        /**
         * @brief Get a string representation of the anchor measurement process
         * @return std::string A string describing the anchor measurement process
         */
        virtual std::string getProcessString() const override;

    private:
        static constexpr double sigma_anchor_ = 0.01;  ///< Anchor noise std dev [meters]
    };

} // namespace utility::slam::measurement

#endif // UTILITY_SLAM_MEASUREMENT_MEASUREMENT_ANCHOR_HPP
