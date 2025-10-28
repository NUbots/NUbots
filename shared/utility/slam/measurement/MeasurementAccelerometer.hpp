/**
 * @file MeasurementAccelerometer.hpp
 * @brief Defines the MeasurementAccelerometer class for accelerometer measurements
 */
#ifndef UTILITY_SLAM_MEASUREMENT_MEASUREMENT_ACCELEROMETER_HPP
#define UTILITY_SLAM_MEASUREMENT_MEASUREMENT_ACCELEROMETER_HPP

#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>

#include "../gaussian/GaussianInfo.hpp"
#include "../system/SystemEstimator.hpp"
#include "MeasurementGaussianLikelihood.hpp"

namespace utility::slam::measurement {

    /**
     * @class MeasurementAccelerometer
     * @brief Class for accelerometer measurements (specific force)
     *
     * This class represents accelerometer measurements in the SLAM system.
     * The accelerometer measures specific force (linear acceleration + gravity) in the body frame.
     *
     * Frames:
     * - World frame (n): NED (North-East-Down)
     * - Body/Camera frame (b): OpenCV convention (z forward, x right, y down)
     * - Torso frame (t): NUbots convention (x forward, y left, z up)
     *
     * The measurement model is:
     * y = R_bn^T * (a_n - g_n) + v
     *
     * where:
     * - y is the measured specific force (3D vector) [m/s²]
     * - R_bn is the rotation from navigation to body frame
     * - a_n is the acceleration in navigation frame (dvBNn/dt transformed to n-frame)
     * - g_n is gravity in navigation frame = [0, 0, 9.81] in NED
     * - v is measurement noise
     *
     * For quasi-static conditions (low dynamics), a_n ≈ 0, so:
     * y ≈ R_nb * g_n = Rz(yaw) * Ry(pitch) * Rx(roll) * g_n
     *
     * This helps constrain orientation (especially roll and pitch).
     *
     * Implementation:
     * - Analytical Jacobian using chain rule on rotation matrices
     * - Analytical Hessian for second-order optimization methods
     * - Efficient computation using pre-computed rotation derivatives
     */
    class MeasurementAccelerometer : public MeasurementGaussianLikelihood {
    public:
        /**
         * @brief Construct a new MeasurementAccelerometer object
         * @param time The time at which the measurement occurs
         * @param measurement The measurement vector (3D specific force in m/s²)
         */
        MeasurementAccelerometer(double time, const Eigen::VectorXd& measurement);

        /**
         * @brief Construct a new MeasurementAccelerometer object with verbosity
         * @param time The time at which the measurement occurs
         * @param measurement The measurement vector (3D specific force in m/s²)
         * @param verbosity The verbosity level for logging and output
         */
        MeasurementAccelerometer(double time, const Eigen::VectorXd& measurement, int verbosity);

        /**
         * @brief Destroy the MeasurementAccelerometer object
         */
        ~MeasurementAccelerometer() override;

        // Inherited virtual functions
        Eigen::VectorXd predict(const Eigen::VectorXd& state_vector,
                                const system::SystemEstimator& system) const override;

        Eigen::VectorXd predict(const Eigen::VectorXd& state_vector,
                                const system::SystemEstimator& system,
                                Eigen::MatrixXd& jacobian) const override;

        Eigen::VectorXd predict(const Eigen::VectorXd& state_vector,
                                const system::SystemEstimator& system,
                                Eigen::MatrixXd& jacobian,
                                Eigen::Tensor<double, 3>& hessian) const override;

        gaussian::GaussianInfo<double> noiseDensity(const system::SystemEstimator& system) const override;

    protected:
        /**
         * @brief Get a string representation of the accelerometer measurement process
         * @return std::string A string describing the accelerometer measurement process
         */
        std::string getProcessString() const override;

    private:
        static constexpr double sigma_accel_ = 0.5;   ///< Accelerometer noise std dev [m/s²]
        static constexpr double gravity_     = 9.81;  ///< Gravitational acceleration [m/s²]
    };

}  // namespace utility::slam::measurement

#endif  // UTILITY_SLAM_MEASUREMENT_MEASUREMENT_ACCELEROMETER_HPP
