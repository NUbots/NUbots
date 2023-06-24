#ifndef MODULE_ACTUATION_FOOTCONTROLLER_HPP
#define MODULE_ACTUATION_FOOTCONTROLLER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::actuation {

    // Conversion: Quaternion --> Fused angles (2D)
    void FusedFromQuat(const Eigen::Quaterniond& q, double& fusedPitch, double& fusedRoll) {
        // Calculate the fused pitch and roll
        double stheta = 2.0 * (q.y() * q.w() - q.x() * q.z());
        double sphi   = 2.0 * (q.y() * q.z() + q.x() * q.w());
        stheta        = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta));  // Coerce stheta to [-1,1]
        sphi          = (sphi >= 1.0 ? 1.0 : (sphi <= -1.0 ? -1.0 : sphi));        // Coerce sphi   to [-1,1]
        fusedPitch    = asin(stheta);
        fusedRoll     = asin(sphi);
    }

    Eigen::Quaterniond QuatFromFused(double fusedPitch, double fusedRoll)  // Assume: fusedYaw = 0, hemi = true
    {
        // Precalculate the sine values
        double sth  = sin(fusedPitch);
        double sphi = sin(fusedRoll);

        // Calculate the sine sum criterion
        double crit = sth * sth + sphi * sphi;

        // Calculate the tilt angle alpha
        double alpha   = (crit >= 1.0 ? M_PI_2 : acos(sqrt(1.0 - crit)));
        double halpha  = 0.5 * alpha;
        double chalpha = cos(halpha);
        double shalpha = sin(halpha);

        // Calculate the tilt axis angle gamma
        double gamma  = atan2(sth, sphi);
        double cgamma = cos(gamma);
        double sgamma = sin(gamma);

        // Return the required quaternion orientation (a rotation about (cgamma, sgamma, 0) by angle alpha)
        return Eigen::Quaterniond(chalpha, cgamma * shalpha, sgamma * shalpha, 0.0);  // Order: (w,x,y,z)
    }

    class FootController : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Gains for the servos
            double servo_gain = 0.0;
            /// @brief Whether or not to use orientation correction
            bool correction_enabled = false;
            /// @brief Proportional gain for torso orientation roll correction
            double roll_p_gain = 0.0;
            /// @brief Proportional gain for torso orientation pitch correction
            double pitch_p_gain = 0.0;
            /// @brief Integral gain for torso orientation roll correction
            double roll_i_gain = 0.0;
            /// @brief Integral gain for torso orientation pitch correction
            double pitch_i_gain = 0.0;
            /// @brief Antiwindup max I error
            double max_i_error = 0.0;
            /// @brief Derivative gain for torso orientation roll correction
            double roll_d_gain = 0.0;
            /// @brief Derivative gain for torso orientation pitch correction
            double pitch_d_gain = 0.0;
        } cfg;

        // *************** //
        // *** Left *** //
        // *************** //
        double left_prev_roll_error      = 0;
        double left_prev_pitch_error     = 0;
        double left_integral_roll_error  = 0;
        double left_integral_pitch_error = 0;
        /// @brief Last time we updated
        NUClear::clock::time_point left_last_update_time{};

        // *************** //
        // *** Right *** //
        // *************** //
        double right_prev_roll_error      = 0;
        double right_prev_pitch_error     = 0;
        double right_integral_roll_error  = 0;
        double right_integral_pitch_error = 0;
        /// @brief Last time we updated
        NUClear::clock::time_point right_last_update_time{};


    public:
        /// @brief Called by the powerplant to build and setup the FootController reactor.
        explicit FootController(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_FOOTCONTROLLER_HPP
