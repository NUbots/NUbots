#ifndef FIELDLOCALISATIONSRIF_HPP
#define FIELDLOCALISATIONSRIF_HPP

#include <nuclear>

namespace module::localisation {

    /**
     * @brief Square-root information filter field localisation
     *
     * Estimates the 6-DOF torso pose in the field frame {f} plus a 2-DOF camera-mount attitude bias, as a Gaussian in
     * square-root information form.Prediction is driven by body-twist inputs finite-differenced from the odometry
     * stream (Sensors.Htw); measurement updates are MAP optimisations (trust-region Newton) over the robust landmark
     * ray likelihood, yielding a Laplace-approximation posterior with a covariance that quantifies the estimate's
     * uncertainty.
     *
     * Landmarks are the YOLO field-line intersections (L/T/X) and goal posts, consumed as unit rays in the camera
     * frame; gravity (accelerometer) and the kinematic torso height provide additional low-rate corrections.
     *
     * Initialisation is a coarse grid-search over (x, y, yaw) on the first usable vision frame, scored by the same
     * landmark likelihood. The field's 180 degree symmetry is broken with the game-context prior that the robot starts
     * in its own half.
     */

    class FieldLocalisationSRIF : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and set up the FieldLocalisationSRIF reactor.
        explicit FieldLocalisationSRIF(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief Configuration parameters for this module
        struct Config {
            /// @brief Process noise PSDs [m/sqrt(s), rad/sqrt(s)]
            filter::SystemLocalisation::Parameters process{};
            /// @brief Landmark measurement noise/association options
            filter::MeasurementFieldLandmarks::Options measurement{};
            /// @brief Initial sqrt-covariance diagonal for the 8-dim state after the grid solve
            Eigen::Matrix<double, 8, 1> initial_sqrt_covariance =
                (Eigen::Matrix<double, 8, 1>() << 1.0, 1.0, 0.05, 0.05, 0.05, 0.5, 0.02, 0.02).finished();
            /// @brief Sign of field-x for the starting half (from game context; breaks the field symmetry)
            double own_half_x_sign = 1.0;
            /// @brief Grid search steps for the initial pose solve
            double grid_step_xy  = 0.35;
            double grid_step_yaw = 18.0 * M_PI / 180.0;
            /// @brief Minimum landmark associations to trust an initial grid solve
            int min_init_associations = 4;
            /// @brief Enable the multi-hypothesis (field-symmetry) Gaussian mixture bank
            bool use_hypothesis_bank = false;
            /// @brief Enable the accelerometer gravity measurement
            bool use_gravity = true;
            /// @brief Enable the kinematic torso-height measurement
            bool use_kinematic_height = true;
            /// @brief Maximum odometry sample spacing to finite-difference across [s]
            double max_odometry_gap = 0.1;
            /// @brief Maximum age of vision-to-sensors pairing [s]
            double max_sensor_pairing_age = 0.1;
        } cfg;

        /**
         * @brief Emit the localisation Field message (and debug graphs) from the current belief.
         * @param paired The odometry sample paired with the processed vision frame
         * @param measurement The processed landmark measurement (nullptr on the bootstrap frame)
         * @param Tbc Camera pose w.r.t. torso at capture time (for the association-line projection)
         */
        void emit_field(const SensorEntry& paired,
                        const filter::MeasurementFieldLandmarks* measurement,
                        const filter::Pose<double>& Tbc);

        /// @brief Seconds since the module time origin
        [[nodiscard]] double seconds(const NUClear::clock::time_point& tp) const {
            return std::chrono::duration_cast<std::chrono::duration<double>>(tp - t0).count();
        }

        /**
         * @brief Coarse global grid search for the initial pose on one vision frame.
         *
         * Roll, pitch and torso height come from the kinematic chain (the odometry
         * world frame is gravity-aligned); (x, y, yaw) are found by maximising the
         * landmark measurement log-likelihood over a grid, and the own-half prior
         * selects between the maximum and its 180 degree mirror.
         *
         * @param candidates Landmark rays extracted from the vision messages
         * @param Tbc Camera pose w.r.t. torso at capture time
         * @param Twt Torso pose in the odometry world frame at capture time
         * @param eta0 Output: initial 8-dim state estimate
         * @return True if enough landmarks associated to trust the solve
         */
        [[nodiscard]] bool solve_initial_pose(
            const std::vector<filter::MeasurementFieldLandmarks::CandidateDetection>& candidates,
            const filter::Pose<double>& Tbc,
            const filter::Pose<double>& Twt,
            Eigen::Matrix<double, 8, 1>& eta0) const;
    }
}  // namespace module::localisation


#endif
