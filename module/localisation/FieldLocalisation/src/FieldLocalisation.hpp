#ifndef MODULE_LOCALISATION_FIELDLOCALISATION_HPP
#define MODULE_LOCALISATION_FIELDLOCALISATION_HPP

#include <nuclear>

#include "message/eye/DataPoint.hpp"
#include "message/localisation/Field.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/FieldLines.hpp"

#include "utility/localisation/OccupancyMap.hpp"
#include "utility/math/stats/multivariate.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::localisation {

    // Particle struct
    struct Particle {
        /// @brief State of the particle (x, y, theta) of world in field space
        Eigen::Vector3d state = Eigen::Vector3d::Zero();
        /// @brief Weight of the particle
        double weight = 1.0;
    };

    class FieldLocalisation : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Size of the grid cells in the occupancy grid [m]
            double grid_size = 0.0;
            /// @brief Number of particles to use in the particle filter
            int n_particles = 0;
            /// @brief Uncertainty in the process model
            Eigen::Matrix3d process_noise = Eigen::Matrix3d::Zero();
            /// @brief Uncertainty in the measurement model
            double measurement_noise = 0.0;
            /// @brief Maximum distance a field line can be from a particle to be considered an observation [m]
            double max_range = 0.0;
            /// @brief Initial state (x,y,theta) of the robot, saved for resetting
            std::vector<Eigen::Vector3d> initial_state{};
            /// @brief Initial covariance matrix of the robot's state, saved for resetting
            Eigen::Matrix3d initial_covariance = Eigen::Matrix3d::Identity();
            /// @brief Bool to enable/disable saving the generated map as a csv file
            bool save_map = false;
            /// @brief Minimum number of field line points for a measurement update
            size_t min_observations = 0;
            /// @brief Penalty factor for observations being outside map
            double outside_map_penalty_factor = 0.0;
            /// @brief Whether to use the hardcoded initial state or not
            bool use_hardcoded_initial_state = false;
        } cfg;

        NUClear::clock::time_point last_time_update_time;

        /// @brief Occupancy grid map of the field lines
        OccupancyMap<double> fieldline_map;

        /// @brief State (x,y,theta) of the robot
        Eigen::Vector3d state = Eigen::Vector3d::Zero();

        /// @brief Covariance matrix of the robot's state
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

        /// @brief Particles used in the particle filter
        std::vector<Particle> particles{};


    public:
        /// @brief Called by the powerplant to build and setup the FieldLocalisation reactor.
        explicit FieldLocalisation(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Transform a point in the robot's coordinate frame into an index in the map
        /// @param particle The state of the particle (x,y,theta)
        /// @param rPWw The field point (x, y) in world space {w} [m]
        /// @return The observation location (x, y) in the map
        Eigen::Vector2i position_in_map(const Eigen::Vector3d particle, const Eigen::Vector3d rPWw);

        /// @brief Get the weight of a particle given a set of observations
        /// @param particle The state of the particle (x,y,theta)
        /// @param observations The observations (x, y) in the robot's coordinate frame [m]
        /// @return The weight of the particle
        double calculate_weight(const Eigen::Vector3d particle, const std::vector<Eigen::Vector3d>& observations);

        /// @brief Get the current mean (state) of the robot
        // @return The current mean (state) of the robot
        Eigen::Vector3d compute_mean();

        /// @brief Get the current covariance matrix of the robot's state
        // @return The current covariance matrix of the robot's state
        Eigen::Matrix3d compute_covariance();

        /// @brief Perform a time update on the particles
        /// @param walk_command The walk command (dx, dy, dtheta)
        /// @param dt The time since the last time update
        void time_update();

        /// @brief Resample the particles based on their weights
        /// @param particles Vector of particles to be resampled
        /// @return The resampled particles
        void resample();

        /// @brief Add some noise to the particles to compensate for the fact that we don't have a perfect model
        /// @param particles Vector of particles to be resampled
        /// @return The particles with noise added
        void add_noise();
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_FIELDLOCALISATION_HPP
