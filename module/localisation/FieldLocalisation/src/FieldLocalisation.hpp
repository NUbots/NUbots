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
        Eigen::Vector3f state = Eigen::Vector3f::Zero();  // (x, y, theta) of world space in field space
        float weight          = 1.0;
    };

    class FieldLocalisation : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Size of the grid cells in the occupancy grid [m]
            float grid_size = 0.0;
            /// @brief Number of particles to use in the particle filter
            int n_particles = 0;
            /// @brief Uncertainty in the process model
            Eigen::Matrix3f process_noise = Eigen::Matrix3f::Zero();
            /// @brief Uncertainty in the measurement model
            float measurement_noise = 0;
            /// @brief Maximum distance a field line can be from a particle to be considered an observation [m]
            float max_range = 0;
            /// @brief Initial state (x,y,theta) of the robot, saved for resetting
            std::vector<Eigen::Vector3f> initial_state{};
            /// @brief Initial covariance matrix of the robot's state, saved for resetting
            Eigen::Matrix3f initial_covariance = Eigen::Matrix3f::Identity();
            /// @brief Bool to enable/disable saving the generated map as a csv file
            bool save_map = false;
            /// @brief Minimum number of field line points for a measurement update
            size_t min_observations = 0;
            /// @brief Penalty factor for observations being outside map
            float outside_map_penalty_factor = 0.0;
        } cfg;

        NUClear::clock::time_point last_time_update_time;

        /// @brief Occupancy grid map of the field lines
        OccupancyMap<float> fieldline_map;

        /// @brief State (x,y,theta) of the robot
        Eigen::Vector3f state = Eigen::Vector3f::Zero();

        /// @brief Covariance matrix of the robot's state
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Identity();

        /// @brief Status of if the robot is falling
        bool falling = false;

        /// @brief Particles used in the particle filter
        std::vector<Particle> particles{};


    public:
        /// @brief Called by the powerplant to build and setup the FieldLocalisation reactor.
        explicit FieldLocalisation(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Transform a point in the robot's coordinate frame into an index in the map
        /// @param particle The state of the particle (x,y,theta)
        /// @param rPRw The field point (x, y) in world space {w} [m]
        /// @return The observation location (x, y) in the map
        Eigen::Vector2i position_in_map(const Eigen::Vector3f particle, const Eigen::Vector3f rPRw);

        /// @brief Get the weight of a particle given a set of observations
        /// @param particle The state of the particle (x,y,theta)
        /// @param observations The observations (x, y) in the robot's coordinate frame [m]
        /// @return The weight of the particle
        float calculate_weight(const Eigen::Vector3f particle, const std::vector<Eigen::Vector3f>& observations);

        /// @brief Get the current mean (state) of the robot
        // @return The current mean (state) of the robot
        Eigen::Vector3f compute_mean();

        /// @brief Get the current covariance matrix of the robot's state
        // @return The current covariance matrix of the robot's state
        Eigen::Matrix3f compute_covariance();

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
