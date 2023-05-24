#ifndef MODULE_LOCALISATION_ROBOTLOCALISATION_HPP
#define MODULE_LOCALISATION_ROBOTLOCALISATION_HPP

#include <nuclear>

#include "RobotModel.hpp"

#include "message/eye/DataPoint.hpp"
#include "message/localisation/Field.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/FieldLines.hpp"

#include "utility/localisation/OccupancyMap.hpp"
#include "utility/math/filter/ParticleFilter.hpp"
#include "utility/math/stats/multivariate.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::localisation {

    class RobotLocalisation : public NUClear::Reactor {
    private:
        /// @brief Particle filter
        utility::math::filter::ParticleFilter<double, RobotModel> filter;

        /// @brief Stores configuration values
        struct Config {
            /// @brief Size of the grid cells in the occupancy grid [m]
            double grid_size = 0.0;

            /// @brief Number of particles to use in the particle filter
            int n_particles = 0;

            /// @brief Uncertainty in the process model
            Eigen::Matrix<double, 3, 3> process_noise = Eigen::Matrix<double, 3, 3>::Zero();

            /// @brief Uncertainty in the measurement model
            double measurement_noise = 0;

            /// @brief Maximum distance a field line can be from a particle to be considered an observation [m]
            double max_range = 0;

            /// @brief Initial state (x,y,theta) of the robot, saved for resetting
            std::vector<Eigen::Matrix<double, 3, 1>> initial_state{};

            /// @brief Initial covariance matrix of the robot's state, saved for resetting
            Eigen::Matrix<double, 3, 3> initial_covariance = Eigen::Matrix<double, 3, 3>::Identity();

            /// @brief Bool to enable/disable saving the generated map as a csv file
            bool save_map = false;

            /// @brief Vector of known goal post positions
            std::vector<Eigen::Vector2d> goal_posts{};
        } cfg;

        NUClear::clock::time_point last_time_update_time;

        /// @brief Occupancy grid map of the field lines
        OccupancyMap fieldline_map;

    public:
        /// @brief Called by the powerplant to build and setup the RobotLocalisation reactor.
        explicit RobotLocalisation(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Converts a unit vector of point from the camera in robot space to a (x,y) point relative to the robot
        /// on the field plane
        /// @param uPCw unit vector of point from the camera in world space
        /// @param Hcw the world from camera transform
        /// @return the field point measurement (x,y) relative to the robot
        Eigen::Vector2d ray_to_field_plane(Eigen::Vector3d uPCw, Eigen::Isometry3d Hcw);

        /// @brief Transform a point in the robot's coordinate frame into an index in the map
        /// @param particle The state of the particle (x,y,theta)
        /// @param rPRw The field point (x, y) in world space {w} [m]
        /// @return The observation location (x, y) in the map
        Eigen::Vector2i position_in_map(const Eigen::Matrix<double, 3, 1> particle, const Eigen::Vector2d rPRw);

        /// @brief Get the weight of a particle given a set of observations
        /// @param particle The state of the particle (x,y,theta)
        /// @param observations The observations (x, y) in the robot's coordinate frame [m]
        /// @return The weight of the particle
        double calculate_weight(const Eigen::Matrix<double, 3, 1> particle,
                                const std::vector<Eigen::Vector2d>& observations);
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_ROBOTLOCALISATION_HPP
