#ifndef MODULE_LOCALISATION_LOCALISATION_HPP
#define MODULE_LOCALISATION_LOCALISATION_HPP

#include <nuclear>

#include "Map.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/support/nusight/DataPoint.hpp"
#include "message/vision/FieldLines.hpp"
#include "message/vision/Goal.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/math/stats/multivariate.hpp"
#include "utility/math/stats/resample/multinomial.hpp"
#include "utility/math/stats/resample/resample.hpp"
#include "utility/math/stats/resample/residual.hpp"
#include "utility/math/stats/resample/stratified.hpp"
#include "utility/math/stats/resample/systematic.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::localisation {

    // Particle struct
    struct Particle {
        Eigen::Matrix<double, 3, 1> state;
        double weight = 1.0;
    };

    class Localisation : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Size of the grid cells in the occupancy grid [m]
            double grid_size = 0.0;
            /// @brief Number of particles to use in the particle filter
            int n_particles = 0;
            /// @brief Uncertainty in the process model
            Eigen::Matrix<double, 3, 3> process_noise = Eigen::Matrix<double, 3, 3>::Zero();
            /// @brief Scaling factor for odometry x velocity
            double scale_x = 0.0;
            /// @brief Scaling factor for odometry y velocity
            double scale_y = 0.0;
            /// @brief Scaling factor for odometry theta velocity
            double scale_theta = 0.0;
            /// @brief Bool to enable/disable saving the generated map as a csv file
            bool save_map = false;
        } cfg;

        NUClear::clock::time_point last_time_update_time;

        /// @brief Occupancy grid map of the field lines
        Map fieldline_map;

        /// @brief Current walk command (dx, dy, dtheta)
        Eigen::Matrix<double, 3, 1> walk_command = Eigen::Matrix<double, 3, 1>::Zero();

        /// @brief State (x,y,theta) of the robot
        Eigen::Matrix<double, 3, 1> state = Eigen::Matrix<double, 3, 1>::Zero();

        /// @brief Covariance matrix of the robot's state
        Eigen::Matrix<double, 3, 3> covariance = Eigen::Matrix<double, 3, 3>::Identity();

        /// @brief Status of walk engine
        bool walk_engine_enabled = false;

        /// @brief Status of if the robot is falling
        bool falling = false;

        /// @brief Particles used in the particle filter
        std::vector<Particle> particles;


    public:
        /// @brief Called by the powerplant to build and setup the Localisation reactor.
        explicit Localisation(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Converts a unit vector of point from the camera in world space to a (x,y) point relative to the robot
        /// on the field plane
        /// @param uPCw unit vector from the camera to the field point in world space
        /// @param Hcw the camera to world transform
        /// @return the field point measurement (x,y) relative to the robot
        Eigen::Vector2d ray_to_field_plane(Eigen::Vector3d uPCw, Eigen::Isometry3d Hcw);

        /// @brief Get the occupancy value of a cell in the map
        /// @param observation The observation (x, y) in the map
        /// @return The occupancy value of the cell
        double get_occupancy(const Eigen::Vector2i observation);

        /// @brief Transform the observation from the robot's coordinate frame into the map's coordinate frame
        /// @param particle The state of the particle (x,y,theta)
        /// @param observation The observation (x, y) in the robot's coordinate frame [m]
        /// @return The observation location (x, y) in the map
        Eigen::Vector2i observation_relative(const Eigen::Matrix<double, 3, 1> particle,
                                             const Eigen::Vector2d observation);

        /// @brief Get the weight of a particle given a set of observations
        /// @param particle The state of the particle (x,y,theta)
        /// @param observations The observations (x, y) in the robot's coordinate frame [m]
        /// @return The weight of the particle
        double calculate_weight(const Eigen::Matrix<double, 3, 1> particle,
                                const std::vector<Eigen::Vector2d>& observations);

        /// @brief Get the current mean (state) of the robot
        // @return The current mean (state) of the robot
        Eigen::Matrix<double, 3, 1> compute_mean();

        /// @brief Get the current covariance matrix of the robot's state
        // @return The current covariance matrix of the robot's state
        Eigen::Matrix<double, 3, 3> compute_covariance();

        /// @brief Perform a time update on the particles
        /// @param walk_command The walk command (dx, dy, dtheta)
        /// @param dt The time since the last time update
        void time_update();

        /// @brief Generates a random sample from a 1-D array using the Eigen library
        /// @param particles Vector of particles to be resampled
        /// @param size Output size (int or tuple of ints)
        /// @return The randomly generated sample of particles
        void resample();

        /// @brief Add some noise to the particles to compensate for the fact that we don't have a perfect model
        void add_noise();

        /// @brief Log a list of particles
        void log_particles(int number = -1);
    };


}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_LOCALISATION_HPP
