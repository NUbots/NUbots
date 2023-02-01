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

namespace module::localisation {

    // Particle struct
    struct Particle {
        Eigen::Matrix<double, 3, 1> state;
        double weight;
    };

    class Localisation : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            double grid_size;
        } cfg;

        NUClear::clock::time_point last_time_update_time;
        NUClear::clock::time_point last_measurement_update_time;

        static constexpr int TIME_UPDATE_FREQUENCY = 10;

        /// @brief The occupancy grid map of the field lines
        Map fieldline_map;

        /// @brief The current walk command (dx, dy, dtheta)
        Eigen::Matrix<double, 3, 1> walk_command;

        /// @brief The state (x,y,theta) of the robot
        Eigen::Matrix<double, 3, 1> state = Eigen::Matrix<double, 3, 1>::Zero();

        /// @brief The covariance matrix of the robot's state
        Eigen::Matrix<double, 3, 3> covariance = 0.005 * Eigen::Matrix<double, 3, 3>::Identity();

        /// @brief Status of walk engine
        bool walk_engine_enabled = false;

        /// @brief Status of if the robot is falling
        bool falling = false;

        int num_particles = 100;

    public:
        /// @brief Called by the powerplant to build and setup the Localisation reactor.
        explicit Localisation(std::unique_ptr<NUClear::Environment> environment);

        Eigen::Vector3d ray2cartesian(Eigen::Vector3d uPCw, Eigen::Isometry3d Hcw, Eigen::Isometry3d Hwf);

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
        double calculate_weight(const Eigen::Matrix<double, 3, 1> particle,
                                const std::vector<Eigen::Vector2d>& observations);
    };


}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_LOCALISATION_HPP
