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

#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::localisation {

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

        /// @brief Status of walk engine
        bool walk_engine_enabled = false;

        /// @brief Status of if the robot is falling
        bool falling = false;

    public:
        /// @brief Called by the powerplant to build and setup the Localisation reactor.
        explicit Localisation(std::unique_ptr<NUClear::Environment> environment);

        Eigen::Vector3d ray2cartesian(Eigen::Vector3d ray, Eigen::Isometry3d Hcw);

        /// @brief Get the occupancy value of a cell in the map
        double get_occupancy(double x, double y);

        /// @brief Get the occupancy value of a cell in the map for each of the observations
        std::vector<double> provide_rating(const Eigen::Matrix<double, 3, 1> state,
                                           const std::vector<Eigen::Vector2d>& observations);

        /// @brief Transform the observation from the robot's coordinate frame into the map's coordinate frame
        Eigen::Vector2d observation_relative(Eigen::Matrix<double, 3, 1> state, Eigen::Vector2d observation);
    };


}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_LOCALISATION_HPP
