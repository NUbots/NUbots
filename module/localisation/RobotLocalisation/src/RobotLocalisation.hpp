#ifndef MODULE_LOCALISATION_ROBOTLOCALISATION_HPP
#define MODULE_LOCALISATION_ROBOTLOCALISATION_HPP

#include <nuclear>

#include "RobotModel.hpp"

#include "message/localisation/Robot.hpp"
#include "message/vision/Robot.hpp"

#include "utility/math/filter/UKF.hpp"

namespace module::localisation {

    using VisionRobot        = message::vision::Robot;
    using VisionRobots       = message::vision::Robots;
    using LocalisationRobot  = message::localisation::Robot;
    using LocalisationRobots = message::localisation::Robots;

    class RobotLocalisation : public NUClear::Reactor {
    private:
        struct Config {
            Config() = default;
            /// @brief UKF config
            struct UKF {
                struct Noise {
                    Noise() = default;
                    struct Measurement {
                        Eigen::Matrix2d position = Eigen::Matrix2d::Zero();
                    } measurement{};
                    struct Process {
                        Eigen::Vector2d position = Eigen::Vector2d::Zero();
                        Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
                    } process{};
                } noise{};
                struct Initial {
                    Initial() = default;
                    struct Mean {
                        Eigen::Vector2d position = Eigen::Vector2d::Zero();
                        Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
                    } mean{};
                    struct Covariance {
                        Eigen::Vector2d position = Eigen::Vector2d::Zero();
                        Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
                    } covariance{};
                } initial{};
            } ukf{};

            /// @brief Initial state of the for the UKF filter
            RobotModel<double>::StateVec initial_mean;

            /// @brief Initial covariance of the for the UKF filter
            RobotModel<double>::StateVec initial_covariance;

        } cfg;

        struct FilteredRobot {
            FilteredRobot() = default;
            /// @brief Id of the robot
            int id = 0;
            /// @brief Time of the last time update
            NUClear::clock::time_point last_time_update;
            /// @brief Unscented Kalman Filter for this robot
            utility::math::filter::UKF<double, RobotModel> ukf{};
        };

        /// @brief List of filtered robots
        std::vector<FilteredRobot> filtered_robots;

    public:
        /// @brief Called by the powerplant to build and setup the RobotLocalisation reactor.
        explicit RobotLocalisation(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Function to add a robot to the list of robots
        /// @param initial_rRWw The initial position of the robot in world space
        /// @return The id of the robot that was added
        int add_new_robot(const Eigen::Vector3d& initial_rRWw);

        /// @brief Data association function
        /// @param vision_robot The robot detection from the vision system
        /// @param filtered_robots The list of current filtered robots
        /// @return The id of the robot that was associated or -1 if no robot was associated
        int data_association(const Eigen::Vector3d& rRWw, const std::vector<FilteredRobot>& filtered_robots);
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_ROBOTLOCALISATION_HPP
