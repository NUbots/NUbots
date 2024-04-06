#ifndef MODULE_LOCALISATION_GOALLOCALISATION_HPP
#define MODULE_LOCALISATION_GOALLOCALISATION_HPP

#include <nuclear>
#include <vector>

#include "GoalModel.hpp"

#include "utility/math/filter/UKF.hpp"

namespace module::localisation {
    class GoalLocalisation : public NUClear::Reactor {
    private:
        struct Config {
            /// @brief UKF config
            struct UKF {
                struct Noise {
                    struct Measurement {
                        Eigen::Matrix2d position = Eigen::Matrix2d::Zero();
                    } measurement{};
                    struct Process {
                        Eigen::Vector2d position = Eigen::Vector2d::Zero();
                        Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
                    } process{};
                } noise{};
                struct InitialCovariance {
                    Eigen::Vector2d position = Eigen::Vector2d::Zero();
                    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
                } initial_covariance{};
            } ukf{};

            /// @brief The maximum distance a measurement or other goal can be from another goal to be associated
            double association_distance = 0.0;

            /// @brief The maximum number of times a goal can be missed consecutively before it is removed
            int max_missed_count = 0;

        } cfg;

        struct TrackedGoal {
            /// @brief Time of the last time update
            NUClear::clock::time_point last_time_update = NUClear::clock::now();
            /// @brief Unscented Kalman Filter for this goal
            utility::math::filter::UKF<double, GoalModel> ukf{};
            /// @brief Whether the goal was and should have been seen in the last vision update
            bool seen = true;
            /// @brief The number of times the goal has been undetected in a row
            long missed_count = 0;
            /// @brief A unique identifier for the goal
            const unsigned long id;

            /// @brief Constructor that sets the state for the UKF
            TrackedGoal(const Eigen::Vector3d& initial_rGWw, const Config::UKF& cfg_ukf, const unsigned long next_id)
                : id(next_id) {
                NUClear::log<NUClear::DEBUG>("Making goal with id: ", id);
                GoalModel<double>::StateVec initial_state = Eigen::Matrix<double, 4, 1>::Zero();
                initial_state.rGWw                        = initial_rGWw.head<2>();

                ukf.set_state(initial_state.getStateVec(),
                              GoalModel<double>::StateVec(cfg_ukf.initial_covariance.position).asDiagonal());
                ukf.model.process_noise = GoalModel<double>::StateVec(cfg_ukf.noise.process.position);
            }

            // Get the goal's position in world space
            Eigen::Vector2d get_rGWw() const {
                return GoalModel<double>::StateVec(ukf.get_state()).rGWw;
            };
        };

        /// @brief List of tracked goals
        std::vector<TrackedGoal> tracked_goals{};

        /// @brief The next id to assign to a goal
        /// This variable will increase by one each time a new goal is added
        /// As it is unbounded, an unsigned long is used to store it
        unsigned long next_id = 0;

    public:
        /// @brief Called by the powerplant to build and setup the GoalLocalisation reactor.
        explicit GoalLocalisation(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Tests if this goal measurement is associated with a tracked goal or if it is a new goal
        /// @param vision_goal The goal detection from the vision system
        void data_association(const Eigen::Vector3d& rGWw);
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_GOALLOCALISATION_HPP
