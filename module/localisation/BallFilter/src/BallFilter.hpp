
#ifndef MODULE_LOCALISATION_BALLFILTER_HPP
#define MODULE_LOCALISATION_BALLFILTER_HPP

#include <nuclear>

#include "BallModel.hpp"

#include "message/input/Sensors.hpp"
#include "message/vision/Ball.hpp"

#include "utility/math/filter/UKF.hpp"

namespace module::localisation {

    class BallFilter : public NUClear::Reactor {
    private:
        struct Config {
            Config() = default;
            /// @brief UKF config
            struct UKF {
                struct Noise {
                    Noise() = default;
                    struct Measurement {
                        Eigen::Matrix2f position = Eigen::Matrix2f::Zero();
                    } measurement{};
                    struct Process {
                        Eigen::Vector2f position = Eigen::Vector2f::Zero();
                        Eigen::Vector2f velocity = Eigen::Vector2f::Zero();
                    } process{};
                } noise{};
                struct Initial {
                    Initial() = default;
                    struct Mean {
                        Eigen::Vector2f position = Eigen::Vector2f::Zero();
                        Eigen::Vector2f velocity = Eigen::Vector2f::Zero();
                    } mean{};
                    struct Covariance {
                        Eigen::Vector2f position = Eigen::Vector2f::Zero();
                        Eigen::Vector2f velocity = Eigen::Vector2f::Zero();
                    } covariance{};
                } initial{};
            } ukf{};

            /// @brief Initial state of the for the UKF filter
            BallModel<float>::StateVec initial_mean;

            /// @brief Initial covariance of the for the UKF filter
            BallModel<float>::StateVec initial_covariance;

        } cfg;

        /// @brief The time of the last time update
        NUClear::clock::time_point last_time_update;

        /// @brief Unscented Kalman Filter for ball filtering
        utility::math::filter::UKF<float, BallModel> ukf{};

    public:
        /// @brief Called by the powerplant to build and setup the BallFilter reactor.
        explicit BallFilter(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_BALLFILTER_HPP
