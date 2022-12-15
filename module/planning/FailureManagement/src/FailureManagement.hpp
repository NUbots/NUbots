#ifndef MODULE_PLANNING_FAILUREMANAGEMENT_HPP
#define MODULE_PLANNING_FAILUREMANAGEMENT_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class FailureManagement : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            struct falling {
                struct {
                    double recovery;
                    double unstable;
                    double falling;
                } gyroscope;
                struct {
                    double mean;
                    double recovery;
                    double unstable;
                    double falling;
                } accelerometer;
            }
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the FailureManagement reactor.
        explicit FailureManagement(std::unique_ptr<NUClear::Environment> environment);

    private:
        bool getting_up = false;

        struct {
            double mag = 0.0;
        } gyro;
        struct {
            double mag   = 0.0;
            double angle = 0.0;
        } acc;
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_FAILUREMANAGEMENT_HPP
