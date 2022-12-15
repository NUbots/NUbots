#ifndef MODULE_PLANNING_FAILUREMANAGEMENT_HPP
#define MODULE_PLANNING_FAILUREMANAGEMENT_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class FailureManagement : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            struct {
                struct {
                    struct {
                        double mean;
                        double unstable;
                        double falling;
                        double smoothing;
                    } mag;
                } gyro;
                struct {
                    struct {
                        double mean;
                        double unstable;
                        double falling;
                        double smoothing;
                    } mag;
                    struct {
                        double mean;
                        double unstable;
                        double falling;
                        double smoothing;
                    } angle;
                } acc;
            } falling;
            struct {
                double angle;
                double gyro_recovery;
                double acc_recovery;
            } getup;
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
