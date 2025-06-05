#ifndef MODULE_LOCALISATION_MOCAP_HPP
#define MODULE_LOCALISATION_MOCAP_HPP

#include <nuclear>

namespace module::localisation {

    class Mocap : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The rigid body id of the robot in motive software
            uint32_t robot_rigid_body_id = 0;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the Mocap reactor.
        explicit Mocap(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_MOCAP_HPP
