#ifndef MODULE_PURPOSE_DEFEND_HPP
#define MODULE_PURPOSE_DEFEND_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/support/FieldDescription.hpp"

namespace module::purpose {

    class Defend : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The clearance distance to avoid other robots
            double clearance = 0.0;
            /// @brief weight for equidistance cost
            double equidist_weight = 0.0;
            /// @brief weight for closeness to neutral point
            double neutral_weight = 0.0;
            /// @brief weight for avoiding other robots
            double avoid_weight = 0.0;
            /// @brief The maximum number of iterations for the optimisation
            int max_iter = 0;
            /// @brief The step size for the optimisation
            double step_size = 0.0;
        } cfg;

        std::vector<Eigen::Vector3d> opponents_in_first_third(const message::localisation::Robots& robots,
                                                              const message::localisation::Field& field,
                                                              const message::support::FieldDescription& field_desc);

    public:
        /// @brief Called by the powerplant to build and setup the Defend reactor.
        explicit Defend(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_DEFEND_HPP
