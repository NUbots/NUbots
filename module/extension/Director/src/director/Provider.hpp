#ifndef MODULE_EXTENSION_DIRECTOR_PROVIDER_HPP
#define MODULE_EXTENSION_DIRECTOR_PROVIDER_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::extension {

    /**
     * A provider instance which is able to execute a specific task type
     */
    struct Provider {

        /**
         * The data held by a when condition in order to monitor when the condition will be met, as well as to
         * compare it to causing statements to determine if those causing statements would satisfy this provider
         */
        struct WhenCondition {
            WhenCondition(const std::type_index& type,
                          bool (*validator)(const int&),
                          int (*current)(),
                          NUClear::threading::ReactionHandle handle)
                : type(type), validator(validator), current(current), handle(std::move(handle)) {}

            /// The type of state that this condition is checking
            std::type_index type;
            /// Expression to determine if the passed state is valid
            bool (*validator)(const int&);
            /// Function to get the current state
            int (*current)();
            /// The reaction handle which is monitoring this state for a valid condition
            NUClear::threading::ReactionHandle handle;
        };

        Provider(const ::extension::behaviour::commands::ProviderAction& action,
                 const std::type_index& type,
                 const std::shared_ptr<NUClear::threading::Reaction>& reaction)
            : action(action), type(type), reaction(reaction), active(false) {}

        /// The action type that this provider is to be used for
        ::extension::behaviour::commands::ProviderAction action;
        /// The data type this provider is for
        std::type_index type;
        /// The reaction object to run in order to run this provider
        std::shared_ptr<NUClear::threading::Reaction> reaction;
        /// Any when conditions that are required by this provider
        std::vector<WhenCondition> when;
        /// A list of types and states that are caused by running this provider
        std::map<std::type_index, int> causing;
        /// A boolean that is true if this provider is currently active and running (can make subtasks)
        bool active;
    };

}  // namespace module::extension

#endif  // MODULE_EXTENSION_DIRECTOR_PROVIDER_HPP
