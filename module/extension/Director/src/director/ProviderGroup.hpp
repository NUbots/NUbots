#ifndef MODULE_EXTENSION_DIRECTOR_PROVIDERGROUP_HPP
#define MODULE_EXTENSION_DIRECTOR_PROVIDERGROUP_HPP

#include <map>
#include <typeindex>
#include <vector>

#include "Provider.hpp"

namespace module::extension {

    struct ProviderGroup {
        enum class State {
            /// This provider currently isn't doing anything
            IDLE,
            /// This provider is currently in an "Entering" state
            ENTERING,
            /// This provider group is currently in a "Normal" state
            NORMAL,
            /// This provider group is currently in a "Leaving" state
            LEAVING,
            /// This provider group is not running because it has requirements that are not met, in order to meet
            /// these requirements it is taking control of another currently running provider
            PROXYING
        };

        /// The current state of this provider group
        State state = State::IDLE;
        /// List of individual providers that can service tasks for this type
        std::vector<std::shared_ptr<Provider>> providers;
        /// The current task that is running on this provider
        std::shared_ptr<const ::extension::behaviour::commands::DirectorTask> active_task;
        /// The queue of tasks waiting to run if the situation changes
        std::vector<std::shared_ptr<const ::extension::behaviour::commands::DirectorTask>> task_queue;
        /// List of subordinate provider groups
        std::vector<std::map<std::type_index, ProviderGroup>::iterator> subordinates;
    };

}  // namespace module::extension

#endif  // MODULE_EXTENSION_DIRECTOR_PROVIDERGROUP_HPP
