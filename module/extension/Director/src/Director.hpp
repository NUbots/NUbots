#ifndef MODULE_EXTENSION_DIRECTOR_HPP
#define MODULE_EXTENSION_DIRECTOR_HPP

#include <nuclear>
#include <typeindex>

#include "extension/Behaviour.hpp"

namespace module::extension {

    class Director : public NUClear::Reactor {

    private:
        /**
         * A provider instance which is able to execute a specific task type
         */
        struct Provider {

            /**
             * The data held by a when condition in order to monitor when the condition will be met, as well as to
             * compare it to causing statements to determine if those causing statements would satisfy this provider
             */
            struct WhenCondition {
                WhenCondition(const std::type_index& type, bool (*validator)(const int&), int (*current)())
                    : type(type), validator(validator), current(current) {}

                /// The type of state that this condition is checking
                std::type_index type;
                /// Expression to determine if the passed state is valid
                bool (*validator)(const int&);
                /// Function to get the current state
                int (*current)();
                /// The reaction handle which is monitoring this state for this condition
                ReactionHandle handle;
            };

            Provider(const ::extension::behaviour::commands::ProviderAction& action,
                     const std::shared_ptr<NUClear::threading::Reaction>& reaction)
                : action(action), reaction(reaction), active(false) {}

            /// The action type that this provider is to be used for
            ::extension::behaviour::commands::ProviderAction action;
            /// The reaction object to run in order to run this provider
            std::shared_ptr<NUClear::threading::Reaction> reaction;
            /// Any when conditions that are required by this provider
            std::vector<WhenCondition> when;
            /// A list of types and states that are caused by running this provider
            std::map<std::type_index, int> causing;
            /// A boolean that is true if this provider is currently active and running (can make subtasks)
            bool active;
        };

        /**
         * Adds a provider for a type if it does not already exist
         *
         * @param data_type The type_index of the data type to provide for
         * @param r The reaction that will be run to provide
         * @throws std::runtime_error when there is more than one Provides in an on statement
         */
        void add_provider(const std::type_index& data_type,
                          const std::shared_ptr<NUClear::threading::Reaction>& r,
                          const ::extension::behaviour::commands::ProviderAction& action);

        /**
         * Removes a provider for a type if it exists
         * @param The id of the reaction we want to remove providers for
         * @throws std::runtime_error when the reaction does not provide anything
         */
        void remove_provider(const uint64_t& id);

    public:
        /// Called by the powerplant to build and setup the Director reactor.
        explicit Director(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// A list of providers grouped by type
        std::multimap<std::type_index, Provider> providers;
        /// Maps a Reaction::id to what it provides
        std::map<uint64_t, std::multimap<std::type_index, Provider>::iterator> reactions;

        /// A list of reaction_task_ids to director_task objects, once the provider has finished running it will emit
        /// all these as a pack so that the director can work out when providers change which subtasks they emit
        std::multimap<uint64_t, ::extension::behaviour::commands::DirectorTask> pack_builder;
    };

}  // namespace module::extension

#endif  // MODULE_EXTENSION_DIRECTOR_HPP
