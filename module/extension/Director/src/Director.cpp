#include "Director.hpp"

#include "extension/Configuration.hpp"

namespace module::extension {

    using ::extension::Configuration;
    using ::extension::behaviour::Task;
    using ::extension::behaviour::commands::CausingExpression;
    using ::extension::behaviour::commands::DirectorTask;
    using ::extension::behaviour::commands::ProviderAction;
    using ::extension::behaviour::commands::ProviderDone;
    using ::extension::behaviour::commands::ProvidesReaction;
    using ::extension::behaviour::commands::WhenExpression;
    using ::NUClear::threading::Reaction;
    using Unbind = NUClear::dsl::operation::Unbind<ProvidesReaction>;

    using TaskPack = std::vector<std::shared_ptr<const DirectorTask>>;

    /**
     * This message gets emitted when a state we are monitoring is updated.
     * When a `When` condition is being waited on a reaction will start monitoring that state.
     * When it updates we check to see if this would now satisfy the conditions and if it does we emit
     */
    struct StateUpdate {
        StateUpdate(const uint64_t& provider_id, const std::type_index& type, const int& state)
            : provider_id(provider_id), type(type), state(state) {}

        /// The provider_id which was waiting on this type
        uint64_t provider_id;
        /// The enum type that this enum was listening for
        std::type_index type;
        /// The new value for the enum
        int state;
    };

    /**
     * Adds a provider to the list of active providers
     *
     * @param data_type the data type that this provider services
     * @param r         the reaction that is used to run this provider
     * @param action    the action type for this provider (entering, leaving, normal)
     * @throws std::runtime_error if `r` has previously had a provider added
     */
    void Director::add_provider(const ProvidesReaction& provides) {

        auto& group = groups[provides.type];

        // Check if we already inserted this element as a provider
        if (providers.count(provides.reaction->id) != 0) {
            throw std::runtime_error("You cannot have multiple provider DSL words in a single on statement.");
        }

        auto provider = std::make_shared<Provider>(provides.action, provides.type, provides.reaction);
        group.providers.emplace_back(provider);
        providers.emplace(provides.reaction->id, provider);
    }

    /**
     * Removes a provider from our list when it is unbound
     *
     * @param id the reaction id for the provider to be removed
     *
     * @throws std::runtime_error if the provider which the id refers to has not been loaded
     */
    void Director::remove_provider(const uint64_t& id) {

        // If we can find it, erase it
        auto it = providers.find(id);
        if (it != providers.end()) {

            // Get the relevant elements
            auto provider = it->second;
            auto& group   = groups[provider->type];

            // Unbind any when state monitors
            for (auto& when : provider->when) {
                when.handle.unbind();
            }

            // Erase the provider from this group
            auto g_it = std::find(group.providers.begin(), group.providers.end(), provider);
            if (g_it != group.providers.end()) {
                group.providers.erase(g_it);
            }

            // Now we need to deal with the cases where this provider was in use when it was unbound
            if (provider->active) {
                if (group.providers.empty()) {
                    // This is now an error, there are no providers to service the task
                    log<NUClear::ERROR>("The last provider for a type was removed while there were still tasks for it");
                }
                else {
                    // TODO run the current task in the group to refresh which provider is chosen
                    // run_task(group.active_task);
                }
            }

            // If there are no providers left in this group erase it too
            if (group.providers.empty()) {
                groups.erase(provider->type);
            }
            // Erase from our list of providers
            providers.erase(id);
        }
        else {
            throw std::runtime_error("Attempted to remove a Provider that was not loaded");
        }
    }

    void Director::add_when(const WhenExpression& when) {
        auto it = providers.find(when.reaction->id);
        if (it != providers.end()) {
            auto provider = it->second;

            // Add a reaction that will listen for changes to this state and notify the director
            auto id               = when.reaction->id;
            auto type             = when.type;
            auto validator        = when.validator;
            ReactionHandle handle = when.binder(*this, [this, id, type, validator](const int& state) {  //
                if (validator(state)) {
                    emit(std::make_unique<StateUpdate>(id, type, state));
                }
            });
            handle.disable();

            // Add it to the list of when conditions
            provider->when.emplace_back(when.type, when.validator, when.current, handle);
        }
        else {
            throw std::runtime_error("When statements must come after a Provides, Entering or Leaving statement");
        }
    }

    void Director::add_causing(const CausingExpression& causing) {
        auto it = providers.find(causing.reaction->id);
        if (it != providers.end()) {
            auto provider = it->second;
            provider->causing.emplace(causing.type, causing.resulting_state);
        }
        else {
            throw std::runtime_error("Causing statements must come after a Provides, Entering or Leaving statement");
        }
    }

    Director::Director(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Director.yaml").then("Configure", [this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        // Removes all the providers for a reaction when it is unbound
        on<Trigger<Unbind>, Sync<Director>>().then("Remove Provider", [this](const Unbind& unbind) {  //
            remove_provider(unbind.id);
        });

        // Add a provider provider
        on<Trigger<ProvidesReaction>, Sync<Director>>().then("Add Provider", [this](const ProvidesReaction& provides) {
            add_provider(provides);
        });

        // Add a when expression to this provider
        on<Trigger<WhenExpression>, Sync<Director>>().then("Add When", [this](const WhenExpression& when) {  //
            add_when(when);
        });

        // Add a causing condition to this provider
        on<Trigger<CausingExpression>, Sync<Director>>().then("Add Causing", [this](const CausingExpression& causing) {
            add_causing(causing);
        });

        // A task has arrived, either it's a root task and send it off, or build up our pack for when it's done
        on<Trigger<DirectorTask>, Sync<DirectorTask>>().then(
            "Director Task",
            [this](std::shared_ptr<const DirectorTask> task) {
                // Root level task, make the TaskPack immediately and send it off to be executed as a root task
                if (providers.count(task->requester_id) == 0) {
                    emit(std::make_unique<TaskPack>(TaskPack({task})));
                }
                // Check if this provider is active and allowed to make subtasks
                else if (providers[task->requester_id]->active) {
                    pack_builder.emplace(task->requester_task_id, task);
                }
                else {
                    // Throw an error so the user can see what a fool they are being
                    log<NUClear::WARN>("The task",
                                       task->name,
                                       "cannot be executed as the provider",
                                       providers[task->requester_id]->reaction->identifier[0],
                                       "is not active and cannot make subtasks");
                }
            });

        // This reaction runs when a provider finishes to send off the task pack to the main director
        on<Trigger<ProviderDone>, Sync<DirectorTask>>().then("Package Tasks", [this](const ProviderDone& done) {
            // Get all the tasks that were emitted by this provider and send it as a task pack
            auto e = pack_builder.equal_range(done.requester_task_id);
            TaskPack tasks;
            for (auto it = e.first; it != e.second; ++it) {
                tasks.emplace_back(it->second);
            }

            // Sort the task pack so highest priority tasks come first
            std::sort(tasks.begin(), tasks.end(), [](const auto& a, const auto& b) {
                return a->priority < b->priority;
            });

            // Emit the task pack
            emit(std::make_unique<TaskPack>(TaskPack(std::move(tasks))));

            // Erase the task pack builder for this id
            pack_builder.erase(done.requester_task_id);
        });

        // A state that we were monitoring is updated, we might be able to run the reaction now
        on<Trigger<StateUpdate>, Sync<Director>>().then("State Updated", [this](const StateUpdate& update) {
            // TODO ALGORITHM SECTION HERE
            // This state update event means the conditions for the provider specified in the update are now met
        });

        // We do things when we receive tasks to run
        on<Trigger<TaskPack>, Sync<Director>>().then("Run Task Pack", [this](const TaskPack& pack) {
            // TODO ALGORITHM SECTION HERE
            // We have just been given a new task pack

            // TODO check if there are tasks that were previously in the pack from this provider and now are gone
            // For those we need to end execution of those tasks

            for (const auto& task : pack) {
                // TODO run this task through the director's algorithm to work out what to do with it
            }
        });
    }

}  // namespace module::extension
