#include "Director.hpp"

#include "extension/Configuration.hpp"

namespace module {
    namespace extension {

        using ::extension::Configuration;
        using ::extension::behaviour::commands::CausingExpression;
        using ::extension::behaviour::commands::DirectorTask;
        using ::extension::behaviour::commands::ProviderAction;
        using ::extension::behaviour::commands::ProviderDone;
        using ::extension::behaviour::commands::ProvidesReaction;
        using ::extension::behaviour::commands::WhenExpression;
        using ::NUClear::threading::Reaction;
        using Unbind = NUClear::dsl::operation::Unbind<ProvidesReaction>;

        /**
         * This object bunches up tasks that are coming from a single provider (or a single root level task) so they can
         * be emitted all at once. This lets the director see if a provider is changing what tasks it's requesting
         * between runs.
         * This is especially important if it emits no tasks since there would be no other way to see this.
         */
        struct TaskPack {
            TaskPack(const uint64_t& requester_id, std::vector<std::shared_ptr<const DirectorTask>> tasks)
                : requester_id(requester_id), tasks(std::move(tasks)) {}

            /// The ID of the requester (the providers reaction id)
            uint64_t requester_id;
            /// The list of task objects that are requested
            std::vector<std::shared_ptr<const DirectorTask>> tasks;
        };

        /**
         * This message gets emitted when a state we are monitoring is updated.
         * When a `When` condition is being waited on a reaction will start monitoring that state, when it updates we
         * emit one of these so we can decide if we want to take action based on that.
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
         */
        void Director::add_provider(const std::type_index& data_type,
                                    const std::shared_ptr<Reaction>& r,
                                    const ProviderAction& action) {

            // Look for our specific item
            auto range = providers.equal_range(data_type);
            auto it    = std::find_if(range.first, range.second, [&](auto item) { return item.second.reaction == r; });

            // No item means add a new one
            if (it == range.second) {
                it = providers.insert(std::make_pair(data_type, Provider(action, r)));
                reactions.insert(std::make_pair(r->id, it));
            }
            else {
                throw std::runtime_error(
                    "You cannot have multiple provider DSL words (Provides, Entering, Leaving) in a single statement.");
            }
        }

        /**
         * Removes a provider from our list when it is unbound
         *
         * @param id the reaction id for the provider to be removed
         */
        void Director::remove_provider(const uint64_t& id) {

            // If we can find it, erase it
            auto it = reactions.find(id);
            if (it != reactions.end()) {

                // Unbind any when state monitors
                for (auto& when : it->second->second.when) {
                    when.handle.unbind();
                }

                // Erase from the list
                providers.erase(it->second);
                reactions.erase(it);
            }
            else {
                throw std::runtime_error("Attempted to remove a Provider that was not loaded");
            }
        }

        Director::Director(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Configuration>("Director.yaml").then("Configure", [this](const Configuration& config) {
                // clang-format off
                std::string lvl = config["log_level"].as<std::string>();
                if (lvl == "TRACE")      { this->log_level = NUClear::TRACE; }
                else if (lvl == "DEBUG") { this->log_level = NUClear::DEBUG; }
                else if (lvl == "INFO")  { this->log_level = NUClear::INFO;  }
                else if (lvl == "WARN")  { this->log_level = NUClear::WARN;  }
                else if (lvl == "ERROR") { this->log_level = NUClear::ERROR; }
                else if (lvl == "FATAL") { this->log_level = NUClear::FATAL; }
                // clang-format on
            });

            // Removes all the providers for a reaction when it is unbound
            on<Trigger<Unbind>, Sync<Director>>().then("Remove Provider", [this](const Unbind& unbind) {  //
                remove_provider(unbind.id);
            });

            // Add a provider provider
            on<Trigger<ProvidesReaction>, Sync<Director>>().then("Add Provider", [this](const ProvidesReaction& r) {  //
                add_provider(r.type, r.reaction, r.action);
            });

            // Add a when expression to this provider
            on<Trigger<WhenExpression>, Sync<Director>>().then("Add When", [this](const WhenExpression& r) {
                auto it = reactions.find(r.reaction->id);
                if (it != reactions.end()) {

                    // Add a reaction that will listen for changes to this state and notify the director
                    auto id               = r.reaction->id;
                    auto type             = r.type;
                    ReactionHandle handle = r.binder(*this, [this, id, type](const int& state) {  //
                        emit(std::make_unique<StateUpdate>(id, type, state));
                    });
                    handle.disable();

                    // Add it to the list of when conditions
                    it->second->second.when.emplace_back(r.type, r.validator, r.current, handle);
                }
                else {
                    throw std::runtime_error(
                        "When statements can only come after a Provides, Entering or Leaving statement");
                }
            });

            // Add a causing condition to this provider
            on<Trigger<CausingExpression>, Sync<Director>>().then("Add Causing", [this](const CausingExpression& r) {
                auto it = reactions.find(r.reaction->id);
                if (it != reactions.end()) {
                    it->second->second.causing.insert(std::make_pair(r.type, r.resulting_state));
                }
                else {
                    throw std::runtime_error(
                        "Causing statements can only come after a Provides, Entering or Leaving statement");
                }
            });

            // A task has arrived, either it's a root task and send it off, or build up our pack for when it's done
            on<Trigger<DirectorTask>, Sync<DirectorTask>>().then(
                "Director Task",
                [this](std::shared_ptr<const DirectorTask> task) {
                    // Root level task, make the TaskPack immediately and send it off to be executed as a root task
                    if (reactions.count(task->requester_id) == 0) {
                        emit(std::make_unique<TaskPack>(task->requester_task_id,
                                                        std::vector<std::shared_ptr<const DirectorTask>>({task})));
                    }
                    // Check if this provider is active and allowed to make subtasks
                    else if (reactions[task->requester_id]->second.active) {
                        pack_builder.emplace(task->requester_task_id, task);
                    }
                    else {
                        // Throw an error so the user can see what a fool they are being
                        throw std::runtime_error(fmt::format(
                            "The task {} cannot be executed as the provider {} is not active and cannot make subtasks",
                            task->name,
                            reactions[task->requester_id]->second.reaction->identifier[0]));
                    }
                });

            // This reaction runs when a provider finishes to send off the task pack to the main director
            on<Trigger<ProviderDone>, Sync<DirectorTask>>().then("Package Tasks", [this](const ProviderDone& done) {
                // Get all the tasks that were emitted by this provider and send it as a task pack
                auto e = pack_builder.equal_range(done.requester_task_id);
                std::vector<std::shared_ptr<const DirectorTask>> tasks;
                for (auto it = e.first; it != e.second; ++it) {
                    tasks.emplace_back(it->second);
                }

                // Sort the task pack so highest priority tasks come first
                std::sort(tasks.begin(), tasks.end(), [](const auto& a, const auto& b) {
                    return a->priority < b->priority;
                });

                emit(std::make_unique<TaskPack>(done.requester_id, std::move(tasks)));

                // Erase the task pack builder for this id
                pack_builder.erase(done.requester_task_id);
            });

            // A state that we were monitoring is updated, we might be able to run the reaction now
            on<Trigger<StateUpdate>, Sync<Director>>().then("State Updated", [this](const StateUpdate& update) {
                // Find the reaction that is mentioned in the update
                // If this new state makes things valid then run the task
            });

            // We do things when we receive tasks to run
            on<Trigger<TaskPack>, Sync<Director>>().then("Run Task Pack", [this](const TaskPack& pack) {
                // This is a root task
                if (reactions.count(pack.requester_id) == 0) {
                    // Root tasks only ever have one task in the pack
                    const auto& task = pack.tasks[0];

                    // TODO add this task to the root task list

                    // If we submit a root task with 0 priority we remove it from the root tasks
                    if (task->priority == 0) {
                        // TODO remove this task from the root task list
                        return;
                    }
                }

                // This task is not active and is not allowed to run subtasks
                else if (!reactions[pack.requester_id]->second.active) {
                    // Throw an error so the user can see what a fool they are being
                    throw std::runtime_error(fmt::format(
                        "The task {} cannot be executed as the provider {} is not active and cannot make subtasks",
                        pack.name,
                        reactions[pack.requester_id]->second.reaction->identifier[0]));
                }

                // Loop through each task
                for (const auto& task : pack.tasks) {
                    // All task executions happen after we have evaluated the actions for all tasks in this pack

                    // Possibilities

                    // case Task is a Task::Done task
                    //      Rerun the provider which is a parent to this task with the same data (but with a done flag)
                    //      If we are a root task then delete the task from the list

                    // case Provider is available (no When condition that limits it) and not in use
                    //      Execute any entering with no causing first
                    //      Otherwise execute main provider

                    // case Provider is in use by us
                    //      Check if running an entering for a causing and it's not met yet run the entering again
                    //      else run normal provider

                    // case Provider is in use by a root task with higher priority than us
                    //      Put our task in this providers queue
                    //          we can start this task:
                    //              if the other task priority drops
                    //              if the other task stops running

                    // case we have a Provider but it is limited by a When condition and not in use
                    //      Enable the ReactionHandle which monitors the when condition for this provider
                    //      Add this task to the queue of tasks to be executed when the provider's conditions are met
                    //
                    //      See if there is a leaving that provides a casing for our type we can use
                    //          The algorithm has major a bug here.
                    //          If the kick engine wants to be "Statically Stable" and the walk engine has a `Leaving`
                    //          provider that causes this, how does the algorithm know that later down the line the kick
                    //          engine would take over the walk engines IK controls making it have to stop running
                    //          running its leaving reaction first. The algorithm might need to have more information
                    //          provided to it
                    //      See if there is an entering for this provider that provides the causing and run that
                    //

                    // case Provider is in use by a root task with lower priority than us
                    //      We take control of this provider and put the old task in the queue of tasks to be executed
                    //      by this provider
                    //
                    //      We traverse up the tree and find the first optional path that we took (first negative
                    //      priority) or the root, and we move all the tasks into the queue of tasks to execute
                    //      If any of these tasks that we are kicking off have a leaving reaction we  we are done with
                    //      this task pack if any of these tasks have an empty leaving reaction we run that to ensure
                    //      that we have a clean exit
                }

                // If this provider was previously active and has tasks that are no longer running
                //      check if there is another task that was queued at a lower priority than us and run it
            });
        }
    }  // namespace extension
}  // namespace module
