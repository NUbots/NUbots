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

            // Removes all the providers for a reaction when it is unbound
            on<Trigger<Unbind>, Sync<Director>>().then([this](const Unbind& unbind) {  //
                remove_provider(unbind.id);
            });

            // Add a provider provider
            on<Trigger<ProvidesReaction>, Sync<Director>>().then([this](const ProvidesReaction& r) {  //
                add_provider(r.type, r.reaction, r.action);
            });

            // Add a when expression to this provider
            on<Trigger<WhenExpression>, Sync<Director>>().then([this](const WhenExpression& r) {
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
            on<Trigger<CausingExpression>, Sync<Director>>().then([this](const CausingExpression& r) {
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
            on<Trigger<DirectorTask>, Sync<DirectorTask>>().then([this](std::shared_ptr<const DirectorTask> task) {
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
            on<Trigger<ProviderDone>, Sync<DirectorTask>>().then([this](const ProviderDone& done) {
                // Get all the tasks that were emitted by this provider and send it as a task pack
                auto e = pack_builder.equal_range(done.requester_task_id);
                std::vector<std::shared_ptr<const DirectorTask>> tasks;
                for (auto it = e.first; it != e.second; ++it) {
                    tasks.emplace_back(it->second);
                }
                emit(std::make_unique<TaskPack>(done.requester_id, std::move(tasks)));

                // Erase the task pack builder for this id
                pack_builder.erase(done.requester_task_id);
            });

            // A state that we were monitoring is updated, we might be able to run the reaction now
            on<Trigger<StateUpdate>, Sync<Director>>().then([this](const StateUpdate& state) {
                // TODO one of the causing states we are monitoring has updated
            });

            // We do things when we receive tasks to run
            on<Trigger<TaskPack>, Sync<Director>>().then([this](const TaskPack& pack) {
                // We should have a current state tree (which initially may be empty), a transitional state tree and a
                // new state tree. The current state tree represents what was last executed.
                //
                // The transitional state tree is used when we are trying to change states, but there are some leaving
                // reactions in the current state tree that are preventing us from changing. Often this tree will be
                // empty after each iteration as completed transitions are moved into the current state tree.
                //
                // The new state tree is the one we are generating and it represents our desired state change.
                // If at some point while generating the new state tree we reach an impasse where we are unable to
                // change states due to a condition our progress so far is stored within the transitional state tree. In
                // these times we will instead run a modified version of the current state tree.
                //
                // The algorithm for generating is recursively done for each level using the global tasks first, and
                // then local tasks from each provider.
                //
                // We take the set of tasks and the current state tree and use them to work out which reactions need to
                // be updated. As we generate a new state tree we compare it to the current state tree. While doing this
                // there are a few situations that can occur
                //      1st: the node we generated is identical to the transitional, or if it does not exist current
                //      state tree and with identical data.
                //          Do nothing (do not execute this reaction) and continue down the tree
                //      2nd: the node we generated is identical to the transitional, or if it does not exist current
                //      state tree, but with different data
                //          Execute this node of the graph, and then continue to generate the state tree
                //      3rd: the node we generated is different to the current state tree
                //          From the bottom of the now dead branch of the current/transitional state tree up, gather any
                //          leaving reactions but do not execute them yet as we may not actually be leaving depending on
                //          what is lower in our own tree
                //      4th: We want to generate a different state tree, but we are blocked by a when condition
                //          The branches of the new state tree that we have generated thus far becomes a part of the
                //          transitional tree. Then from the bottom of the now dead branch up of the current state tree
                //          up, in addition to any we gathered in a 3rd step from above check to see if any of them have
                //          a leaving reaction that will fulfill our when condition and if so, execute that branch of
                //          the tree. Otherwise, try to find an entering reaction for our current branch that fulfills
                //          the condition and execute that instead.
                //      5th: We want to generate a different state tree, but we are blocked by another task with a
                //      higher global priority, or by another task from our current provider with a higher local
                //      priority.
                //          We can't continue here, we perform the transition technique like when there is a when
                //          condition that we cannot execute. This branch ends here.
                //          TODO what happens when our local provider has another task to execute as well that isn't
                //          blocked? Do we allow that second task to execute or perform a block one block all. Both
                //          cases seem to exist (walk engine doesn't need arms) so maybe there needs to be a way to emit
                //          groups of tasks where it is accept all, or accept none.
                //      6th: We have reached a leaf node of our existing tree without changing anything in our subgraph
                //          In this case, we want to run the parent of this component again so that it will generate a
                //          new task for the leaf node. This scheme can be used to allow things to continuously execute
                //          and draw more data as needed. For example, if the task was to move the leg through an IK
                //          pose, once it reached the target position the Provider who asked to move the leg to that
                //          position would execute again, potentially providing a new pose. Or if that provider had
                //          finished its task, it may emit an empty task in which case it's parent would run (perhaps a
                //          walk engine step phase).
            });
        }
    }  // namespace extension
}  // namespace module
