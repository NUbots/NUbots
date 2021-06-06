#include "Director.hpp"

#include "extension/Behaviour.h"
#include "extension/Configuration.h"

namespace module {
    namespace extension {

        using ::extension::Configuration;
        using ::extension::behaviour::commands::CausingExpression;
        using ::extension::behaviour::commands::DirectorTask;
        using ::extension::behaviour::commands::EnteringReaction;
        using ::extension::behaviour::commands::LeavingReaction;
        using ::extension::behaviour::commands::ProvidesReaction;
        using ::extension::behaviour::commands::WhenExpression;
        using ::NUClear::threading::Reaction;
        using Unbind = NUClear::dsl::operation::Unbind<ProvidesReaction>;

        void Director::add_provider(const std::type_index& data_type,
                                    const std::shared_ptr<Reaction>& r,
                                    const Provider::Type& provider_type) {

            // Find our specific item
            auto range = providers.equal_range(data_type);
            auto it    = std::find_if(range.first, range.second, [&](auto item) { return item.second.reaction == r; });

            // No item means add a new one
            if (it == range.second) {
                it = providers.insert(std::make_pair(data_type, Provider(provider_type, r)));
                reactions.insert(std::make_pair(r->id, it));
            }
            else {
                throw std::runtime_error("There cannot be multiple provider statements in a single on statement.");
            }
        }

        void Director::remove_provider(const uint64_t& id) {

            // If we can find it, erase it
            auto it = reactions.find(id);
            if (it != reactions.end()) {
                providers.erase(it->second);
                reactions.erase(it);
            }
            else {
                throw std::runtime_error("Attempted to remove a Provider that was not loaded");
            }
        }

        Director::Director(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // Remove a provider
            on<Trigger<Unbind>>().then([this](const Unbind& unbind) {  //
                remove_provider(unbind.id);
            });

            // Add a normal provider
            on<Trigger<ProvidesReaction>>().then([this](const ProvidesReaction& r) {  //
                add_provider(r.type, r.reaction, Provider::NORMAL);
            });

            // Add an entering provider
            on<Trigger<EnteringReaction>>().then([this](const EnteringReaction& r) {  //
                add_provider(r.type, r.reaction, Provider::ENTERING);
            });

            // Add a leaving provider
            on<Trigger<LeavingReaction>>().then([this](const LeavingReaction& r) {  //
                add_provider(r.type, r.reaction, Provider::LEAVING);
            });

            // Add a when expression to this provider
            on<Trigger<WhenExpression>>().then([this](const WhenExpression& r) {
                auto it = reactions.find(r.reaction->id);
                if (it != reactions.end()) {
                    it->second->second.when.emplace_back(r.type, r.expr, r.current);
                }
                else {
                    throw std::runtime_error(
                        "When statements can only come after a Provides, Entering or Leaving statement");
                }
            });

            // Add a causing condition to this provider
            on<Trigger<CausingExpression>>().then([this](const CausingExpression& r) {
                auto it = reactions.find(r.reaction->id);
                if (it != reactions.end()) {
                    it->second->second.causing.insert(std::make_pair(r.type, r.resulting_state));
                }
                else {
                    throw std::runtime_error(
                        "Causing statements can only come after a Provides, Entering or Leaving statement");
                }
            });

            on<Trigger<DirectorTask>>().then([this](const DirectorTask& task) {
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
