#include "Director.h"

#include "extension/Configuration.h"

namespace module {
namespace extension {

    using extension::Configuration;

    Director::Director(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Director.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Director.yaml
        });

        on<Trigger<EnteringReaction>>().then([this](const EnteringReaction& r) {
            // TODO add an entering flag for this provider
        });

        on<Trigger<LeavingReaction>>().then([this](const LeavingReaction& r) {
            // TODO add a leaving flag for this provider
        });

        on<Trigger<ProvidesReaction>>().then([this](const ProvidesReaction& r) {
            // TODO add a provider flag for this provider
        });

        on<Trigger<WhenExpression>>().then([this](const WhenReaction& r) {
            // TODO add a when expression for this provider
        });

        on<Trigger<CausingExpression>>().then([this](const CausingReaction& r) {
            // TODO add a causing expression for this provider
        });

        on<Trigger<DirectorTask>>().then([this](const DirectorTask& task) {
            // Check which provider emitted this task (if any) to get its scope

            // TODO if the cause_id is not any of our reactions remove it from scope
        });

        // Main director function
        on<Every<90, Per<std::chrono::seconds>>>().then([this] {
            // Sort tasks by priority
            std::sort(global_tasks.begin(), global_tasks.end(), bypriority);

            for (const auto& t : global_tasks) {

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
                //          a leaving reaction that will fulfil our when condition and if so, execute that branch of the
                //          tree. Otherwise, try to find an entering reaction for our current branch that fulfils the
                //          condition and execute that instead.
                //      5th: We have reached a leaf node of our existing tree without changing anything in our subgraph
                //          In this case, we want to run the parent of this component again so that it will generate a
                //          new task for the leaf node. This scheme can be used to allow things to continuously execute
                //          and draw more data as needed. For example, if the task was to move the leg through an IK
                //          pose, once it reached the target position the Provider who asked to move the leg to that
                //          position would execute again, potentially providing a new pose. Or if that provider had
                //          finished its task, it may emit an empty task in which case it's parent would run (perhaps a
                //          walk engine step phase).

                // TODO
                // Basically we have a tree of providers that we can use to complete tasks
                // We take each of our global tasks and based on priority we send them through the tree of providers
                // If a previous task has taken a provider, it is not available for future providers
                // There are also enter/leave/causing that we have to deal with
                // We only execute a task if its command has changed from the last time it ran, or if the child of it's
                // command has no tasks This way we can keep a constant flow of information through the system without
                // running things unnecessarily
                // Also you don't run rerun the parent if you were stuck on an entering/leaving provider, you just
                // switch to the appropriate provider


                // Basic idea is that global task with highest priority gets first grab at the providers
                // If at any point it has a when that is not met, it'll grab the entering or leaving that gives it
                // control However if there is a generic leaving we have to run that too

                // This is a tricky algorithm!
                // What we need to do, is give access to providers based first on their global priority
                // However some of them may have when conditions that we must try to fulfil which may change our path to
                // the entering/leaving tasks that cause our whens to be true To know this we have to go through the
                // current task tree as well to see what is assigned


                // TODO Features
                // when a child has no tasks with priority of their own, rerun the parent of that task
            }
        });
    }
}  // namespace extension
}  // namespace module
