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
