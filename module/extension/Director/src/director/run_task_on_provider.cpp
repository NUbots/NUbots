#include "Director.hpp"

namespace module::extension {

    using provider::Provider;

    void Director::run_task_on_provider(const std::shared_ptr<BehaviourTask>& task,
                                        const std::shared_ptr<provider::Provider>& provider) {

        // Update the active provider and task
        auto& group              = provider->group;
        bool run_start_providers = group.active_provider == nullptr;
        group.active_task        = task;

        // If there was no active provider, then this is the first time running this group
        // Therefore we should run the "start" providers
        if (run_start_providers) {
            for (auto& provider : group.providers) {
                if (provider->classification == Provider::Classification::START) {
                    // We have to swap to this as the active provdider so it can actually run
                    group.active_provider = provider;
                    auto task             = provider->reaction->get_task();
                    if (task) {
                        task->run(std::move(task));
                    }
                }
            }
        }

        // Set the active provider ready for running
        group.active_provider = provider;

        // Run the reaction
        auto reaction_task = provider->reaction->get_task();
        if (reaction_task) {
            powerplant.submit(std::move(reaction_task));
        }
    }

}  // namespace module::extension
