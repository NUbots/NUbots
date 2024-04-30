/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "Director.hpp"

#include "extension/Configuration.hpp"

namespace module::extension {

    using component::DirectorTask;
    using component::Provider;
    using component::ProviderGroup;
    using ::extension::Configuration;
    using ::extension::behaviour::RunInfo;
    using ::extension::behaviour::commands::BehaviourTask;
    using ::extension::behaviour::commands::CausingExpression;
    using ::extension::behaviour::commands::NeedsExpression;
    using ::extension::behaviour::commands::ProviderDone;
    using ::extension::behaviour::commands::ProvideReaction;
    using ::extension::behaviour::commands::WhenExpression;
    using Unbind = NUClear::dsl::operation::Unbind<ProvideReaction>;


    thread_local RunInfo::RunReason Director::current_run_reason = RunInfo::RunReason::OTHER_TRIGGER;

    /**
     * This message gets emitted when a state we are monitoring is updated.
     * When a `When` condition is being waited on a reaction will start monitoring that state.
     * When it updates we check to see if this would now satisfy the conditions and if it does we emit
     */
    struct StateUpdate {
        StateUpdate(const uint64_t& provider_id_, const std::type_index& type_, const int& state_)
            : provider_id(provider_id_), type(type_), state(state_) {}

        /// The reaction id of the Provider which was waiting on this type
        uint64_t provider_id;
        /// The enum type that this enum was listening for
        std::type_index type;
        /// The new value for the enum
        int state;
    };

    void Director::add_provider(const ProvideReaction& provide) {

        // Create if it doesn't already exist
        if (!groups.contains(provide.type)) {
            groups.emplace(provide.type, ProviderGroup(provide.type));
        }
        auto& group = groups.at(provide.type);

        // Check if we already inserted this element as a Provider
        if (providers.contains(provide.reaction->id)) {
            throw std::runtime_error("You cannot have multiple Provider DSL words in a single on statement.");
        }

        auto provider = std::make_shared<Provider>(group,
                                                   provide.reaction->id,
                                                   Provider::Classification(provide.classification),
                                                   provide.type,
                                                   provide.reaction);
        group.providers.push_back(provider);
        providers.emplace(provide.reaction->id, provider);
    }

    void Director::remove_provider(const uint64_t& id) {

        // If we can find it, erase it
        auto it = providers.find(id);
        if (it != providers.end()) {

            // Get the relevant elements
            auto provider = it->second;
            auto& group   = provider->group;

            // Unbind any when state monitors
            for (auto& when : provider->when) {
                when->handle.unbind();
            }

            // Erase the Provider from this group
            group.providers.erase(std::remove(group.providers.begin(), group.providers.end(), provider),
                                  group.providers.end());

            // Now we need to deal with the cases where this Providers was in use when it was unbound
            if (provider == group.active_provider) {
                if (group.providers.empty()) {
                    // This is now an error, there are no Providers to service the task
                    log<NUClear::ERROR>("The last Provider for a type was removed while there were still tasks for it");
                }
                else {
                    // Reevaluate the group to see if the loss of this provider changes anything
                    // Since we already removed the provider from the group it won't get reassigned
                    reevaluate_group(group);
                }
            }

            // If there are no Providers left in this group erase it too
            if (group.providers.empty()) {
                groups.erase(provider->type);
            }
            // Erase from our list of Providers
            providers.erase(id);
        }
    }

    std::shared_ptr<component::Provider> Director::get_root_provider(const std::type_index& root_type) {
        // Create a root provider for this task if one doesn't already exist and use it
        if (!groups.contains(root_type)) {
            groups.emplace(root_type, ProviderGroup(root_type));
        }
        auto& group = groups.at(root_type);
        if (group.providers.empty()) {
            // We subtract from unique_id_source here so that we fill numbers from the top while regular
            // reaction_ids are filling from the bottom
            uint64_t unique = --unique_id_source;
            auto provider =
                std::make_shared<Provider>(group, unique, Provider::Classification::ROOT, root_type, nullptr);
            group.providers.push_back(provider);
            providers.emplace(unique, provider);
            group.active_provider = provider;
        }
        auto root_provider = group.providers.front();

        return root_provider;
    }

    void Director::add_when(const WhenExpression& when) {
        auto it = providers.find(when.reaction->id);
        if (it != providers.end()) {
            auto provider = it->second;

            // Can't add when to a Start or Stop
            if (provider->classification == Provider::Classification::STOP
                || provider->classification == Provider::Classification::START) {
                throw std::runtime_error("You cannot use the 'When' DSL word with Start or Stop.");
            }

            // If the initial read throws an exception then it starts as false
            bool current = false;
            try {
                // This throws an exception when the state hasn't been emitted yet
                current = when.validator(when.current());
            }
            catch (...) {
            }

            // The when condition object
            auto w  = std::make_shared<component::Provider::WhenCondition>(when.type, when.validator, current);
            auto id = when.reaction->id;

            // Add a reaction that will listen for changes to this state and notify the director
            w->handle = when.binder(*this, [this, id, w](const int& state) {
                // Only update if the validity changes
                bool valid = w->validator(state);
                if (valid != w->current) {
                    w->current = valid;
                    emit<Scope::DIRECT>(std::make_unique<StateUpdate>(id, w->type, state));
                }
            });

            // Add it to the list of when conditions
            provider->when.push_back(w);
        }
        else {
            throw std::runtime_error("When statements must come after a Provide statement");
        }
    }

    void Director::add_causing(const CausingExpression& causing) {
        auto it = providers.find(causing.reaction->id);
        if (it != providers.end()) {
            auto provider = it->second;

            // Can't add causing to a Start or Stop
            if (provider->classification == Provider::Classification::STOP
                || provider->classification == Provider::Classification::START) {
                throw std::runtime_error("You cannot use the 'Causing' DSL word with Start or Stop.");
            }

            provider->causing.emplace(causing.type, causing.resulting_state);
        }
        else {
            throw std::runtime_error("Causing statements must come after a Provide statement");
        }
    }

    void Director::add_needs(const NeedsExpression& needs) {
        auto it = providers.find(needs.reaction->id);

        if (it != providers.end()) {
            auto provider = it->second;

            // Can't add needs to a Start or Stop
            if (provider->classification == Provider::Classification::STOP
                || provider->classification == Provider::Classification::START) {
                throw std::runtime_error("You cannot use the 'Needs' DSL word with Start or Stop.");
            }

            provider->needs.emplace(needs.type);
        }
        else {
            throw std::runtime_error("Needs statements must come after a Provide statement");
        }
    }

    Director::Director(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        if (source != nullptr) {
            throw std::runtime_error("Multiple behaviour directors are not allowed.");
        }
        source = this;

        on<Configuration>("Director.yaml").then("Configure", [this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        // Removes all the Providers for a reaction when it is unbound
        on<Trigger<Unbind>>().then("Remove Provider", [this](const Unbind& unbind) {  //
            std::lock_guard<std::recursive_mutex> lock(director_mutex);
            remove_provider(unbind.id);
        });

        // Add a Provider
        on<Trigger<ProvideReaction>>().then("Add Provider", [this](const ProvideReaction& provide) {
            std::lock_guard<std::recursive_mutex> lock(director_mutex);
            add_provider(provide);
        });

        // Add a when expression to this Provider
        on<Trigger<WhenExpression>>().then("Add When", [this](const WhenExpression& when) {  //
            std::lock_guard<std::recursive_mutex> lock(director_mutex);
            add_when(when);
        });

        // Add a causing condition to this Provider
        on<Trigger<CausingExpression>>().then("Add Causing", [this](const CausingExpression& causing) {
            std::lock_guard<std::recursive_mutex> lock(director_mutex);
            add_causing(causing);
        });

        // Add a needs relationship to this Provider
        on<Trigger<NeedsExpression>>().then("Add Needs", [this](const NeedsExpression& needs) {  //
            std::lock_guard<std::recursive_mutex> lock(director_mutex);
            add_needs(needs);
        });

        // A task has arrived, either it's a root task so we send it off immediately, or we build up our pack for when
        // the Provider has finished executing
        on<Trigger<BehaviourTask>>().then("Director Task", [this](const BehaviourTask& t) {
            std::lock_guard<std::recursive_mutex> lock(director_mutex);
            // Make our own mutable director task from the behaviour task
            auto task = std::make_shared<DirectorTask>(t);

            // Root level task, make the pack immediately and send it off to be executed as a root task
            if (!providers.contains(task->requester_id)) {
                // Get the root provider from the task root type
                auto root_provider = get_root_provider(t.root_type);

                // Modify the task we received to look like it came from this provider
                task->requester_id = root_provider->id;

                emit(std::make_unique<TaskPack>(TaskPack(root_provider, {task})));
            }
            else {
                auto& p = providers.at(task->requester_id);
                auto id = p->reaction->identifiers.name;
                if (p->classification == Provider::Classification::START) {
                    log<NUClear::WARN>("The task",
                                       task->name,
                                       "cannot be executed as Provider",
                                       id,
                                       "is a start provider.");
                }
                else if (p->classification == Provider::Classification::STOP) {
                    log<NUClear::WARN>("The task",
                                       task->name,
                                       "cannot be executed as Provider",
                                       id,
                                       "is a stop provider.");
                }
                // Everything is fine
                else {
                    pack_builder.emplace(task->requester_task_id, task);
                }
            }
        });

        // This reaction runs when a Provider finishes to send off the task pack to the main director
        on<Trigger<ProviderDone>>().then("Package Tasks", [this](const ProviderDone& done) {
            std::lock_guard<std::recursive_mutex> lock(director_mutex);
            // Get all the tasks that were emitted by this provider and send it as a task pack
            auto range  = pack_builder.equal_range(done.requester_task_id);
            auto pack   = std::make_unique<TaskPack>();
            pack->first = providers.at(done.requester_id);
            for (auto it = range.first; it != range.second; ++it) {
                pack->second.push_back(it->second);
            }

            // Sort the task pack so highest priority tasks come first
            // We sort by direct priority not challenge priority since they're all the same pack
            std::stable_sort(pack->second.rbegin(), pack->second.rend(), [](const auto& a, const auto& b) {
                return direct_priority(a, b);
            });

            // Emit the task pack
            emit(pack);

            // Erase the task pack builder for this id
            pack_builder.erase(done.requester_task_id);
        });

        // A state that we were monitoring is updated, we might be able to run the task now
        on<Trigger<StateUpdate>>().then("State Updated", [this](const StateUpdate& update) {
            std::lock_guard<std::recursive_mutex> lock(director_mutex);
            // Get the group that had a state update
            auto p  = providers.at(update.provider_id);
            auto& g = p->group;

            // Go check if this state update has changed any of the tasks that are queued
            reevaluate_group(g);
        });

        // We have a new task pack to run
        on<Trigger<TaskPack>, Sync<Director>>().then("Run Task Pack", [this](const TaskPack& pack) {  //
            std::lock_guard<std::recursive_mutex> lock(director_mutex);
            run_task_pack(pack);
        });
    }

    Director::~Director() {
        // Remove this director as the information source
        source = nullptr;
    }

}  // namespace module::extension
