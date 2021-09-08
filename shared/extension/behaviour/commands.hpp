/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2021 NUbots <nubots@nubots.net>
 */
#ifndef EXTENSION_BEHAVIOUR_COMMANDS_HPP
#define EXTENSION_BEHAVIOUR_COMMANDS_HPP

#include <functional>
#include <memory>
#include <nuclear>
#include <typeindex>
#include <utility>

/**
 * This namespace holds all of the communication primitives that are used by the Behaviour header to send messages to
 * the Director algorithm. You shouln't be emitting any of these messages outside of the behaviour header.
 */
namespace extension::behaviour::commands {

    /**
     * The classification type of the Provider
     */
    enum class ProviderClassification {
        /// NORMAL Providers are the usual Providers that execute in a loop if needed
        NORMAL,
        /// ENTERING Providers are the first point of entry for a type before the main NORMAL reaction runs
        ENTERING,
        /// LEAVING Providers are executed as a module is relinquishing control
        LEAVING,
    };

    /**
     * Message used to tell the Director about a new Provides reaction
     */
    struct ProvidesReaction {

        /**
         * @brief Construct a new Provides Reaction object to send to the Director
         *
         * @param reaction_         the reaction for this provides object
         * @param type_             the type that this Provider provides for
         * @param classification_   what kind of provider this ProvidesReaction is for
         */
        ProvidesReaction(std::shared_ptr<NUClear::threading::Reaction> reaction_,
                         const std::type_index& type_,
                         const ProviderClassification& classification_)
            : reaction(std::move(reaction_)), type(type_), classification(classification_) {}

        /// The reaction for this Provider
        std::shared_ptr<NUClear::threading::Reaction> reaction;
        /// The data type that this Provider services
        std::type_index type;
        /// The action type that this Provider is using (normal, entering, leaving)
        ProviderClassification classification;
    };

    /**
     * Message used to tell the Director about a new When condition on a Provider
     */
    struct WhenExpression {

        /**
         * Constructs a new WhenExpression object to send to the Director
         *
         * @param reaction_     the reaction this when condition is for
         * @param type_         the enum type that this when condition is monitoring
         * @param validator_    a function that will determine if the passed value satisfies this when condition
         * @param current_      a function that will get the current state of the reaction
         * @param binder_       a function that can be used to bind a reaction that monitors when the state changes
         */
        WhenExpression(std::shared_ptr<NUClear::threading::Reaction> reaction_,
                       const std::type_index& type_,
                       std::function<bool(const int&)> validator_,
                       std::function<int()> current_,
                       std::function<NUClear::threading::ReactionHandle(NUClear::Reactor&,
                                                                        std::function<void(const int&)>)> binder_)
            : reaction(std::move(reaction_))
            , type(type_)
            , validator(std::move(validator_))
            , current(std::move(current_))
            , binder(std::move(binder_)) {}

        /// The Provider reaction that is contingent on this when condition
        std::shared_ptr<NUClear::threading::Reaction> reaction;
        /// The enum type that this when expression is looking at
        std::type_index type;
        /// Function to determine if the passed state is valid
        std::function<bool(const int&)> validator;
        /// Function to get the current state from the global cache
        std::function<int()> current;
        /// Function to bind a reaction to monitor when this state changes
        std::function<NUClear::threading::ReactionHandle(NUClear::Reactor&, std::function<void(const int&)>)> binder;
    };

    /**
     * Message used to tell the Director about a new Causing relationship for this Provider
     */
    struct CausingExpression {

        /**
         * Construct a new Causing Expression object to send to the Director
         *
         * @param reaction          the reaction this causing condition is for
         * @param type              the enum type that this causing condition is going to manipulate
         * @param resulting_state   the state the enum will be in once the causing has succeeded
         */
        CausingExpression(std::shared_ptr<NUClear::threading::Reaction> reaction,
                          const std::type_index& type,
                          const int& resulting_state)
            : reaction(std::move(reaction)), type(type), resulting_state(resulting_state) {}
        /// The Provider reaction that will cause this state
        std::shared_ptr<NUClear::threading::Reaction> reaction;
        /// The enum type that this `When` expression claims to manipulate
        std::type_index type;
        /// The resulting state this Provider is claiming to provide if it is executed
        int resulting_state;
    };

    /**
     * Message to tell the Director that a Provider task has finished executing
     */
    struct ProviderDone {

        /**
         * @brief Construct a new Provider Done object
         *
         * @param requester_id      the reaction_id of the NUClear reaction that finished executing
         * @param requester_task_id the reaction_task_id of the NUClear reaction task that finished executing
         */
        ProviderDone(const uint64_t& requester_id, const uint64_t& requester_task_id)
            : requester_id(requester_id), requester_task_id(requester_task_id) {}

        /// The reaction_id of the Provider that finished
        uint64_t requester_id;
        /// The specific task_id of the Provider that finished
        uint64_t requester_task_id;
    };

    /**
     * A task message that is to be sent to the Director for execution
     */
    struct DirectorTask {

        /**
         * Construct a new Task object to send to the Director
         *
         * @param type                  the type that this task is for
         * @param requester_id          the id of the reaction that emitted this task
         * @param requester_task_id     the task_id of the reaction task that emitted this
         * @param data                  the task data to be sent to the provider
         * @param name                  a string name for this task for use in debugging
         * @param priority              the priority that this task is to run with
         * @param optional              whether this task is optional or not
         */
        DirectorTask(const std::type_index& type_,
                     const uint64_t& requester_id_,
                     const uint64_t& requester_task_id_,
                     std::shared_ptr<void> data_,
                     std::string name_,
                     const int& priority_,
                     const bool& optional_)
            : type(type_)
            , requester_id(requester_id_)
            , requester_task_id(requester_task_id_)
            , data(std::move(data_))
            , name(std::move(name_))
            , priority(priority_)
            , optional(optional_) {}

        /// The Provider type this Director task is for
        std::type_index type;
        /// The reaction id of the requester (could be the id of a Provider)
        uint64_t requester_id;
        /// The reaction task id of the requester (if it is a Provider later a ProviderDone will be emitted)
        uint64_t requester_task_id;
        /// The data for the command, (the data that will be given to the Provider)
        std::shared_ptr<void> data;
        /// A name for this task to be shown in debugging systems
        std::string name;
        /// The priority with which this task is to be executed
        int priority;
        /// If this task is optional
        bool optional;
    };


}  // namespace extension::behaviour::commands

#endif  // EXTENSION_BEHAVIOUR_COMMANDS_HPP
