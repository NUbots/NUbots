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
 * Copyright 2022 NUbots <nubots@nubots.net>
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
 * the behaviour system. You shouldn't be emitting any of these messages outside of the behaviour header.
 */
namespace extension::behaviour::commands {

    /**
     * The classification type of the Provider
     */
    enum class ProviderClassification {
        /// Provide providers are typical providers that run when given a task
        PROVIDE,
        /// Start providers run once when a provider group first gets a task
        START,
        /// Stop providers run once there are no tasks running for any provider of this group
        STOP
    };

    template <typename T>
    struct RootType {
        RootType() = delete;
    };

    /**
     * Message used to tell the behaviour system about a new Provides reaction
     */
    struct ProvideReaction {

        /**
         * @brief Construct a new Provide Reaction object to send to the behaviour system
         *
         * @param reaction_         the reaction for this provides object
         * @param type_             the type that this Provider provides for
         * @param classification_   what kind of provider this ProvideReaction is for
         */
        ProvideReaction(std::shared_ptr<NUClear::threading::Reaction> reaction_,
                        const std::type_index& type_,
                        const ProviderClassification& classification_)
            : reaction(std::move(reaction_)), type(type_), classification(classification_) {}

        /// The reaction for this Provider
        std::shared_ptr<NUClear::threading::Reaction> reaction;
        /// The data type that this Provider services
        std::type_index type;
        /// The action type that this Provider is using
        ProviderClassification classification;
    };

    /**
     * Message used to tell the behaviour system about a new When condition on a Provider
     */
    struct WhenExpression {

        /**
         * Constructs a new WhenExpression object to send to the behaviour system
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
     * Message used to tell the behaviour system about a new Causing relationship for this Provider
     */
    struct CausingExpression {

        /**
         * Construct a new Causing Expression object to send to the behaviour system
         *
         * @param reaction_         the reaction this causing condition is for
         * @param type_             the enum type that this causing condition is going to manipulate
         * @param resulting_state_  the state the enum will be in once the causing has succeeded
         */
        CausingExpression(std::shared_ptr<NUClear::threading::Reaction> reaction_,
                          const std::type_index& type_,
                          const int& resulting_state_)
            : reaction(std::move(reaction_)), type(type_), resulting_state(resulting_state_) {}
        /// The Provider reaction that will cause this state
        std::shared_ptr<NUClear::threading::Reaction> reaction;
        /// The enum type that this `When` expression claims to manipulate
        std::type_index type;
        /// The resulting state this Provider is claiming to provide if it is executed
        int resulting_state;
    };

    /**
     * Message used to tell the behaviour system about a new Needs relationship on a Provider
     */
    struct NeedsExpression {

        /**
         * Construct a new Needs Expression object to send to the behaviour system
         *
         * @param reaction_         the reaction this needs relationship is for
         * @param type_             the provider type that this needs relationship is referring to
         */
        NeedsExpression(std::shared_ptr<NUClear::threading::Reaction> reaction_, const std::type_index& type_)
            : reaction(std::move(reaction_)), type(type_) {}
        /// The Provider reaction that has this relationship
        std::shared_ptr<NUClear::threading::Reaction> reaction;
        /// The provider type that this needs relationship is referring to
        std::type_index type;
    };

    /**
     * Message to tell the behaviour system that a Provider task has finished executing
     */
    struct ProviderDone {

        /**
         * @brief Construct a new Provider Done object
         *
         * @param requester_id_      the reaction_id of the NUClear reaction that finished executing
         * @param requester_task_id_ the reaction_task_id of the NUClear reaction task that finished executing
         */
        ProviderDone(const uint64_t& requester_id_, const uint64_t& requester_task_id_)
            : requester_id(requester_id_), requester_task_id(requester_task_id_) {}

        /// The reaction_id of the Provider that finished
        uint64_t requester_id;
        /// The specific task_id of the Provider that finished
        uint64_t requester_task_id;
    };

    /**
     * A task message that is to be sent to the behaviour system for execution
     */
    struct BehaviourTask {

        /**
         * Construct a new Task object to send to the behaviour system
         *
         * @param type_                 the type that this task is for
         * @param root_type_            a secondary type to use if this is a root task
         * @param requester_id_         the id of the reaction that emitted this task
         * @param requester_task_id_    the task_id of the reaction task that emitted this
         * @param data_                 the task data to be sent to the provider
         * @param name_                 a string name for this task for use in debugging
         * @param priority_             the priority that this task is to run with
         * @param optional_             whether this task is optional or not
         */
        BehaviourTask(const std::type_index& type_,
                      const std::type_index& root_type_,
                      const uint64_t& requester_id_,
                      const uint64_t& requester_task_id_,
                      std::shared_ptr<void> data_,
                      std::string name_,
                      const int& priority_,
                      const bool& optional_)
            : type(type_)
            , root_type(root_type_)
            , requester_id(requester_id_)
            , requester_task_id(requester_task_id_)
            , data(std::move(data_))
            , name(std::move(name_))
            , priority(priority_)
            , optional(optional_) {}

        /// The Provider type this task is for
        std::type_index type;
        /// A secondary provider type to use if this is a root task
        std::type_index root_type;
        /// The reaction id of the requester (could be the id of a Provider)
        uint64_t requester_id;
        /// The reaction task id of the requester (if it is a Provider later a ProviderDone will be emitted)
        uint64_t requester_task_id;
        /// The data for the command, (the data that will be given to the Provider) if null counts as no task
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
