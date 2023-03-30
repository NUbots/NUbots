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

#ifndef MODULE_EXTENSION_DIRECTOR_PROVIDER_HPP
#define MODULE_EXTENSION_DIRECTOR_PROVIDER_HPP

#include <nuclear>
#include <utility>

#include "extension/Behaviour.hpp"

namespace module::extension::component {

    struct ProviderGroup;

    /**
     * A Provider instance which is able to execute a specific task type
     */
    struct Provider {

        enum class Classification {
            /// Provide providers are typical providers that run when given a task
            PROVIDE = int(::extension::behaviour::commands::ProviderClassification::PROVIDE),
            /// Start providers run once when a provider group first gets a task
            START = int(::extension::behaviour::commands::ProviderClassification::START),
            /// Stop providers run once there are no tasks running for any provider of this group
            STOP = int(::extension::behaviour::commands::ProviderClassification::STOP),
            /// A root virtual provider to use when working with root tasks
            ROOT,
        };

        /**
         * The data held by a `When` condition in order to monitor when the condition will be met, as well as to
         * compare it to `Causing` statements to determine if those `Causing` statements would satisfy this Provider
         */
        struct WhenCondition {
            WhenCondition(const std::type_index& type_,
                          std::function<bool(const int&)> validator_,
                          const bool& current_)
                : type(type_), validator(std::move(validator_)), current(current_) {}
            /// The type of state that this condition is checking
            std::type_index type;
            /// Expression to determine if the passed state is valid
            std::function<bool(const int&)> validator;
            /// The current state of the condition when it was last seen by the Director
            bool current;
            /// The reaction handle which is monitoring this state for a valid condition
            NUClear::threading::ReactionHandle handle;
        };

        Provider(ProviderGroup& group_,
                 const uint64_t& id_,
                 const Classification& classification_,
                 const std::type_index& type_,
                 std::shared_ptr<NUClear::threading::Reaction> reaction_)
            : group(group_), id(id_), classification(classification_), type(type_), reaction(std::move(reaction_)) {}

        /// The provider group this provider belongs to
        ProviderGroup& group;
        /// The id for this provider. Will either be reaction->id or a generated one for root providers
        uint64_t id;
        /// The classification of this Provider
        Classification classification;
        /// The data type this Provider is for
        std::type_index type;
        /// The reaction object to run in order to run this Provider
        std::shared_ptr<NUClear::threading::Reaction> reaction;
        /// Any when conditions that are required by this Provider
        std::vector<std::shared_ptr<WhenCondition>> when;
        /// A list of types and states that are caused by running this Provider
        std::map<std::type_index, int> causing;
        /// A list of provider types that this provider needs in order to run
        std::set<std::type_index> needs;
    };

}  // namespace module::extension::component

#endif  // MODULE_EXTENSION_DIRECTOR_PROVIDER_HPP
