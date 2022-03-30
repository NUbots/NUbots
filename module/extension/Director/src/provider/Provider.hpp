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
#ifndef MODULE_EXTENSION_DIRECTOR_PROVIDER_HPP
#define MODULE_EXTENSION_DIRECTOR_PROVIDER_HPP

#include <nuclear>
#include <utility>

#include "extension/Behaviour.hpp"

namespace module::extension::provider {

    /**
     * A Provider instance which is able to execute a specific task type
     */
    struct Provider {

        /**
         * The data held by a `When` condition in order to monitor when the condition will be met, as well as to
         * compare it to `Causing` statements to determine if those `Causing` statements would satisfy this Provider
         */
        struct WhenCondition {
            WhenCondition(const std::type_index& type_,
                          std::function<bool(const int&)> validator_,
                          std::function<int()> current_,
                          NUClear::threading::ReactionHandle handle_)
                : type(type_)
                , validator(std::move(validator_))
                , current(std::move(current_))
                , handle(std::move(handle_)) {}

            /// The type of state that this condition is checking
            std::type_index type;
            /// Expression to determine if the passed state is valid
            std::function<bool(const int&)> validator;
            /// Function to get the current state
            std::function<int()> current;
            /// The reaction handle which is monitoring this state for a valid condition
            NUClear::threading::ReactionHandle handle;
        };

        Provider(const ::extension::behaviour::commands::ProviderClassification& classification_,
                 const std::type_index& type_,
                 std::shared_ptr<NUClear::threading::Reaction> reaction_)
            : classification(classification_), type(type_), reaction(std::move(reaction_)) {}

        /// The classification of this Provider
        ::extension::behaviour::commands::ProviderClassification classification;
        /// The data type this Provider is for
        std::type_index type;
        /// The reaction object to run in order to run this Provider
        std::shared_ptr<NUClear::threading::Reaction> reaction;
        /// Any when conditions that are required by this Provider
        std::vector<WhenCondition> when;
        /// A list of types and states that are caused by running this Provider
        std::map<std::type_index, int> causing;
        /// A boolean that is true if this Provider is currently active and running (can make subtasks)
        bool active = false;
    };

}  // namespace module::extension::provider

#endif  // MODULE_EXTENSION_DIRECTOR_PROVIDER_HPP
