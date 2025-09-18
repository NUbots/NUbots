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
                 const NUClear::id_t& id_,
                 const Classification& classification_,
                 const std::type_index& type_,
                 std::shared_ptr<NUClear::threading::Reaction> reaction_)
            : group(group_), id(id_), classification(classification_), type(type_), reaction(std::move(reaction_)) {}

        /// The provider group this provider belongs to
        ProviderGroup& group;
        /// The id for this provider. Will either be reaction->id or a generated one for root providers
        NUClear::id_t id;
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
