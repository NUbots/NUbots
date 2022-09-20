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
#ifndef EXTENSION_BEHAVIOUR_INFORMATIONSOURCE_HPP
#define EXTENSION_BEHAVIOUR_INFORMATIONSOURCE_HPP

#include <memory>
#include <typeindex>

namespace extension::behaviour::information {

    /**
     * @brief This class is an abstract class that allows the static Behaviour DSL to access the relevant state from the
     * algorithm that is backing it.
     *
     * Extend this class to provide whatever information is needed for outputs in the DSL.
     */
    class InformationSource {
    protected:
        /// This static variable gets set by whatever class is acting as the information source
        static InformationSource* source;

        /**
         * @brief Internal get task data function. Is overridden by the class that is acting as the information source.
         *
         * @param reaction_id the reaction id of the reaction that is asking for data
         *
         * @return a void shared pointer to the data for this reaction
         */
        virtual std::shared_ptr<void> _get_task_data(const uint64_t& reaction_id) = 0;

    public:
        virtual ~InformationSource() = default;

        /**
         * @brief Static method that redirects to the singleton instance providing the InformationSource
         *
         * @param reaction_id the reaction id of the reaction that is asking for data
         *
         * @return a void shared pointer to the data for this reaction
         */
        static std::shared_ptr<void> get_task_data(const uint64_t& reaction_id) {
            return source->_get_task_data(reaction_id);
        }
    };

}  // namespace extension::behaviour::information

#endif  // EXTENSION_BEHAVIOUR_INFORMATIONSOURCE_HPP
