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

#ifndef EXTENSION_BEHAVIOUR_INFORMATIONSOURCE_HPP
#define EXTENSION_BEHAVIOUR_INFORMATIONSOURCE_HPP

#include <memory>
#include <typeindex>

namespace extension::behaviour {

    /**
     * Provides information about how and why this provider was executed.
     */
    struct RunInfo {
        enum RunReason {
            /// Something other than the Director caused this reaction to execute
            OTHER_TRIGGER,
            /// A new task has been given to this provider
            NEW_TASK,
            /// The provider has been started (will happen on on<Start<X>>)
            STARTED,
            /// The provider has been stopped (will happen on on<Stop<X>>)
            STOPPED,
            /// A subtask has finished and emitted a Done message
            SUBTASK_DONE,
            /// Another task requires a causing from this provider and pushed it to run
            PUSHED
        };

        /// The reason we executed
        RunReason run_reason;

        /// Whether we are currently in a Done state or not
        bool done;
    };

    /**
     * Provides information about the current state of a provider group
     */
    struct GroupInfo {
        enum RunState {
            /// The group has not emitted the task
            NO_TASK,
            /// The group is running the task
            RUNNING,
            /// The group has the task queued
            QUEUED
        };

        /// The current run state of the group
        RunState run_state;

        /// Whether the task is done or not, regardless of if this provider group is running it
        bool done;
    };

}  // namespace extension::behaviour

namespace extension::behaviour::information {

    /**
     * This class is an abstract class that allows the static Behaviour DSL to access the relevant state from the
     * algorithm that is backing it.
     *
     * Extend this class to provide whatever information is needed for outputs in the DSL.
     */
    class InformationSource {
    protected:
        /// This static variable gets set by whatever class is acting as the information source
        static InformationSource* source;

        /**
         * Internal get task data function. Is overridden by the class that is acting as the information source.
         *
         * @param reaction_id the reaction id of the reaction that is asking for data
         *
         * @return a void shared pointer to the data for this reaction
         */
        virtual std::shared_ptr<void> _get_task_data(const uint64_t& reaction_id) = 0;

        /**
         * Internal get run information. Is overridden by the class that is acting as the information source.
         *
         * @param reaction_id the reaction id of the reaction that is asking for data
         *
         * @return a RunInfo struct containing information about the current run
         */
        virtual RunInfo _get_run_info(const uint64_t& reaction_id) = 0;

        /**
         * Internal get group information. Is overridden by the class that is acting as the information source.
         *
         * @param reaction_id the reaction id of the reaction that is asking for data
         * @param type the type of the provider group group to get information about
         * @param root_type the secondary type to use if this is a root task
         *
         * @return a GroupInfo struct containing information about the current group
         */
        virtual GroupInfo _get_group_info(const uint64_t& reaction_id,
                                          const std::type_index& type,
                                          const std::type_index& root_type) = 0;

    public:
        virtual ~InformationSource() = default;

        /**
         * Gets the data for the passed providers reaction id from the information source
         *
         * @param reaction_id the reaction id of the reaction that is asking for data
         *
         * @return a void shared pointer to the data for this reaction
         */
        static inline std::shared_ptr<void> get_task_data(const uint64_t& reaction_id) {
            return source->_get_task_data(reaction_id);
        }

        /**
         * Gets information about why the current task is running from the information source
         *
         * @param reaction_id the reaction id of the reaction that is asking for data
         *
         * @return a RunInfo struct containing information about the current run
         */
        static inline RunInfo get_run_info(const uint64_t& reaction_id) {
            return source->_get_run_info(reaction_id);
        }

        /**
         * Get the group info object
         *
         * @param reaction_id the reaction id of the reaction that is asking for data
         * @param type the type of the group to get the info for
         * @param root_type the secondary type to use if this is a root task
         *
         * @return the group info object containing information about the requested group
         */
        static inline GroupInfo get_group_info(const uint64_t& reaction_id,
                                               const std::type_index& type,
                                               const std::type_index& root_type) {
            return source->_get_group_info(reaction_id, type, root_type);
        }
    };

}  // namespace extension::behaviour::information

#endif  // EXTENSION_BEHAVIOUR_INFORMATIONSOURCE_HPP
