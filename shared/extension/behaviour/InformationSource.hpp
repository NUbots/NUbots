/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
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
            /// The provider has been started (will happen on on<Started<X>>)
            STARTED,
            /// The provider has been stopped (will happen on on<Stopped<X>>)
            STOPPED,
            /// A subtask has finished and emitted a Done message
            SUBTASK_DONE,
            /// Another task requires a causing from this provider and pushed it to run
            PUSHED
        };

        /// The reason we executed
        RunReason run_reason;
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
