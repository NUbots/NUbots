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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "NUsight.h"

#include "message/support/nuclear/ReactionStatistics.h"
#include "utility/time/time.h"

namespace module {
namespace support {
    using ReactionStatisticsProto = message::support::nuclear::ReactionStatistics;
    using NUClear::message::ReactionStatistics;
    using std::chrono::duration_cast;
    using std::chrono::microseconds;
    using utility::time::getUtcTimestamp;

    void NUsight::provideReactionStatistics() {
        // handles["reaction_statistics"].push_back(on<Trigger<ReactionStatistics>>().then([this](const
        // ReactionStatistics& stats) {

        //     ReactionStatisticsProto reactionStatistics;
        //     reactionStatistics.set_reactionid(stats.reactionId);
        //     reactionStatistics.set_taskid(stats.taskId);
        //     reactionStatistics.set_causereactionid(stats.causeReactionId);
        //     reactionStatistics.set_causetaskid(stats.causeTaskId);
        //     reactionStatistics.set_emitted(getUtcTimestamp<microseconds>(stats.emitted));
        //     reactionStatistics.set_started(getUtcTimestamp<microseconds>(stats.started));
        //     reactionStatistics.set_finished(getUtcTimestamp<microseconds>(stats.finished));
        //     reactionStatistics.set_name(stats.identifier[0]);
        //     reactionStatistics.set_triggername(stats.identifier[1]);
        //     reactionStatistics.set_functionname(stats.identifier[2]);

        //     send(reactionStatistics);
        // }));
    }
}  // namespace support
}  // namespace module
