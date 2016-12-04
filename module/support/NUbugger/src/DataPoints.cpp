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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "NUbugger.h"

#include "message/support/nubugger/proto/DataPoint.pb.h"

#include "utility/time/time.h"

namespace module {
namespace support {
    using utility::time::getUtcTimestamp;

    using message::support::nubugger::proto::DataPoint;

    void NUbugger::provideDataPoints() {

        handles["data_point"].push_back(on<Trigger<DataPoint>>().then([this](const DataPoint& dataPoint) {

            uint filterId = dataPointFilterIds.find(dataPoint.label()) == dataPointFilterIds.end()
                ? dataPointFilterIds.insert(std::make_pair(dataPoint.label(), dataPointFilterId++)).first->second
                : dataPointFilterIds[dataPoint.label()];

            send(dataPoint, filterId);
        }));
    }
}
}
