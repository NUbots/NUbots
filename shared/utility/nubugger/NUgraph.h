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

#ifndef NUGRAPH_H
#define NUGRAPH_H

#include <nuclear>
#include "messages/support/nubugger/proto/DataPoint.pb.h"

namespace utility {
namespace nubugger{

    template<typename... Values>
    inline std::unique_ptr<messages::support::nubugger::proto::DataPoint> graph(std::string label, Values... values) {


    	auto dataPoint = std::make_unique<messages::support::nubugger::proto::DataPoint>();
    	dataPoint->set_label(label);
    	for(const auto& value : { float(values)... }) {
    		dataPoint->add_value(value);
    	}
    	return std::move(dataPoint);
    }

}
}

#endif
