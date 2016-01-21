/*
 * This file is part of NUbots Codebase.
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

#include "GrimReaper.h"

namespace module {
namespace support {

    using NUClear::message::ReactionStatistics;

    GrimReaper::GrimReaper(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {


        on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics& stats) {

            // If there was an exception
            if(stats.exception) {

                std::cout << "There was an exception/segfault in the following module!" << std::endl;

                // Print out loads of information!
                for(auto& id : stats.identifier) {
                    if(!id.empty()) {
                        std::cout << id << std::endl;
                    }
                }

                try {
                    std::rethrow_exception(stats.exception);
                }
                catch(std::exception e) {
                    std::cout << "Exception message is as follows:" << std::endl;
                    std::cout << e.what() << std::endl;
                }
                // FOR ALL THOSE CRAZY PEOPLE WHO THROW RANDOM OBJECTS
                catch(...) {

                }

                // Kill the system
                std::exit(1);
            }
        });
    }
}
}

