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

#ifndef MODULES_RESEARCH_AUTOCLASSIFIERPROVIDER_H
#define MODULES_RESEARCH_AUTOCLASSIFIERPROVIDER_H

#include <nuclear>
#include "messages/vision/LookUpTable.h"
#include "messages/vision/proto/LookUpTable.pb.h"
#include "messages/input/Image.h"

namespace modules {
namespace research {

    class AutoClassifierProvider : public NUClear::Reactor {
    public:
        static constexpr const char* CONFIGURATION_PATH = "AutoClassifierProvider.yaml";
        /// @brief Called by the powerplant to build and setup the AutoClassifier reactor.
        explicit AutoClassifierProvider(std::unique_ptr<NUClear::Environment> environment);

    private:
        int ballEdgeBuffer  = 0;
        int goalEdgeBuffer  = 0;
        int fieldEdgeBuffer = 0;
        int lineEdgeBuffer  = 0;

        ReactionHandle ballProvider;
        ReactionHandle goalProvider;
        ReactionHandle fieldProvider;
        ReactionHandle lineProvider;
    };

}
}


#endif