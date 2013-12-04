/*
 * This file is part of ScriptOptimizer.
 *
 * ScriptOptimizer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScriptOptimizer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScriptOptimizer.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_RESEARCH_SCRIPTOPTIMIZER_H
#define MODULES_RESEARCH_SCRIPTOPTIMIZER_H

#include <nuclear>

namespace modules {
    namespace research {

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        class ScriptOptimizer : public NUClear::Reactor {
        public:
            explicit ScriptOptimizer(std::unique_ptr<NUClear::Environment> environment);
        };
    }
}
#endif

