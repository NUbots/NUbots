/*
 * This file is part of Localisation.
 *
 * Localisation is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Localisation is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Localisation.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_LOCALISATIONFIELDOBJECT_H
#define MODULES_LOCALISATIONFIELDOBJECT_H

#include <armadillo>

namespace modules {
namespace localisation {

class LocalisationFieldObject {
public:
    LocalisationFieldObject();

    enum class LFOId {
        kInvalid,
        kBall,
        kFieldLine,
        kCorner,  
        kCentre_circle,
        kObstacle,
        kGoalL,
        kGoalR,
        kGoalU,  
        kGoalYL,
        kGoalYR,
        kGoalYU,
        kGoalBL,
        kGoalBR,
        kGoalBU,
    };

private:
    arma::vec2 location_;
    LFOId id_;
    std::string name_;
};

}
}

#endif
