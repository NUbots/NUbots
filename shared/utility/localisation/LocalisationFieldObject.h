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

#ifndef UTILITY_LOCALISATION_LOCALISATIONFIELDOBJECT_H
#define UTILITY_LOCALISATION_LOCALISATIONFIELDOBJECT_H

#include <iomanip>
#include <armadillo>

namespace utility {
namespace localisation {

enum class LFOId {
    kInvalid,
    kBall,
    kGoalYL,
    kGoalYR,
    kGoalBL,
    kGoalBR,
};

class LocalisationFieldObject {
private:
    arma::vec2 location_;
    LFOId id_;
    std::string name_;

public:
    LocalisationFieldObject() { } // Necessary?

    LocalisationFieldObject(arma::vec2 location, LFOId id, const std::string& name)
        : location_(location), id_(id), name_(name) { }

    LFOId id() const { return id_; }
    arma::vec2 location() const { return location_; }
    std::string name() const { return name_; }

    friend std::ostream& operator<<(std::ostream &os, const LocalisationFieldObject& o) {
        return os
            << "{ "
            << "name: " << o.name_ << ", "
            << "position: ["
                << std::setw(7) << o.location_(0) << ", "
                << std::setw(7) << o.location_(1) << "]"
            << " }";
    }
};

}
}

#endif

