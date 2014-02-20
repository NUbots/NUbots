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

#ifndef MODULES_FIELDDESCRIPTION_H
#define MODULES_FIELDDESCRIPTION_H

#include <armadillo>

#include "localisation/LocalisationFieldObject.h"
#include "messages/support/Configuration.h"

using messages::support::Configuration;

namespace modules {
namespace localisation {

struct FieldDescriptionConfig {
    static constexpr const char* CONFIGURATION_PATH = "FieldDescription.json";
};

class FieldDescription {
public:
    FieldDescription(Configuration<FieldDescriptionConfig> config);

    // Should be a std::unordered_set?
    std::vector<LocalisationFieldObject> field_objects_;
};

}
}

#endif
