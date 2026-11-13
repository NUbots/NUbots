/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
/**
 * @file FieldMapFromDescription.hpp
 * @brief Build FieldDimensions from a message::support::FieldDescription. Separate from
 *        FieldMap.hpp so the filter core does not depend on the message library.
 */
#ifndef MODULE_LOCALISATION_SRIF_FIELDMAPFROMDESCRIPTION_HPP
#define MODULE_LOCALISATION_SRIF_FIELDMAPFROMDESCRIPTION_HPP

#include "srif/FieldMap.hpp"

#include "message/support/FieldDescription.hpp"

namespace module::localisation::srif {

    /// @brief FieldDimensions from a FieldDescription message
    inline FieldDimensions field_dimensions(const message::support::FieldDescription& fd) {
        FieldDimensions dims;
        dims.line_width             = fd.dimensions.line_width;
        dims.field_length           = fd.dimensions.field_length;
        dims.field_width            = fd.dimensions.field_width;
        dims.goal_depth             = fd.dimensions.goal_depth;
        dims.goal_width             = fd.dimensions.goal_width;
        dims.goal_area_length       = fd.dimensions.goal_area_length;
        dims.goal_area_width        = fd.dimensions.goal_area_width;
        dims.penalty_mark_distance  = fd.dimensions.penalty_mark_distance;
        dims.centre_circle_diameter = fd.dimensions.center_circle_diameter;
        dims.penalty_area_length    = fd.dimensions.penalty_area_length;
        dims.penalty_area_width     = fd.dimensions.penalty_area_width;
        dims.goalpost_width         = fd.dimensions.goalpost_width;
        dims.border_strip_min_width = fd.dimensions.border_strip_min_width;
        return dims;
    }

}  // namespace module::localisation::srif

#endif  // MODULE_LOCALISATION_SRIF_FIELDMAPFROMDESCRIPTION_HPP
