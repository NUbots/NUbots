/**
 * @file FieldMapFromDescription.hpp
 * @brief Build a FieldMap/FieldDimensions from a NUbots message::support::FieldDescription.
 *
 * This is kept separate from FieldMap.hpp so the estimator core (FieldMap and the measurement/system
 * headers that include it) stays free of the message library: only code that actually has a
 * FieldDescription to hand pulls in the protobuf dependency by including this header.
 */
#ifndef UTILITY_SLAM_FIELDMAPFROMDESCRIPTION_HPP
#define UTILITY_SLAM_FIELDMAPFROMDESCRIPTION_HPP

#include "FieldMap.hpp"

#include "message/support/FieldDescription.hpp"

namespace utility::slam {

    /**
     * @brief FieldDimensions from a NUbots FieldDescription message.
     *
     * Maps message::support::FieldDescription::FieldDimensions (snake_case, metres) onto the
     * estimator's FieldDimensions. Fields the map does not use (goalpost cross-section, crossbar,
     * net, etc.) are ignored.
     */
    inline FieldDimensions field_dimensions(const message::support::FieldDescription& fd) {
        FieldDimensions dims;
        dims.lineWidth            = fd.dimensions.line_width;
        dims.fieldLength          = fd.dimensions.field_length;
        dims.fieldWidth           = fd.dimensions.field_width;
        dims.goalDepth            = fd.dimensions.goal_depth;
        dims.goalWidth            = fd.dimensions.goal_width;
        dims.goalAreaLength       = fd.dimensions.goal_area_length;
        dims.goalAreaWidth        = fd.dimensions.goal_area_width;
        dims.penaltyMarkDistance  = fd.dimensions.penalty_mark_distance;
        dims.centreCircleDiameter = fd.dimensions.center_circle_diameter;
        dims.penaltyAreaLength    = fd.dimensions.penalty_area_length;
        dims.penaltyAreaWidth     = fd.dimensions.penalty_area_width;
        dims.goalpostWidth        = fd.dimensions.goalpost_width;
        dims.borderStripMinWidth  = fd.dimensions.border_strip_min_width;
        return dims;
    }

    /// @brief Build a FieldMap from a NUbots FieldDescription message.
    inline FieldMap field_map(const message::support::FieldDescription& fd) {
        return FieldMap(field_dimensions(fd));
    }

}  // namespace utility::slam

#endif  // UTILITY_SLAM_FIELDMAPFROMDESCRIPTION_HPP
