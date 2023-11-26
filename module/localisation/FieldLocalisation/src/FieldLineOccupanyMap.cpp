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
 * Copyright 2023 NUBots <nubots@nubots.net>
 */


#include "FieldLocalisation.hpp"

namespace module::localisation {

    void FieldLocalisation::setup_fieldline_map(const FieldDescription& fd) {
        // Resize map to the field dimensions
        int map_width  = (fd.dimensions.field_width + 2 * fd.dimensions.border_strip_min_width) / cfg.grid_size;
        int map_length = (fd.dimensions.field_length + 2 * fd.dimensions.border_strip_min_width) / cfg.grid_size;
        fieldline_map.resize(map_width, map_length);

        // Calculate line width in grid cells
        int line_width = fd.dimensions.line_width / cfg.grid_size;

        // Add outer field lines
        int field_x0     = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
        int field_y0     = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
        int field_width  = (fd.dimensions.field_width) / cfg.grid_size;
        int field_length = (fd.dimensions.field_length) / cfg.grid_size;
        fieldline_map.add_rectangle(field_x0, field_y0, field_length, field_width, line_width);

        // Add left goal area box
        int left_goal_box_x0 = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
        int left_goal_box_y0 =
            (fd.dimensions.border_strip_min_width + (fd.dimensions.field_width - fd.dimensions.goal_area_width) / 2)
            / cfg.grid_size;
        int left_goal_box_width  = (fd.dimensions.goal_area_length) / cfg.grid_size;
        int left_goal_box_length = (fd.dimensions.goal_area_width) / cfg.grid_size;
        fieldline_map.add_rectangle(left_goal_box_x0,
                                    left_goal_box_y0,
                                    left_goal_box_width,
                                    left_goal_box_length,
                                    line_width);

        // Add right goal area box
        int right_goal_box_x0 =
            (fd.dimensions.border_strip_min_width + fd.dimensions.field_length - fd.dimensions.goal_area_length)
            / cfg.grid_size;
        int right_goal_box_y0     = left_goal_box_y0;
        int right_goal_box_width  = left_goal_box_width;
        int right_goal_box_length = left_goal_box_length;
        fieldline_map.add_rectangle(right_goal_box_x0,
                                    right_goal_box_y0,
                                    right_goal_box_width,
                                    right_goal_box_length,
                                    line_width);

        // Add left penalty area box
        int left_penalty_box_x0 = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
        int left_penalty_box_y0 =
            (fd.dimensions.border_strip_min_width + (fd.dimensions.field_width - fd.dimensions.penalty_area_width) / 2)
            / cfg.grid_size;
        int left_penalty_box_width  = (fd.dimensions.penalty_area_width) / cfg.grid_size;
        int left_penalty_box_length = (fd.dimensions.penalty_area_length) / cfg.grid_size;
        fieldline_map.add_rectangle(left_penalty_box_x0,
                                    left_penalty_box_y0,
                                    left_penalty_box_length,
                                    left_penalty_box_width,
                                    line_width);

        // Add right penalty area box
        int right_penalty_box_x0 =
            (fd.dimensions.border_strip_min_width + fd.dimensions.field_length - fd.dimensions.penalty_area_length)
            / cfg.grid_size;
        int right_penalty_box_y0     = left_penalty_box_y0;
        int right_penalty_box_width  = left_penalty_box_width;
        int right_penalty_box_length = left_penalty_box_length;
        fieldline_map.add_rectangle(right_penalty_box_x0,
                                    right_penalty_box_y0,
                                    right_penalty_box_length,
                                    right_penalty_box_width,
                                    line_width);

        // Add centre line
        int centre_line_x0    = (fd.dimensions.border_strip_min_width + fd.dimensions.field_length / 2) / cfg.grid_size;
        int centre_line_y0    = (fd.dimensions.border_strip_min_width) / cfg.grid_size;
        int centre_line_width = (fd.dimensions.field_width) / cfg.grid_size;
        int centre_line_length = (fd.dimensions.line_width) / cfg.grid_size;
        fieldline_map.add_rectangle(centre_line_x0, centre_line_y0, centre_line_length, centre_line_width, line_width);

        // Add left penalty cross in centre of penalty area
        int left_penalty_cross_x0 =
            (fd.dimensions.border_strip_min_width + fd.dimensions.penalty_mark_distance) / cfg.grid_size;
        int left_penalty_cross_y0 =
            (fd.dimensions.border_strip_min_width + fd.dimensions.field_width / 2) / cfg.grid_size;
        int left_penalty_cross_width = 0.1 / cfg.grid_size;
        fieldline_map.add_cross(left_penalty_cross_x0, left_penalty_cross_y0, left_penalty_cross_width, line_width);

        // Add right penalty cross in centre of penalty area
        int right_penalty_cross_x0 =
            (fd.dimensions.border_strip_min_width + fd.dimensions.field_length - fd.dimensions.penalty_mark_distance)
            / cfg.grid_size;
        int right_penalty_cross_y0    = left_penalty_cross_y0;
        int right_penalty_cross_width = 0.1 / cfg.grid_size;
        fieldline_map.add_cross(right_penalty_cross_x0, right_penalty_cross_y0, right_penalty_cross_width, line_width);

        // Add centre cross in centre of field
        int centre_cross_x0 =
            (fd.dimensions.border_strip_min_width + fd.dimensions.field_length / 2) / cfg.grid_size + line_width / 2;
        int centre_cross_y0    = left_penalty_cross_y0;
        int centre_cross_width = 0.1 / cfg.grid_size;
        fieldline_map.add_cross(centre_cross_x0, centre_cross_y0, centre_cross_width, line_width);

        // Add centre circle
        int centre_circle_x0 = (fd.dimensions.border_strip_min_width + fd.dimensions.field_length / 2) / cfg.grid_size;
        int centre_circle_y0 = (fd.dimensions.border_strip_min_width + fd.dimensions.field_width / 2) / cfg.grid_size;
        int centre_circle_r  = (fd.dimensions.center_circle_diameter / 2) / cfg.grid_size;
        fieldline_map.add_circle(centre_circle_x0, centre_circle_y0, centre_circle_r, line_width);

        // Precompute the distance map
        fieldline_map.create_distance_map(cfg.grid_size);
    }

}  // namespace module::localisation
