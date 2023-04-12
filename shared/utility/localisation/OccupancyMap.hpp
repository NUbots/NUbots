#ifndef UTILITY_LOCALISATION_OCCUPANCYMAP_HPP
#define UTILITY_LOCALISATION_OCCUPANCYMAP_HPP

#include <Eigen/Core>
#include <queue>

namespace module::localisation {

    class OccupancyMap {

    public:
        /// @brief Getter for the map data
        Eigen::MatrixXd get_map();

        /// @brief Get the width (number of columns) of the map
        int get_width();

        /// @brief Get the length (number of rows) of the map
        int get_length();

        /// @brief Get the value of the map at the specified coordinates or return -1 if the coordinates are outside
        /// the map bounds
        /// @param x The x-coordinate of the map
        /// @param y The y-coordinate of the map
        /// @return The value of the map at the specified coordinates
        double get_occupancy_value(int x, int y);

        /**
         * @brief Resize the map to specified width and length, and set all values to zero
         * @param width The new width of the map
         * @param length The new length of the map
         */
        void resize(int width, int length);

        /**
         * @brief Add a horizontal line to the map
         * @param X0 The x-coordinate of the origin of the line
         * @param Y0 The y-coordinate of the origin of the line
         * @param length The length of the line
         * @param width The width of the line
         */
        void add_horizontal_line(int x0, int y0, int length);

        /**
         * @brief Add a vertical line to the map
         * @param X0 The x-coordinate of the origin of the line
         * @param Y0 The y-coordinate of the origin of the line
         * @param length The length of the line
         * @param width The width of the line
         */
        void add_vertical_line(int x0, int y0, int length);

        /**
         * @brief Add a rectangle to the map with width of 1 grid cell
         * @param X0 The x-coordinate of the origin of the rectangle
         * @param Y0 The y-coordinate of the origin of the rectangle
         * @param width The width of the rectangle
         * @param height The height of the rectangle
         */
        void add_rectangle(int x0, int y0, int width, int height);

        /**
         * @brief Add a rectangle to the map with specified inner width
         * @param X0 The x-coordinate of the origin of the rectangle
         * @param Y0 The y-coordinate of the origin of the rectangle
         * @param width The width of the rectangle
         * @param height The height of the rectangle
         * @param inner_width The width of the inner rectangle
         */
        void add_rectangle(int x0, int y0, int width, int height, int inner_width);

        /**
         * @brief Add a circle to the map with specified origin, radius and line width
         * @param origin The origin (x,y) of the circle
         * @param radius The radius of the circle
         * @param inner_width The width of the circle line
         */
        void add_circle(const int X0, const int Y0, const int radius, const int inner_width);

        /**
         * @brief Add a rectangular cross to the map with specified origin, radius and line width
         * @param origin The origin (x,y) of the circle
         * @param radius The radius of the circle
         * @param inner_width The width of the circle line
         */
        void add_cross(const int X0, const int Y0, const int radius, const int inner_width);

        /**
         * @brief Fills the surrounding cells with a decreasing occupancy value from 1 for a user-specified range.
         * @param map The pre-filled Map
         * @param range The range for the surrounding cells
         */
        void fill_surrounding_cells(int range);

        /**
         * @brief Replace map with a new map with occupancy values which encode the minimum distance to the
         * closest occupied cell
         * @param map The pre-filled Map with occupancy values of 0 or 1
         * @param grid_size The size of the grid cells
         */
        void create_distance_map(double grid_size);

    private:
        /// @brief Eigen matrix which stores the map data
        Eigen::MatrixXd map;
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_MAP_HPP
