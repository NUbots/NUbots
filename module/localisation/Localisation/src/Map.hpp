#ifndef MODULE_LOCALISATION_MAP_HPP
#define MODULE_LOCALISATION_MAP_HPP

#include <Eigen/Core>

namespace module::localisation {

    class Map {
    public:
        // Map data
        Eigen::MatrixXd map;

        /**
         * @brief Resize the map to specified width and length, and set all values to zero
         * @param width The new width of the map
         * @param length The new length of the map
         */
        void resize(int width, int length) {
            map.resize(width, length);
            map.setZero();
        }

        /**
         * @brief Add a horizontal line to the map
         * @param X0 The x-coordinate of the origin of the line
         * @param Y0 The y-coordinate of the origin of the line
         * @param length The length of the line
         * @param width The width of the line
         */
        void add_horizontal_line(int x0, int y0, int length) {
            map.block(y0, x0, 1, length).setOnes();
        }

        /**
         * @brief Add a vertical line to the map
         * @param X0 The x-coordinate of the origin of the line
         * @param Y0 The y-coordinate of the origin of the line
         * @param length The length of the line
         * @param width The width of the line
         */
        void add_vertical_line(int x0, int y0, int length) {
            map.block(y0, x0, length, 1).setOnes();
        }

        /// @brief Add a rectangle to the map
        void add_rectangle(int x0, int y0, int width, int height) {
            // Add the top and bottom lines
            add_horizontal_line(x0, y0, width);
            add_horizontal_line(x0, y0 + height - 1, width);
            // Add the left and right lines
            add_vertical_line(x0, y0, height);
            add_vertical_line(x0 + width - 1, y0, height);
        }

        /**
         * @brief Add a rectangle to the map
         * @param X0 The x-coordinate of the origin of the rectangle
         * @param Y0 The y-coordinate of the origin of the rectangle
         * @param width The width of the rectangle
         * @param height The height of the rectangle
         * @param inner_width The width of the inner rectangle
         */
        void add_rectangle(int x0, int y0, int width, int height, int inner_width) {
            for (int i = 0; i < inner_width; i++) {
                add_rectangle(x0 + i, y0 + i, width - (2 * i), height - (2 * i));
            }
        }

        /**
         * @brief Add a circle to the map with specified origin, radius and line width
         * @param origin The origin (x,y) of the circle
         * @param radius The radius of the circle
         * @param inner_width The width of the circle line
         */
        void add_circle(const int X0, const int Y0, const int radius, const int inner_width) {
            for (int y = Y0 - radius; y <= Y0 + radius; y++) {
                for (int x = X0 - radius; x <= X0 + radius; x++) {
                    if (std::pow(x - X0, 2) + std::pow(y - Y0, 2) <= std::pow(radius, 2)) {
                        int dist = std::sqrt(std::pow(x - X0, 2) + std::pow(y - Y0, 2));
                        if (dist >= radius - inner_width && dist <= radius) {
                            if (x >= 0 && x < map.cols() && y >= 0 && y < map.rows()) {
                                map(y, x) = 1;
                            }
                        }
                    }
                }
            }
        }

        /**
         * @brief Add a rectangular cross to the map with specified origin, radius and line width
         * @param origin The origin (x,y) of the circle
         * @param radius The radius of the circle
         * @param inner_width The width of the circle line
         */
        void add_cross(const int X0, const int Y0, const int radius, const int inner_width) {
            for (int i = Y0 - radius; i <= Y0 + radius; i++) {
                if (i >= 0 && i < map.rows()) {
                    int width   = inner_width / 2;
                    int start_x = X0 - width;
                    int end_x   = X0 + width;
                    if (start_x >= 0 && end_x < map.cols()) {
                        map.block(i, start_x, 1, end_x - start_x + 1) = Eigen::MatrixXd::Ones(1, end_x - start_x + 1);
                    }
                }
            }

            for (int i = X0 - radius; i <= X0 + radius; i++) {
                if (i >= 0 && i < map.cols()) {
                    int width   = inner_width / 2;
                    int start_y = Y0 - width;
                    int end_y   = Y0 + width;
                    if (start_y >= 0 && end_y < map.rows()) {
                        map.block(start_y, i, end_y - start_y + 1, 1) = Eigen::MatrixXd::Ones(end_y - start_y + 1, 1);
                    }
                }
            }
        }

        /**
         * @brief Fills the surrounding cells with a decreasing occupancy value from 1 for a user-specified range.
         * @param map The pre-filled Map
         * @param range The range for the surrounding cells
         */
        void fill_surrounding_cells(int range) {
            int rows = map.rows();
            int cols = map.cols();
            for (int y = 0; y < rows; y++) {
                for (int x = 0; x < cols; x++) {
                    if (map(y, x) == 1) {
                        for (int i = 1; i <= range; i++) {
                            double value = (1.0 * (range - i) / range) * 0.5;
                            for (int j = -i; j <= i; j++) {
                                if (x - i >= 0 && y + j >= 0 && y + j < rows) {
                                    if (map(y + j, x - i) < value) {
                                        map(y + j, x - i) = value;
                                    }
                                }
                                if (x + i < cols && y + j >= 0 && y + j < rows) {
                                    if (map(y + j, x + i) < value) {
                                        map(y + j, x + i) = value;
                                    }
                                }
                                if (y - i >= 0 && x + j >= 0 && x + j < cols) {
                                    if (map(y - i, x + j) < value) {
                                        map(y - i, x + j) = value;
                                    }
                                }
                                if (y + i < rows && x + j >= 0 && x + j < cols) {
                                    if (map(y + i, x + j) < value) {
                                        map(y + i, x + j) = value;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    };


}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_MAP_HPP
