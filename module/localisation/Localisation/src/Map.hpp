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
    };


}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_MAP_HPP
