#ifndef UTILITY_LOCALISATION_OCCUPANCYMAP_HPP
#define UTILITY_LOCALISATION_OCCUPANCYMAP_HPP

#include <Eigen/Core>
#include <queue>

namespace module::localisation {

    template <typename Scalar>
    class OccupancyMap {

    public:
        /// @brief Getter for the map data
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> get_map() {
            return map;
        }

        /// @brief Get the width (number of columns) of the map
        int get_width() {
            return map.cols();
        }

        /// @brief Get the length (number of rows) of the map
        int get_length() {
            return map.rows();
        }

        /// @brief Get the value of the map at the specified coordinates or return -1 if the coordinates are outside
        /// the map bounds
        /// @param x The x-coordinate of the map
        /// @param y The y-coordinate of the map
        /// @return The value of the map at the specified coordinates
        Scalar get_occupancy_value(int x, int y) {
            // Check if the observation is within the map
            if (x < 0 || x >= get_length() || y < 0 || y >= get_width()) {
                return -1;
            }
            else {
                return map(x, y);
            }
        }

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

        /**
         * @brief Add a rectangle to the map with width of 1 grid cell
         * @param X0 The x-coordinate of the origin of the rectangle
         * @param Y0 The y-coordinate of the origin of the rectangle
         * @param width The width of the rectangle
         * @param height The height of the rectangle
         */
        void add_rectangle(int x0, int y0, int width, int height) {
            // Add the top and bottom lines
            add_horizontal_line(x0, y0, width);
            add_horizontal_line(x0, y0 + height - 1, width);
            // Add the left and right lines
            add_vertical_line(x0, y0, height);
            add_vertical_line(x0 + width - 1, y0, height);
        }

        /**
         * @brief Add a rectangle to the map with specified inner width
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
                        map.block(i, start_x, 1, end_x - start_x + 1) =
                            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Ones(1, end_x - start_x + 1);
                    }
                }
            }

            for (int i = X0 - radius; i <= X0 + radius; i++) {
                if (i >= 0 && i < map.cols()) {
                    int width   = inner_width / 2;
                    int start_y = Y0 - width;
                    int end_y   = Y0 + width;
                    if (start_y >= 0 && end_y < map.rows()) {
                        map.block(start_y, i, end_y - start_y + 1, 1) =
                            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Ones(end_y - start_y + 1, 1);
                    }
                }
            }
        }

        /**
         * @brief Replace map with a new map with occupancy values which encode the minimum distance to the
         * closest occupied cell
         * @param map The pre-filled Map with occupancy values of 0 or 1
         * @param grid_size The size of the grid cells
         */
        void create_distance_map(Scalar grid_size) {
            // Initialize the distance map with infinite distances
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> dist_map =
                Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Constant(map.rows(),
                                                                                map.cols(),
                                                                                std::numeric_limits<float>::infinity());

            // Create a queue to hold the cells to be visited
            std::queue<std::pair<int, int>> q;

            // Mark all occupied cells as visited and set their distances to 0
            for (int y = 0; y < map.rows(); y++) {
                for (int x = 0; x < map.cols(); x++) {
                    if (map(y, x) == 1) {
                        dist_map(y, x) = 0;
                        q.push(std::make_pair(y, x));
                    }
                }
            }

            // Run BFS algorithm to compute the distances of all unoccupied cells to the nearest occupied cell
            while (!q.empty()) {
                std::pair<int, int> curr = q.front();
                q.pop();
                int y = curr.first;
                int x = curr.second;

                // Update the distances of the neighbors of the current cell
                for (int j = -1; j <= 1; j++) {
                    for (int i = -1; i <= 1; i++) {
                        // Skip the current cell
                        if (i == 0 && j == 0) {
                            continue;
                        }

                        int new_y = y + j;
                        int new_x = x + i;

                        // Check if the neighbor is within the map boundaries
                        if (new_y >= 0 && new_y < map.rows() && new_x >= 0 && new_x < map.cols()) {
                            float weight = std::sqrt(std::pow(i, 2) + std::pow(j, 2));

                            // Check if the neighbor is unoccupied and has a shorter distance through the current cell
                            if (map(new_y, new_x) == 0 && dist_map(new_y, new_x) > dist_map(y, x) + weight) {
                                dist_map(new_y, new_x) = dist_map(y, x) + weight;
                                q.push(std::make_pair(new_y, new_x));
                            }
                        }
                    }
                }
            }

            // Replace the original map with the distance map
            map = dist_map * grid_size;
        }

    private:
        /// @brief Eigen matrix which stores the map data
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> map;
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_MAP_HPP
